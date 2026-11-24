"""Manus 장갑: SDK Integrated 모드로 서버 PC의 동글에 직접 접속해 조작자 양손의 손가락 굽힘을 측정한다.

- MANUS Core 없이 SDK가 동글을 직접 연다 (Integrated). 접속은 SDK 내부 가상 호스트(SDKIntegrated)에 붙는 방식이다
- 장갑 스레드가 SDK 초기화 → 호스트 접속 → 상태 점검 루프(자동 페어링·로그)를 돈다
  초기화·접속에 실패하면 config.MANUS_RECONNECT_S 주기로 재시도한다
- 측정값은 raw skeleton 스트림(손 노드 위치)으로 받는다. 노드 정보(체인 종류·관절 종류)로 손목과
  손가락별 관절·끝 위치를 골라 MediaPipe 순서 키포인트 21개를 만든다 (노드 정보는 장갑별로 1회 조회해 둔다)
- landscape에서 페어링·허용된 장갑의 손만 쓴다
- 키포인트를 손 리타게팅(hand_retargeter.py, dex-retargeting DexPilot)으로 공통 형식 손가락 6개(0=펼침, 1=굽힘)로
  변환한다. 리타게팅은 손별로 config.HAND_RETARGET_RATE_HZ 이하로 돈다
- SDK가 없으면(이미지 미설치) 장갑은 비활성이고 서버는 계속 돈다 — 손가락은 조작 샘플 fingers로 대체된다

SDK 콜백은 SDK 내부 스레드에서 호출되므로 측정값은 락으로 보호하고, 소비자(제어 루프)는 스냅샷만 가져간다.
"""

import logging
import threading
import time

import numpy as np

from teleop.glove.hand_retargeter import KEYPOINT_COUNT, HandRetargeter
from teleop.utils import config

logger = logging.getLogger("manus_glove")

SIDES = ("left", "right")

# SDK ChainType: 손 체인과 손가락 체인 → 키포인트 시작 인덱스 (엄지 1, 검지 5, 중지 9, 약지 13, 새끼 17)
_CHAIN_HAND = 13
_FINGER_CHAIN_KEYPOINT_BASE = {5: 1, 6: 5, 7: 9, 8: 13, 9: 17}

# 손가락 체인당 키포인트 수 (끝 쪽 노드 4개: 네 손가락 MCP·PIP·DIP·끝, 엄지 CMC·MCP·IP·끝)
_FINGER_KEYPOINTS = 4

# SDK FingerJointType Invalid (노드 정렬에서 제외)
_FINGER_JOINT_INVALID = 0

# 손 크기 로그용 키포인트: 손목, 중지 끝
_WRIST_KEYPOINT = 0
_MIDDLE_TIP_KEYPOINT = 12

# SDK Side enum 값 → 손 이름
_SDK_SIDE = {1: "left", 2: "right"}

# SDK DevicePairedState enum: Paired, Unpaired
_PAIRED_STATE_PAIRED = 1
_PAIRED_STATE_UNPAIRED = 2

# SDK LogSeverity → 파이썬 로그 레벨 (Debug·Info는 debug, Warn은 warning, Error는 error)
_SDK_LOG_LEVEL = {0: logging.DEBUG, 1: logging.DEBUG, 2: logging.WARNING, 3: logging.ERROR}

# 장갑 스레드 종료 대기 시간(초)
_STOP_TIMEOUT_S = 5.0


def keypoint_node_indices(node_infos: list[tuple[int, int, int, int]]) -> list[int] | None:
    """raw skeleton 노드 정보 [(nodeId, parentId, chainType, fingerJointType)]로 키포인트 21개의 노드 배열 인덱스를 반환한다.

    손목 = 손 체인 노드 (없으면 부모가 목록에 없는 루트 노드), 손가락 = 체인별 관절 종류 순으로 끝 쪽 노드 4개.
    구성할 수 없으면 None을 반환한다.
    """
    keypoint_nodes: list[int | None] = [None] * KEYPOINT_COUNT
    finger_nodes = {base: [] for base in _FINGER_CHAIN_KEYPOINT_BASE.values()}
    node_ids = {node_id for node_id, _, _, _ in node_infos}

    # 노드 분류: 손 체인(손목) 후보, 루트 노드, 손가락 체인별 (관절 종류, 배열 인덱스)
    root_node = None
    for node_index, (node_id, parent_id, chain_type, joint_type) in enumerate(node_infos):
        if chain_type == _CHAIN_HAND and keypoint_nodes[0] is None:
            keypoint_nodes[0] = node_index
        if parent_id not in node_ids or parent_id == node_id:
            root_node = node_index if root_node is None else root_node
        if chain_type in _FINGER_CHAIN_KEYPOINT_BASE and joint_type != _FINGER_JOINT_INVALID:
            finger_nodes[_FINGER_CHAIN_KEYPOINT_BASE[chain_type]].append((joint_type, node_index))

    # 손목: 손 체인 노드가 없으면 루트 노드
    if keypoint_nodes[0] is None:
        keypoint_nodes[0] = root_node
    if keypoint_nodes[0] is None:
        return None

    # 손가락: 관절 종류 순 정렬 후 끝 쪽 4개
    for keypoint_base, nodes in finger_nodes.items():
        if len(nodes) < _FINGER_KEYPOINTS:
            return None
        for offset, (_, node_index) in enumerate(sorted(nodes)[-_FINGER_KEYPOINTS:]):
            keypoint_nodes[keypoint_base + offset] = node_index
    return keypoint_nodes


def glove_allowed(glove_id: int) -> bool:
    """장갑 id가 config.MANUS_ALLOWED_GLOVE_IDS에 허용되면 True (목록이 비어 있으면 모두 허용)."""
    return not config.MANUS_ALLOWED_GLOVE_IDS or glove_id in config.MANUS_ALLOWED_GLOVE_IDS


class ManusGlove:
    """SDK 접속 스레드와 양손 손가락 측정 상태를 소유한다."""

    def __init__(self):
        """상태를 초기화한다. SDK 로드·접속은 start() 이후 장갑 스레드에서 한다."""
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._initialized = False
        self._connected = False

        # SDK 바인딩(lib·ffi·콜백 래퍼)과 SDK enum (로드 시 채움)
        self._lib = None
        self._ffi = None
        self._sdk = None
        self._return_code_enum = None
        self._axis_view_enum = None
        self._axis_polarity_enum = None
        self._side_enum = None

        # 측정값·장갑 목록 보호용 락 (SDK 콜백 스레드·제어 루프·장갑 스레드 공유)
        self._lock = threading.Lock()

        # 손별 손 리타게팅 (SDK 콜백 스레드 전용)
        self._retargeters = {side: HandRetargeter(side) for side in SIDES}
        # 손별 마지막 리타게팅 시각(monotonic, SDK 콜백 스레드 전용)
        self._retarget_mono = {side: 0.0 for side in SIDES}

        # 장갑별 키포인트 노드 배열 인덱스 {(장갑 id, 노드 수): 인덱스 목록 | None} (SDK 콜백 스레드 전용)
        self._keypoint_nodes: dict[tuple[int, int], list[int] | None] = {}

        # 손별 손가락 굽힘(0-1), 키포인트(21,3), 마지막 수신 시각·키포인트 변화 시각(monotonic)
        self._fingers: dict[str, list[float] | None] = {side: None for side in SIDES}
        self._keypoints: dict[str, np.ndarray | None] = {side: None for side in SIDES}
        self._rx_mono = {side: 0.0 for side in SIDES}
        self._keypoints_changed_mono = {side: 0.0 for side in SIDES}
        self._skeleton_count = 0
        self._retarget_count = 0

        # 통계 주기 동안 버린 항목 수: 미등록 장갑 id, 비유한 위치, 조회 실패·범위를 벗어난 스트림·landscape
        self._unmapped_entry_count = 0
        self._nonfinite_count = 0
        self._invalid_stream_count = 0

        # landscape 장갑 목록 [{id, side, paired, battery}], 장갑 id → 손 이름, 동글 수
        self._gloves: list[dict] = []
        self._glove_sides: dict[int, str] = {}
        self._dongle_count = 0

        # 장갑 스레드 전용: 마지막으로 로그한 장갑 목록, 페어링 실패 id, 마지막 통계 시각
        self._logged_gloves: tuple | None = None
        self._pair_failed: set[int] = set()
        self._stats_t = 0.0

    @property
    def connected(self) -> bool:
        """SDK 호스트(Integrated)에 접속되어 있으면 True."""
        return self._connected

    # ── 외부 인터페이스 ──────────────────────────────────────────────────────

    def start(self):
        """장갑 스레드를 시작한다 (SDK 초기화·접속 실패해도 스레드가 재시도한다). 반환값 없음."""
        # 이전 스레드가 실행·종료 중이면 시작 생략
        if self._thread is not None and self._thread.is_alive():
            if self._stop_event.is_set():
                logger.warning("Manus 장갑 이전 스레드가 아직 종료 중 — 시작 생략")
            return

        # 장갑 스레드 전용 상태 초기화
        self._logged_gloves = None
        self._pair_failed = set()
        self._stats_t = 0.0

        # 장갑 스레드 시작
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="manus-glove", daemon=True)
        self._thread.start()
        logger.info("Manus 장갑 시작 (SDK Integrated 모드)")

    def stop(self):
        """장갑 스레드를 멈추고 SDK를 종료한다. 반환값 없음."""
        # 정지 요청 후 스레드 종료 대기
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=_STOP_TIMEOUT_S)

            # 제한 시간 안에 끝나지 않으면 스레드를 유지한 채 경고
            if self._thread.is_alive():
                logger.warning(f"Manus 장갑 스레드가 {_STOP_TIMEOUT_S:.0f}초 안에 종료되지 않음 (SDK 종료 대기 중)")
                return
            self._thread = None
        logger.info("Manus 장갑 정지")

    def fingers(self) -> dict[str, list[float]]:
        """신선한(MANUS_STALE_S 이내) 손의 손가락 굽힘 {side: [6] 0-1}를 반환한다 (없는 손은 빠진다)."""
        now = time.monotonic()

        # 신선한 손만 복사해 반환
        with self._lock:
            return {
                side: list(self._fingers[side])
                for side in SIDES
                if self._fingers[side] is not None and now - self._rx_mono[side] < config.MANUS_STALE_S
            }

    # ── 장갑 스레드 ──────────────────────────────────────────────────────────

    def _run(self):
        """SDK 로드 후 초기화·접속·자동 페어링·상태 로그 루프를 돈다 (장갑 스레드 본체)."""
        if not self._load_sdk():
            return
        try:
            while not self._stop_event.is_set():
                try:
                    # 미초기화면 초기화 재시도
                    if not self._initialized:
                        self._initialized = self._initialize_sdk()
                        if not self._initialized:
                            self._stop_event.wait(config.MANUS_RECONNECT_S)
                            continue

                    # 미접속이면 접속 재시도
                    if not self._connected:
                        self._connected = self._connect()
                        if not self._connected:
                            self._stop_event.wait(config.MANUS_RECONNECT_S)
                            continue

                    # 장갑 상태 점검·자동 페어링·통계 로그
                    self._log_glove_changes()
                    self._pair_unpaired_gloves()
                    self._log_stats()
                    self._stop_event.wait(config.MANUS_POLL_S)
                except Exception:
                    # 점검 중 예외를 기록하고 대기 후 루프 유지
                    logger.exception("Manus 장갑 스레드 오류 — 재시도 대기")
                    self._stop_event.wait(config.MANUS_RECONNECT_S)
        finally:
            # 초기화된 SDK 종료
            self._connected = False
            if self._initialized:
                self._initialized = False
                self._lib.CoreSdk_ShutDown()
                logger.info("Manus SDK 종료")

    def _load_sdk(self) -> bool:
        """manus_sdk 패키지와 SDK enum을 불러온다. 반환: 성공 여부 (실패면 장갑 비활성)."""
        # 패키지 로드
        try:
            import manus_sdk
            from manus_sdk.generated._enums import AxisPolarity, AxisView, SDKReturnCode, Side
        except ImportError as e:
            logger.error(f"manus_sdk를 불러올 수 없음, 장갑 비활성: {e}")
            return False

        # 네이티브 바인딩 확인
        if manus_sdk.lib is None:
            logger.error("manus_sdk 네이티브 바인딩 로드 실패, 장갑 비활성 (libzmq5·libgomp1·cffi 설치 확인)")
            return False

        # 바인딩·콜백 래퍼·enum 보관
        self._lib = manus_sdk.lib
        self._ffi = manus_sdk.ffi
        self._sdk = manus_sdk.ManusSDK()
        self._return_code_enum = SDKReturnCode
        self._axis_view_enum = AxisView
        self._axis_polarity_enum = AxisPolarity
        self._side_enum = Side
        return True

    def _initialize_sdk(self) -> bool:
        """Integrated 모드 초기화·콜백 등록·좌표계 설정을 한다. 반환: 성공 여부 (실패면 SDK를 종료해 둔다)."""
        return_codes = self._return_code_enum

        # Integrated 모드 초기화
        result = self._lib.CoreSdk_InitializeIntegrated()
        if result != return_codes.Success:
            logger.error(f"Manus SDK Integrated 초기화 실패: {return_codes(result).name} — 재시도 대기")
            return False

        # 콜백 등록 (SDK 내부 스레드에서 호출된다)
        registrations = (
            ("log", self._sdk.register_log_callback, self._on_sdk_log),
            ("landscape", self._sdk.register_landscape_callback, self._on_landscape),
            ("raw skeleton", self._sdk.register_raw_skeleton_callback, self._on_raw_skeleton),
            ("disconnected", self._sdk.register_disconnected_callback, self._on_disconnected),
        )
        for callback_name, register, callback in registrations:
            result = register(callback)
            if result != return_codes.Success:
                logger.error(f"Manus {callback_name} 콜백 등록 실패: {return_codes(result).name} — 재시도 대기")
                self._lib.CoreSdk_ShutDown()
                return False

        # 좌표계 설정
        coord_system = self._ffi.new("CoordinateSystemVUH*")
        coord_system.view = self._axis_view_enum.XFromViewer
        coord_system.up = self._axis_polarity_enum.PositiveZ
        coord_system.handedness = self._side_enum.Right
        coord_system.unitScale = 1.0
        result = self._lib.CoreSdk_InitializeCoordinateSystemWithVUH(coord_system[0], True)
        if result != return_codes.Success:
            logger.error(f"Manus 좌표계 설정 실패: {return_codes(result).name} — 재시도 대기")
            self._lib.CoreSdk_ShutDown()
            return False

        logger.info("Manus SDK Integrated 초기화 완료")
        return True

    def _connect(self) -> bool:
        """SDK 호스트(Integrated 가상 호스트)를 찾아 접속한다. 반환: 성공 여부."""
        return_codes = self._return_code_enum

        # 호스트 탐색
        result = self._lib.CoreSdk_LookForHosts(1, False)
        if result != return_codes.Success:
            logger.warning(f"Manus 호스트 탐색 실패: {return_codes(result).name}")
            return False
        host_count = self._ffi.new("uint32_t*")
        result = self._lib.CoreSdk_GetNumberOfAvailableHostsFound(host_count)
        if result != return_codes.Success or host_count[0] == 0:
            logger.warning("Manus 호스트 없음 — 재시도")
            return False

        # 호스트 목록 조회
        hosts = self._ffi.new("ManusHost[]", host_count[0])
        result = self._lib.CoreSdk_GetAvailableHostsFound(hosts, host_count[0])
        if result != return_codes.Success:
            logger.warning(f"Manus 호스트 목록 조회 실패: {return_codes(result).name}")
            return False

        # 첫 호스트에 접속
        host_name = self._ffi.string(hosts[0].hostName).decode("utf-8", errors="replace")
        result = self._lib.CoreSdk_ConnectToHost(hosts[0])
        if result != return_codes.Success:
            logger.warning(f"Manus 호스트 접속 실패 ({host_name}): {return_codes(result).name}")
            return False
        logger.info(f"Manus 호스트 접속: {host_name}")
        return True

    def _pair_unpaired_gloves(self):
        """landscape에 보이는 허용된 미페어링 장갑을 동글에 페어링한다 (MANUS_AUTO_PAIR일 때만). 반환값 없음."""
        if not config.MANUS_AUTO_PAIR:
            return

        # 허용된 미페어링 장갑 선택
        with self._lock:
            unpaired = [
                glove for glove in self._gloves
                if glove["paired"] == _PAIRED_STATE_UNPAIRED and glove_allowed(glove["id"])
            ]

        # 장갑별 페어링 시도
        for glove in unpaired:
            paired_result = self._ffi.new("bool*")
            result = self._lib.CoreSdk_PairGlove(glove["id"], paired_result)
            if result == self._return_code_enum.Success and paired_result[0]:
                logger.info(f"Manus 장갑 페어링 성공: 0x{glove['id']:X} ({glove['side']})")
                self._pair_failed.discard(glove["id"])
            elif glove["id"] not in self._pair_failed:
                # 실패 로그는 장갑당 1회 남김
                logger.warning(f"Manus 장갑 페어링 실패: 0x{glove['id']:X} ({glove['side']}), 결과 {result}")
                self._pair_failed.add(glove["id"])

    def _log_glove_changes(self):
        """동글 수·장갑 목록(id·손·페어링 상태)이 바뀌면 로그로 남긴다. 반환값 없음."""
        # 현재 동글 수·장갑 목록 스냅샷
        with self._lock:
            glove_state = (
                self._dongle_count,
                tuple((glove["id"], glove["side"], glove["paired"]) for glove in self._gloves),
            )

        # 바뀌지 않았으면 생략
        if glove_state == self._logged_gloves:
            return
        self._logged_gloves = glove_state

        # 목록 로그와 미인식 경고
        dongle_count, gloves = glove_state
        summary = ", ".join(
            f"0x{glove_id:X}({side}, paired={paired})" for glove_id, side, paired in gloves
        ) or "없음"
        logger.info(f"Manus 동글 {dongle_count}개, 장갑 목록: {summary}")
        if dongle_count == 0:
            logger.warning("Manus 동글 미인식 — USB 연결·컨테이너 장치 권한 확인")
        elif not gloves:
            logger.warning("Manus 동글은 인식됐으나 장갑 없음 — 장갑 전원·페어링·수신 거리 확인")

    def _log_stats(self):
        """주기적으로 수신·리타게팅 주기, 배터리, 손별 손 크기·굽힘, 버린 항목, 값 고정을 로그로 남긴다. 반환값 없음."""
        # 통계 주기 확인
        now = time.monotonic()
        if now - self._stats_t < config.MANUS_STATS_INTERVAL_S:
            return
        elapsed_s = now - self._stats_t if self._stats_t > 0 else config.MANUS_STATS_INTERVAL_S
        self._stats_t = now

        # 카운터·측정값 스냅샷 후 카운터 초기화
        with self._lock:
            skeleton_rate_hz = self._skeleton_count / elapsed_s
            retarget_rate_hz = self._retarget_count / elapsed_s
            self._skeleton_count = 0
            self._retarget_count = 0
            dropped_counts = (self._unmapped_entry_count, self._nonfinite_count, self._invalid_stream_count)
            self._unmapped_entry_count = 0
            self._nonfinite_count = 0
            self._invalid_stream_count = 0
            battery_by_side = {glove["side"]: glove["battery"] for glove in self._gloves}
            snapshot = {
                side: (
                    self._keypoints[side],
                    self._fingers[side],
                    now - self._rx_mono[side],
                    now - self._keypoints_changed_mono[side],
                )
                for side in SIDES
            }

        # 손별 상태: 손 크기(손목→중지 끝, cm) / 굽힘(0-1) little ring middle index thumb_bend thumb_rotate
        side_texts = []
        frozen_sides = []
        for side in SIDES:
            keypoints, fingers, age_s, unchanged_s = snapshot[side]
            if keypoints is None or fingers is None or age_s >= config.MANUS_STALE_S:
                side_texts.append(f"{side}: 미수신")
                continue
            if unchanged_s >= config.MANUS_FROZEN_WARN_S:
                frozen_sides.append(f"{side} {unchanged_s:.0f}초")
            battery_percent = battery_by_side.get(side)
            battery_text = "?" if battery_percent is None else str(battery_percent)
            hand_length_cm = 100.0 * float(np.linalg.norm(keypoints[_MIDDLE_TIP_KEYPOINT] - keypoints[_WRIST_KEYPOINT]))
            finger_text = " ".join(f"{finger:.2f}" for finger in fingers)
            side_texts.append(f"{side}(배터리 {battery_text}%): 손 크기 {hand_length_cm:.1f}cm 굽힘[{finger_text}]")
        logger.info(
            f"Manus raw skeleton {skeleton_rate_hz:.1f} Hz, 리타게팅 {retarget_rate_hz:.1f} Hz | " + " | ".join(side_texts)
        )

        # 버린 항목 경고
        unmapped_count, nonfinite_count, invalid_count = dropped_counts
        if unmapped_count or nonfinite_count or invalid_count:
            logger.warning(
                f"Manus 항목 무시: 페어링·허용 장갑 아님(또는 landscape 이전) {unmapped_count}, "
                f"비유한 위치 {nonfinite_count}, 조회 실패·범위 벗어난 스트림·landscape {invalid_count}"
            )

        # 키포인트가 그대로인 손 경고 (값은 계속 사용)
        if frozen_sides:
            logger.warning(f"Manus 손 위치 변화 없음: {', '.join(frozen_sides)} — 장갑 착용·센서 상태 확인")

    # ── SDK 콜백 (SDK 내부 스레드) ───────────────────────────────────────────

    def _on_raw_skeleton(self, stream_info):
        """raw skeleton 스트림을 손별 키포인트로 만들고 리타게팅해 굽힘으로 보관한다. 반환값 없음."""
        try:
            now = time.monotonic()
            with self._lock:
                glove_sides = dict(self._glove_sides)

            # 스켈레톤별 키포인트 수집 (페어링·허용된 장갑의 손만)
            keypoints_by_side = {}
            unmapped_count = 0
            nonfinite_count = 0
            invalid_count = 0
            for skeleton_index in range(int(stream_info.skeletonsCount)):
                keypoints, status = self._read_keypoints(skeleton_index, glove_sides)
                if status == "unmapped":
                    unmapped_count += 1
                elif status == "nonfinite":
                    nonfinite_count += 1
                elif status == "invalid":
                    invalid_count += 1
                elif keypoints is not None:
                    keypoints_by_side[status] = keypoints

            # 손별 리타게팅 (주기 상한, 미수신 뒤 재개면 리타게팅 상태 초기화)
            fingers_by_side = {}
            for side in keypoints_by_side:
                if now - self._retarget_mono[side] < 1.0 / config.HAND_RETARGET_RATE_HZ:
                    continue
                if now - self._rx_mono[side] >= config.MANUS_STALE_S:
                    self._retargeters[side].reset()
                fingers_by_side[side] = self._retargeters[side].retarget(keypoints_by_side[side])
                self._retarget_mono[side] = now

            # 손별 키포인트·굽힘·수신 시각과 카운터 보관
            with self._lock:
                for side, keypoints in keypoints_by_side.items():
                    if self._keypoints[side] is None or not np.array_equal(keypoints, self._keypoints[side]):
                        self._keypoints_changed_mono[side] = now
                    self._keypoints[side] = keypoints
                    self._rx_mono[side] = now
                for side, fingers in fingers_by_side.items():
                    self._fingers[side] = fingers
                self._skeleton_count += 1
                self._retarget_count += len(fingers_by_side)
                self._unmapped_entry_count += unmapped_count
                self._nonfinite_count += nonfinite_count
                self._invalid_stream_count += invalid_count
        except Exception as e:
            logger.error(f"raw skeleton 콜백 오류: {type(e).__name__}: {e}")

    def _read_keypoints(self, skeleton_index: int, glove_sides: dict[int, str]) -> tuple[np.ndarray | None, str]:
        """스켈레톤 하나의 노드 위치에서 키포인트(21,3)를 만든다 (SDK 콜백 안에서만 호출).

        반환: (키포인트, 손 이름) 또는 (None, 사유 "unmapped"·"invalid"·"nonfinite"·"no_layout").
        """
        return_codes = self._return_code_enum

        # 스켈레톤 정보(장갑 id·노드 수)와 손
        info = self._ffi.new("RawSkeletonInfo*")
        if self._lib.CoreSdk_GetRawSkeletonInfo(skeleton_index, info) != return_codes.Success:
            return None, "invalid"
        glove_id = int(info.gloveId)
        node_count = int(info.nodesCount)
        side = glove_sides.get(glove_id)
        if side is None:
            return None, "unmapped"

        # 노드 위치 조회
        nodes = self._ffi.new("SkeletonNode[]", node_count)
        if self._lib.CoreSdk_GetRawSkeletonData(skeleton_index, nodes, node_count) != return_codes.Success:
            return None, "invalid"

        # 키포인트 노드 배치 (장갑별 1회 조회)
        node_indices = self._keypoint_layout(glove_id, node_count)
        if node_indices is None:
            return None, "no_layout"

        # 키포인트 위치 배열과 유한값 검사
        keypoints = np.array(
            [
                [nodes[node_index].transform.position.x, nodes[node_index].transform.position.y,
                 nodes[node_index].transform.position.z]
                for node_index in node_indices
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(keypoints)):
            return None, "nonfinite"
        return keypoints, side

    def _keypoint_layout(self, glove_id: int, node_count: int) -> list[int] | None:
        """장갑의 키포인트 노드 배열 인덱스를 반환한다 (처음 한 번 노드 정보를 조회해 보관, 구성 불가면 None)."""
        layout_key = (glove_id, node_count)
        if layout_key in self._keypoint_nodes:
            return self._keypoint_nodes[layout_key]

        # 노드 정보(계층·체인·관절 종류) 조회 (실패하면 보관하지 않고 다음 콜백에서 재시도)
        node_info_array = self._ffi.new("NodeInfo[]", node_count)
        result = self._lib.CoreSdk_GetRawSkeletonNodeInfoArray(glove_id, node_info_array, node_count)
        if result != self._return_code_enum.Success:
            return None
        node_infos = [
            (int(node_info.nodeId), int(node_info.parentId), int(node_info.chainType), int(node_info.fingerJointType))
            for node_info in node_info_array
        ]

        # 키포인트 배치 구성과 보관
        node_indices = keypoint_node_indices(node_infos)
        self._keypoint_nodes[layout_key] = node_indices
        if node_indices is None:
            logger.error(f"Manus 장갑 0x{glove_id:X}: raw skeleton 노드로 손 키포인트를 구성할 수 없음 — 노드 정보 {node_infos}")
        else:
            logger.info(f"Manus 장갑 0x{glove_id:X}: raw skeleton 노드 {node_count}개 → 키포인트 21개 배치")
        return node_indices

    def _on_landscape(self, landscape):
        """landscape(장치 목록)에서 장갑 id·손·페어링 상태·배터리를 보관한다. 반환값 없음."""
        try:
            devices = landscape.gloveDevices

            # 장치 수 검사
            glove_count = int(devices.gloveCount)
            dongle_count = int(devices.dongleCount)
            if glove_count > len(devices.gloves) or dongle_count > len(devices.dongles):
                with self._lock:
                    self._invalid_stream_count += 1
                return

            # 장갑 목록 복사 (배터리는 0-100 밖이면 None)
            gloves = []
            for glove_index in range(glove_count):
                glove = devices.gloves[glove_index]
                battery_percent = int(glove.batteryPercentage)
                gloves.append({
                    "id": int(glove.id),
                    "side": _SDK_SIDE.get(int(glove.side)),
                    "paired": int(glove.pairedState),
                    "battery": battery_percent if 0 <= battery_percent <= 100 else None,
                })

            # 보관 (손 매핑은 페어링·허용된 장갑만)
            with self._lock:
                self._dongle_count = dongle_count
                self._gloves = gloves
                self._glove_sides = {
                    glove["id"]: glove["side"]
                    for glove in gloves
                    if glove["side"] is not None
                    and glove["paired"] == _PAIRED_STATE_PAIRED
                    and glove_allowed(glove["id"])
                }
        except Exception as e:
            logger.error(f"landscape 콜백 오류: {type(e).__name__}: {e}")

    def _on_disconnected(self, host):
        """호스트 접속 끊김을 표시한다 (장갑 스레드가 재접속). 반환값 없음."""
        self._connected = False
        logger.warning("Manus 호스트 접속 끊김 — 재접속 대기")

    def _on_sdk_log(self, severity, message, length):
        """SDK 내부 로그를 파이썬 로그로 옮긴다. 반환값 없음."""
        try:
            # C 문자열을 디코딩해 대응 레벨로 기록
            text = self._ffi.string(message, length).decode("utf-8", errors="replace")
            logger.log(_SDK_LOG_LEVEL.get(int(severity), logging.INFO), f"[SDK] {text}")
        except Exception as e:
            logger.error(f"SDK 로그 콜백 오류: {type(e).__name__}: {e}")
