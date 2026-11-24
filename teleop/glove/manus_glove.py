"""Manus 장갑: SDK Integrated 모드로 서버 PC의 동글에 직접 접속해 조작자 양손의 손가락 굽힘을 측정한다.

- MANUS Core 없이 SDK가 동글을 직접 연다 (Integrated). 접속은 SDK 내부 가상 호스트(SDKIntegrated)에 붙는 방식이다
- 장갑 스레드가 SDK 초기화 → 호스트 접속 → 상태 점검 루프(자동 페어링·로그)를 돈다
- 측정값은 ergonomics 스트림(관절각, deg)으로 받는다. 배열 하나에 왼손 0-19, 오른손 20-39 인덱스가 있고
  손가락마다 [Spread, MCP, PIP, DIP] (엄지는 [CMC Spread, CMC, MCP, IP]) 순서다
- 공통 형식 손가락 6개(0=펼침, 1=굽힘)로 변환:
    네 손가락 = MCP+PIP+DIP 굴곡 합, 엄지 굽힘 = MCP+IP 굴곡 합, 엄지 회전 = CMC spread
  정규화 범위는 config.MANUS_*_RANGE_DEG (상태 로그에 원시 각도를 남겨 실기기 보정에 쓴다)
- SDK가 없거나(이미지 미설치) 초기화에 실패해도 서버는 계속 돈다 — 손가락은 조작 샘플 fingers로 대체된다

SDK 콜백은 SDK 내부 스레드에서 호출되므로 측정값은 락으로 보호하고, 소비자(제어 루프)는 스냅샷만 가져간다.
"""

import logging
import threading
import time

import numpy as np

from teleop.utils import config

logger = logging.getLogger("manus_glove")

SIDES = ("left", "right")

# ergonomics 배열에서 손별 시작 인덱스 (ErgonomicsDataType: 왼손 0-19, 오른손 20-39)
_SIDE_OFFSET = {"left": 0, "right": 20}
# 손 블록 내 손가락 시작 오프셋 (손가락당 4개)
_THUMB = 0
_INDEX = 4
_MIDDLE = 8
_RING = 12
_PINKY = 16
# 손가락 블록 내 항목 오프셋: [Spread, MCP, PIP, DIP] / 엄지 [CMC Spread, CMC, MCP, IP]
_SPREAD = 0
_JOINT_1 = 1
_JOINT_2 = 2
_JOINT_3 = 3
# 공통 형식 순서(little, ring, middle, index)에 맞춘 손가락 시작 오프셋
_CURL_FINGER_BASES = (_PINKY, _RING, _MIDDLE, _INDEX)

# SDK Side enum 값 → 손 이름
_SDK_SIDE = {1: "left", 2: "right"}
# SDK DevicePairedState enum: Unpaired
_PAIRED_STATE_UNPAIRED = 2
# SDK LogSeverity → 파이썬 로그 레벨 (Debug·Info는 양이 많아 debug로 내린다)
_SDK_LOG_LEVEL = {0: logging.DEBUG, 1: logging.DEBUG, 2: logging.WARNING, 3: logging.ERROR}


def finger_angles(data, side: str) -> list[float]:
    """ergonomics 배열에서 해당 손의 손가락 각도 6개(deg)를 반환한다.

    순서: [little, ring, middle, index 굴곡 합(MCP+PIP+DIP), 엄지 굽힘(MCP+IP), 엄지 회전(CMC spread)].
    """
    base = _SIDE_OFFSET[side]

    # 네 손가락 굴곡 합
    angles = [
        data[base + finger + _JOINT_1] + data[base + finger + _JOINT_2] + data[base + finger + _JOINT_3]
        for finger in _CURL_FINGER_BASES
    ]

    # 엄지 굽힘(MCP + IP)과 회전(CMC spread)
    angles.append(data[base + _THUMB + _JOINT_2] + data[base + _THUMB + _JOINT_3])
    angles.append(data[base + _THUMB + _SPREAD])
    return angles


def angles_to_fingers(angles: list[float]) -> list[float]:
    """손가락 각도 6개(deg)를 config 범위로 정규화한 공통 형식 굽힘 6개(0-1)를 반환한다."""
    ranges = [config.MANUS_FINGER_CURL_RANGE_DEG] * 4 + [
        config.MANUS_THUMB_BEND_RANGE_DEG,
        config.MANUS_THUMB_ROTATE_RANGE_DEG,
    ]
    return [
        float(np.clip((angle - open_deg) / (closed_deg - open_deg), 0.0, 1.0))
        for angle, (open_deg, closed_deg) in zip(angles, ranges)
    ]


class ManusGlove:
    """SDK 접속 스레드와 양손 손가락 측정 상태를 소유한다."""

    def __init__(self):
        """상태를 초기화한다. SDK 로드·접속은 start() 이후 장갑 스레드에서 한다."""
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lib = None
        self._ffi = None
        self._sdk = None
        self._connected = False

        # SDK 콜백 스레드와 제어 루프·장갑 스레드가 교차 접근하므로 락으로 보호
        self._lock = threading.Lock()
        # 손별 손가락 굽힘(0-1), 원시 각도(deg, 보정 로그용), 마지막 수신 시각(monotonic)
        self._fingers: dict[str, list[float] | None] = {side: None for side in SIDES}
        self._angles: dict[str, list[float] | None] = {side: None for side in SIDES}
        self._rx_mono = {side: 0.0 for side in SIDES}
        self._ergo_count = 0
        # landscape 장갑 목록 [{id, side, paired, battery}]과 장갑 id → 손 이름
        self._gloves: list[dict] = []
        self._glove_sides: dict[int, str] = {}

        # 장갑 스레드 전용: 로그 중복 방지용 마지막 장갑 목록·페어링 실패 id, 통계 시각
        self._logged_gloves: list[tuple] | None = None
        self._pair_failed: set[int] = set()
        self._stats_t = 0.0

    @property
    def connected(self) -> bool:
        """SDK 호스트(Integrated)에 접속되어 있으면 True."""
        return self._connected

    # ── 외부 인터페이스 ──────────────────────────────────────────────────────

    def start(self):
        """장갑 스레드를 시작한다 (SDK 접속 실패해도 스레드가 재시도한다). 반환값 없음."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="manus-glove", daemon=True)
        self._thread.start()
        logger.info("Manus 장갑 시작 (SDK Integrated 모드)")

    def stop(self):
        """장갑 스레드를 멈추고 SDK를 종료한다. 반환값 없음."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        logger.info("Manus 장갑 정지")

    def fingers(self) -> dict[str, list[float]]:
        """신선한(MANUS_STALE_S 이내) 손의 손가락 굽힘 {side: [6] 0-1}를 반환한다 (없는 손은 빠진다)."""
        now = time.monotonic()
        with self._lock:
            return {
                side: list(self._fingers[side])
                for side in SIDES
                if self._fingers[side] is not None and now - self._rx_mono[side] < config.MANUS_STALE_S
            }

    # ── 장갑 스레드 ──────────────────────────────────────────────────────────

    def _run(self):
        """SDK 로드·초기화 후 접속·자동 페어링·상태 로그 루프를 돈다 (장갑 스레드 본체)."""
        if not self._load_sdk() or not self._initialize_sdk():
            return
        try:
            while not self._stop_event.is_set():
                # 미접속이면 접속 재시도
                if not self._connected:
                    self._connected = self._connect()
                    if not self._connected:
                        self._stop_event.wait(config.MANUS_RECONNECT_S)
                        continue

                self._log_glove_changes()
                self._pair_unpaired_gloves()
                self._log_stats()
                self._stop_event.wait(config.MANUS_POLL_S)
        finally:
            self._connected = False
            self._lib.CoreSdk_ShutDown()
            logger.info("Manus SDK 종료")

    def _load_sdk(self) -> bool:
        """manus_sdk 패키지를 불러온다. 반환: 성공 여부 (실패면 장갑 비활성)."""
        try:
            import manus_sdk
        except ImportError as e:
            logger.error(f"manus_sdk를 불러올 수 없음, 장갑 비활성: {e}")
            return False
        if manus_sdk.lib is None:
            logger.error("manus_sdk 네이티브 바인딩 로드 실패, 장갑 비활성 (libzmq5·libgomp1·cffi 설치 확인)")
            return False
        self._lib = manus_sdk.lib
        self._ffi = manus_sdk.ffi
        self._sdk = manus_sdk.ManusSDK()
        return True

    def _initialize_sdk(self) -> bool:
        """Integrated 모드 초기화·콜백 등록·좌표계 설정을 한다. 반환: 성공 여부."""
        from manus_sdk.generated._enums import AxisPolarity, AxisView, SDKReturnCode, Side

        result = self._lib.CoreSdk_InitializeIntegrated()
        if result != SDKReturnCode.Success:
            logger.error(f"Manus SDK Integrated 초기화 실패: {SDKReturnCode(result).name}")
            return False

        # 콜백 등록 (SDK 내부 스레드에서 호출된다)
        self._sdk.register_log_callback(self._on_sdk_log)
        self._sdk.register_landscape_callback(self._on_landscape)
        self._sdk.register_ergonomics_callback(self._on_ergonomics)
        self._sdk.register_disconnected_callback(self._on_disconnected)

        # 좌표계 설정 (ergonomics 관절각에는 영향 없지만 SDK가 접속 전 설정을 요구한다)
        coord_system = self._ffi.new("CoordinateSystemVUH*")
        coord_system.view = AxisView.XFromViewer
        coord_system.up = AxisPolarity.PositiveZ
        coord_system.handedness = Side.Right
        coord_system.unitScale = 1.0
        result = self._lib.CoreSdk_InitializeCoordinateSystemWithVUH(coord_system[0], True)
        if result != SDKReturnCode.Success:
            logger.error(f"Manus 좌표계 설정 실패: {SDKReturnCode(result).name}")
            self._lib.CoreSdk_ShutDown()
            return False

        logger.info("Manus SDK Integrated 초기화 완료")
        return True

    def _connect(self) -> bool:
        """SDK 호스트(Integrated 가상 호스트)를 찾아 접속한다. 반환: 성공 여부."""
        from manus_sdk.generated._enums import SDKReturnCode

        # 호스트 탐색
        result = self._lib.CoreSdk_LookForHosts(1, False)
        if result != SDKReturnCode.Success:
            logger.warning(f"Manus 호스트 탐색 실패: {SDKReturnCode(result).name}")
            return False
        count = self._ffi.new("uint32_t*")
        result = self._lib.CoreSdk_GetNumberOfAvailableHostsFound(count)
        if result != SDKReturnCode.Success or count[0] == 0:
            logger.warning("Manus 호스트 없음 — 재시도")
            return False

        # 첫 호스트에 접속 (Integrated 모드는 로컬 가상 호스트 하나뿐)
        hosts = self._ffi.new("ManusHost[]", count[0])
        result = self._lib.CoreSdk_GetAvailableHostsFound(hosts, count[0])
        if result != SDKReturnCode.Success:
            logger.warning(f"Manus 호스트 목록 조회 실패: {SDKReturnCode(result).name}")
            return False
        host_name = self._ffi.string(hosts[0].hostName).decode("utf-8", errors="replace")
        result = self._lib.CoreSdk_ConnectToHost(hosts[0])
        if result != SDKReturnCode.Success:
            logger.warning(f"Manus 호스트 접속 실패 ({host_name}): {SDKReturnCode(result).name}")
            return False
        logger.info(f"Manus 호스트 접속: {host_name}")
        return True

    def _pair_unpaired_gloves(self):
        """landscape에 보이는 미페어링 장갑을 동글에 페어링한다. 반환값 없음."""
        with self._lock:
            unpaired = [glove for glove in self._gloves if glove["paired"] == _PAIRED_STATE_UNPAIRED]

        for glove in unpaired:
            paired = self._ffi.new("bool*")
            result = self._lib.CoreSdk_PairGlove(glove["id"], paired)
            if result == 0 and paired[0]:
                logger.info(f"Manus 장갑 페어링 성공: 0x{glove['id']:X} ({glove['side']})")
                self._pair_failed.discard(glove["id"])
            elif glove["id"] not in self._pair_failed:
                # 실패 로그는 장갑당 1회 (매 점검 주기 재시도)
                logger.warning(f"Manus 장갑 페어링 실패: 0x{glove['id']:X} ({glove['side']}), 결과 {result}")
                self._pair_failed.add(glove["id"])

    def _log_glove_changes(self):
        """장갑 목록(id·손·페어링 상태)이 바뀌면 로그로 남긴다. 반환값 없음."""
        with self._lock:
            gloves = [(glove["id"], glove["side"], glove["paired"]) for glove in self._gloves]
        if gloves == self._logged_gloves:
            return
        self._logged_gloves = gloves
        summary = ", ".join(f"0x{gid:X}({side}, paired={paired})" for gid, side, paired in gloves) or "없음"
        logger.info(f"Manus 장갑 목록: {summary}")

    def _log_stats(self):
        """주기적으로 수신률·배터리·손별 원시 각도(보정용)·정규화 굽힘을 로그로 남긴다. 반환값 없음."""
        now = time.monotonic()
        if now - self._stats_t < config.MANUS_STATS_INTERVAL_S:
            return
        dt = now - self._stats_t if self._stats_t > 0 else config.MANUS_STATS_INTERVAL_S
        self._stats_t = now

        with self._lock:
            hz = self._ergo_count / dt
            self._ergo_count = 0
            battery = {glove["side"]: glove["battery"] for glove in self._gloves}
            snapshot = {side: (self._angles[side], self._fingers[side], now - self._rx_mono[side]) for side in SIDES}

        # 손별 상태: 각도(deg) little ring middle index thumb_bend thumb_rotate / 굽힘(0-1)
        parts = []
        for side in SIDES:
            angles, fingers, age = snapshot[side]
            if angles is None or age >= config.MANUS_STALE_S:
                parts.append(f"{side}: 미수신")
                continue
            angle_text = " ".join(f"{a:.0f}" for a in angles)
            finger_text = " ".join(f"{f:.2f}" for f in fingers)
            parts.append(f"{side}(배터리 {battery.get(side, '?')}%): 각도[{angle_text}] 굽힘[{finger_text}]")
        logger.info(f"Manus ergonomics {hz:.1f} Hz | " + " | ".join(parts))

    # ── SDK 콜백 (SDK 내부 스레드) ───────────────────────────────────────────

    def _on_ergonomics(self, stream):
        """ergonomics 스트림을 손별 각도·굽힘으로 변환해 보관한다. 반환값 없음."""
        try:
            now = time.monotonic()
            with self._lock:
                glove_sides = dict(self._glove_sides)

            # 항목별 대상 손: 장갑 id면 landscape의 손, 사용자 id면 양손 모두
            updates = {}
            for i in range(stream.dataCount):
                entry = stream.data[i]
                data = list(entry.data)
                if entry.isUserID:
                    target_sides = SIDES
                else:
                    target_sides = (glove_sides[entry.id],) if entry.id in glove_sides else ()
                for side in target_sides:
                    updates[side] = finger_angles(data, side)

            # 보관
            with self._lock:
                for side, angles in updates.items():
                    self._angles[side] = angles
                    self._fingers[side] = angles_to_fingers(angles)
                    self._rx_mono[side] = now
                self._ergo_count += 1
        except Exception as e:
            logger.error(f"ergonomics 콜백 오류: {type(e).__name__}: {e}")

    def _on_landscape(self, landscape):
        """landscape(장치 목록)에서 장갑 id·손·페어링 상태·배터리를 보관한다. 반환값 없음."""
        try:
            devices = landscape.gloveDevices
            gloves = []
            for i in range(devices.gloveCount):
                glove = devices.gloves[i]
                gloves.append({
                    "id": int(glove.id),
                    "side": _SDK_SIDE.get(int(glove.side)),
                    "paired": int(glove.pairedState),
                    "battery": int(glove.batteryPercentage),
                })
            with self._lock:
                self._gloves = gloves
                self._glove_sides = {glove["id"]: glove["side"] for glove in gloves if glove["side"] is not None}
        except Exception as e:
            logger.error(f"landscape 콜백 오류: {type(e).__name__}: {e}")

    def _on_disconnected(self, host):
        """호스트 접속 끊김을 표시한다 (장갑 스레드가 재접속). 반환값 없음."""
        self._connected = False
        logger.warning("Manus 호스트 접속 끊김 — 재접속 대기")

    def _on_sdk_log(self, severity, message, length):
        """SDK 내부 로그를 파이썬 로그로 옮긴다. 반환값 없음."""
        try:
            text = self._ffi.string(message, length).decode("utf-8", errors="replace")
            logger.log(_SDK_LOG_LEVEL.get(int(severity), logging.INFO), f"[SDK] {text}")
        except Exception as e:
            logger.error(f"SDK 로그 콜백 오류: {type(e).__name__}: {e}")
