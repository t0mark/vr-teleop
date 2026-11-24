"""Inspire RH56E2 다지 손: 양손을 각각 Modbus TCP(config.HAND_ADDRESSES)로 구동한다.

- 손마다 전용 스레드가 접속 → 초기화(HAND_ID·오류 확인, 속도·힘 임계값 1회 기록) → 구동 루프를 돈다
- 구동 루프(config.HAND_RATE_HZ): 목표가 바뀌었거나 재전송 주기가 지나면 ANGLE_SET을 쓰고, ANGLE_ACT를 읽는다
- 각도 규약: 레지스터 0-1000에서 1000 = 펼침 → ANGLE_SET = round(1000·(1 − 굽힘)), 굽힘 = 1 − ANGLE_ACT/1000
- 첫 목표가 오기 전(재접속 직후 포함)에는 ANGLE_SET을 쓰지 않는다 — 접속만으로 손이 움직이지 않게
- 접속이 끊기면 로그를 1회 남기고 그 손만 준비 해제 후 재접속을 반복한다 (팔 제어는 막지 않는다)

레지스터 주소 체계(실측): 주소는 매뉴얼의 바이트 주소이고, 다중 읽기/쓰기는 시작 주소부터 바이트를
2개씩 묶은 워드(리틀 엔디언)로 다룬다. short 6개 그룹은 count=6으로, 바이트 그룹(ERROR·STATUS·TEMP)은
워드 하나에 손가락 2개(하위 바이트 = 앞 손가락)가 실린다.
버스 접근은 전부 손별 스레드에서만 한다. 외부는 목표·측정값만 락으로 주고받는다.
"""

import logging
import threading
import time

import numpy as np

from teleop.robot.robot_interface import HandBackend
from teleop.utils import config
from teleop.utils.inputs import FINGER_COUNT

logger = logging.getLogger("inspire_hand")

# 레지스터 주소 (RH56E2 매뉴얼 2.6, 손가락 순서 little, ring, middle, index, thumb_bend, thumb_rotate)
REG_HAND_ID = 1000
REG_ANGLE_SET = 1486
REG_FORCE_SET = 1498
REG_SPEED_SET = 1522
REG_ANGLE_ACT = 1546
REG_FORCE_ACT = 1582
# ERROR(1606, 6바이트)·STATUS(1612, 6바이트)·TEMP(1618, 6바이트)를 연속 9워드로 한 번에 읽는다
REG_ERROR = 1606
BYTE_GROUP_WORDS = 9

# ANGLE 레지스터 최대값 (1000 = 펼침)
ANGLE_FULL_SCALE = 1000

SIDES = ("left", "right")


class InspireHand(HandBackend):
    """양손 Modbus 접속 스레드와 손가락 목표·측정 상태를 소유한다."""

    def __init__(self, addresses: dict = config.HAND_ADDRESSES, rate_hz: float = config.HAND_RATE_HZ):
        """addresses: {side: (IP, 포트)}. rate_hz: 손별 구동 루프 주기(Hz)."""
        self._addresses = dict(addresses)
        self._period = 1.0 / rate_hz
        self._stop_event = threading.Event()
        self._threads: dict[str, threading.Thread] = {}

        # set_fingers(제어 루프)·fingers(상태 서비스)와 손 스레드가 교차 접근하므로 락으로 보호
        self._lock = threading.Lock()
        # 목표 굽힘(첫 목표 전에는 None)과 갱신 번호(스레드가 새 목표인지 판단)
        self._target: dict[str, np.ndarray | None] = {side: None for side in SIDES}
        self._target_seq = {side: 0 for side in SIDES}
        # 측정 굽힘·힘(g)과 손별 준비 상태
        self._measured: dict[str, np.ndarray | None] = {side: None for side in SIDES}
        self._force: dict[str, list[int] | None] = {side: None for side in SIDES}
        self._side_ready = {side: False for side in SIDES}

    @property
    def ready(self) -> bool:
        """양손 모두 접속·초기화가 끝나 목표를 받을 수 있으면 True."""
        return all(self._side_ready[side] for side in self._addresses)

    # ── 외부 인터페이스 ──────────────────────────────────────────────────────

    def start(self):
        """손별 접속·구동 스레드를 시작한다 (접속 실패해도 스레드가 재시도한다). 반환값 없음."""
        self._stop_event.clear()
        for side in self._addresses:
            if side in self._threads and self._threads[side].is_alive():
                continue
            thread = threading.Thread(target=self._run_side, args=(side,), name=f"inspire-hand-{side}", daemon=True)
            self._threads[side] = thread
            thread.start()
        logger.info(f"Inspire 손 시작: {self._addresses} ({1.0 / self._period:.0f} Hz)")

    def stop(self):
        """구동 스레드를 멈추고 접속을 닫는다 (정지 시 동작 명령은 보내지 않는다). 반환값 없음."""
        self._stop_event.set()
        for thread in self._threads.values():
            thread.join(timeout=2.0)
        self._threads = {}
        logger.info("Inspire 손 정지")

    def set_fingers(self, fingers: dict[str, list[float]]):
        """목표 손가락 굽힘 {left/right: [6] 0-1}를 설정한다 (준비 안 된 손·형식이 틀린 값은 무시). 반환값 없음."""
        if fingers is None:
            return
        with self._lock:
            for side in self._addresses:
                values = fingers.get(side)
                if not self._side_ready[side] or values is None or len(values) != FINGER_COUNT:
                    continue
                self._target[side] = np.clip(np.asarray(values, dtype=float), 0.0, 1.0)
                self._target_seq[side] += 1

    def fingers(self) -> dict[str, list[float]] | None:
        """측정 손가락 굽힘 스냅샷을 반환한다 (준비된 손만 담고, 준비된 손이 없으면 None)."""
        with self._lock:
            snapshot = {
                side: self._measured[side].tolist()
                for side in self._addresses
                if self._side_ready[side] and self._measured[side] is not None
            }
        return snapshot or None

    # ── 스레드 본체 ──────────────────────────────────────────────────────────

    def _run_side(self, side: str):
        """한 손의 접속 → 초기화 → 구동 루프를 정지 요청까지 반복한다 (끊기면 백오프 후 재접속)."""
        from pymodbus.client import ModbusTcpClient

        host, port = self._addresses[side]
        reconnect_s = config.HAND_RECONNECT_S
        while not self._stop_event.is_set():
            client = ModbusTcpClient(host, port=port, timeout=config.HAND_TIMEOUT_S, retries=1)

            # 접속·초기화: 실패하면 대기 시간을 늘려 가며 재시도
            try:
                if not client.connect():
                    raise ConnectionError("TCP 접속 실패")
                self._initialize(side, client)
            except Exception as e:
                client.close()
                logger.warning(f"손 {side}({host}:{port}) 접속 실패, {reconnect_s:.1f}초 후 재시도: {e!r}")
                self._stop_event.wait(reconnect_s)
                reconnect_s = min(reconnect_s * 2.0, config.HAND_RECONNECT_MAX_S)
                continue
            reconnect_s = config.HAND_RECONNECT_S

            # 구동 루프: 끊기면 그 손만 준비 해제하고 재접속으로 돌아간다
            try:
                self._loop(side, client)
            except Exception as e:
                logger.error(f"손 {side}({host}:{port}) 통신 끊김, 재접속 시도: {e!r}")
            finally:
                with self._lock:
                    self._side_ready[side] = False
                client.close()

        logger.info(f"손 {side} 스레드 종료")

    def _initialize(self, side: str, client):
        """HAND_ID·오류를 1회 읽어 로그하고 속도·힘 임계값을 기록한 뒤 손을 준비 상태로 만든다. 반환값 없음."""
        hand_id = self._read(client, REG_HAND_ID, 1)[0] & 0xFF
        errors, status, temps = self._read_byte_groups(client)
        angle_act = self._read(client, REG_ANGLE_ACT, FINGER_COUNT)
        logger.info(
            f"손 {side} 접속: HAND_ID {hand_id}, ANGLE_ACT {angle_act}, ERROR {errors}, STATUS {status}, TEMP {temps}"
        )

        # 속도·힘 임계값은 동작을 일으키지 않는 설정이라 접속 시 1회 기록
        self._write(client, REG_SPEED_SET, [config.HAND_SPEED] * FINGER_COUNT)
        self._write(client, REG_FORCE_SET, [config.HAND_FORCE_LIMIT] * FINGER_COUNT)
        logger.info(f"손 {side} 속도 {config.HAND_SPEED}, 힘 임계값 {config.HAND_FORCE_LIMIT} g 설정")

        # 재접속 직후에도 예전 목표로 움직이지 않도록 목표를 비우고 측정값부터 채운다
        with self._lock:
            self._target[side] = None
            self._measured[side] = self._angles_to_curl(angle_act)
            self._side_ready[side] = True
        logger.info(f"손 {side} 준비 완료 (첫 목표 전까지 명령 없음)")

    def _loop(self, side: str, client):
        """목표 기록·측정 읽기를 주기마다 반복한다 (정지 요청 시 반환, 통신 오류 시 예외)."""
        sent_seq = 0
        sent_angles: list[int] | None = None
        sent_t = 0.0
        cycle = 0
        stats_t = time.monotonic()
        stats_cycles = 0
        stats_writes = 0

        while not self._stop_event.is_set():
            t0 = time.monotonic()

            # 새 목표가 있거나 재전송 주기가 지났으면 ANGLE_SET 기록 (첫 목표 전에는 기록 없음)
            with self._lock:
                target = self._target[side]
                target_seq = self._target_seq[side]
            if target is not None:
                angles = self._curl_to_angles(target)
                if (target_seq != sent_seq and angles != sent_angles) or t0 - sent_t >= config.HAND_RESEND_S:
                    self._write(client, REG_ANGLE_SET, angles)
                    sent_angles = angles
                    sent_t = t0
                    stats_writes += 1
                sent_seq = target_seq

            # 실측 각도는 매 주기, 실측 힘은 분주해서 읽는다
            angle_act = self._read(client, REG_ANGLE_ACT, FINGER_COUNT)
            force_act = None
            if cycle % config.HAND_FORCE_READ_DIVIDER == 0:
                force_act = self._read(client, REG_FORCE_ACT, FINGER_COUNT)
            with self._lock:
                self._measured[side] = self._angles_to_curl(angle_act)
                if force_act is not None:
                    self._force[side] = force_act
            cycle += 1
            stats_cycles += 1

            # 주기적으로 루프 주기·기록 횟수·오류·온도·힘을 로그
            if t0 - stats_t >= config.HAND_STATS_INTERVAL_S:
                errors, status, temps = self._read_byte_groups(client)
                elapsed = t0 - stats_t
                with self._lock:
                    force = self._force[side]
                logger.info(
                    f"손 {side}: {stats_cycles / elapsed:.1f} Hz, ANGLE_SET 기록 {stats_writes}회, "
                    f"ANGLE_ACT {angle_act}, 목표 {sent_angles}, FORCE_ACT {force} g, "
                    f"ERROR {errors}, STATUS {status}, TEMP {temps}"
                )
                if any(errors):
                    logger.warning(f"손 {side} 구동기 오류 코드 {errors} (bit0 끼임, bit1 과열, bit2 과전류, bit3 모터 이상, bit4 통신)")
                stats_t = t0
                stats_cycles = 0
                stats_writes = 0

            self._stop_event.wait(max(0.0, self._period - (time.monotonic() - t0)))

    # ── Modbus 입출력 ────────────────────────────────────────────────────────

    def _read(self, client, address: int, count: int) -> list[int]:
        """레지스터 count개를 부호 있는 short 목록으로 읽어 반환한다 (오류 응답이면 예외)."""
        response = client.read_holding_registers(address, count=count, device_id=config.HAND_DEVICE_ID)
        if response.isError():
            raise RuntimeError(f"레지스터 {address} 읽기 오류: {response}")
        return [value - 0x10000 if value > 0x7FFF else value for value in response.registers]

    def _write(self, client, address: int, values: list[int]):
        """레지스터 여러 개(FC16)를 기록한다 (오류 응답이면 예외). 반환값 없음."""
        response = client.write_registers(
            address, [value & 0xFFFF for value in values], device_id=config.HAND_DEVICE_ID,
        )
        if response.isError():
            raise RuntimeError(f"레지스터 {address} 쓰기 오류: {response}")

    def _read_byte_groups(self, client) -> tuple[list[int], list[int], list[int]]:
        """ERROR·STATUS·TEMP(손가락별 1바이트)를 읽어 (오류, 상태, 온도℃) 목록으로 반환한다."""
        words = self._read(client, REG_ERROR, BYTE_GROUP_WORDS)

        # 워드 하나에 손가락 2개: 하위 바이트가 앞 손가락
        data = []
        for word in words:
            data.extend([word & 0xFF, (word >> 8) & 0xFF])
        n = FINGER_COUNT
        return data[0:n], data[n:2 * n], data[2 * n:3 * n]

    # ── 단위 변환 ────────────────────────────────────────────────────────────

    @staticmethod
    def _curl_to_angles(curl: np.ndarray) -> list[int]:
        """굽힘 [6] 0-1을 ANGLE_SET 값 [6] 0-1000(1000 = 펼침)으로 변환해 반환한다."""
        return [int(round(ANGLE_FULL_SCALE * (1.0 - value))) for value in curl]

    @staticmethod
    def _angles_to_curl(angles: list[int]) -> np.ndarray:
        """ANGLE_ACT 값 [6]을 굽힘 [6] 0-1로 변환해 반환한다."""
        return np.clip(1.0 - np.asarray(angles, dtype=float) / ANGLE_FULL_SCALE, 0.0, 1.0)
