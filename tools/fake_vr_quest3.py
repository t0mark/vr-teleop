"""Quest 대체 합성 포즈 송신기 (dev 모드용): 서버와 WebRTC로 협상해 가짜 업링크 샘플을 보낸다.

실기기 없이 조작 시나리오 전체를 재현해 IK·제어·그리퍼 경로를 자극한다.
샘플 형식은 web/src/xr/pose_reader.js의 실제 업링크와 동일하다 (WebXR 좌표: x=우, y=상, -z=전방, m).
시그널링(HTTP POST /rtc/offer)·state 채널 송신은 web/src/net/rtc_link.js와 같은 방식이다.

시나리오 (1사이클, 초):
     0-2   대기 (손 rest 자세)
     2     오른손 A 상승 에지 → 제어 시작 + ready 복귀
     2-8   ready 복귀 대기
     8-18  grip 쥐고 양손 원운동 (클러치 추종)
    12-16  트리거 삼각 램프 0→1→0 (그리퍼 개폐)
    18     오른손 B 상승 에지 → 제어 해제
    18-21  대기

사용법 (컨테이너 내부, 서버 기동 후):
    python tools/fake_quest.py [--host localhost] [--rate 50] [--cycles 1]
"""

import argparse
import asyncio
import json
import logging
import math
import ssl
import sys
import time
from pathlib import Path

# 스크립트 직접 실행(python tools/fake_quest.py) 시 sys.path[0]은 tools/라 teleop 패키지가 안 보인다
# → 저장소 루트를 넣어 어느 위치에서 실행해도 동작하게 한다
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import aiohttp
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription

from teleop.utils import config
from teleop.utils.log import setup_logging

logger = logging.getLogger("fake_vr_quest3")

# 시나리오 구간 경계(초): (이름, 시작, 끝)
_PHASES = [
    ("idle", 0.0, 2.0),
    ("press_A", 2.0, 2.4),
    ("wait_ready", 2.4, 8.0),
    ("clutch_motion", 8.0, 18.0),
    ("press_B", 18.0, 18.4),
    ("idle_end", 18.4, 21.0),
]
_CYCLE_S = _PHASES[-1][2]

# 사람 기하 (WebXR, m): 어깨선 y=1.4, 손 rest 위치, 원운동 중심·반경·각속도
_SHOULDER_Y = 1.40
_HAND_REST = {"right": (0.30, 1.00, -0.25), "left": (-0.30, 1.00, -0.25)}
_CIRCLE_CENTER = {"right": (0.30, 1.20, -0.40), "left": (-0.30, 1.20, -0.40)}
_CIRCLE_RADIUS = 0.15
_CIRCLE_OMEGA = 1.25

# STUN 서버 없음: 로봇 내장 PC 안에서 자기 자신에게 접속하는 것도, Quest 직결 환경도
# NAT 통과가 필요 없다 (teleop/net/rtc_hub.py와 동일한 이유)
_RTC_CONFIG = RTCConfiguration(iceServers=[])


def _phase_at(t: float) -> str:
    """사이클 내 시각 t(초)의 구간 이름을 반환한다."""
    for name, start, end in _PHASES:
        if start <= t < end:
            return name
    return "idle_end"


def _triangle(t: float, start: float, end: float) -> float:
    """[start, end] 구간에서 0→1→0으로 오르내리는 삼각파 값을 반환한다 (구간 밖은 0)."""
    if not (start <= t < end):
        return 0.0
    half = (end - start) / 2.0
    u = (t - start) / half
    return u if u <= 1.0 else 2.0 - u


class FakeQuest:
    """시나리오 시각 → 업링크 샘플 생성을 담당한다 (송수신 통계 포함)."""

    def __init__(self, rate_hz: float, cycles: int):
        self._rate_hz = rate_hz
        self._cycles = cycles
        self._sent = 0
        self._rx_state = 0
        self._rx_pcd = 0
        self._last_phase = ""

    # ── 샘플 생성 ────────────────────────────────────────────────────────────

    def sample(self, t_cycle: float) -> dict:
        """사이클 내 시각 t(초)의 업링크 샘플(JSON dict)을 만들어 반환한다."""
        phase = _phase_at(t_cycle)
        if phase != self._last_phase:
            logger.info(f"시나리오 구간: {phase} (t={t_cycle:.1f}s)")
            self._last_phase = phase

        hands = {}
        for side in ("left", "right"):
            pos = self._hand_pos(side, t_cycle, phase)
            engaged = phase == "clutch_motion"
            trigger = _triangle(t_cycle, 12.0, 16.0) if engaged else 0.0
            a = 1 if (side == "right" and phase == "press_A") else 0
            b = 1 if (side == "right" and phase == "press_B") else 0
            hands[side] = {
                "pos": list(pos),
                "quat": [0.0, 0.0, 0.0, 1.0],
                "trigger": trigger,
                "squeeze": 1.0 if engaged else 0.0,
                # xr-standard 버튼 순서: 0=trigger, 1=squeeze, 4=A/X, 5=B/Y
                "buttons": [1 if trigger > 0.5 else 0, 1 if engaged else 0, 0, 0, a, b],
            }

        return {"t": time.monotonic() * 1000.0, "body": {"joints": self._body_joints(hands)}, "hands": hands}

    def _hand_pos(self, side: str, t: float, phase: str) -> tuple[float, float, float]:
        """구간별 손목 위치를 반환한다 (클러치 구간은 원운동, 그 외 rest).

        클러치 체결과 같은 샘플에서 위치가 rest→원 시작점으로 점프하지만, 매퍼가 체결 순간을
        앵커로 캡처하므로 체결 이후 델타는 연속이다 (체결 전 점프는 무해).
        """
        if phase != "clutch_motion":
            return _HAND_REST[side]
        # 좌우 반주기 위상차: 양팔이 다른 목표를 갖게 해 좌우 뒤바뀜 회귀를 눈으로 구분
        theta = _CIRCLE_OMEGA * (t - 8.0) + (math.pi if side == "left" else 0.0)
        cx, cy, cz = _CIRCLE_CENTER[side]
        return (cx + _CIRCLE_RADIUS * math.cos(theta), cy + _CIRCLE_RADIUS * math.sin(theta), cz)

    def _body_joints(self, hands: dict) -> dict:
        """어깨·팔꿈치 등 body-tracking 조인트를 손 위치와 정합되게 합성해 반환한다.

        팔꿈치는 어깨 볼과 손목의 중점을 아래로 내린 근사 — 팔꿈치 방향 코스트가 그럴듯한
        방향을 받는 정도면 충분하다. 이름·계층은 Meta 규약(shoulder=쇄골, arm-upper=어깨 볼).
        """
        joints = {
            "hips": {"pos": [0.0, 1.0, 0.0], "quat": [0.0, 0.0, 0.0, 1.0]},
            "head": {"pos": [0.0, 1.62, 0.0], "quat": [0.0, 0.0, 0.0, 1.0]},
        }
        for side, sx in (("right", 1.0), ("left", -1.0)):
            shoulder = (sx * 0.18, _SHOULDER_Y, 0.0)
            upper = (sx * 0.22, _SHOULDER_Y - 0.02, 0.0)
            hand = hands[side]["pos"]
            elbow = (
                (upper[0] + hand[0]) / 2.0,
                (upper[1] + hand[1]) / 2.0 - 0.08,
                (upper[2] + hand[2]) / 2.0,
            )
            joints[f"{side}-shoulder"] = {"pos": list(shoulder), "quat": [0.0, 0.0, 0.0, 1.0]}
            joints[f"{side}-arm-upper"] = {"pos": list(upper), "quat": [0.0, 0.0, 0.0, 1.0]}
            joints[f"{side}-arm-lower"] = {"pos": list(elbow), "quat": [0.0, 0.0, 0.0, 1.0]}
        return joints

    # ── 송수신 루프 ──────────────────────────────────────────────────────────

    async def run(self, host: str, port: int):
        """서버와 WebRTC로 협상해 시나리오를 사이클 수만큼 송신한다. 반환값 없음."""
        # 자체 서명 인증서 환경이므로 검증을 끈다 (Quest 브라우저의 인증서 예외 허용과 같은 역할)
        ssl_ctx = ssl.create_default_context()
        ssl_ctx.check_hostname = False
        ssl_ctx.verify_mode = ssl.CERT_NONE

        pc = RTCPeerConnection(configuration=_RTC_CONFIG)
        state_ch = pc.createDataChannel("state", ordered=True)
        pcd_ch = pc.createDataChannel("pcd", ordered=False, maxRetransmits=0)
        opened = asyncio.Event()

        @state_ch.on("open")
        def _on_open():
            opened.set()

        @state_ch.on("message")
        def _on_state_message(data):
            # 실제 클라이언트라면 반드시 소비해야 하는 다운링크(robot_state 등) — 여기선 개수만 센다
            if '"robot_state"' in data:
                self._rx_state += 1

        @pcd_ch.on("message")
        def _on_pcd_message(_data):
            self._rx_pcd += 1

        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        while pc.iceGatheringState != "complete":
            await asyncio.sleep(0.05)

        offer_url = f"https://{host}:{port}{config.RTC_OFFER_PATH}"
        logger.info(f"WebRTC 협상: {offer_url}, {self._rate_hz:.0f} Hz, {self._cycles}사이클 ({_CYCLE_S:.0f}초/사이클)")
        async with aiohttp.ClientSession() as session:
            async with session.post(
                offer_url,
                json={"sdp": pc.localDescription.sdp, "type": pc.localDescription.type},
                ssl=ssl_ctx,
            ) as resp:
                answer = await resp.json()
        await pc.setRemoteDescription(RTCSessionDescription(sdp=answer["sdp"], type=answer["type"]))

        await opened.wait()
        try:
            await self._send_loop(state_ch)
        finally:
            await pc.close()
        logger.info(f"종료: 송신 {self._sent}건, 수신 robot_state {self._rx_state}건, 포인트클라우드 {self._rx_pcd}건")

    async def _send_loop(self, channel):
        """고정 주기로 샘플을 송신한다 (사이클 완료까지). 반환값 없음."""
        period = 1.0 / self._rate_hz
        t0 = time.monotonic()
        stats_t = t0
        total = self._cycles * _CYCLE_S
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= total:
                return
            channel.send(json.dumps(self.sample(elapsed % _CYCLE_S)))
            self._sent += 1
            if now - stats_t >= 5.0:
                logger.info(
                    f"송신 {self._sent / (now - t0):.1f} Hz 평균, 수신 robot_state {self._rx_state}건, "
                    f"포인트클라우드 {self._rx_pcd}건 (경과 {elapsed:.0f}/{total:.0f}초)"
                )
                stats_t = now
            await asyncio.sleep(max(0.0, period - (time.monotonic() - now)))


def main():
    """인자를 해석하고 송신기를 실행한다."""
    parser = argparse.ArgumentParser(description="dev 모드용 합성 포즈 송신기 (Quest 대체)")
    parser.add_argument("--host", default="localhost", help="서버 주소 (기본 localhost)")
    parser.add_argument("--port", type=int, default=config.HTTPS_PORT, help=f"HTTPS 포트 (기본 {config.HTTPS_PORT})")
    parser.add_argument("--rate", type=float, default=50.0, help="송신 주기 Hz (기본 50, Quest 실측 45)")
    parser.add_argument("--cycles", type=int, default=1, help="시나리오 반복 횟수 (기본 1)")
    args = parser.parse_args()

    setup_logging()
    try:
        asyncio.run(FakeQuest(args.rate, args.cycles).run(args.host, args.port))
    except KeyboardInterrupt:
        logger.info("중단 요청 수신, 종료")


if __name__ == "__main__":
    main()
