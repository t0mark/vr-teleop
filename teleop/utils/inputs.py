"""공통 샘플 해석: 업링크 샘플에서 손 포즈·추종 여부·손가락 굽힘·명령을 꺼낸다.

입력 기기(Quest 3 컨트롤러 손 위치, 데스크톱 수동 조작)는 웹 입력 모듈이 모두 같은
공통 샘플 형식으로 변환해 보낸다. 서버는 이 모듈의 접근자로만 샘플을 읽어 기기를 구분하지 않는다.
VR 샘플에는 fingers가 없다 — 손가락은 서버에 붙은 Manus 장갑(teleop/glove/manus_glove.py)이 직접 준다.
    sample = {
        t,
        hands: {left/right: {pos[3], quat[x,y,z,w], follow: bool, fingers[6](선택)} | None},
        head: {pos[3], quat[x,y,z,w]} | None,                   # 조작자 시점(XR viewer) 포즈 (선택)
        command: "start" | "ready" | "stop" | "home" | None,   # 발생한 순간의 샘플에만 실린다
        body: {supported, jointCount, joints} | None,
    }
fingers 순서: [little, ring, middle, index, thumb_bend, thumb_rotate], 0=펼침, 1=굽힘.
"""

import numpy as np

# 손가락 자유도 수 (RH56E2: 새끼·약지·중지·검지·엄지 굽힘·엄지 회전)
FINGER_COUNT = 6
# 서버가 받아들이는 명령 목록 (home: ready pose로 복귀한 뒤 제어 해제)
COMMANDS = ("start", "ready", "stop", "home")


def _hand(sample: dict | None, side: str) -> dict | None:
    """해당 손 항목을 반환한다 (샘플·손이 없으면 None)."""
    hands = (sample or {}).get("hands") or {}
    return hands.get(side)


def hand_pose(sample: dict | None, side: str) -> tuple[list, list] | None:
    """해당 손의 (pos[3], quat[x,y,z,w])를 반환한다 (손이 없으면 None)."""
    hand = _hand(sample, side)
    if hand is None or "pos" not in hand or "quat" not in hand:
        return None
    return hand["pos"], hand["quat"]


def head_pose(sample: dict | None) -> tuple[list, list] | None:
    """조작자 시점의 (pos[3], quat[x,y,z,w])를 반환한다 (없으면 None)."""
    head = (sample or {}).get("head")
    if not isinstance(head, dict) or "pos" not in head or "quat" not in head:
        return None
    return head["pos"], head["quat"]


def hand_follow(sample: dict | None, side: str) -> bool:
    """해당 팔이 손을 추종해야 하면 True (손이 없으면 False)."""
    hand = _hand(sample, side)
    return bool(hand.get("follow", False)) if hand else False


def hand_fingers(sample: dict | None, side: str) -> list[float] | None:
    """해당 손의 손가락 굽힘 6개(0-1 클립)를 반환한다 (손이 없거나 형식이 틀리면 None)."""
    hand = _hand(sample, side)
    fingers = hand.get("fingers") if hand else None
    if not isinstance(fingers, (list, tuple)) or len(fingers) != FINGER_COUNT:
        return None
    try:
        values = np.clip(np.asarray(fingers, dtype=float), 0.0, 1.0)
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(values)):
        return None
    return values.tolist()


def sample_command(sample: dict | None) -> str | None:
    """샘플에 실린 명령(start/ready/stop/home)을 반환한다 (없거나 알 수 없는 값이면 None)."""
    command = (sample or {}).get("command")
    return command if command in COMMANDS else None
