"""VR 컨트롤러 입력 해석: 업링크 샘플에서 버튼·스틱 값을 꺼낸다.

IK 서비스(ready 복귀)와 제어 서비스(제어 시작·해제, 모바일 베이스)가 같은 샘플 형식을 읽으므로
해석을 한곳에 둔다. 샘플 형식은 web/src/xr/pose_reader.js의 hands 항목을 따른다:
    hands[side] = {pos, quat, trigger, squeeze, thumbstick[2], buttons[]}
buttons는 xr-standard 게임패드 순서(0=trigger, 1=squeeze, 4=A/X, 5=B/Y)의 눌림 여부다.
"""


def hand_button(sample: dict | None, side: str, index: int) -> bool:
    """해당 손 컨트롤러의 index번 버튼이 눌려 있으면 True (손·버튼이 없으면 False)."""
    hand = (sample or {}).get("hands", {}).get(side)
    buttons = hand.get("buttons", []) if hand else []
    return len(buttons) > index and bool(buttons[index])


def hand_thumbstick(sample: dict | None, side: str) -> tuple[float, float]:
    """해당 손 스틱 (x, y)를 반환한다 (없으면 (0, 0)). 게임패드 규약: 오른쪽 +x, 앞으로 밀기 -y."""
    hand = (sample or {}).get("hands", {}).get(side)
    if not hand:
        return 0.0, 0.0
    axis = hand.get("thumbstick") or [0.0, 0.0]
    return float(axis[0]), float(axis[1])


def hand_trigger(sample: dict | None, side: str) -> float | None:
    """해당 손 트리거 값(0-1)을 반환한다 (손이 없으면 None)."""
    hand = (sample or {}).get("hands", {}).get(side)
    return float(hand.get("trigger", 0.0)) if hand else None
