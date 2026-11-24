"""로봇 백엔드 추상 인터페이스.

다른 로봇을 붙이거나 실로봇 없이 시뮬레이션으로 돌릴 때 상위 서비스를 바꾸지 않기 위한 경계다.
관절값은 항상 {관절명: 각도(rad)} 사전으로 주고받는다 — SDK와 URDF의 관절 순서가 다르고
(SDK는 torso→right→left, pinocchio 축소 모델은 left 먼저) 인덱스로 다루면 좌우가 뒤바뀐다.

제어 경로는 세 단계로 나뉜다.
    prepare_control()  전원·서보·Control Manager를 명령 수신 가능 상태로 만든다 (1회, 블로킹)
    start_stream()     연속 명령 스트림을 연다 (제어 시작마다)
    send_targets()     목표 관절각을 한 주기 보낸다 (30 Hz)
    stop_stream()      스트림을 닫는다 — 로봇은 서보가 잡은 채 현재 자세를 유지한다
"""

from abc import ABC, abstractmethod


class JointSource(ABC):
    """관절 측정값 공급자. 실로봇 백엔드와 시뮬레이션(IK 결과)이 함께 구현한다."""

    @abstractmethod
    def joint_positions(self) -> dict[str, float] | None:
        """최신 관절값 {관절명: 각도(rad)}를 반환한다. 아직 값이 없거나 갱신이 끊겨 오래됐으면 None.

        URDF 관절명과 같은 이름을 써야 한다 (torso_0-5, right_arm_0-6, left_arm_0-6, head_0-1 등).
        """

    @abstractmethod
    def hand_fingers(self) -> dict[str, list[float]] | None:
        """손가락 측정 굽힘 {left/right: [6] 0(펼침)-1(굽힘)}를 반환한다. 값이 없으면 None.

        순서: [little, ring, middle, index, thumb_bend, thumb_rotate].
        """


class RobotBackend(JointSource):
    """실로봇 연결·상태 구독·명령 스트리밍의 공통 인터페이스."""

    @abstractmethod
    def connect(self) -> bool:
        """로봇에 연결하고 상태 구독을 시작한다. 반환: 성공 여부."""

    @abstractmethod
    def disconnect(self):
        """스트림·상태 구독을 멈추고 연결을 닫는다. 반환값 없음."""

    @property
    @abstractmethod
    def connected(self) -> bool:
        """현재 연결 상태를 반환한다."""

    @abstractmethod
    def set_velocity_limits(self, limits: dict[str, float]):
        """명령에 실을 관절별 속도 상한 {관절명: rad/s}을 설정한다. 반환값 없음."""

    @abstractmethod
    def prepare_control(self) -> bool:
        """전원·서보·제어 관리자를 명령 수신 가능 상태로 만든다. 반환: 성공 여부 (블로킹)."""

    @abstractmethod
    def start_stream(self) -> bool:
        """연속 명령 스트림을 연다. 반환: 성공 여부 (블로킹, 제어 준비 대기 포함)."""

    @abstractmethod
    def send_targets(self, named: dict[str, float], dt: float) -> bool:
        """한 주기의 목표를 보낸다. 반환: 성공 여부 (실패면 스트림이 이미 닫힌 상태).

        named: 몸통 관절(torso·양팔·머리) 목표 {관절명: rad}.
        dt: 명령 주기(초) — minimum_time·hold_time 계산에 쓴다.
        """

    @abstractmethod
    def stop_stream(self):
        """명령 스트림을 닫는다 (열려 있지 않으면 무시). 반환값 없음."""

    @property
    @abstractmethod
    def streaming(self) -> bool:
        """명령 스트림이 열려 있으면 True."""


class HandBackend(ABC):
    """다지 손 구동 인터페이스. 본체와 별도 버스를 쓰는 손(RH56E2 등)을 위해 백엔드와 분리한다.

    손가락 값은 굽힘 0(펼침)-1(굽힘), 순서 [little, ring, middle, index, thumb_bend, thumb_rotate].
    """

    @abstractmethod
    def start(self):
        """장치를 열고 초기화를 백그라운드에서 시작한다. 반환값 없음."""

    @abstractmethod
    def stop(self):
        """구동을 멈추고 장치를 놓는다. 반환값 없음."""

    @property
    @abstractmethod
    def ready(self) -> bool:
        """초기화가 끝나 목표를 받을 수 있으면 True."""

    @abstractmethod
    def set_fingers(self, fingers: dict[str, list[float]]):
        """목표 손가락 굽힘 {left/right: [6] 0-1}를 설정한다. 반환값 없음 (준비 전이면 무시)."""

    @abstractmethod
    def fingers(self) -> dict[str, list[float]] | None:
        """측정 손가락 굽힘 {left/right: [6] 0-1}를 반환한다. 준비 전이면 None."""
