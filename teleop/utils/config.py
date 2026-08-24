"""전역 설정: 경로·포트·IK·카메라 파라미터 등 모듈 간 공유 파라미터를 한곳에 모은다.

호출 사이 유지 상태가 없으므로 모듈 수준 상수로 정의한다.
"""

import os
from pathlib import Path

# 저장소 루트 (컨테이너에서는 /workspace). 이 파일 위치 기준으로 역산해 마운트 경로에 독립적으로 동작
REPO_ROOT = Path(__file__).resolve().parents[2]

# 프론트엔드 정적 파일 루트와 자체 서명 인증서 출력 위치
WEB_ROOT = REPO_ROOT / "web"
CERT_DIR = WEB_ROOT / "certs"
CERT_FILE = CERT_DIR / "server.crt"
KEY_FILE = CERT_DIR / "server.key"

# RBY1 모델 리소스: IK용 URDF와 브라우저 시각화용 GLB 메시.
# pip 패키지(rby1_sdk)에는 포함되지 않아 이미지 빌드 시 SDK 저장소에서 받아 굽는다(docker/Dockerfile).
# 저장소 안에 두지 않으므로 기본값은 이미지 내부 경로이고, 다른 위치를 쓸 때만 환경 변수로 덮어쓴다.
RBY1_MODEL_ROOT = Path(os.environ.get("RBY1_MODEL_ROOT", "/opt/rby1/models/rby1a"))
RBY1_URDF = RBY1_MODEL_ROOT / "urdf" / "model.urdf"
RBY1_MESH_DIR = RBY1_URDF.parent / "meshes"
MESH_URL_PREFIX = "/models/"

# HTTPS/WSS 바인딩 (Quest 직결·PC 모니터링 공용, WebXR secure context 요건상 HTTPS 필수)
HTTP_HOST = "0.0.0.0"
HTTPS_PORT = 8443

# 접속 편의 포트: 443 = https://IP 만으로 접속되는 별칭, 80 = http 접속을 HTTPS로 리다이렉트
HTTPS_STD_PORT = 443
HTTP_REDIRECT_PORT = 80

# WSS 경로: 포즈 업링크(VR → 서버), 모니터 다운링크(서버 → 데스크톱 뷰)
POSE_WS_PATH = "/ws/pose"
MONITOR_WS_PATH = "/ws/monitor"

# 포즈 파이프라인 상태 로그 주기(초)와 샘플 신선도 한계(초, 초과 시 IK 입력 무효)
POSE_STATS_INTERVAL_S = 5.0
POSE_STALE_S = 0.5

# 그리퍼 TCP(집는 지점)의 ee 프레임 로컬 오프셋(m): 손가락 원점 -0.073 + 손가락 절반 길이
# 사용자 손(grip 중심)이 이 지점에 매핑된다
GRIPPER_TCP_OFFSET = (0.0, 0.0, -0.12)

# IK 루프·솔버 파라미터 (sqpmethod: warm start 연속 추종에서는 소수 반복이면 충분)
IK_RATE_HZ = 30.0
IK_MAX_ITER = 6
# 코스트 가중치: TCP 위치 최우선, 자세는 손목 관절 추종이 체감되는 수준,
# 팔꿈치는 자세 유도 보조 (body-tracking 있을 때만), ready 바이어스는 여유 자유도 해소용
IK_W_POS = 50.0
IK_W_ROT = 8.0
IK_W_ELBOW = 10.0
IK_W_REG = 0.5
# ready 바이어스는 null-space 투영 없이 소프트 코스트로 넣으므로 추종을 해치지 않게 약하게
IK_W_READY = 0.02
# 관절 속도 상한: URDF velocity의 비율로 관절별 계산하되 전역 상한도 둔다 (감사 지적: 4.0은
# 하드웨어 한계 초과 — arm_6은 2.09 rad/s). 실기기 이전 시 그대로 안전 상한으로 사용
IK_VEL_FRACTION = 0.5
IK_MAX_JOINT_VEL = 1.5

# 안전 관절 한계 (실사용 검증된 custom-teleop의 add_joint_limit 값, rad):
# arm_3(팔꿈치) 완전 신전 금지 — 특이점·몸통 스침 방지 / arm_5 역방향 꺾임 금지 — 콜리전 방지
SAFETY_JOINT_LIMITS = {
    "right_arm_3": (-2.6, -0.5),
    "left_arm_3": (-2.6, -0.5),
    "right_arm_5": (0.2, 1.9),
    "left_arm_5": (0.2, 1.9),
}

# 클러치: grip(squeeze)이 이 값을 넘는 동안만 추종 (custom-teleop과 동일 임계값)
GRIP_ENGAGE_THRESHOLD = 0.8

# 오른손 A 버튼(xr-standard 게임패드 인덱스 4) → ready pose 복귀
GAMEPAD_BUTTON_A = 4

# 사람→로봇 팔 매핑: 클러치 델타 방식 (1m = 1m). 가슴 폴백 높이는 몸통 프레임 기준점용
ARM_SCALE = 1.0
FALLBACK_CHEST_HEIGHT = 1.35

# Quest 직결 서버 IP: PC 이더넷 인터페이스를 NetworkManager "공유" 모드로 사용
# (공유 모드가 PC를 10.42.0.1로 고정하고 Quest에 DHCP 할당). 인증서 SAN 기본값으로도 사용
DIRECT_LINK_SERVER_IP = "10.42.0.1"

# 실로봇 연결: 로봇 제어 gRPC 주소 (내장 PC에서는 br0 대역의 로봇 내부 주소).
# 상태 구독·시각화 방송 주기는 IK와 같은 30 Hz로 맞춘다
ROBOT_ADDRESS = "192.168.30.1:50051"
ROBOT_STATE_RATE_HZ = 30.0

# 실로봇 명령 스트리밍: IK 주기와 같은 30 Hz. minimum_time은 주기보다 약간 길게 잡아 명령이 끊기지 않게
# 하고(custom-teleop 규약 dt×1.01), control_hold_time은 다음 명령이 늦어도 자세를 유지하는 여유(dt×10)
CONTROL_RATE_HZ = 30.0
CONTROL_MIN_TIME_FACTOR = 1.01
CONTROL_HOLD_TIME_FACTOR = 10.0
CONTROL_SEND_TIMEOUT_MS = 200
# Control Manager unlimited 모드: custom-teleop과 동일하게 True 고정.
# 제한 모드(False)는 활성화 시점에 관절 펌웨어 한계 검사를 하는데, head_1이 한계 밖이면(위를 본 자세)
# 활성화 자체가 거부된다 (실기기 확인: unlimited면 "skipping limit check"로 통과).
# 속도 안전은 명령 쪽(set_velocity_limit + 서비스 클램프)에서 계속 보장한다
CM_UNLIMITED_MODE = True
# 머리 관절 위치 PID 게인 (custom-teleop 값: 큰 D로 감쇠). CM 활성화 전에 1회 적용
HEAD_PID_GAINS = {"head_0": (200, 0, 8000), "head_1": (200, 0, 8000)}
# 제어 준비(wait_for_control_ready) 대기 상한
CONTROL_READY_TIMEOUT_MS = 1000

# 안전장치
# - 입력 두절: VR 업링크가 이 시간 이상 끊기면 제어를 해제한다 (IK는 마지막 자세를 유지하지만,
#   조작자가 사라진 상태로 스트림을 유지할 이유가 없다)
CONTROL_INPUT_TIMEOUT_S = 3.0
# - 추적 오차: 명령과 측정 관절각 차이가 이 값(rad)을 넘으면 로봇이 막힌 것으로 보고 해제한다
CONTROL_MAX_TRACKING_ERR = 0.5
# - 팔 외 관절(토르소·머리)을 ready로 보내는 속도는 URDF 한계에 이 비율을 곱한다 (팔은 IK_VEL_FRACTION).
#   토르소는 무거우므로 보수적으로 (0.15 → torso_0-2 약 0.3 rad/s, 45°→80° 이동에 4-5초)
CONTROL_BODY_VEL_FRACTION = 0.15

# 오른손 B 버튼(xr-standard 게임패드 인덱스 5) → 제어 해제
GAMEPAD_BUTTON_B = 5

# 모바일 베이스: 오른손 스틱. 전진 밀기(axes[3] < 0)가 +x, 왼쪽 밀기(axes[2] < 0)가 +yaw(CCW).
# 실기기에서 방향이 반대면 MOBILE_LINEAR_SIGN·MOBILE_ANGULAR_SIGN으로 뒤집는다
MOBILE_LINEAR_MAX = 0.3
MOBILE_ANGULAR_MAX = 0.5
MOBILE_LINEAR_SIGN = 1.0
MOBILE_ANGULAR_SIGN = 1.0
MOBILE_STICK_DEADZONE = 0.15
# 스틱 → 속도 1차 필터 시간 상수(초): 급출발·급정지 완화
MOBILE_SMOOTHING_TAU = 0.3

# 그리퍼: 별도 Dynamixel 버스(UPC 규약 장치명), 모터 ID = 0(오른쪽)·1(왼쪽).
# 왼쪽 모터는 기계적으로 방향이 반대라 열림 정도를 뒤집어 보낸다 (custom-teleop gripper 규약)
GRIPPER_DEVICE = "/dev/rby1_gripper"
# 그리퍼 모터 전원은 본체 툴 플랜지 출력(V)에서 나온다 — 공급 전에는 버스 ping이 실패한다
GRIPPER_TOOL_VOLTAGE = 12
GRIPPER_POWER_SETTLE_S = 0.5
GRIPPER_BAUDRATE = 2_000_000
GRIPPER_MOTOR_IDS = {"right": 0, "left": 1}
GRIPPER_INVERT = {"right": False, "left": True}
GRIPPER_RATE_HZ = 10.0
# 호밍: 약한 전류로 양끝까지 밀어 엔코더 범위를 잰다. 엔코더 변화가 EPS(rad) 미만인 읽기가
# STALL_COUNT회(×0.1 s) 이어지면 끝에 닿은 것으로 판정한다 (엔코더 미세 떨림 약 0.002 rad 실측 —
# 완전 일치 조건은 영원히 끝나지 않는다). 한 방향이 TIMEOUT을 넘기면 실패로 본다
GRIPPER_HOMING_CURRENT = 0.5
GRIPPER_HOMING_STALL_COUNT = 30
GRIPPER_HOMING_STALL_EPS = 0.01
GRIPPER_HOMING_TIMEOUT_S = 30.0
GRIPPER_HOLD_CURRENT = 5.0

# D455 캡처: depth·color 동일 해상도로 열고 depth를 color에 정렬한다
# (정렬된 depth + color 내부 파라미터로 deproject하면 색상 포인트클라우드가 바로 나옴)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_STATS_INTERVAL_S = 5.0

# 카메라 장착: 머리 팬틸트 끝 링크에 고정. 삼각대 나사 마운트 = URDF에 주석으로 남은 head_mount 조인트
# (link_head_2 기준 xyz 0.022 0 0.04). depth 광학 중심은 나사 마운트에서 realsense-ros 규약
# (+0.0158, +0.0475, +0.0145) m 만큼 떨어져 있으므로 더해 둔다 (몸체가 팬틸트 중앙에 옴).
# 광학 프레임 규약(x=우, y=하, z=전방)을 링크 프레임(x=전방, y=좌, z=상)에 대응:
# link_x = cam_z, link_y = -cam_x, link_z = -cam_y. 실기기 장착 후 실측값으로 보정 가능
CAMERA_MOUNT_LINK = "link_head_2"
CAMERA_MOUNT_POS = (0.022 + 0.0158, 0.0475, 0.04 + 0.0145)
CAMERA_MOUNT_ROT = (
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
)

# 가상 로봇의 카메라 표시 메시: Intel realsense-ros의 d455.stl을 camera_link 프레임(m)으로 구운 GLB
# (web/assets/, 정적 루트로 서빙). 원점 = 좌측 IR(depth) 광학 중심, 몸체는 -y(우측)로 124 mm 뻗음
CAMERA_MESH_URL_PREFIX = "/assets/"
CAMERA_MESH_FILE = "d455.glb"

# 포인트클라우드: 픽셀 스트라이드 다운샘플(640x480/4 → 최대 19,200점), 유효 깊이 범위(m), 송신 주기
PCD_STRIDE = 4
PCD_MIN_DEPTH = 0.2
# 유효 깊이 상한: 실내에서 4 m는 짧아 벽·바닥 대부분이 잘려 나간다 (실측: 유효점 775/19200).
# 점 수는 스트라이드 격자(19,200)로 이미 묶여 있어 상한을 늘려도 대역폭이 튀지 않는다
PCD_MAX_DEPTH = 8.0
PCD_RATE_HZ = 10.0
