"""전역 설정: 경로·포트·IK·카메라 파라미터 등 모듈 간 공유 파라미터를 한곳에 모은다.

호출 사이 유지 상태가 없으므로 모듈 수준 상수로 정의한다.
"""

from pathlib import Path

# 저장소 루트 (컨테이너에서는 /workspace). 이 파일 위치 기준으로 역산해 마운트 경로에 독립적으로 동작
REPO_ROOT = Path(__file__).resolve().parents[2]

# 프론트엔드 정적 파일 루트와 자체 서명 인증서 출력 위치
WEB_ROOT = REPO_ROOT / "web"
CERT_DIR = WEB_ROOT / "certs"
CERT_FILE = CERT_DIR / "server.crt"
KEY_FILE = CERT_DIR / "server.key"

# RBY1 모델 리소스 (rby1-sdk 동봉): IK용 URDF와 브라우저 시각화용 GLB 메시
RBY1_URDF = REPO_ROOT / "oss" / "rby1-sdk" / "models" / "rby1a" / "urdf" / "model.urdf"
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
PCD_MAX_DEPTH = 4.0
PCD_RATE_HZ = 10.0
# 누적: 복셀(m) 격자에 최신 관측을 유지하고, PCD_ACCUM_FRAMES 프레임 동안 재관측되지 않은 복셀은 버린다
# (같은 표면이 프레임마다 중복 쌓이지 않고, 깊이 홀·노이즈는 시간 누적으로 메워짐)
PCD_VOXEL_M = 0.02
PCD_ACCUM_FRAMES = 20
