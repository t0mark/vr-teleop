"""전역 설정: 경로·포트·IK·카메라 파라미터 등 모듈 간 공유 파라미터를 모듈 수준 상수로 한곳에 모은다."""

import os
from pathlib import Path

# 저장소 루트 (이 파일 위치에서 역산, 컨테이너에서는 /workspace)
REPO_ROOT = Path(__file__).resolve().parents[2]

# 초기 실행 모드 (환경 변수 TELEOP_MODE, 기본 dev):
#   dev = 시뮬 로봇·시뮬 손·합성 카메라로 전체 파이프라인 구동 (하드웨어 불필요)
#   deploy = 실로봇(gRPC)·Inspire 손·D455 사용 (로봇 내장 PC), 로봇 연결 실패 시 기동 실패
# 기동 후 웹 데스크톱 뷰의 모드 버튼으로 런타임 전환한다 (main.py의 ModeManager, 전환 실패 시 현재 모드 유지)
TELEOP_MODE = os.environ.get("TELEOP_MODE", "dev")

# 서버 로그 파일 경로와 크기 기준 회전 설정 (파일당 최대 바이트, 보관할 이전 파일 수)
LOG_FILE = REPO_ROOT / "logs" / "web_api.log"
LOG_MAX_BYTES = 10 * 1024 * 1024
LOG_BACKUP_COUNT = 5

# dev 모드 시뮬 로봇: 명령 추종 1차 지연 시간 상수(초)
SIM_TRACKING_TAU = 0.15
# dev 모드 시뮬 손: ready가 되기까지의 가짜 호밍 소요 시간(초)
SIM_HAND_HOMING_S = 1.0
# dev 모드 시뮬 손: 손가락 목표 추종 1차 지연 시간 상수(초)
SIM_HAND_TAU = 0.2

# 프론트엔드 정적 파일 루트
WEB_ROOT = REPO_ROOT / "web"

# 자체 서명 인증서 출력 위치 (정적 서빙 루트 밖)
CERT_DIR = REPO_ROOT / "certs"
CERT_FILE = CERT_DIR / "server.crt"
KEY_FILE = CERT_DIR / "server.key"

# 정적 자산 루트: 로봇 모델(rby1a/)과 시각화 에셋(avatar.glb, d455.glb)을 한곳에 모은다.
# 서버가 /assets/ URL로 서빙한다 (web_server.py)
ASSETS_DIR = REPO_ROOT / "assets"
# 정적 자산 서빙 라우트 경로
ASSETS_ROUTE_PREFIX = "/assets/"

# RBY1 모델: IK용 URDF와 브라우저 시각화용 GLB 메시 (assets/rby1a, 약 14MB — rby1-sdk v0.9.1의
# rby1a/urdf에서 model.urdf와 glb만 추린 것. dae는 미사용: pinocchio는 URDF XML만 파싱하고
# 브라우저는 glb만 그린다). 저장소 상대경로라 어디서든 동작하고, 환경 변수로 덮어쓸 수 있다.
# SDK를 올릴 때는 pip 버전과 같은 태그의 models/rby1a로 이 폴더를 갱신할 것 (버전 어긋남 주의)
RBY1_MODEL_ROOT = Path(os.environ.get("RBY1_MODEL_ROOT", str(ASSETS_DIR / "rby1a")))
RBY1_URDF = RBY1_MODEL_ROOT / "urdf" / "model.urdf"
RBY1_MESH_DIR = RBY1_URDF.parent / "meshes"
# RBY1 메시 서빙 라우트 경로
MESH_ROUTE_PREFIX = "/models/"
# 브라우저에 보내는 RBY1 메시 URL 접두어 (index.html 기준 상대 경로)
MESH_URL_PREFIX = "models/"

# Inspire RH56E2 손 모델: RBY1 URDF의 그리퍼 자리(ee_right/ee_left)에 붙일 손 URDF 조각·메시 경로와 메시 URL 접두어
HAND_MODEL_DIR = ASSETS_DIR / "inspire_hand"
HAND_MESH_DIR = HAND_MODEL_DIR / "meshes"
HAND_MESH_URL_PREFIX = "assets/inspire_hand/meshes/"
# 손가락 굽힘(0-1) → 손 모델 관절각: 네 손가락 마디별 최대 각(deg)
HAND_FINGER_MAX_DEG = 80.0
# 엄지 회전·굽힘: (모델 기준 자세의 굽힘 값, 전체 각도 범위 deg)
HAND_THUMB_ROTATE = (0.7, 75.0)
HAND_THUMB_BEND = (0.45, 83.0)

# HTTPS 서버 바인딩 주소와 본 포트
HTTP_HOST = "0.0.0.0"
HTTPS_PORT = 8443

# 접속 편의 포트: 443은 HTTPS 별칭, 80은 HTTPS 리다이렉트
HTTPS_STD_PORT = 443
HTTP_REDIRECT_PORT = 80

# WebRTC 시그널링 경로: SDP offer를 POST하면 answer를 JSON으로 반환한다
RTC_OFFER_PATH = "/rtc/offer"

# state 채널 송신 대기 바이트 상한: 초과한 피어에는 robot_state 등 방송을 건너뛴다
RTC_STATE_BUFFER_LIMIT_BYTES = 256 * 1024
# mesh 채널 송신 대기 바이트 경고 기준: 초과하면 경고 로그만 남긴다
RTC_MESH_BUFFER_WARN_BYTES = 8 * 1024 * 1024

# 포즈 파이프라인 상태 로그 주기(초)와 샘플 신선도 한계(초, 초과 시 IK 입력 무효)
POSE_STATS_INTERVAL_S = 5.0
POSE_STALE_S = 0.5

# 손 TCP(파지 중심)의 ee 프레임 로컬 오프셋(m): 장착면에서 손바닥 앞 파지 중심까지 (RH56E2 STEP 실측)
# 사용자 손(grip 중심)이 이 지점에 매핑된다
HAND_TCP_OFFSET = (-0.035, 0.0, -0.11)

# IK 주기(Hz)
IK_RATE_HZ = 30.0

# IK(Pink 미분 IK) 작업 비용: 손 TCP 위치(1/m)·손 자세(1/rad)·ready 자세 유지(1/rad)
# 위치 비용을 가장 크게 두어 손 위치 정확도를 우선하고, ready 자세 유지는 남는 자유도(팔꿈치)만 정한다
IK_POSITION_COST = 1.0
IK_ORIENTATION_COST = 0.3
IK_POSTURE_COST = 1e-3
# IK 손 작업의 Levenberg-Marquardt 감쇠 (특이 자세 근처 관절 속도 폭주 방지)
IK_LM_DAMPING = 1e-3
# IK QP 솔버 이름(qpsolvers)과 전체 감쇠
IK_QP_SOLVER = "daqp"
IK_DAMPING = 1e-6
# IK 관절 한계 복원 게인 (0-1, 한계 밖에서 안으로 돌아오는 속도 비율)
IK_CONFIG_LIMIT_GAIN = 0.5

# 팔 관절 속도 상한(rad/s): URDF velocity × IK_VEL_FRACTION, 전역 상한 IK_MAX_JOINT_VEL
IK_VEL_FRACTION = 0.5
IK_MAX_JOINT_VEL = 1.5

# 속도 제한 스텝에 쓰는 경과 시간 상한 = IK 주기(1 / IK_RATE_HZ) × 이 배수
IK_MAX_DT_FACTOR = 2.0

# IK 입력 거부·솔브 실패 경고를 모아서 남기는 주기(초)
IK_WARN_INTERVAL_S = 1.0

# 안전 관절 한계 {관절명: (하한, 상한) rad}: URDF 한계와 교집합으로 IK에 적용한다
SAFETY_JOINT_LIMITS = {
    "right_arm_3": (-2.6, -0.5),
    "left_arm_3": (-2.6, -0.5),
}

# 사람→로봇 팔 매핑: 클러치 델타 방식 (1m = 1m)
ARM_SCALE = 1.0

# Quest 직결 서버 IP: PC 이더넷 인터페이스를 NetworkManager "공유" 모드로 사용
# (공유 모드가 PC를 10.42.0.1로 고정하고 Quest에 DHCP 할당). 인증서 SAN 기본값으로도 사용
DIRECT_LINK_SERVER_IP = "10.42.0.1"

# 실로봇 gRPC 주소
ROBOT_ADDRESS = "192.168.30.1:50051"
# 상태 구독·시각화 방송 주기(Hz)
ROBOT_STATE_RATE_HZ = 30.0
# 상태 구독 신선도 한계(초): 마지막 수신 후 이 시간이 지나면 측정값 무효
ROBOT_STATE_STALE_S = 0.5

# 실로봇 명령 스트리밍 주기(Hz)
CONTROL_RATE_HZ = 30.0
# 명령 minimum_time·control_hold_time = 명령 주기 × 배수
CONTROL_MIN_TIME_FACTOR = 1.01
CONTROL_HOLD_TIME_FACTOR = 10.0
# 명령 전송 타임아웃(ms)
CONTROL_SEND_TIMEOUT_MS = 200
# Control Manager unlimited 모드 사용 여부 (활성화 시 관절 펌웨어 한계 검사 생략)
CM_UNLIMITED_MODE = True
# 머리 관절 위치 PID 게인 {관절명: (P, I, D)}: Control Manager 활성화 전에 1회 적용
HEAD_PID_GAINS = {"head_0": (200, 0, 8000), "head_1": (200, 0, 8000)}
# 제어 준비(wait_for_control_ready) 대기 상한(ms)
CONTROL_READY_TIMEOUT_MS = 1000
# 관절 임피던스 게인 {torso/arm: stiffness(Nm/rad)·토크 제한(Nm)·감쇠비}
IMPEDANCE_GAINS = {
    "torso": {"stiffness": [500.0] * 6, "torque_limit": [600.0] * 6, "damping_ratio": 0.7},
    "arm": {
        "stiffness": [80.0, 80.0, 80.0, 60.0, 60.0, 60.0, 60.0],
        "torque_limit": [40.0, 40.0, 40.0, 30.0, 30.0, 30.0, 30.0],
        "damping_ratio": 0.85,
    },
}

# 업링크 두절 해제 시간(초): VR 입력이 이 시간 이상 끊기면 제어를 해제한다
CONTROL_INPUT_TIMEOUT_S = 3.0
# 추적 오차 한계(rad): 명령-측정 차이가 넘으면 명령·IK 상태를 측정 자세로 재동기화한다
CONTROL_MAX_TRACKING_ERR = 0.5
# 토르소 명령 속도 상한 = URDF 속도 한계 × 이 비율
CONTROL_BODY_VEL_FRACTION = 0.15
# 머리 명령 속도 상한 = URDF 속도 한계 × 이 비율
CONTROL_HEAD_VEL_FRACTION = 0.5
# 제어 루프 주기 처리 예외의 로그 최소 간격(초)
CONTROL_ERROR_LOG_INTERVAL_S = 1.0

# 다지 손(Inspire RH56E2) Modbus TCP 주소 {side: (IP, 포트)}
HAND_ADDRESSES = {"right": ("192.168.11.211", 6000), "left": ("192.168.11.210", 6000)}
# Modbus 장치 ID
HAND_DEVICE_ID = 0xFF
# 손별 구동 루프 주기(Hz)
HAND_RATE_HZ = 50.0
# 실측 힘(FORCE_ACT) 읽기 분주: 루프 N회마다 1회
HAND_FORCE_READ_DIVIDER = 5
# 손가락 속도(0-1000)와 힘 제어 임계값(g, 0-3000): 접속 시 1회 기록
HAND_SPEED = 500
HAND_FORCE_LIMIT = 500
# 같은 목표를 변경 없이 재전송하는 주기(초)
HAND_RESEND_S = 1.0
# 접속·요청 타임아웃(초)과 재접속 대기(초, 실패할 때마다 두 배로 늘려 상한까지)
HAND_TIMEOUT_S = 0.5
HAND_RECONNECT_S = 1.0
HAND_RECONNECT_MAX_S = 10.0
# 손 상태 로그 주기(초)
HAND_STATS_INTERVAL_S = 5.0
# 목표 거부(유한하지 않은 값·준비 전)와 구동 루프 처리 오류 경고의 손별 최소 간격(초)
HAND_REJECT_LOG_INTERVAL_S = 5.0

# Manus 장갑 (조작자 손가락 입력): SDK Integrated 모드로 서버 PC의 동글에 직접 접속한다
# SDK 초기화·접속 재시도 대기(초), 장갑 상태 점검(자동 페어링) 주기(초), 상태 로그 주기(초)
MANUS_RECONNECT_S = 2.0
MANUS_POLL_S = 1.0
MANUS_STATS_INTERVAL_S = 5.0

# raw skeleton 미수신이 이 시간(초)을 넘은 손은 장갑 값 무효 (조작 샘플 fingers로 대체)
MANUS_STALE_S = 0.5

# 손 키포인트가 이 시간(초) 이상 완전히 같으면 경고 로그 (값은 계속 사용)
MANUS_FROZEN_WARN_S = 3.0

# 미페어링 장갑 자동 페어링 여부, 사용할 장갑 id 목록 (비어 있으면 모든 장갑 허용, 예: (0x7736F1B4,))
MANUS_AUTO_PAIR = True
MANUS_ALLOWED_GLOVE_IDS: tuple[int, ...] = ()

# 손 리타게팅(dex-retargeting DexPilot): 손별 최대 계산 주기(Hz)
HAND_RETARGET_RATE_HZ = 50.0
# 손 리타게팅용 Inspire 손 URDF 폴더 (dex-urdf, 손끝 링크·mimic 관절 포함, 계산 전용)
HAND_RETARGET_URDF_DIR = HAND_MODEL_DIR / "retarget"
# 사람 손 → 로봇 손 크기 배율 (Inspire 손이 사람 손보다 크다)
HAND_RETARGET_SCALING = 1.15
# 리타게팅 결과 저역 통과 필터 계수 (작을수록 부드럽고 지연이 크다)
HAND_RETARGET_LOW_PASS_ALPHA = 0.2

# 카메라 캡처: depth·color 공통 해상도(픽셀)·프레임률(Hz), 프레임률 로그 주기(초)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_STATS_INTERVAL_S = 5.0
# D455 끊김(USB 재연결 등) 시 파이프라인 재오픈 재시도 간격(초)
CAMERA_RECONNECT_S = 2.0
# D455 프레임셋 수신 대기 한도(ms), 캡처 스레드 정지 대기 여유(초)
CAMERA_FRAME_TIMEOUT_MS = 2000
CAMERA_STOP_MARGIN_S = 3.0
# D455 depth 프리셋 (pyrealsense2 rs400_visual_preset 이름): high_accuracy는 신뢰도 낮은 깊이를 버린다
CAMERA_DEPTH_PRESET = "high_accuracy"
# D455 depth 후처리: 공간 필터 (평활 강도 0.25-1, 경계 보존 기준 1-50, 반복 1-5)
CAMERA_SPATIAL_ALPHA = 0.5
CAMERA_SPATIAL_DELTA = 20
CAMERA_SPATIAL_MAGNITUDE = 2
# D455 depth 후처리: 시간 필터 (평활 강도 0-1, 경계 보존 기준 1-100)
CAMERA_TEMPORAL_ALPHA = 0.4
CAMERA_TEMPORAL_DELTA = 20

# 카메라 장착: 머리 팬틸트 끝 링크(link_head_2) 기준 depth 광학 프레임 포즈 (POS: m, ROT: 열 = 광학 x·y·z 축).
# 광학 프레임(x=우, y=하, z=전방)과 링크 프레임(x=전방, y=좌, z=상)의 대응은 대략
# link_x ≈ cam_z, link_y ≈ -cam_x, link_z ≈ -cam_y 이다
CAMERA_MOUNT_LINK = "link_head_2"
CAMERA_MOUNT_POS = (0.0106, 0.045, 0.0752)
CAMERA_MOUNT_ROT = (
    (-0.10159, 0.04796, 0.99367),
    (-0.99454, -0.02857, -0.1003),
    (0.02358, -0.99844, 0.0506),
)

# 가상 로봇의 카메라 표시 메시: Intel realsense-ros의 d455.stl을 camera_link 프레임(m)으로 구운 GLB
# (assets/, /assets/ URL로 서빙). 원점 = 좌측 IR(depth) 광학 중심, 몸체는 -y(우측)로 124 mm 뻗음
CAMERA_MESH_URL_PREFIX = "assets/"
CAMERA_MESH_FILE = "d455.glb"

# 인식(메시 재구성) 처리 주기(Hz): 카메라 프레임에 포즈·마스크를 붙여 TSDF 워커로 넘기는 주기
PERCEPTION_RATE_HZ = 10.0
# 작업 영역 상자 (base 프레임 m, (x), (y), (z)): 고해상도 작업 영역 메시 범위 (바닥 z<0.05 제외)
WORKSPACE_BOX = ((0.05, 1.3), (-0.9, 0.9), (0.05, 1.9))

# 프레임 포즈: 관절 측정 폴링 주기(Hz)
POSED_JOINT_POLL_HZ = 60.0
# 프레임 포즈: 관절 측정 이력 보관 길이(초)
POSED_HISTORY_S = 1.0
# 프레임 포즈: 머리 관절 속도를 구하는 이력 구간 길이(초)
POSED_HEAD_SPEED_WINDOW_S = 0.1

# 자기 몸 마스크 링크 그룹 {링크 이름 접두사: 깊이 여유(m)}: 팔·FT 센서는 넓게, 손은 좁게
SELF_MASK_GROUPS = {
    ("link_right_arm_", "link_left_arm_", "FT_sensor_"): 0.05,
    ("right_", "left_"): 0.04,
}
# 자기 몸 마스크: 링크 표면 균일 샘플 간격(m)
SELF_MASK_POINT_SPACING = 0.004
# 자기 몸 마스크: z-buffer 빈틈 메우기 반경(격자 칸)
SELF_MASK_FILL_CELLS = 2
# 자기 몸 마스크: 최종 마스크 팽창 반경(격자 칸)
SELF_MASK_DILATE_CELLS = 2
# 픽셀 판정(자기 몸·작업 영역) 격자 스트라이드(픽셀)
PIXEL_MASK_STRIDE = 2

# TSDF 환경 층: 복셀 크기(m)·블록 생성 깊이 상한(m)
TSDF_VOXEL = 0.03
TSDF_DEPTH_MAX = 4.0
# TSDF 작업 영역 층: 복셀 크기(m)·블록 생성 깊이 상한(m)
TSDF_WS_VOXEL = 0.006
TSDF_WS_DEPTH_MAX = 2.5
# TSDF 공통: 메시 추출 가중치 임계값
TSDF_WEIGHT_THRESHOLD = 3.0
# TSDF 공통: 초기 블록 용량 (부족하면 늘어난다)
TSDF_BLOCK_COUNT = 1000
# TSDF 공통: 메시 추출·송신 주기(초)
TSDF_EXTRACT_S = 1.0
# TSDF 빈 공간 갱신용 관측 깊이 상한(m)
TSDF_CARVE_DEPTH_MAX = 10.0
# TSDF 복셀 가중치 상한 (50 = 10 Hz에서 약 5초 분량)
TSDF_MAX_WEIGHT = 50.0
# TSDF 워커 프로세스가 쓸 CPU 코어 수 (사용 가능한 코어 중 뒤쪽 번호)
TSDF_WORKER_CPUS = 4
# TSDF 워커 프로세스가 죽었을 때 재시작을 시도하는 최소 간격(초)
TSDF_WORKER_RESTART_S = 5.0
# TSDF 입력에서 제외할 머리 관절 속도 상한(rad/s)
TSDF_MAX_HEAD_SPEED = 0.3

# 메시 전송 블록 한 변(m): 환경 층·작업 영역 층
MESH_SPLIT_SIZE = 0.48
MESH_SPLIT_SIZE_WS = 0.12
# 블록 변경 판정 요약의 양자화: 정점 위치 간격(m)과 색 채널 유지 비트 수
MESH_DIGEST_POS_STEP = 0.002
MESH_DIGEST_COLOR_BITS = 5
# 메시 패킷 하나의 최대 크기(바이트)
MESH_MAX_MESSAGE_BYTES = 192 * 1024
