"""전역 설정: 경로·포트·IK·카메라 파라미터 등 모듈 간 공유 파라미터를 한곳에 모은다.

호출 사이 유지 상태가 없으므로 모듈 수준 상수로 정의한다.
"""

import os
from pathlib import Path

# 저장소 루트 (컨테이너에서는 /workspace). 이 파일 위치 기준으로 역산해
# 마운트 경로에 독립적으로 동작한다
REPO_ROOT = Path(__file__).resolve().parents[2]

# 초기 실행 모드: dev = 가짜 부품(시뮬 로봇·시뮬 손·합성 카메라)으로 전체 파이프라인 구동 (하드웨어 불필요),
#                deploy = 실로봇(gRPC)·실물 D455 사용 (로봇 내장 PC).
# 실로봇을 움직이는 쪽이 명시적 선택이어야 하므로 기본값은 dev다.
# 기동 후에는 웹 데스크톱 뷰의 모드 버튼으로 런타임 전환할 수 있다 (main.py의 ModeManager).
# 초기 모드가 deploy인데 로봇 연결이 실패하면 폴백 없이 기동 실패로 처리하고,
# 웹에서 deploy로 전환하다 실패하면 현재 모드를 유지한 채 오류만 알린다.
TELEOP_MODE = os.environ.get("TELEOP_MODE", "dev")

# dev 모드 시뮬 로봇: 명령 추종 1차 지연 시간 상수(초).
# 0보다 크게 두면 명령-측정 오차가 실제로 생겨 추적 오차 재동기화(CONTROL_MAX_TRACKING_ERR)·속도 클램프
# 경로까지 dev에서 자극된다. 팔 속도 상한(1.5 rad/s) × τ ≈ 정상 상태 오차 0.23 rad < 재동기화 임계 0.5
SIM_TRACKING_TAU = 0.15
# dev 모드 시뮬 손: 가짜 호밍 소요 시간(초) — 기동 직후 ready 전 상태 처리를 시험하기 위한 지연
SIM_HAND_HOMING_S = 1.0
# dev 모드 시뮬 손: 손가락 목표 추종 1차 지연 시간 상수(초)
SIM_HAND_TAU = 0.2

# 프론트엔드 정적 파일 루트와 자체 서명 인증서 출력 위치
WEB_ROOT = REPO_ROOT / "web"
CERT_DIR = WEB_ROOT / "certs"
CERT_FILE = CERT_DIR / "server.crt"
KEY_FILE = CERT_DIR / "server.key"

# 정적 자산 루트: 로봇 모델(rby1a/)과 시각화 에셋(avatar.glb, d455.glb)을 한곳에 모은다.
# 서버가 /assets/ URL로 서빙한다 (web_server.py)
ASSETS_DIR = REPO_ROOT / "assets"

# RBY1 모델: IK용 URDF와 브라우저 시각화용 GLB 메시 (assets/rby1a, 약 14MB — rby1-sdk v0.9.1의
# rby1a/urdf에서 model.urdf와 glb만 추린 것. dae는 미사용: pinocchio는 URDF XML만 파싱하고
# 브라우저는 glb만 그린다). 저장소 상대경로라 어디서든 동작하고, 환경 변수로 덮어쓸 수 있다.
# SDK를 올릴 때는 pip 버전과 같은 태그의 models/rby1a로 이 폴더를 갱신할 것 (버전 어긋남 주의)
RBY1_MODEL_ROOT = Path(os.environ.get("RBY1_MODEL_ROOT", str(ASSETS_DIR / "rby1a")))
RBY1_URDF = RBY1_MODEL_ROOT / "urdf" / "model.urdf"
RBY1_MESH_DIR = RBY1_URDF.parent / "meshes"
MESH_URL_PREFIX = "/models/"

# Inspire RH56E2 손 모델 (assets/inspire_hand, tools/build_inspire_hand.py가 STEP에서 생성): 손 URDF 조각을
# 로딩할 때 RBY1 URDF의 그리퍼 자리(ee_right/ee_left)에 붙인다. 손 메시는 /assets/ URL로 서빙된다
HAND_MODEL_DIR = ASSETS_DIR / "inspire_hand"
HAND_MESH_DIR = HAND_MODEL_DIR / "meshes"
HAND_MESH_URL_PREFIX = "/assets/inspire_hand/meshes/"
# 손가락 굽힘(0 펼침 - 1 굽힘) → 손 모델 관절각: 네 손가락은 근위·원위 마디가 각각 0 → HAND_FINGER_MAX_DEG
# (합 160°, 매뉴얼 굽힘 범위 156°에 대응). 엄지 회전·굽힘은 (기준 굽힘 값, 전체 범위 deg) — 기준 값일 때가
# 모델 기준 자세(STEP 자세)이고, 범위는 매뉴얼 각도 범위(회전 75°, 굽힘 83°)
HAND_FINGER_MAX_DEG = 80.0
HAND_THUMB_YAW = (0.7, 75.0)
HAND_THUMB_BEND = (0.45, 83.0)

# HTTPS/WSS 바인딩 (Quest 직결·PC 모니터링 공용, WebXR secure context 요건상 HTTPS 필수)
HTTP_HOST = "0.0.0.0"
HTTPS_PORT = 8443

# 접속 편의 포트: 443 = https://IP 만으로 접속되는 별칭, 80 = http 접속을 HTTPS로 리다이렉트
HTTPS_STD_PORT = 443
HTTP_REDIRECT_PORT = 80

# WebRTC 시그널링 경로: SDP offer를 POST하면 answer를 JSON으로 반환한다 (지속 연결 아님 — 1회 왕복).
# 이후 통신(포즈 업링크·robot_state·메시 블록 등)은 전부 DataChannel로 흐른다 (rtc_hub.py 참고)
RTC_OFFER_PATH = "/rtc/offer"

# 포즈 파이프라인 상태 로그 주기(초)와 샘플 신선도 한계(초, 초과 시 IK 입력 무효)
POSE_STATS_INTERVAL_S = 5.0
POSE_STALE_S = 0.5

# 손 TCP(파지 중심)의 ee 프레임 로컬 오프셋(m): 장착면에서 손바닥 앞 파지 중심까지 (RH56E2 STEP 실측)
# 사용자 손(grip 중심)이 이 지점에 매핑된다
HAND_TCP_OFFSET = (-0.035, 0.0, -0.11)

# IK 루프·솔버 파라미터 (sqpmethod: warm start 연속 추종에서는 소수 반복이면 충분)
IK_RATE_HZ = 30.0
IK_MAX_ITER = 6
# 코스트 가중치: TCP 위치 최우선, 자세는 손목 관절 추종이 체감되는 수준,
# 팔꿈치는 자세 유도 보조 (body-tracking 있을 때만), ready 바이어스는 여유 자유도 해소용.
# 위치:자세 비율이 작으면 작업 공간 경계에서 솔버가 자세를 지키려고 위치를 포기해 팔이 덜 뻗는다
# (오프라인 비교: 50:8이면 옆 0.46 m·아래 0.34 m에서 멈춤, 2000:8이면 기구학 한계 0.55·0.43 m까지 도달).
# 위치가 닿는 영역에서는 7자유도 여유로 자세도 그대로 맞으므로 자세 추종은 손해 보지 않는다
IK_W_POS = 2000.0
IK_W_ROT = 8.0
IK_W_ELBOW = 10.0
IK_W_REG = 0.5
# ready 바이어스는 null-space 투영 없이 소프트 코스트로 넣으므로 추종을 해치지 않게 약하게
IK_W_READY = 0.02
# 관절 속도 상한: URDF velocity의 비율로 관절별 계산하되 전역 상한도 둔다 (감사 지적: 4.0은
# 하드웨어 한계 초과 — arm_6은 2.09 rad/s). 실기기 이전 시 그대로 안전 상한으로 사용
IK_VEL_FRACTION = 0.5
IK_MAX_JOINT_VEL = 1.5

# 안전 관절 한계 (Unity teleop 예제의 팔 개별 제어 모드 add_joint_limit 값, rad):
# arm_3(팔꿈치) 완전 신전 금지 — 특이점·몸통 스침 방지.
# arm_5(손목 피치)는 예제와 같이 URDF 한계(-1.57 - 1.92)를 그대로 쓴다 — (0.2, 1.9)로 묶으면
# 손목을 아래로 꺾는 자세가 막혀 실사용 중 arm_5가 하한에 박히고 팔 도달 범위까지 줄었다
SAFETY_JOINT_LIMITS = {
    "right_arm_3": (-2.6, -0.5),
    "left_arm_3": (-2.6, -0.5),
}

# 사람→로봇 팔 매핑: 클러치 델타 방식 (1m = 1m)
ARM_SCALE = 1.0

# Quest 직결 서버 IP: PC 이더넷 인터페이스를 NetworkManager "공유" 모드로 사용
# (공유 모드가 PC를 10.42.0.1로 고정하고 Quest에 DHCP 할당). 인증서 SAN 기본값으로도 사용
DIRECT_LINK_SERVER_IP = "10.42.0.1"

# 실로봇 연결: 로봇 제어 gRPC 주소 (내장 PC에서는 br0 대역의 로봇 내부 주소).
# 상태 구독·시각화 방송 주기는 IK와 같은 30 Hz로 맞춘다
ROBOT_ADDRESS = "192.168.30.1:50051"
ROBOT_STATE_RATE_HZ = 30.0
# 상태 구독 신선도 한계(초): 마지막 수신 후 이 시간이 지나면 측정값 무효 (제어 중이면 해제)
ROBOT_STATE_STALE_S = 0.5

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
# 관절 임피던스 게인 (토르소·양팔, Unity teleop 예제의 스트리밍 값). 제어기 토크는
# τ = K(q_d - q) + D(q̇_d - q̇) + τ_중력보상 이고 관절별로 토크 제한에서 포화된다 (SDK 규약).
# 토크 제한은 중력 보상분까지 포함한 전체 토크에 걸리므로 자기 무게를 버틸 만큼 남겨야 한다.
# stiffness(Nm/rad)가 접촉 시 밀리는 정도를 정한다: 팔 80 Nm/rad면 0.1 rad 밀릴 때 8 Nm로 버틴다
IMPEDANCE_GAINS = {
    "torso": {"stiffness": [500.0] * 6, "torque_limit": [600.0] * 6, "damping_ratio": 0.7},
    "arm": {
        "stiffness": [80.0, 80.0, 80.0, 60.0, 60.0, 60.0, 60.0],
        "torque_limit": [40.0, 40.0, 40.0, 30.0, 30.0, 30.0, 30.0],
        "damping_ratio": 0.85,
    },
}

# 안전장치
# - 입력 두절: VR 업링크가 이 시간 이상 끊기면 제어를 해제한다 (IK는 마지막 자세를 유지하지만,
#   조작자가 사라진 상태로 스트림을 유지할 이유가 없다)
CONTROL_INPUT_TIMEOUT_S = 3.0
# - 추적 오차: 명령과 측정 관절각 차이가 이 값(rad)을 넘으면 명령·IK 상태를 측정 자세로 재동기화한다
#   (제어는 유지). 임피던스 제어라 접촉하면 밀리는 게 정상이지만, 명령을 둔 채 크게 밀렸다 풀리면
#   스프링처럼 튀어 돌아오고 이 복귀는 명령 속도 클램프가 막지 못한다. 0.5 rad × 팔 80 Nm/rad = 40 Nm로
#   토크 제한과 비슷한 지점에서 명령을 측정 자세로 끌어온다
CONTROL_MAX_TRACKING_ERR = 0.5
# - 팔 외 관절(토르소·머리)을 ready로 보내는 속도는 URDF 한계에 이 비율을 곱한다 (팔은 IK_VEL_FRACTION).
#   토르소는 무거우므로 보수적으로 (0.15 → torso_0-2 약 0.3 rad/s, 45°→80° 이동에 4-5초)
CONTROL_BODY_VEL_FRACTION = 0.15
# - 머리(head_0·head_1)가 조작자 시선을 따라가는 속도: URDF 한계(3.14 rad/s)에 이 비율을 곱한다 (0.5 → 1.57 rad/s)
CONTROL_HEAD_VEL_FRACTION = 0.5

# 다지 손(Inspire RH56E2) Modbus TCP 주소 {side: (IP, 포트)}. 손은 외부 24V 전원으로 구동되고,
# 호스트 USB-LAN 어댑터(192.168.11.222)와 같은 대역에 있다
HAND_ADDRESSES = {"right": ("192.168.11.211", 6000), "left": ("192.168.11.210", 6000)}
# Modbus 장치 ID (매뉴얼 규정 0xFF, TCP에서는 IP로 구분하므로 펌웨어가 무시한다)
HAND_DEVICE_ID = 0xFF
# 손별 구동 루프 주기: 실측 읽기 지연(6레지스터 p50 0.4 ms, p95 5.6 ms, 최대 10 ms)이 20 ms 주기에 충분히 들어간다
HAND_RATE_HZ = 50.0
# 실측 힘(FORCE_ACT) 읽기 분주: 루프 N회마다 1회 (50 Hz / 5 = 10 Hz)
HAND_FORCE_READ_DIVIDER = 5
# 손가락 속도(0-1000)와 힘 제어 임계값(g, 0-3000): 접속 시 1회 기록. 기본값(1000, 2000)보다 보수적으로
HAND_SPEED = 500
HAND_FORCE_LIMIT = 500
# 같은 목표를 변경 없이도 재전송하는 주기(초): 손 재부팅 등으로 목표가 사라져도 복구되게 한다
HAND_RESEND_S = 1.0
# 접속·요청 타임아웃(초)과 재접속 대기(초, 실패할 때마다 두 배로 늘려 상한까지)
HAND_TIMEOUT_S = 0.5
HAND_RECONNECT_S = 1.0
HAND_RECONNECT_MAX_S = 10.0
# 상태 로그 주기(초)
HAND_STATS_INTERVAL_S = 5.0

# Manus 장갑 (조작자 손가락 입력): SDK Integrated 모드 — MANUS Core 없이 서버 PC에 꽂힌 동글에 직접 접속한다.
# 페어링되지 않은 장갑이 보이면 자동으로 동글에 페어링한다
# - SDK 접속 재시도 대기(초), 장갑 상태 점검(자동 페어링) 주기(초), 상태 로그 주기(초)
MANUS_RECONNECT_S = 2.0
MANUS_POLL_S = 1.0
MANUS_STATS_INTERVAL_S = 5.0
# - ergonomics 미수신이 이 시간(초)을 넘으면 그 손의 장갑 값은 무효 (조작 샘플 fingers로 대체)
MANUS_STALE_S = 0.5
# - ergonomics 관절각(deg) → 굽힘 0(펼침)-1(굽힘) 정규화 범위 (펼침 각, 굽힘 각). 양손 실측 분포 기준:
#   네 손가락(MCP+PIP+DIP 굴곡 합): 편 손 -15~28, 주먹 206~240
#   엄지 굽힘(MCP+IP 굴곡 합): 편 손 -37~-8, 주먹 29~77 (손바닥 안으로 접으면 최대 130)
#   엄지 회전(CMC spread, +가 손바닥 쪽 palmar abduction): 편 손 -34~-18, 주먹 +14~+28
MANUS_FINGER_CURL_RANGE_DEG = (10.0, 210.0)
MANUS_THUMB_BEND_RANGE_DEG = (-20.0, 90.0)
MANUS_THUMB_ROTATE_RANGE_DEG = (-25.0, 20.0)

# D455 캡처: depth·color 동일 해상도로 열고 depth를 color에 정렬한다
# (정렬된 depth + color 내부 파라미터로 deproject하면 색상 포인트클라우드가 바로 나옴)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_STATS_INTERVAL_S = 5.0
# D455 끊김(USB 재연결 등) 시 파이프라인 재오픈 재시도 간격(초)
CAMERA_RECONNECT_S = 2.0

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
CAMERA_MESH_URL_PREFIX = "/assets/"
CAMERA_MESH_FILE = "d455.glb"

# 인식(메시 재구성) 처리 주기(Hz): 카메라 프레임마다 포즈·마스크를 붙여 TSDF 워커로 넘기는 주기
PERCEPTION_RATE_HZ = 10.0
# 작업 영역 상자 (base 프레임 m, (x), (y), (z)): 로봇 앞 손이 닿는 공간 = 고해상도 작업 영역 메시 범위.
# 바닥(z<0.05)은 환경 메시가 보여 주므로 뺀다
WORKSPACE_BOX = ((0.05, 1.3), (-0.9, 0.9), (0.05, 1.9))

# 프레임 포즈: 관절 측정 이력 폴링 주기(Hz)와 보관 길이(초). 프레임 시각의 관절값을 이력 보간으로 구한다
POSED_JOINT_POLL_HZ = 60.0
POSED_HISTORY_S = 1.0

# 로봇 자기 몸 마스크 (메시에 팔·손이 구워지지 않게 depth에서 제거): 팔·손 링크 메시를 FK로 depth 영상에 투영해
# 관측 깊이가 로봇 표면보다 그룹별 여유(m) 넘게 앞이 아닌 픽셀을 지운다 (손에 쥔 물체처럼 손보다 앞에 있는 것은 남는다).
# 그룹 {링크 이름 접두사: 깊이 여유}: 팔·FT 센서는 URDF에 없는 어댑터·케이블과 보정 오차를 덮도록 넉넉하게,
# 손은 쥔 물체를 살리도록 좁게
SELF_MASK_GROUPS = {
    ("link_right_arm_", "link_left_arm_", "FT_sensor_"): 0.10,
    ("right_", "left_"): 0.04,
}
# - 표면 샘플 간격(m): 삼각형 면적 비례 균일 샘플 — CAD 메시는 큰 삼각형이라 정점만으로는 앞면 가운데가 비어
#   뒷면 깊이가 잡힌다. 투영 빈틈 메우기 반경(격자 칸)
SELF_MASK_POINT_SPACING = 0.004
SELF_MASK_FILL_CELLS = 2
# 최종 마스크 팽창 반경(격자 칸): 로봇 실루엣 경계의 섞인 깊이(flying pixel)까지 지운다
SELF_MASK_DILATE_CELLS = 2
# 픽셀 판정(자기 몸·작업 영역) 격자 스트라이드: 이 간격으로 역투영해 판정하고 원 해상도로 확장한다
PIXEL_MASK_STRIDE = 2

# 메시 재구성 (TSDF, 별도 프로세스): 두 층으로 나눈다 — 환경 층(방 전체, 거친 복셀)과 작업 영역 층(WORKSPACE_BOX 안,
# 고운 복셀). 층마다 복셀 크기(m)와 블록 생성 깊이 상한(m)을 둔다. 작업 영역 층은 손 작업 물체(수 cm)의 형상을 살리는 크기
TSDF_VOXEL = 0.03
TSDF_DEPTH_MAX = 5.0
TSDF_WS_VOXEL = 0.006
TSDF_WS_DEPTH_MAX = 2.5
# 공통: 메시 추출 가중치 임계값, 초기 블록 용량(부족하면 늘어난다), 메시 추출·송신 주기(초)
TSDF_WEIGHT_THRESHOLD = 3.0
TSDF_BLOCK_COUNT = 1000
TSDF_EXTRACT_S = 1.0
# 동적 물체(지나간 사람 등) 지우기: 시야 안 기존 블록도 매 프레임 적분해 카메라-표면 사이를 빈 공간으로 갱신한다.
# - 빈 공간 갱신용 관측 깊이 상한(m): 블록 생성(TSDF_DEPTH_MAX)보다 멀리 본 광선도 그 앞 공간을 지울 수 있게 크게 둔다
# - 복셀 가중치 상한: 오래 본 표면도 상한 프레임 수(30 = 10 Hz에서 약 3초) 안에 새 관측으로 바뀐다
TSDF_CARVE_DEPTH_MAX = 10.0
TSDF_MAX_WEIGHT = 30.0
# TSDF 워커 프로세스가 쓸 CPU 코어 수: Open3D의 TBB 스레드 풀은 프로세스 CPU affinity만큼 스레드를 만들므로
# 워커를 코어 몇 개에 묶어 제어 프로세스와 CPU를 다투지 않게 한다 (사용 가능한 코어 중 뒤쪽 번호를 쓴다)
TSDF_WORKER_CPUS = 4
# 머리 관절 속도(rad/s)가 이 값을 넘는 프레임은 적분하지 않는다 (프레임-관절 시각 오차로 맵이 번지는 것 방지)
TSDF_MAX_HEAD_SPEED = 0.3
# 메시 전송 블록 한 변(m, 층별): 추출한 메시를 이 격자로 나눠 바뀐 블록만 보낸다. 블록 하나가 메시지 상한
# (MESH_MAX_MESSAGE_BYTES)을 넘지 않는 크기여야 한다 — 작업 영역 층은 삼각형이 촘촘해(6 mm 복셀) 작게 나눈다
MESH_SPLIT_SIZE = 0.48
MESH_SPLIT_SIZE_WS = 0.12
# 메시 패킷 하나의 최대 크기(바이트): DataChannel 메시지 상한(256 KB) 아래로 블록을 나눠 담는다
MESH_MAX_MESSAGE_BYTES = 192 * 1024
