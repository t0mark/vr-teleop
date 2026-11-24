# RBY1 Web VR Teleop

Quest 3 브라우저(WebXR + Babylon.js)에서 Rainbow Robotics **RBY1** 휴머노이드의 양팔·Inspire 다지 손·머리를 원격 조작하는 웹 기반 텔레오퍼레이션 시스템이다.
조작자의 손 위치는 손등에 부착한 Quest 3 컨트롤러로, 손가락은 Manus 장갑으로 측정한다.
로봇 머리의 RealSense D455로 주변을 3D 메시로 재구성해 VR 안에 함께 보여 준다.

- 서버: Python(asyncio) 단일 프로세스 + TSDF 워커 프로세스. Docker 컨테이너에서 실행한다
- 클라이언트: 별도 설치 없이 브라우저로 접속한다. Quest는 조작용, PC 브라우저는 모니터링·dev 조작용이다
- 통신: HTTPS로 페이지를 받고, WebRTC DataChannel로 포즈·로봇 상태·메시를 주고받는다

---

## 목차

1. [시스템 구성](#1-시스템-구성)
2. [폴더 구조](#2-폴더-구조)
3. [호스트 준비 (최초 1회)](#3-호스트-준비-최초-1회)
4. [설치·기동](#4-설치기동)
5. [운영 명령](#5-운영-명령)
6. [접속과 조작](#6-접속과-조작)
7. [핵심 알고리즘](#7-핵심-알고리즘)
8. [설정 (config.py)](#8-설정-configpy)
9. [도구](#9-도구)
10. [문제 해결](#10-문제-해결)
11. [외부 자산·라이선스](#11-외부-자산라이선스)

---

## 1. 시스템 구성

### 장비

| 장비 | 역할 | 연결 |
|---|---|---|
| RBY1 (로봇 본체) | 토르소 6 + 양팔 7×2 + 머리 2 관절 | gRPC (`rby1.local:50051`, mDNS) |
| Inspire RH56E2 손 ×2 | 다지 손 (손가락 4 + 엄지 2 자유도) | Modbus TCP (USB-LAN, `192.168.11.21x:6000`), 외부 24V 전원 |
| Intel RealSense D455 | 로봇 머리 카메라 (depth + color) | USB |
| Manus 장갑 ×2 + 동글 | 조작자 손가락 측정 | 동글 USB (SDK Integrated 모드, Integrated 라이선스 필요) |
| Meta Quest 3 + 컨트롤러 ×2 | 조작자 시점·손 위치 (컨트롤러는 손등에 부착) | 유선 직결 (`10.42.0.1`) |

### 실행 모드

| 모드 | 로봇 | 손 | 카메라 | 용도 |
|---|---|---|---|---|
| `dev` | 시뮬레이션 (1차 지연 추종) | 시뮬레이션 | 합성 장면 | 하드웨어 없이 개발·시험 |
| `deploy` | 실로봇 (gRPC) | Inspire (Modbus TCP) | D455 | 운용 |

기동할 때의 모드는 `docker/.env`의 `TELEOP_MODE`로 정한다(기본값 `dev`).
기동한 뒤에는 PC 브라우저의 모드 버튼으로 dev와 deploy를 바꿀 수 있다. 단, 제어 중에는 전환을 거부한다.
deploy로 처음 기동할 때 로봇 연결에 실패하면 서버는 dev로 넘어가지 않고 종료한다.

### 데이터 흐름

```
[Quest 브라우저]                                         [서버 (teleop.main)]
 컨트롤러 포즈·body·head ── state 채널 (JSON, ≤60Hz) ──▶ RtcHub (최신 샘플 1개 + 명령 래치)
                                                            │
 Manus 장갑 ── raw skeleton ──▶ ManusGlove ── HandRetargeter (DexPilot) ── 손가락 6값
                                                            ▼
                                            RobotController (30Hz)
                                             ├ ArmRetargeter: 클러치 델타 → 손 TCP 목표
                                             ├ ArmIkSolver (Pink): 양팔 14관절
                                             ├ 속도·위치 클램프 → 로봇 백엔드 (임피던스 명령)
                                             └ 손 백엔드 (Inspire 각도 명령)
 로봇 모델 표시 ◀── state 채널 (robot_state, 30Hz) ── RobotStateBroadcaster
 재구성 메시 표시 ◀── mesh 채널 (MSH1 바이너리) ── PerceptionService ◀ D455 + TSDF 워커
```

---

## 2. 폴더 구조

```
vr-teleop/
├── teleop/                      # Python 서버 (python -m teleop.main)
│   ├── main.py                  # 엔트리포인트: 공통 스택 조립, ModeManager (dev/deploy 부품 소유·런타임 전환)
│   ├── robot/
│   │   ├── robot_controller.py  # 30Hz 제어 루프: 명령 처리, IK 호출, 안전 클램프, 로봇·손 명령 전송
│   │   ├── robot_model.py       # URDF 로드 (그리퍼 → Inspire 손 교체), IK용 축소 모델(양팔 14 DOF), FK
│   │   ├── robot_state_broadcaster.py  # 링크 포즈·관절·손가락을 robot_state로 방송
│   │   ├── robot_interface.py   # 로봇·손 백엔드 추상 인터페이스
│   │   ├── rby1_robot.py        # 실로봇 백엔드 (rby1-sdk gRPC)
│   │   ├── sim_robot.py         # 시뮬 로봇 백엔드 (dev)
│   │   ├── inspire_hand.py      # Inspire 손 백엔드 (Modbus TCP, 손마다 스레드 1개)
│   │   └── sim_hand.py          # 시뮬 손 백엔드 (dev)
│   ├── ik/
│   │   ├── arm_retargeter.py    # 팔 리타게팅: follow 클러치 + 상대 델타 → 손 TCP 목표
│   │   └── ik_solver.py         # 양팔 IK: Pink 미분 IK (QP, 관절 위치·속도 한계는 제약으로 처리)
│   ├── glove/
│   │   ├── manus_glove.py       # Manus SDK 접속·자동 페어링, raw skeleton → 손 키포인트 21개
│   │   └── hand_retargeter.py   # 손 리타게팅: dex-retargeting DexPilot → Inspire 굽힘 6값
│   ├── camera/
│   │   ├── camera_interface.py  # 카메라 추상 인터페이스 (최신 프레임 1장 보관)
│   │   ├── d455_camera.py       # D455 캡처 (프리셋·후처리 필터, depth를 color에 정렬)
│   │   └── synthetic_camera.py  # 합성 카메라 (dev)
│   ├── perception/
│   │   ├── perception_service.py # 10Hz 인식 루프: 포즈 부착 → 자기 몸 마스크 → TSDF 워커 → 메시 방송
│   │   ├── posed_frame.py       # 촬영 시각 관절 보간 → 카메라 포즈 (FK)
│   │   ├── self_mask.py         # 로봇 팔·손 메시를 투영해 depth에서 로봇 자신을 지움
│   │   ├── projection.py        # depth 역투영·작업 영역 상자 판정
│   │   ├── tsdf_worker.py       # 별도 프로세스: Open3D TSDF 적분·메시 추출·블록 변경 비교
│   │   └── packets.py           # MSH1 메시 패킷 인코딩
│   ├── net/
│   │   ├── rtc_hub.py           # WebRTC 시그널링·DataChannel·포즈 소유 피어·명령 래치
│   │   └── web_server.py        # HTTPS 정적 서빙 (8443, 443 별칭, 80 리다이렉트)
│   └── utils/
│       ├── config.py            # 모든 설정값 (8절)
│       ├── inputs.py            # 공통 업링크 샘플 형식과 접근자
│       ├── transforms.py        # WebXR ↔ 로봇 좌표 변환
│       └── log.py               # 콘솔 + 회전 파일 로그
├── web/                         # 브라우저 클라이언트 (정적 파일, 빌드 없음)
│   ├── index.html
│   ├── src/
│   │   ├── main.js              # 앱 기동, VR/데스크톱 분기, 메시지 라우팅, XR 수명주기
│   │   ├── manual_control.js    # dev 데스크톱 수동 조작 패널·키보드
│   │   ├── input/               # 컨트롤러·body tracking 입력 → 공통 샘플
│   │   ├── net/                 # RTC 링크, MSH1 디코더
│   │   ├── render/              # 씬, 로봇·아바타·메시·HUD 렌더링
│   │   └── xr/                  # WebXR 세션
│   └── vendor/                  # Babylon.js 번들
├── assets/
│   ├── rby1a/                   # RBY1 URDF·GLB (rby1-sdk 0.9.1의 models/rby1a)
│   ├── inspire_hand/            # Inspire 손 URDF 조각·GLB (시각화·IK 모델용, tools/build_inspire_hand.py 산출)
│   │   └── retarget/            # 손 리타게팅 계산용 Inspire URDF (dex-urdf, 손끝 링크·mimic 관절 포함)
│   ├── hand_meshes/             # Babylon WebXR 손 메시 (현재 코드에서는 참조하지 않음)
│   ├── avatar.glb               # 데스크톱 뷰 사람 아바타
│   └── d455.glb                 # 가상 로봇의 카메라 표시 메시
├── tools/                       # 보조 도구 (9절)
├── docker/
│   ├── Dockerfile               # 서버 이미지
│   ├── compose.yml              # 서비스 정의 (host 네트워크, 자동 재시작)
│   └── .env                     # [gitignore] 호스트별 기동 모드
├── certs/                       # [gitignore] 자체 서명 HTTPS 인증서
├── logs/                        # [gitignore] 서버 로그 (web_api.log)
├── reference/                   # [gitignore] 외부 SDK·참고 코드 (Manus SDK 필수)
└── _tmp/                        # [gitignore] 작업용 임시 폴더
```

---

## 3. 호스트 준비 (최초 1회)

서버 PC 기준이다. `deploy` 모드를 쓰지 않는 개발 PC라면 3.1~3.2만 하면 된다.

### 3.1 Docker

Docker Engine과 Compose 플러그인을 설치한다. 다른 의존성(Python, Pinocchio, Open3D, SDK 등)은 모두 이미지 안에 들어간다.

### 3.2 Manus SDK

MANUS 배포 SDK를 `reference/manusSDK/`에 풀어 둔다. `reference/`는 저장소에 포함되지 않는다.
이미지를 빌드할 때 `reference/manusSDK/Python/manus_sdk`가 이미지에 복사된다(`docker/compose.yml`의 `additional_contexts`).

```
reference/manusSDK/Python/manus_sdk/   ← 이 폴더가 있어야 빌드된다
```

### 3.3 네트워크 (deploy)

| 연결 | 설정 |
|---|---|
| 로봇 gRPC | 로봇 내장 PC의 mDNS 이름 `rby1.local`로 접속한다(`config.ROBOT_ADDRESS`). 유선 경로로 해석되는지 확인한다 (10절) |
| Quest 직결 | 서버 PC 이더넷 포트를 NetworkManager **공유(Shared to other computers)** 모드로 설정한다. PC가 `10.42.0.1`이 되고 Quest는 DHCP로 주소를 받는다 |
| Inspire 손 | 손이 연결된 LAN 포트에 호스트 IP `192.168.11.222/24`를 붙인다. 손 주소는 오른손 `192.168.11.211:6000`, 왼손 `192.168.11.210:6000` |

Inspire 손 포트에 IP를 자동으로 붙이는 NetworkManager dispatcher 예시이다. 인터페이스 이름은 호스트마다 다르니 `ip -br link`로 확인해서 넣는다.

```bash
# /etc/NetworkManager/dispatcher.d/90-inspire-hand-ip  (root 소유, chmod 755)
HAND_IFACE="enx..."              # 손이 연결된 인터페이스 이름
HAND_HOST_ADDR="192.168.11.222/24"

if [ "$1" = "$HAND_IFACE" ] && [ "$2" = "up" ]; then
    ip addr replace "$HAND_HOST_ADDR" dev "$HAND_IFACE"
    logger -t inspire-hand-ip "$HAND_IFACE 에 $HAND_HOST_ADDR 추가"
fi
```

### 3.4 USB 장치 (deploy)

D455와 Manus 동글을 서버 PC에 연결한다. 컨테이너는 `/dev`를 마운트하고 USB(189)·V4L(81) 문자 장치만 허용한다(`compose.yml`의 `device_cgroup_rules`). 따라서 호스트에 별도 udev 설정은 필요 없다.

---

## 4. 설치·기동

```bash
cd docker

# 1) 이미지 빌드
docker compose build

# 2) HTTPS 인증서 생성 (컨테이너를 띄우기 전에 해야 한다)
#    SAN 기본값: localhost, 127.0.0.1, 10.42.0.1. 다른 IP로도 접속하면 --ip로 추가한다
docker compose run --rm teleop python /workspace/tools/make_https_cert.py
#    예) docker compose run --rm teleop python /workspace/tools/make_https_cert.py --ip 10.42.0.1 192.168.0.6

# 3) 기동 모드 지정 (실물 하드웨어가 붙은 호스트만 작성, 없으면 dev)
echo "TELEOP_MODE=deploy" > .env

# 4) 기동
docker compose up -d
```

- 인증서가 없으면 서버가 기동 중에 종료되고, 재시작 정책 때문에 재시작이 반복된다. 인증서·개인키는 `certs/`에 생기며, 정적 서빙 루트(`web/`) 밖이라 브라우저에 노출되지 않는다.
- 컨테이너에는 `restart: unless-stopped`가 걸려 있다. PC를 재부팅하거나 서버가 죽어도 자동으로 다시 뜬다. 단, `docker compose stop`으로 직접 멈췄다면 다시 뜨지 않는다.
- 저장소 전체가 컨테이너의 `/workspace`에 마운트된다. 그래서 코드를 고친 뒤에는 이미지를 다시 빌드할 필요 없이 재시작만 하면 된다.

---

## 5. 운영 명령

| 목적 | 명령 |
|---|---|
| 서버 로그 (회전 파일) | `tail -f logs/web_api.log` |
| 콘솔 출력 (TSDF 워커·크래시 포함) | `docker logs -f airlab_hw_teleop` |
| 코드·`config.py` 변경 반영 | `docker restart airlab_hw_teleop` |
| `Dockerfile`·`compose.yml` 변경 반영 | `cd docker && docker compose up -d --build` |
| 종료 (재부팅해도 다시 켜지 않음) | `cd docker && docker compose stop` |
| 컨테이너 셸 | `docker exec -it airlab_hw_teleop bash` |

멈추거나 재시작할 때(`stop`·`restart`)는 종료 정리를 거친다. 제어를 해제하고 로봇·손·카메라를 순서대로 해제하며, 최대 30초(`stop_grace_period`)까지 기다린다.

주요 로그의 의미 (5초 주기):

| 로거 | 내용 |
|---|---|
| `robot_controller` | 제어 상태, 명령 전송률, 속도 클램프·추적 오차 재동기화 횟수, IK 계산 시간 |
| `manus_glove` | raw skeleton·리타게팅 주기, 손별 손 크기(cm)·굽힘 6값, 배터리 |
| `inspire_hand` | 손별 루프 주기, 실측 각도·힘·온도·에러 |
| `perception` | 인식 주기·처리 시간, 층별 메시 블록·삼각형 수, 적분·추출 시간 |
| `d455_camera` | 캡처 fps, 불완전 프레임 드롭 |
| `rtc_hub` | 업링크 수신률, 형식 오류 샘플 |

---

## 6. 접속과 조작

### 6.1 접속

| 클라이언트 | 주소 | 비고 |
|---|---|---|
| Quest (조작) | `https://10.42.0.1` (유선 직결) | 최초 1회 인증서 예외를 허용한 뒤 `Enter VR` |
| PC 브라우저 (모니터링) | `https://localhost:8443` 또는 `https://<서버 IP>` | 좌상단 모드 버튼, 우상단 dev 수동 조작 패널 |

8443이 본 포트이고, 443은 별칭, 80은 HTTPS로 리다이렉트된다.

### 6.2 VR 조작 (Quest)

장비: Quest 3 헤드셋, 손등에 부착한 Quest 3 컨트롤러(양손), Manus 장갑(양손)

- **제어 시작**: `Enter VR` 2초 뒤 자동으로 시작한다. 로봇이 ready 자세로 이동한 다음 추종을 시작한다
- **팔**: 컨트롤러가 추적되는 동안 컨트롤러의 위치·자세 변화량(델타)을 1m = 1m로 따라간다
- **손가락**: Manus 장갑의 손 모양을 리타게팅한 값을 따라간다
- **머리**: 조작자 시선의 yaw·pitch를 따라간다. VR 시점은 로봇 머리 카메라 위치에 고정된다
- **토르소**: ready 자세로 고정한다
- **제어 종료**: Quest 시스템 메뉴로 VR을 끝내면 로봇이 ready 자세로 돌아간 뒤 제어를 해제한다
  - 브라우저가 꺼지는 등 종료 신호가 오지 않으면, 업링크가 3초 끊긴 시점에 그 자리에서 해제한다
  - 제어를 해제하면 명령 스트림만 닫는다. 서보는 켜진 채로 현재 자세를 유지한다
- Quest의 손 추적이 켜져 있어도 이 페이지는 컨트롤러 입력만 쓴다

### 6.3 dev 모드 수동 조작 (PC 브라우저)

우상단 패널의 체크박스나 `M` 키로 켠다. dev 모드에서 전환 중이 아닐 때만 보인다.

| 키 | 동작 (`Shift`를 누르고 있으면 왼손, 아니면 오른손) |
|---|---|
| `G` | grip 토글 (켜진 동안 추종 = 클러치) |
| `W` / `S` | 손 앞 / 뒤 |
| `A` / `D` | 손 좌 / 우 |
| `R` / `F` | 손 위 / 아래 |
| `T` (누르고 있기) | 손가락 굽힘 |
| `Enter` | A 버튼 = start (제어 시작) |
| `Backspace` | B 버튼 = stop (제어 해제) |

VR 없이 전체 시나리오(시작 → ready → 원운동 추종 → 손가락 굽힘 → 해제)를 돌려 보려면 다음을 실행한다.

```bash
docker exec airlab_hw_teleop bash -c 'cd /workspace && python tools/fake_vr_client.py --cycles 1'
```

---

## 7. 핵심 알고리즘

### 7.1 팔: 클러치 델타 + Pink 미분 IK

1. **리타게팅** (`ik/arm_retargeter.py`)
   - `follow`가 켜지는 순간 조작자 손 포즈와 로봇 손 TCP 포즈를 앵커로 잡는다.
   - 이후에는 손의 이동·회전량만 TCP 앵커에 더한다. 그래서 사람과 로봇의 체격 차이와 무관하고, 추종을 시작할 때 로봇이 튀지 않는다.
   - 이동 방향은 앵커 시점의 사람 몸통 방향(어깨선) 기준으로 해석한다.
2. **IK** (`ik/ik_solver.py`, Pink)
   - 매 주기 QP를 한 번 풀어 관절 속도를 구하고 적분한다.
   - 비용: 손 TCP 위치(가장 큼) > 손 자세 > ready 자세 유지(남는 자유도인 팔꿈치만 정함)
   - 관절 위치 한계(URDF ∩ `SAFETY_JOINT_LIMITS`)와 속도 한계는 **제약**으로 걸어서 해가 넘지 않는다
   - 손 TCP = ee 프레임 + `HAND_TCP_OFFSET` (파지 중심)
3. **안전 클램프** (`robot/robot_controller.py`)
   - 명령을 보내기 직전에 관절별 변화량을 속도 상한으로 다시 자르고 URDF 위치 한계로 자른다.
   - 명령과 측정값의 차이가 0.5 rad를 넘으면 측정 자세로 재동기화한다.

### 7.2 손: dex-retargeting (DexPilot)

1. Manus raw skeleton 노드에서 MediaPipe 순서의 키포인트 21개를 만든다(손목 + 손가락마다 관절 3개 + 끝).
2. 손목 프레임을 추정해 MANO 규약(x 손바닥 법선, y 검지 쪽, z 손가락 방향)으로 옮긴다.
3. DexPilot이 손목→손끝 벡터와 손끝↔손끝 벡터를 Inspire 손 FK와 맞추는 관절각을 푼다. 손끝 사이 관계가 보존되므로 집기에 유리하다.
4. 구동 관절 6개를 관절 한계 기준으로 0~1로 정규화한다: `[little, ring, middle, index, thumb_bend, thumb_rotate]`
5. Inspire 레지스터 값으로 변환한다: `1000 × (1 − 굽힘)`, 50Hz.

### 7.3 환경 메시 재구성

- **카메라 포즈**: 영상으로 추정하지 않고, 로봇 관절 측정값으로 FK를 계산한다. 촬영 시각에 맞춰 관절값을 보간한다.
- **입력 처리**:
  - D455 `high_accuracy` 프리셋과 disparity 공간·시간 필터를 적용한다.
  - 머리 관절 속도가 0.3 rad/s를 넘는 프레임은 버린다.
  - 로봇 팔·손 메시를 투영해서 depth에서 로봇 자신을 지운다.
- **융합**: 별도 프로세스에서 Open3D VoxelBlockGrid TSDF로 적분한다. 층은 두 개다.
  - 환경 층: 30mm 복셀, 깊이 4m 이내
  - 작업 영역 층(로봇 앞 상자): 6mm 복셀
- **동적 물체**: 시야 안의 기존 블록도 매 프레임 빈 공간으로 갱신하고, 가중치 상한을 50(약 5초)으로 둔다. 그래서 치운 물체는 몇 초 안에 지워진다.
- **전송**: 1초마다 메시를 추출하고, 바뀐 블록만 MSH1 패킷(int16 mm 좌표, rgb, uint32 인덱스)으로 보낸다.

---

## 8. 설정 (config.py)

모든 설정은 `teleop/utils/config.py` 한 곳에 있다. 바꾼 뒤에는 `docker restart airlab_hw_teleop`로 반영한다.

| 그룹 | 주요 항목 |
|---|---|
| 모드·로그 | `TELEOP_MODE`(환경 변수), `LOG_FILE` |
| 네트워크 | `ROBOT_ADDRESS`, `HAND_ADDRESSES`, `DIRECT_LINK_SERVER_IP`, `HTTPS_PORT` |
| 팔 IK | `IK_POSITION_COST`, `IK_ORIENTATION_COST`, `IK_POSTURE_COST`, `IK_LM_DAMPING`, `IK_VEL_FRACTION`, `IK_MAX_JOINT_VEL`, `SAFETY_JOINT_LIMITS`, `HAND_TCP_OFFSET`, `ARM_SCALE` |
| 로봇 제어 | `CONTROL_RATE_HZ`, `IMPEDANCE_GAINS`, `CONTROL_INPUT_TIMEOUT_S`, `CONTROL_MAX_TRACKING_ERR`, `CONTROL_BODY_VEL_FRACTION`, `CONTROL_HEAD_VEL_FRACTION` |
| Inspire 손 | `HAND_RATE_HZ`, `HAND_SPEED`, `HAND_FORCE_LIMIT` |
| Manus·손 리타게팅 | `MANUS_AUTO_PAIR`, `MANUS_ALLOWED_GLOVE_IDS`(비우면 모든 장갑 허용), `HAND_RETARGET_SCALING`, `HAND_RETARGET_LOW_PASS_ALPHA`, `HAND_RETARGET_RATE_HZ` |
| 카메라 | `CAMERA_WIDTH/HEIGHT/FPS`, `CAMERA_DEPTH_PRESET`, `CAMERA_SPATIAL_*`, `CAMERA_TEMPORAL_*`, `CAMERA_MOUNT_*`(장착 보정) |
| 인식·TSDF | `PERCEPTION_RATE_HZ`, `WORKSPACE_BOX`, `SELF_MASK_GROUPS`, `TSDF_VOXEL`, `TSDF_WS_VOXEL`, `TSDF_DEPTH_MAX`, `TSDF_MAX_WEIGHT`, `TSDF_EXTRACT_S`, `TSDF_WORKER_CPUS` |

---

## 9. 도구

| 도구 | 실행 위치 | 설명 |
|---|---|---|
| `tools/make_https_cert.py [--ip IP ...] [--days 825]` | 컨테이너 | 자체 서명 인증서를 `certs/`에 생성한다 (SAN에 접속 IP를 넣어야 한다) |
| `tools/fake_vr_client.py [--host] [--port] [--rate 50] [--cycles 1]` | 컨테이너 | VR 없이 합성 포즈로 21초 조작 시나리오를 재현한다 (dev 시험용) |
| `tools/build_inspire_hand.py <RH56E2-R STEP>` | 호스트 (`pip install cadquery-ocp open3d`) | Inspire STEP에서 링크별 GLB와 손 URDF 조각을 만든다 (1회성, 결과는 `assets/inspire_hand/`) |

---

## 10. 문제 해결

| 증상 | 확인 |
|---|---|
| 컨테이너가 계속 재시작된다 | `docker logs airlab_hw_teleop`을 본다. `certs/`가 없거나, deploy 기동 시 로봇 연결에 실패한 경우다 |
| 로봇이 무선 경로로 연결된다 | `rby1.local`은 로봇 PC의 모든 인터페이스 주소로 해석된다. 로그의 `로봇 연결:` 줄을 확인하고, 필요하면 mDNS가 유선 인터페이스에서만 응답하도록 설정한다 |
| Quest에서 접속이 안 된다 | 이더넷 공유 모드(`10.42.0.1`)와 인증서 SAN에 접속 IP가 들어 있는지 확인한다 |
| 손가락이 움직이지 않는다 | `grep manus_glove logs/web_api.log \| tail`로 동글·장갑 목록과 페어링 상태를 본다. 손 백엔드는 제어 중일 때만 명령을 보낸다 |
| 손가락 값이 이상하다 | `manus_glove` 로그의 `손 크기`가 17~20cm 정도인지 본다(값이 너무 작으면 노드 좌표 문제). 크기 차이는 `HAND_RETARGET_SCALING`으로 조정한다 |
| Inspire 손이 연결되지 않는다 | 24V 전원과 호스트 IP(`192.168.11.222`)를 확인한다. 손이 연결되지 않아도 팔 제어는 동작하고, 손 스레드는 재접속을 계속 시도한다 |
| 메시가 비거나 번진다 | `perception`·`d455_camera` 로그를 본다. 필요하면 `CAMERA_DEPTH_PRESET`(`"default"`와 비교), `TSDF_MAX_WEIGHT`, `SELF_MASK_GROUPS`를 조정한다. 로봇 손이 계속 가리는 영역은 관측되지 않아 구멍으로 남는다 |
| `pinocchio` import 시 `CXXABI` 오류 | 시스템 libstdc++가 먼저 로드된 경우다. 이미지의 `LD_LIBRARY_PATH=/opt/venv/lib` 설정을 확인한다 |

---

## 11. 외부 자산·라이선스

| 자산 | 출처 |
|---|---|
| `assets/rby1a/` | rby1-sdk v0.9.1의 `models/rby1a` (URDF·GLB). SDK 버전을 올리면 같은 태그로 갱신한다 |
| `assets/inspire_hand/` (URDF 조각·GLB) | Inspire 공식 RH56E2 STEP을 `tools/build_inspire_hand.py`로 변환한 것 |
| `assets/inspire_hand/retarget/` | [dex-urdf](https://github.com/dexsuite/dex-urdf)의 Inspire 손 URDF (`LICENSE.txt` 포함) |
| `assets/d455.glb` | Intel realsense-ros의 `d455.stl`을 변환한 것 |
| 손 리타게팅 | [dex-retargeting](https://github.com/dexsuite/dex-retargeting) (DexPilot) |
| 팔 IK | [Pink](https://github.com/stephane-caron/pink) (Pinocchio 기반 미분 IK) |
