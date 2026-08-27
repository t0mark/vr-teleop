# RBY1 웹 기반 VR Teleop 전환 플랜

## 목표

Unity 기반 rby1-vr-teleop을 웹 스택으로 전환한다.
Quest 브라우저 온디바이스 렌더링(PCVR 미사용), Babylon.js + WebXR 기반.
기존 eef(손목) 포즈 단독 추적을 손목 + 팔꿈치·어깨 포즈 추적으로 개선한다.

## 아키텍처

```
[브라우저 (단일 페이지, Babylon.js)]
  - PC 접속: 데스크톱 뷰 (모니터링) / Quest 접속: Enter VR 버튼 → immersive-vr 진입
  - VR 모드: body-tracking(팔꿈치·어깨) + 손목 포즈 + 컨트롤러 입력 업링크
  - 공통: 카메라 영상 수신(WebRTC), 포인트클라우드 수신·렌더링(WebSocket 바이너리)
        │ Quest: 유선 이더넷 (USB-C 이더넷 어댑터 ↔ PC 직결)
        │ PC: 일반 네트워크 접속
        │ WSS (포즈/입력 업링크) + WebRTC + WSS (다운링크)
        ▼
[서버 (PC, Docker 컨테이너)]
  - web_api: HTTPS 정적 파일 + WSS (WebXR secure context 요건, 자체 서명 인증서)
  - ik: xr_teleoperate 방식 (Pinocchio + CasADi), RBY1 URDF 사용
  - robot_sdk: rby1-sdk (기존 python/main.py 제어 로직 재사용)
  - stream: RealSense D455 RGB → GStreamer/aiortc → WebRTC
  - depth_to_3d: D455 depth → 다운샘플 → WSS 바이너리 브로드캐스트
  - utils: 공용 카메라 캡처, 세션 기록, 설정, 로그, 좌표 변환
        │ gRPC (rby1-sdk)
        ▼
[RBY1 로봇]
```

- VR - PC 연결: 유선 이더넷 직결 (Quest USB-C 이더넷 어댑터 ↔ PC), 양쪽 고정 IP로 같은 서브넷 구성
- localhost 접속이 아니므로 WebXR secure context 요건상 HTTPS 필수 → 자체 서명 인증서 + Quest 브라우저에서 인증서 예외 허용
- 모니터링: 같은 페이지에 PC 브라우저로 접속하면 비VR 데스크톱 렌더링으로 영상·포인트클라우드·상태 확인

## 폴더 구조

```
teleop/                      # 저장소 루트
├── web/                     # 프론트엔드 (단일 페이지: 데스크톱 뷰 + Enter VR)
│   ├── index.html
│   ├── certs/               # 자체 서명 인증서 (gitignore)
│   └── src/
│       ├── xr/              # WebXR 세션, body-tracking, 입력 수집
│       ├── net/             # WebSocket 업링크, WebRTC 수신, 포인트클라우드 수신
│       ├── render/          # Babylon.js 씬, 영상 텍스처, 포인트클라우드, HUD
│       └── main.js
├── teleop/                  # 백엔드 파이썬 패키지 (파일명만 보고 역할이 보이게 명명)
│   ├── main.py              # 엔트리포인트: 공통 스택 조립 + ModeManager(dev/deploy 부품 조립·웹 런타임 전환)
│   ├── ik/                  # 포즈 → 관절각 (순수 계산기만 — 루프 없음, robot_controller가 호출)
│   │   ├── ik_solver.py     #   IK 최적화 (Pinocchio + CasADi)
│   │   └── pose_to_target.py #  사람 포즈 → IK 목표 (grip 클러치 + 손 델타 매핑)
│   ├── pointcloud/          # depth → 포인트클라우드
│   │   ├── depth_to_pcd.py  #   depth 프레임 → 3D 점군 변환 + 전송용 바이너리 패킹
│   │   └── pcd_streamer.py  #   10Hz 루프: 새 프레임을 변환해 전 클라이언트 방송
│   ├── net/                 # 통신
│   │   ├── web_server.py    #   HTTPS 정적 서빙 + WSS 라우트 (3개 포트) + 자산 gzip 사전 압축
│   │   └── websocket.py     #   웹소켓 접속 관리: 포즈 수신·전 클라이언트 방송·명령 라우팅
│   │                        #   (6단계에서 video_stream 추가 예정)
│   ├── robot/               # 로봇
│   │   ├── robot_model.py   #   URDF 로드·FK (시각화·IK·카메라 외부 파라미터 공용)
│   │   ├── robot_interface.py # 로봇·그리퍼 추상 인터페이스 (계약)
│   │   ├── robot_controller.py # 30Hz 제어 루프: 입력 해석 → IK 호출 → 안전 클램프 → 명령 전송 (A/B 상태 머신)
│   │   ├── robot_state_broadcaster.py # 로봇 상태 읽어 FK → 브라우저 방송 (30Hz)
│   │   ├── rby1_robot.py    #   [배포] rby1-sdk gRPC 백엔드
│   │   ├── rby1_gripper.py  #   [배포] Dynamixel 그리퍼
│   │   ├── sim_robot.py     #   [dev] 시뮬 로봇 (1차 지연 추종)
│   │   └── sim_gripper.py   #   [dev] 시뮬 그리퍼
│   ├── camera/              # RGB-D 소스
│   │   ├── camera_interface.py # 프레임 소스 추상 인터페이스 (계약)
│   │   ├── d455_camera.py   #   [배포] 실물 D455
│   │   └── synthetic_camera.py # [dev] 합성 장면 생성
│   └── utils/               # 공용 유틸 (config: 전역 설정·TELEOP_MODE, transforms, inputs, log;
│                            #             record는 8단계에서 추가)
├── assets/                  # 정적 자산: rby1a/(URDF+GLB 메시, rby1-sdk v0.9.1에서 추림, 14MB),
│                            # avatar.glb(아바타), d455.glb(카메라) — /assets/ URL로 서빙
├── tools/                   # 실행 도구 (fake_vr_quest3.py: Quest 3 대체 합성 포즈 송신,
│                            #            make_https_cert.py: 자체 서명 인증서 생성)
├── docker/
│   ├── compose.yml
│   └── Dockerfile
├── logs/                    # 기록 출력 (gitignore)
└── oss/                     # rby1-sdk, rby1-examples-vr-teleop (참조용, gitignore)
```

## 구성 요소별 계획

### 1. 프론트엔드 (`web/`)

- 단일 페이지: PC 브라우저는 데스크톱 뷰(마우스 궤도 카메라, 모니터링용), VR 기기는 Enter VR 버튼으로 immersive-vr 진입 (Babylon.js `WebXRDefaultExperience` 기본 흐름)
- Quest 설정: `chrome://flags/`에서 WebXR Experiments 활성화 필요 (body-tracking)
- 접속: 이더넷 직결 후 Quest 브라우저에서 `https://{PC IP}:{포트}` 접속, 최초 1회 인증서 예외 허용
- 입력 수집 (VR 모드)
    - `XRBodyTracking`: 어깨·팔꿈치 조인트
    - 컨트롤러/hand-tracking: 손목 포즈, 트리거·그립(그리퍼), 스틱(모바일 베이스)
- 좌표계: WebXR(오른손, y-up) → 로봇(z-up) 변환. 기존 `T_conv` 대응 행렬 재정의
- 표시(공통): 로봇 카메라 영상(WebRTC video 텍스처), 포인트클라우드, 상태 HUD
- 업링크: 포즈 패킷을 고정 주기(50-100 Hz)로 WebSocket 전송 (데스크톱 뷰는 수신 전용)

### 2. IK (`teleop/core/ik/`)

- xr_teleoperate(Pinocchio + CasADi) 방식 차용, RBY1 URDF(rby1-sdk `models/`)로 교체
- 입력: 양팔 (어깨, 팔꿈치, 손목) 포즈 / 출력: RBY1 팔 관절각
- 기존 eef 단독 추적과 달리, 손목 포즈 추종 + 팔꿈치·어깨 포즈 코스트를 함께 최적화해 사람 팔 자세를 재현
- 관절 한계·속도 제한, 특이점 근처 damping

### 3. 로봇 제어 (`teleop/robot_sdk/`)

- `base.py`: 로봇 공통 추상 인터페이스 (연결·초기화·관절 명령·그리퍼), 추후 다른 로봇 추가 대비
- `rby1/`: 기존 `oss/rby1-examples-vr-teleop/python/main.py`에서 재사용
    - 연결·전원·서보·Control Manager 초기화
    - ready pose, 토르소·모바일 제어, 그리퍼(`gripper.py`)
- 변경: UDP/ZMQ 수신부 제거 → IK 출력(관절각)을 직접 스트리밍 명령으로 전달

### 4. 영상 스트리밍 (`teleop/net/stream/`)

- D455 RGB → H.264 하드웨어 인코딩 → WebRTC (aiortc 또는 GStreamer webrtcbin)
- 시그널링은 web_api WSS에 통합, 다중 피어(VR + 모니터링) 지원
- 목표 지연 < 150 ms

### 5. 포인트클라우드 (`teleop/core/depth_to_3d/`)

- D455 depth → voxel 다운샘플 → float32/uint16 바이너리 패킹 → WSS 전송 (5-15 Hz)
- 접속한 모든 클라이언트에 브로드캐스트

### 6. 웹 API (`teleop/net/web_api/`)

- `web/` HTTPS 정적 서빙, 포즈 업링크 수신, 포인트클라우드 송신, WebRTC 시그널링
- 자체 서명 인증서 생성 스크립트 포함 (`web/certs/`에 출력)

### 7. 공통 유틸 (`teleop/utils/`)

- `record/`: 포즈 입력, IK 출력, 로봇 상태를 타임스탬프와 함께 바이너리 기록 → `logs/`
- `camera/`: D455 디바이스 1회 오픈, RGB는 stream·depth는 depth_to_3d로 프레임 분배
- `config.py`: 포트, IK 게인, 카메라 해상도·프레임레이트 등 설정 일원화
- `log.py`: 실행 중 모니터링 로그 포맷 통일
- `transforms.py`: 쿼터니언·행렬 변환, WebXR(y-up) → 로봇(z-up) 좌표 변환
- 원칙: 2개 이상 모듈이 공유하는 코드만 추가 (잡동사니화 방지)

### 8. Docker (`docker/`)

- compose 기반, 컨테이너 이름 `airlab_hw_{이름}`, 이미지 `t0mark251/{이름}`
- 서버 컨테이너: Python + Pinocchio/CasADi + rby1-sdk + librealsense(최소 구성) + aiortc
- librealsense는 뎁스·RGB 스트림 취득에 필요한 최소 구성만 소스 빌드
    - core + pyrealsense2 바인딩만 설치
    - GUI·툴·예제 제외: `BUILD_EXAMPLES=OFF`, `BUILD_GRAPHICAL_EXAMPLES=OFF`, `BUILD_TOOLS=OFF`
- 호스트 환경 변경 금지, D455는 device 마운트

## 단계별 진행

1. **환경 구축**: Docker 이미지(서버 의존성), rby1-sdk·URDF 로드 확인
2. **프론트 뼈대**: HTTPS 웹 서버 + 이더넷 직결 + Babylon.js WebXR 세션 진입, Quest에서 body-tracking·손목 포즈 취득 확인
3. **포즈 파이프라인**: WSS 업링크 + 좌표 변환, 서버에서 포즈 수신·시각화 검증
4. **IK**: Pinocchio + CasADi IK 구현, 시뮬레이션(관절각 시각화)으로 검증 후 실로봇 연결
5. **제어 통합**: 기존 제어 로직 이식 (ready pose, 그리퍼, 토르소, 모바일 베이스)
6. **영상**: D455 WebRTC 스트림 → Babylon.js 텍스처 표시
7. **포인트클라우드 + 데스크톱 뷰**: WSS 바이너리 스트림·렌더링, PC 브라우저에서 영상·포인트클라우드·상태 확인
8. **기록**: 포즈·IK·로봇 상태 로깅
9. **통합 테스트·튜닝**: 지연 측정, IK 게인·필터 튜닝, 안전 장치(급격한 포즈 점프 차단)
