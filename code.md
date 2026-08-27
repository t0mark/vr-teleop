# 작업 기록

## docker/Dockerfile

- 이미지: `t0mark251/rby1-web-teleop:latest` (1.3GB)
- 단일 빌드: debian:bookworm-slim + micromamba(/opt/bin/micromamba) + conda 환경(/opt/venv)
- **모델은 이미지에 없다 (2026-08-25 저장소로 이관)**: 원래 2단계 빌드로 rby1-sdk 태그에서
  sparse checkout 해 `/opt/rby1/models`에 굽고 `RBY1_MODEL_ROOT` ENV로 노출했으나,
  개발 PC마다 이미지 재빌드가 필요해지는 문제로 저장소 포함으로 결정 변경 (아래 URDF 항목 참고).
  **함정: 내장 PC의 기존 이미지에는 `ENV RBY1_MODEL_ROOT=/opt/rby1/models/rby1a`가 남아 있어**
  config의 env 우선 규칙 때문에 저장소 모델 대신 이미지 사본(v0.9.1, 현재는 내용 동일)을 쓴다.
  저장소 모델을 갱신하면 어긋나므로 내장 PC에서는 이미지 재빌드(권장) 또는 기동 시
  `RBY1_MODEL_ROOT=` 명시로 해제할 것
- 핵심 스택 설치 경로가 이원화되어 있음
    - conda-forge: python 3.11, pinocchio 4.1.0, casadi 3.7.2, numpy, scipy
    - pip: rby1-sdk 0.9.1(고정), aiortc, websockets, aiohttp, pyrealsense2
- 특이사항
    - **Pinocchio를 pip(pin 패키지)으로 설치하면 안 됨**: PyPI wheel은 전 버전(2.x-4.x)에서 casadi 지원 없이 빌드되어
      `pinocchio.casadi`(cpin) import 불가. conda-forge 빌드만 포함. IK가 cpin에 의존하므로 conda 필수
    - **모델 자산은 pip 패키지에 없다**: `rby1_sdk` wheel 내용물은 `_bindings.so` + `__init__.py`뿐이고
      URDF·메시는 GitHub 저장소에만 있다 → 저장소 `models/rby1a`에 추려서 포함 (위 URDF 항목 참고)
    - rby1-sdk PyPI 최신은 **0.9.1**이다 (`>=0.7.0`으로 설치하면 0.9.1이 잡힘)
    - librealsense 소스 빌드 대신 pyrealsense2 PyPI wheel 사용 (D455 취득에 충분). 문제 시 최소 소스 빌드로 회귀
    - `/opt/venv/bin`을 PATH 앞에 두어 컨테이너의 기본 `python` = conda 환경
    - GStreamer(NVENC)는 미포함. 내장 PC에 NVIDIA GPU가 없으므로 영상 단계(6단계)는 소프트웨어 인코딩 전제로 설계한다

## docker/compose.yml

- 사용법: `cd docker && docker compose up -d`, 진입은 `docker exec -it airlab_hw_teleop bash`
- 컨테이너: `airlab_hw_teleop`, 상시 기동, 저장소 루트 → /workspace 마운트, network_mode: host
- D455: **`/dev:/dev` 마운트** + device_cgroup_rules(c 189/81/188)
    - pyrealsense2 PyPI wheel은 **V4L2 백엔드**라 `/dev/bus/usb`만으로는 장치가 0대로 잡힘 (libusb 백엔드 아님).
      `/dev/video*`가 필요하고 번호가 재연결마다 바뀌어 /dev 전체를 노출. 열 수 있는 장치는 cgroup 규칙으로 제한됨
    - **c 188(USB 시리얼)은 그리퍼 버스용**: `/dev/rby1_gripper → ttyUSB1`. 규칙이 없으면 root인데도
      `open()`이 EPERM으로 실패한다 (SDK 로그는 "Error opening serial port"만 찍음)
- 특이사항
    - `name: rby1-web-teleop` 프로젝트 이름 고정 필수 (타 프로젝트 orphan 오인 방지)
    - **GPU 예약(deploy.resources) 없음**: RBY1 내장 PC에는 NVIDIA GPU가 없어, 예약이 남아 있으면 `up`이 실패한다
    - URDF·메시가 이미지에 있으므로 모델용 추가 마운트는 필요 없다
    - 컨테이너(root)가 마운트 볼륨에 만든 파일은 소유권 이관 필요:
      `docker exec airlab_hw_teleop chown -R 1001:1001 /workspace/logs /workspace/web/certs`
      (내장 PC의 airlab uid는 1001. 컨테이너 umask 때문에 server.key가 0666으로 생기므로 호스트에서 600으로 조인다)

## 실행 환경 (RBY1 내장 PC)

- 서버는 로봇 내장 PC(hostname `rby1`)에서 동작한다. i5-1250PE 16스레드 / 15GB RAM / **NVIDIA GPU 없음**
- 로봇 제어: 내장 PC에서 gRPC `192.168.30.1:50051` 도달 확인 (`app_main` 프로세스가 서빙, `br0` 대역)
- 네트워크: `br0` 192.168.30.1(로봇), `wlp0s20f3`(사내망), `enx00e04c2f6858`(Quest 직결), `can0-3`
- **docker는 snap 판**이고 `dockerd --group docker`로 뜨는데 그 그룹이 없어 소켓이 root:root로 떨어져 있었다
    - 해결: `groupadd docker` + `usermod -aG docker airlab` + `chgrp docker /var/run/docker.sock`
    - **데몬을 재시작하면 안 된다**: 내장 PC에서 도는 로봇 웹 스택 컨테이너(`backend`·`frontend`·`jupyterlab`)가
      전부 `restart=no`라 재시작 시 그대로 죽는다. 살아 있는 소켓의 그룹만 바꿔 재시작을 피했다
      (그룹이 존재하므로 다음 부팅에는 `--group docker`가 자동 적용됨)
    - 재로그인 전까지는 셸에 그룹이 반영되지 않으므로 `sg docker -c '<명령>'`으로 우회한다
- 포트: 80/443/8443은 비어 있다 (로봇 웹 스택은 8000·8888·5173 사용). 충돌 없음
- 디스크가 빠듯하다: / 109GB 중 여유 12GB. 빌드 캐시에 회수 가능 10.9GB가 있으므로
  모자라면 `docker builder prune`으로 확보한다 (이미지·컨테이너 영향 없음)

## RBY1 URDF·메시 관련

- 위치: **저장소 `models/rby1a/urdf/model.urdf`**, 메시는 같은 폴더 meshes/ (glb 36개만 포함, 14MB).
  경로는 `config.RBY1_MODEL_ROOT`(기본 저장소 상대경로, 환경 변수로 덮어쓰기 가능)로 결정된다
    - 2026-08-25 이미지 내장 → 저장소 포함으로 결정 변경 (개발 PC마다 재빌드가 필요한 문제).
      출처는 rby1-sdk 태그 v0.9.1의 models/rby1a — **pip 버전을 올리면 같은 태그의 모델로 이 폴더를
      함께 갱신할 것** (URDF와 제어 API의 릴리스 묶음이 수동 관리로 바뀌었다)
    - dae는 넣지 않는다(47MB): pinocchio는 URDF XML만 파싱하고 브라우저는 glb만 그린다
- **표준 urdfdom으로 원본 로드 불가**: wheel 관절 limit에 effort 누락 → `robot_model.load_fixed_urdf_xml()`이 보정 후 `buildModelFromXML` 사용. collision capsule 에러 출력은 비치명(무시)
- **GLB 정점 데이터는 URDF 링크 프레임(z-up) 그대로** (bbox를 URDF collision과 대조해 확인).
  glb 내부 루트 노드는 전부 identity — 축 변환이 파일에 안 구워져 있음
- 팔 체인: arm_0-2(어깨 3축) → arm_3(팔꿈치) → arm_4-6(손목 3축), 손목 링크 = link_*_arm_6
- ready pose(기존 예제 Right-A 자세): 토르소 [0,45,-80,45,0,0]°(크라우치), 오른팔 [0,-15,0,-120,0,30,0]°, 왼팔 [0,15,0,-120,0,30,0]°,
  **머리 [0, 0.4] rad** (custom-teleop 기본 0.7은 바닥을 과하게 봐서 0.4≈23°로 완화)
- 카메라 장착점: URDF에 주석 처리된 `head_mount` 조인트 (link_head_2 + xyz 0.022 0 0.04)를 삼각대 나사
  마운트로 보고, depth 광학 중심 = 나사 + (0.0158, 0.0475, 0.0145) (realsense-ros `_d455.urdf.xacro` 값)

## teleop/ 백엔드

- `main.py`: 조립 — RobotModel + ArmIkSolver + TargetMapper + PoseHub + IkService + **RobotStateService**
  + **RobotControlService + Rby1Gripper**(실로봇 연결 시에만) + WebAPIServer
    - **시각화 공급자를 기동 시 1회 선택**: 실로봇 연결 성공이면 측정 관절값, 실패면 IK 결과(시뮬레이션)
    - 실로봇 연결 시 순서: connect(읽기 전용) → `power_tool_flange()`(그리퍼 12V) → 그리퍼 스레드 시작(호밍)
      → 제어 서비스 시작(대기 상태). **기동만으로 팔은 움직이지 않는다** (그리퍼 호밍만 동작)
    - 기동: `docker exec -d airlab_hw_teleop bash -c 'cd /workspace && python -m teleop.main >> /workspace/logs/web_api.log 2>&1'`
    - 종료: 컨테이너에 pgrep/pkill 없음 → `/proc/*/cmdline`을 훑어 teleop.main 프로세스에 SIGTERM
- `robot_sdk/`: 로봇 SDK 추상화 (plan.md 5단계)
    - `base.py`: `JointSource`(관절값·그리퍼 공급자) + `RobotBackend`(연결·구독 + `prepare_control` /
      `start_stream` / `send_targets` / `stop_stream`) + `GripperBackend`(start/stop/set_openness/openness).
      실로봇과 IK 시뮬레이션이 같은 인터페이스를 구현해 상위 서비스가 어느 쪽인지 몰라도 되게 한다
    - `rby1/robot.py`: `Rby1Backend`
        - `connect()`는 **읽기 전용**: `create_robot_a` → `connect` → `start_state_update(30Hz)`. 전원은 꺼져
          있을 때만 켠다 (꺼져 있으면 인코더가 전부 0). 서보·CM은 건드리지 않는다
        - `prepare_control()`: 전원 → 서보 on → CM 폴트 리셋 → 머리 PID 게인 1회 적용(P200/I0/D8000,
          CM 비활성 상태에서만 반영되므로 켜져 있으면 껐다 적용) → `enable_control_manager(unlimited=True)`.
          이미 된 단계는 건너뛴다. **제어 시작(A) 시에만 호출** — 기동만으로 서보·CM을 바꾸지 않는다
        - **CM_UNLIMITED_MODE는 True 고정 (custom-teleop과 동일)**: 제한 모드(False)는 활성화 시점에 관절
          펌웨어 한계 검사를 해서 head_1이 한계 밖이면(위를 본 자세, -0.66 < -0.35) 활성화가 거부된다.
          unlimited는 "skipping limit check"로 통과 (실기기 확인). 속도 안전은 명령 쪽에서 계속 보장
        - `start_stream()`: 진행 중 제어가 있으면 `cancel_control` → `wait_for_control_ready` → `create_command_stream`
        - `send_targets(named, (v, w), dt)`: `BodyComponentBasedCommand`(torso/right/left 각각 `JointPositionCommand`)
          + head `JointPositionCommand` + `SE2VelocityCommand`. 관절 순서는 이름에서 조립(torso_0.., right_arm_0.., left_arm_0..,
          head_0..)하므로 사전 순서 무관. minimum_time = dt×1.01, hold_time = dt×10 (custom-teleop 규약).
          `set_velocity_limits()`로 받은 관절별 상한을 SDK 쪽 안전 상한으로 싣는다. 예외·누락 관절이면 스트림을 닫고 False
        - `power_tool_flange(12)`: 양쪽 툴 플랜지 12V — **그리퍼 Dynamixel 전원**. 이걸 안 켜면 버스 ping이 전부 False
        - `disconnect()`는 스트림만 닫고 서보·CM은 끄지 않는다 (자세를 잡은 채 남는 편이 안전)
    - `rby1/gripper.py`: `Rby1Gripper` — custom-teleop gripper.py 이식. 전용 스레드에서 초기화 → 호밍 → 10Hz 구동
        - 모터 ID 0=오른쪽, 1=왼쪽. 열림 규약 0=닫힘 1=열림, **왼쪽은 방향 반전**(`GRIPPER_INVERT`, 원본의
          right=trigger / left=1-trigger 비대칭을 옮긴 것). **실기기에서 트리거 → 닫힘 방향 미확인**, 반대면 INVERT를 뒤집는다
        - 호밍: 0.5 전류로 양끝까지 밀어 엔코더 범위 측정 (실측 약 4.5 rad, 약 9초). **원본의 "엔코더 완전 일치 30회"
          조건은 미세 떨림(약 0.002 rad) 때문에 영원히 끝나지 않았다** → 변화량 < 0.01 rad를 정지로 판정 + 방향당 30초 타임아웃
        - 호밍은 **서버 기동마다 실제로 그리퍼를 열었다 닫는다** (물건을 쥔 채 재기동 금지)
        - 엔코더를 10Hz로 읽어 측정 열림 정도를 돌려주므로 시각화·HUD가 실제 그리퍼를 본다 (`Rby1Backend.gripper_openness` 위임)
    - `control.py`: `RobotControlService` — 30Hz asyncio 루프, 상태 2개 (대기/제어)
        - **대기**: 로봇을 안 움직이고 `IkService.sync_to(측정값)`를 매 주기 호출 → 시뮬과 실로봇이 같은 자세.
          제어 시작 순간 명령이 현재 자세에서 연속 출발 = 점프 구조적 불가
        - **오른손 A** → `prepare_control` → `start_stream` → 첫 명령 = 측정값 → IkService ready 복귀 시작 → 이후 클러치 추종.
          토르소·머리는 IK 밖이라 여기서 ready로 속도 제한 보간 (`CONTROL_BODY_VEL_FRACTION` 0.15 ≈ 토르소 0.3 rad/s)
        - **해제 조건**: 오른손 B / 업링크 두절 3초 / 명령-측정 추적 오차 > 0.5 rad (끼임·충돌) / 전송 실패.
          해제 = 스트림 취소, 서보는 켜진 채 현재 자세 유지
        - 마지막 관문 안전 클램프: 이전 명령 대비 관절별 변화량 ≤ 속도 상한 × dt (IK 솔버 제한과 이중)
        - 모바일 베이스: 오른손 스틱 → (전진 v, yaw w), 데드존 0.15, 1차 필터 τ=0.3s. **실기기에서 부호 미확인**
          (`MOBILE_LINEAR_SIGN`/`MOBILE_ANGULAR_SIGN`으로 뒤집는다)
        - 그리퍼 목표는 제어 중에만 보낸다 (트리거 → `IkService.gripper_openness`)
        - 검증: 가짜 백엔드로 동기화·연속 출발·클램프·ready 도달·주행·B/추적 오차/입력 두절 해제 전부 통과.
          실 SDK로는 연결·명령 빌더 조립·CM 활성화(unlimited)까지 확인. **실기기 A 버튼 추종은 미확인** —
          로봇이 명령보다 느리면 추적 오차로 해제될 수 있다 → `CONTROL_MAX_TRACKING_ERR` 조정
    - 상태 콜백은 SDK 스레드에서 오므로 최신 값 한 벌만 락으로 보호해 보관하고 소비자는 스냅샷만 가져간다
    - **SDK 관절 이름이 URDF와 정확히 일치**한다 (torso_0-5, right_arm_0-6, left_arm_0-6, head_0-1, 바퀴 2개)
      → 이름 기반 매핑이 그대로 성립. 순서는 다르므로 인덱스로 다루면 안 된다
- `robot_sdk/service.py`: `RobotStateService` — 30Hz로 관절값을 읽어 FK → WebXR 변환 → robot_state 방송.
  robot_config(메시 구성) 등록도 여기서 한다. 메시지에 `source`(robot/sim)·`live`를 실어 브라우저 HUD가
  실로봇인지 시뮬레이션인지 표시한다
- `core/ik/robot_model.py`: `RobotModel` — **모델 두 벌**
    - 전체 모델(nq=30): 시각화 FK·카메라 외부 파라미터 전용. 실로봇의 토르소·머리까지 그대로 반영해야 하므로
      축소 모델로는 안 된다
    - 축소 모델(nq=14): IK 최적화 전용 (양팔만 자유, 나머지 ready 고정)
    - 관절값은 항상 {관절명: rad} 사전으로 주고받는다. **연속 관절(바퀴)은 pinocchio q를 2칸(cos·sin)**
      차지하므로 인덱스 가정 금지
    - `fk_visual(named, gripper_open)` / `camera_pose(named)` / `camera_link_pose(named)`
- `core/ik/service.py`: `IkService` — 30Hz로 IK만 푼다. **방송하지 않는다** (JointSource로서 시뮬레이션 공급자 역할)
    - **입력 두절 시 마지막 자세 유지** (ready 점프 금지 — 실로봇에서 순간 텔레포트 명령이 됨) + 클러치 해제
    - 진단 로그: 5초마다 한계 도달 관절 + shoulder/arm-upper 좌우 간격 실측
    - `sync_to(측정값)` = **거울 모드**: 제어 대기 중 solve를 건너뛰고 외부 값을 그대로 둔다. solve는 클러치
      미체결이어도 ready 바이어스가 섞인 최적화라 동기화 값을 매 주기 덮어썼다 (테스트에서 발견).
      `begin_return_to_ready()`가 거울 모드를 풀고 ready 보간을 시작한다
- `utils/config.py`: 경로·포트·IK 파라미터·카메라 장착·포인트클라우드·로봇 주소·**제어/안전/그리퍼/모바일 파라미터** 일원화
- `utils/inputs.py`: 업링크 샘플의 버튼·스틱·트리거 해석 (IK 서비스와 제어 서비스가 공유). xr-standard 인덱스 4=A, 5=B
- `utils/transforms.py`: WebXR↔로봇 좌표 변환 (양방향), 샘플 전체 변환
- `net/web_api/server.py`: HTTPS(8443/443) + HTTP 리다이렉트(80) + WSS 라우트 + `/models/` GLB 정적 서빙
- `net/web_api/precompress.py`: 기동 시 정적 자산의 `.gz` 형제 파일 생성
    - aiohttp 정적 핸들러는 `Accept-Encoding: gzip`이면 `<파일>.gz`가 옆에 있을 때 그걸 대신 보낸다
      → 서버 코드 변경 없이 전송량만 준다
    - **효과 실측: 페이지 1회 로드 39개 파일 24.83 MB → 8.02 MB (32%)**.
      babylon.js 8.26→1.78 MB, GLB는 37% 수준까지 줄어든다
    - 대상은 32 KB 이상, 잘 안 바뀌는 경로만(`web/vendor`, `web/assets`, 모델 메시 디렉터리).
      **`web/src`는 일부러 제외** — 압축본 갱신이 기동 시점에만 일어나므로 편집이 잦은 파일을 넣으면
      낡은 .gz가 서빙되는 함정이 생긴다 (합계 수십 KB라 이득도 없다)
    - 산출물은 gitignore(`*.gz`)
    - **로봇 GLB 간소화는 시도했다가 접었다** (재시도 금지): 텍스처는 0이고 전부 순수 형상,
      삼각형 277,505개(LINK_6 하나가 62,314개)라 quadric decimation으로 24%까지 줄면 13.47→2.86 MB가 된다.
      다만 형상이 눈에 띄게 각져 데모 품질이 떨어진다. 남은 로딩 시간은 감수할 만한 수준이라 원본을 쓴다
        - 하게 된다면: 정점 병합(`merge_vertices`) 후 간소화하고 `fix_normals()`로 감김을 정리해야 한다.
          병합을 건너뛰면 CAD 익스포트의 분리된 정점 때문에 표면이 조각나 얼룩진다.
          재질은 `TextureVisuals(material=...)`로 새로 물릴 것 — `visual.copy()`는 원본 정점 수 기준
          UV까지 끌고 와 배색이 깨진다
    - **모든 응답 Cache-Control: no-cache** — 브라우저 휴리스틱 캐시가 옛 JS를 계속 서빙해
      수정이 반영 안 되는 문제가 있었음. 재검증은 정상 동작한다(실측: 재요청 시 304, 본문 0바이트)이므로
      **재접속은 이미 싸다. 느린 것은 첫 로드**였고, 원인은 캐시가 아니라 무압축 전송이었다
- `net/web_api/pose_hub.py`: `PoseHub` — 업링크 수신·변환·보관, robot_config 접속 시 1회 전송,
  `broadcast_state()`(JSON)·`broadcast_bytes()`(바이너리)로 전 클라이언트(VR+모니터)에 방송
    - **송신 격리 구조**: 클라이언트마다 큐(깊이 8) + 전용 writer 태스크. 방송은 큐에 넣기만 하고 블로킹 없음.
      큐가 차면 오래된 메시지 드롭(최신 우선), 드롭 수는 5초 로그에 누적 표시
    - 장애 이력: 한 클라이언트 소켓이 읽지 않아 `await send`가 영구 대기 → 같은 `_send_to`를 쓰던 IK 루프와
      포인트클라우드 루프가 **동시에 멈춤**(크래시 아님, 로그 없음). 증상: 송신 Hz가 서서히 떨어지다 통계 로그가 사라짐
    - 소켓 단절 시 예외는 `ConnectionResetError`가 아니라 일반 `ConnectionError("Connection lost")`로 올 수 있음
- `core/ik/service.py`·`core/depth_to_3d/service.py`·`robot_sdk/service.py`: 루프 태스크에 done 콜백으로 예외 로그
- `core/ik/solver.py`: `ArmIkSolver` — **EE(그리퍼) 포즈 트래킹 + 팔꿈치 방향 소프트 코스트** (어깨 절대 트래킹은 없음.
  body-tracking이 잡힐 때만 팔꿈치 항 활성, 미검출 시 자동 off)
    - **솔버는 sqpmethod + convexify_strategy=eigen-reflect** — ipopt는 호출당 30-50ms로 30Hz 불가
      (실사용 딜레이의 원인이었음). sqp는 동일 잔차에 0.1-2ms. convexify 없으면 발산하므로 필수
    - 코스트: 그리퍼 TCP 위치 + 자세(회전행렬 Frobenius) + 이전 해 정규화 + **ready pose 바이어스**
      (w=0.02, 여유 자유도 해소 — 기존 Unity teleop의 nullspace target 역할. null-space 투영 없는
      소프트 코스트라 크면 추종을 해침: 0.3→3-6cm 오차, 0.02→평균 3.5mm)
    - **log6 사용 금지**: 오차 0(수렴점)에서 미분 NaN → Frobenius로 대체함
    - 위치 기준점은 TCP(집는 지점) = ee 프레임 로컬 (0,0,-0.12) — fk_frames의 wrist 위치에도 동일 적용
    - 미검출 팔은 현재 자세 유지. **관절별 속도 상한** = min(URDF velocity × 0.5, 1.5 rad/s) × 실경과시간
      (기존 전역 4.0 rad/s는 arm_6 하드웨어 한계 2.09를 초과했었음 — 감사 지적)
    - **안전 관절 한계 적용** (custom-teleop의 add_joint_limit와 동일): arm_3 [-2.6,-0.5](팔꿈치 신전 금지),
      arm_5 [0.2,1.9](손목 역꺾임 금지) — URDF 원값은 콜리전 영역 진입을 허용하므로 교집합 필수
    - NaN/inf 결과 가드 (error_on_fail=False의 방어)
    - 형태학적 한계: 로봇 전완+그리퍼(0.53m)가 사람(~0.25m)보다 길어 가슴 바로 앞 목표는 잔차 수 cm 발생
- `core/ik/mapper.py`: `TargetMapper` — **grip 클러치 + 상대 델타 매핑** (실사용 검증된 custom-teleop 구조)
    - **클러치**: grip(squeeze) > 0.8을 쥐고 있는 동안만 추종. 놓으면 팔 현재 자세 유지
    - **델타 매핑**: engagement 순간 손 포즈·로봇 TCP 포즈를 앵커로 캡처, 이후 손의 델타만 이식
      (위치 1m=1m 평행이동, 자세는 월드 프레임 델타 좌곱 ΔR·R_tcp앵커)
      → 목표가 항상 현재 자세에서 연속 시작 = **VR 진입/트래킹 점프 시 폭주가 구조적으로 불가능**
      → 절대 자세 오프셋 캘리브레이션 자체가 불필요 (앵커에 흡수)
    - 델타는 몸통 프레임(어깨선+중력)으로 재표현 → 서 있는 방향 무관 (yaw 불변성 검증)
    - **IK 목표는 그리퍼 TCP** (플랜지 기준이면 손이 그리퍼보다 15cm 뒤 위화감)
    - **부위별 길이비 스케일링 금지** (로봇 전완이 사람 2배 → 그리퍼가 손을 앞지름, 겪은 회귀)
    - 조인트 의미: 어깨 볼 = *-arm-upper 원점 (*-shoulder는 쇄골). 가슴·yaw 계산에 사용
    - **절대 자세 매핑의 함정 (독립 감사로 발견)**: 손 쿼터니언은 상사변환(C·R·Cᵀ)되므로 "자연 자세"의
      로봇 프레임 값은 항등이다. 이전 절대 매핑의 _R_GRIP_NATURAL은 한쪽 변환(C·R) 규약과 혼합되어
      목표 자세가 120° 틀어졌고 이것이 "VR 진입 시 팔 폭주·콜리전 진입"의 주범이었음.
      절대 매핑을 되살릴 일이 있으면 반드시 이 규약을 확인할 것
- `core/ik/service.py`: `IkService` — 30Hz 루프, executor에서 solve, FK → WebXR 변환 → robot_state 방송
    - **입력 두절 시 마지막 자세 유지** (ready 점프 금지 — 실로봇에서 순간 텔레포트 명령이 됨) + 클러치 해제
    - 진단 로그: 5초마다 한계 도달 관절 + shoulder/arm-upper 좌우 간격 실측
    - **실로봇 연결 지점**: solve 결과 q를 robot_sdk로 보내면 됨. **주의: q 순서는 pinocchio 축소 모델
      순서(left 먼저)로 SDK 관례(torso→right→left)와 다름** — robot_config의 joint_names(q 순서 관절명)로
      반드시 이름 기반 매핑할 것 (감사에서 좌우 뒤바뀜 함정으로 지적됨)

- `utils/camera/`: `FrameSource`(abc, base.py) + `D455Camera`(d455.py) — **장치 1회 오픈**, 캡처 스레드가
  depth를 color에 정렬(rs.align)해 `RGBDFrame(depth m float32, color rgb8, seq, t_mono)`를 최신 1장만 보관.
  소비자(포인트클라우드, 추후 영상 스트림)는 `latest()`로 스냅샷만 가져감 (seq로 중복 판별)
    - 640x480@30 → 실측 30 fps. SDK 버퍼는 재사용되므로 복사본 보관 필수
    - 카메라 없으면 main.py가 경고 후 포인트클라우드만 비활성 (포즈·IK는 정상 기동)
- `core/depth_to_3d/`: `PointCloudBuilder`(builder.py) → `pack_pointcloud`(packet.py) → `PointCloudService`(service.py)
    - **단일 프레임만 보낸다** (시간 누적 없음). 패킷 하나 = 그 프레임에서 본 표면.
      로봇·머리가 움직여도 잔상이 남지 않고, 대역폭·처리 비용이 누적 방식보다 훨씬 싸다
    - 실측 대비: 누적(복셀 0.02m×20프레임) 92k점 8.0 MB/s·8-13 ms/프레임, 모니터 송신 드롭 누적 8842건
      → 단일 프레임 15.3k점 **1.34 MB/s·1-2 ms/프레임, 드롭 0**
    - 카메라 외부 파라미터는 머리·토르소 FK에 딸린 값이라 **프레임마다 갱신**한다
      (`builder.set_extrinsics`). FK는 이벤트 루프 쪽에서 — RobotModel의 pinocchio Data를 다른 서비스와
      공유하므로 executor 스레드에서 부르면 경합이 난다
    - 역투영: 정규화 좌표 격자 (u-ppx)/fx 를 1회 계산, 스트라이드 4 서브샘플(최대 19,200점), 깊이 0.2-8.0 m만 유지
    - **유효 깊이 상한을 4 → 8 m로 올렸다**: 실내에서 4 m는 짧아 대부분이 잘려 나갔다
      (실측 유효점 775/19,200 → 17,137). 점 수는 스트라이드 격자로 이미 묶여 있어 대역폭이 튀지 않는다
      (1.5 MB/s)
    - **머리 각도 규약: `head_1` 양수 = 아래 보기.** URDF 기준 실측 — head_1=0°면 광학축이 이미 10° 하향,
      +23°면 33° 하향, **-37.7°면 27.7° 상향**이다. 로봇 머리가 위를 보고 있으면 점이 천장에만 찍힌다
      ("pcd가 안 나온다"의 정체였음 — 좌표 버그가 아니라 카메라가 천장을 보고 있던 것)
    - 외부 파라미터는 `RobotModel.camera_pose()` — 머리 링크(config.CAMERA_MOUNT_LINK) FK × 장착 오프셋·광학 축 대응
      (config.CAMERA_MOUNT_POS/ROT). ready(머리 pitch 0.4) 기준 base 위치 약 [0.19, 0.05, 1.28], 광학축 23° 하향
    - D455는 실제로 로봇 머리에 장착되어 있다 (2026-08-23 확인). 오차가 보이면 CAMERA_MOUNT_POS를 실측 보정
    - `RobotModel.camera_visual_config()`가 장착 정보를 WebXR 기저로 바꿔 robot_config.camera로 1회 전송
    - 패킷: `PCD1` 매직 + count + float64 t + **int16 xyz(mm)** + uint8 rgb = 16 + 9N 바이트
      (float32 대비 절반, 13k점 기준 약 120 KB, 10 Hz → 1.2 MB/s 실측)
    - 대역폭은 장면에 좌우된다: 내장 PC 실측(넓은 실내, 최대 깊이 8 m) 17.1k점 1.5 MB/s.
      더 줄여야 하면 `PCD_MAX_DEPTH`·`PCD_STRIDE`로 조절한다
    - 방송은 `PoseHub.broadcast_bytes()` (VR 업링크 소켓 + 모니터 소켓 모두에 바이너리 프레임)

## web/ 프론트엔드 — 렌더링 좌표 규약 (중요, 어기면 로봇이 분해되어 보임)

- **시점 두 가지** (main.js가 기기에 따라 갈라 구성한다)
    - VR(Quest): **로봇 시점**. XR 카메라를 로봇 머리 장착 카메라 위치에 고정하고 로봇은 팔 계통만 그린다
        - XR 카메라 위치는 헤드셋이 매 프레임 덮어쓰므로 직접 못 옮긴다 → 부모 TransformNode를 두고
          `부모 위치 = 목표 − 카메라 로컬 위치`로 잡아 월드 위치를 고정한다. **회전은 건드리지 않는다**
          (고개 돌리기는 그대로 동작). 갱신은 XR 카메라가 갱신된 뒤인 `onBeforeRenderObservable`에서
        - 위치를 완전히 고정하므로 사용자가 걸어도 시점은 안 움직인다. 로봇 머리가 사용자를 따라 돌지 않는
          현 단계에서는 이게 맞다 — 머리 추종을 넣으면 이 고정을 다시 검토할 것
        - 몸통·머리 메시를 그대로 두면 **시점이 머리 메시 안이라 내부 면이 화면을 덮는다** → "arms" 모드 필수
    - PC: **관찰 시점**. 로봇 전체 + 조작자 아바타를 나란히 놓고 포인트클라우드까지 함께 본다
- 로봇 base = 씬 원점(ROBOT_OFFSET = 0). 포인트클라우드도 같은 원점을 공유해 로봇 주변에 그려진다.
  사람 아바타만 HUMAN_OFFSET(로봇 왼쪽 1m)에 세운다

- 씬은 Babylon 기본 **왼손 좌표계 유지 필수**
    - `scene.useRightHandedSystem = true` 금지: 카메라 전방이 -z가 되어 HUD(+z 고정)가 안 보임
    - **glTF 로더의 `FORCE_RIGHT_HANDED` 모드도 금지**: 씬 전체를 조용히 RH로 강제 전환해
      HUD 실종 + 카메라 조작 시 프러스텀 컬링 붕괴(전부 빈 화면) 회귀를 일으켰음
- 포즈 표시(WebXR 오른손 → 왼손 씬): z-거울 켤레 — pos (x,y,-z), quat (-x,-y,z,w)
- **GLB 메시 기저 보정 유도** (robot_view.js `meshfix` 래퍼):
    - 링크 포즈를 WebXR 기저로 상사변환(Cᵀ·T·C)하면 링크 로컬 기저도 재정의됨.
      GLB 정점은 URDF 링크 프레임(z-up) 데이터라 로컬에 Cᵀ 추가 적용 필요
    - Babylon glTF 로더는 LH 씬에서 __root__에 x-거울(M_x = Ry180·scale(1,1,-1))을 넣음
    - 최종 래퍼 W = M_z·Cᵀ·M_x = 순수 회전 120° (축 (-1,1,1)/√3) = **쿼터니언 (-0.5, 0.5, 0.5, 0.5)**
    - visual origin도 같은 이유로 서버에서 WebXR 기저로 변환해 보내고, 클라이언트는 포즈와 동일한 z-거울 켤레만 적용
- 검증 방법(재사용 가능): 전체 모델 제로 포즈(직립)의 링크 포즈를 주입해 조립 상태 확인 — 직립이 무너지면 좌표 규약 회귀
- 기타: Babylon 9.22.0 + babylonjs.loaders.min.js 로컬 벤더링(오프라인), 컨트롤러 메시 로드 비활성,
  PC(immersive-vr 미지원)에서는 XR 경험 자체를 생성하지 않음 (유령 XR 카메라 방지)
- `src/net/uplink.js`: 포즈 송신(60Hz 상한, Quest 실측 45Hz) + robot_config/state 수신
- `src/render/hud.js`: VR **왼쪽 아래 구석에 월드 고정**된 작은 패널(0.48×0.30m) + EXIT 버튼
  (라운드 필 + 로그아웃 아이콘, 레이 호버 하이라이트, 클릭 → 세션 종료) + 데스크톱 오버레이
    - **카메라 고정이 아니라 앵커 고정**: 카메라에 붙이면 고개를 돌려도 화면 같은 자리에 따라와
      "구석에 두고 필요할 때 본다"가 불가능. VR 진입 후 첫 유효 머리 포즈(높이>0.3m)에서
      수평 시선 기준 전방 0.9·왼쪽 0.8·아래 0.6m(약 42° 좌·34° 하)에 앵커를 두고 머리를 향해 기울임
    - 패널의 읽히는 면은 로컬 -z → lookAt(2·pos - head)로 +z를 머리 반대편에 향하게 함
    - 세션마다 재배치 (detach → attach). 캔버스 2D 드로잉만 사용 (외부 에셋 없음)
- `src/render/human_view.js`: `HumanView` — 조작자 아바타 (데스크톱 전용, VR에서는 자기 자신이라 안 그림)
    - 에셋 `web/assets/avatar.glb` = Mixamo **X Bot** (three.js 예제 배포본, 2.8MB). 회색 마네킹이라
      SMPL 렌더와 비슷한 인상이고 표준 humanoid 리그를 쓴다. 처음 쓴 CesiumMan은 생김새가 부적절해 교체함
    - **리그는 이름 목록이 아니라 접미사 패턴 + 계층 탐색으로 해석**한다 (`mixamorig:LeftForeArm` 등).
      ForeArm/UpLeg가 Arm/Leg 패턴에 걸리지 않도록 후방탐색 `(?<!fore)arm$` · `(?<!up)leg$`를 쓴다.
      팔·다리는 좌우로 가른 뿌리 본에서 계층을 타고 내려가 짝을 맞춘다 (슬롯마다 독립적으로 좌우를
      정하면 리그에 따라 좌우가 섞일 수 있다)
    - **리타게팅은 방향 정합(position-based)**: 본마다 rest 방향 d0와 추적 조인트로 만든 목표 방향 d1의
      최단호 회전 q를 구해 `q ⊗ restRot`을 본의 월드 회전으로 넣는다. 조인트 쿼터니언을 직접 쓰지 않는 이유는
      트래킹 조인트와 리그 본의 로컬 축 규약이 달라 상사변환 규약을 틀리면 통째로 틀어지기 때문
      (매퍼의 `_R_GRIP_NATURAL` 함정과 같은 종류). 대신 본 축 회전(roll)은 잃는다
    - **함정 3종 (전부 실제로 겪음)**
        1. glTF 로더는 본을 TransformNode에 link해 **매 프레임 노드 → 본으로 덮어쓴다** → 로드 직후
           `bone.linkTransformNode(null)`로 끊지 않으면 본을 돌려도 결과가 사라진다
        2. **본 회전은 스켈레톤 공간에서 넣어야 한다** (mesh 인자 없이). `getRotationQuaternion(WORLD, mesh)` /
           `setRotationQuaternion(..., WORLD, mesh)`는 메시 월드 행렬을 함께 분해하는데, 이 아바타는
           Armature에 0.01 배율이 있고 로더가 __root__에 z-거울(det<0)을 넣어 둬서 분해가 어긋나
           본 로컬 행렬에 배율이 섞여 **메시가 100배로 부푼다**. 목표 방향은 메시 월드 행렬의 역행렬로
           선형 변환해 스켈레톤 공간으로 가져온 뒤 rest 방향과 비교한다
        3. **자세 샘플이 없을 때도 컨테이너를 기본 자리(로봇 옆)에 둔다**. 원점에 그대로 두면
           rest(T) 자세로 로봇과 겹쳐 보인다. 숨기는 게 아니라 자리를 잡아 주는 게 맞다 —
           Quest가 안 붙어 있을 때 아바타가 사라지면 그것대로 이상하다
    - 좌우 판별만은 **월드 좌표** rest 위치로 한다 — 스켈레톤 공간은 거울 때문에 x 부호가 뒤집혀 있다
      (위치 변환은 분해가 없어 안전하고, 회전만 위험하다)
    - 배율: 사람의 엉덩이-머리 거리 / 아바타 rest 거리 (첫 샘플에서 1회, 0.5-2.0 클램프).
      컨테이너 배율은 위 2번을 스켈레톤 공간 방식으로 고친 뒤로는 안전하다
    - 배치 간격 HUMAN_OFFSET = 로봇 왼쪽 **1.5 m** (1.0 m는 양쪽이 팔을 벌리면 닿는다)
    - 좌우 판별은 본 이름이 아니라 **rest 자세의 x 부호**로 한다 (리그마다 L/R 명명이 제각각)
    - 배치: 어깨선으로 사람 yaw를 구해 로봇과 같은 방향(+z)으로 정규화 후 엉덩이를 HUMAN_OFFSET으로 이동.
      `θ = atan2(-sz, -sx)` (s = 오른어깨→왼어깨의 xz 단위벡터). 파이썬 이식으로 yaw 불변성 수치 검증함
    - 루트 배치(`_placeAt`): 현재 엉덩이 본의 월드 위치와 목표의 차이만큼 컨테이너를 민다
      (배율·거울에 무관한 방식). 로드 직후에는 목표를 기본 자리로 잡아 rest 자세로 세워 둔다
    - `setDebugJoints(true)`로 추적 조인트 마커 표시 (리타게팅 디버깅용)
- `src/net/pointcloud.js`: 패킷 디코더 (헤더 16바이트라 Int16Array 뷰를 복사 없이 생성)
- `src/render/pointcloud_view.js`: `PointCloudView` — 점 메시 1개, 버퍼는 (수신 점수 × 1.5) 용량으로 할당 후
  `updateVerticesData`로 덮어씀. 남는 슬롯은 NaN(그려지지 않음). z-거울만 적용하고
  `ROBOT_OFFSET`(robot_view.js에서 export) 아래 배치 → 로봇 base 기준 위치가 그대로 맞음
    - 서버가 단일 프레임만 보내므로 클라이언트도 누적하지 않는다 (패킷 = 이번에 그릴 전체 점집합)
- `src/render/robot_view.js` `setVisibleParts("all"|"arms")`: 링크 변환 노드는 그대로 두고 **메시 노드만** 끈다
  → 숨긴 상태에서도 FK 체인과 시점 앵커가 유효하다. 팔 계통 판별 정규식은 `/_arm_\d|^ee_|^FT_sensor/`
- `src/render/robot_view.js` `cameraAnchorPosition()`: 머리 장착 카메라 프레임의 월드 위치 (VR 시점 앵커).
  메시 표시와 무관하게 살아 있어야 해서 D455 메시와 **별도 노드**로 둔다
- `src/render/robot_view.js` `_buildCamera`: robot_config.camera {link, origin(WebXR 기저 링크 로컬), mesh_base, mesh}로
  **실제 D455 GLB**를 link_head_2 아래에 붙임. 링크 GLB와 같은 체인(`_attachMesh`: origin → meshfix 래퍼 → GLB)
    - `web/assets/d455.glb`: Intel realsense-ros `realsense2_description/meshes/d455.stl`(mm)을 xacro visual origin
      (xyz 4.65mm -47.5mm 0, rpy π/2 0 π/2, scale 0.001)대로 **camera_link 프레임(x전방 y좌 z상, m)에 구운** 파일.
      원점 = 좌측 IR(depth) 광학 중심, 몸체는 -y로 124 mm. 변환은 컨테이너에 trimesh를 설치해 1회 수행(임시 스크립트 삭제)
    - camera_link 축 규약이 URDF 링크와 같아 로봇 링크 GLB와 동일한 기저 보정 쿼터니언을 그대로 씀
    - `alwaysSelectAsActiveMesh = true` 필수: 경계 상자가 갱신 안 되어 컬링으로 사라지는 것 방지
    - 소켓은 `binaryType = "arraybuffer"`, uplink/monitor 둘 다 두 번째 콜백(onBinary)으로 전달
    - HUD에 `pointcloud: NHz, N점` 표시. 디버깅 훅 `window.__teleop.pcdView`
- `src/xr/pose_reader.js`: body 조인트 + 컨트롤러 수집
    - **필터는 '손가락 제외'** (이전엔 상체 조인트만 통과시켰다). 아바타 전신 자세에 몸통·머리·다리가
      필요하고, 손가락을 빼면 20여 개라 대역폭 부담이 없다
    - **Meta body 조인트 의미 (계층: Shoulder→Scapula→ArmUpper→ArmLower)**:
      `*-shoulder`=쇄골 관절(몸 안쪽), `*-arm-upper` 원점=어깨 볼(상완골 시작), `*-arm-lower` 원점=팔꿈치.
      어깨 기준점으로 *-shoulder를 쓰면 팔꿈치 방향이 안쪽으로 편향되어 robot arm_1이 한계(+1°)에
      고착되는 문제가 있었음 (매퍼는 arm-upper 사용, shoulder는 폴백)
    - 서버 진단 로그: 5초마다 한계 도달 관절 목록 + shoulder/arm-upper 좌우 간격 실측 (의미 검증용)
- 디버깅 훅: `window.__teleop` = { engine, scene, robotView, pcdView, human, robotStatus }
- **프론트 진단 방법 (매우 유효, 이번에 아바타 버그를 잡은 경로)**: 컨테이너에 크로미움을 넣어 헤드리스로
  띄우고 콘솔·스크린샷·CDP를 쓴다. 런타임 의존이 아니라 이미지에는 넣지 않았다 — 필요할 때 설치한다
    - 설치: `docker exec airlab_hw_teleop apt-get install -y --no-install-recommends chromium`
    - 스크린샷 + 콘솔: `chromium --headless=new --no-sandbox --use-gl=angle --use-angle=swiftshader
      --enable-unsafe-swiftshader --ignore-certificate-errors --window-size=1280,800
      --virtual-time-budget=25000 --screenshot=<경로> --enable-logging=stderr --v=0 https://localhost:8443/`
    - 씬 상태 조회: `--remote-debugging-port=9333`으로 띄운 뒤 `/json`에서 페이지 타겟을 찾아
      `Runtime.evaluate`로 `window.__teleop.*` 값을 읽는다 (컨테이너에 websockets 있음)
    - 주의: 컨테이너에 한글 폰트가 없어 스크린샷의 HUD 글자는 네모로 보인다 (표시 문제일 뿐)
    - 사람 아바타는 자세 샘플이 있어야 보인다 — Quest가 없으면 합성 샘플을 `/ws/pose`로 흘려 넣어 확인한다

## Quest 직결 네트워크 (확정)

- 내장 PC의 USB 이더넷(`enx00e04c2f6858`)을 NetworkManager 공유 모드로 설정. 연결 이름 `quest-link`,
  PC 10.42.0.1/24 고정, Quest는 DHCP 자동 (dnsmasq·iptables는 내장 PC에 이미 설치되어 있음)
    - `nmcli connection modify "Wired connection 1" connection.id quest-link ipv4.method shared
      ipv4.addresses 10.42.0.1/24 ipv6.method ignore connection.autoconnect yes`
    - 어댑터의 기본 프로필은 DHCP를 기다리며 "getting IP configuration"에서 멈춰 있으므로 그 프로필을 바꾼다
- 접속: Quest 브라우저에서 `10.42.0.1` 입력 (80→HTTPS 리다이렉트, 443 별칭도 동작)
- 인증서 SAN에 직결 IP와 무선 IP를 함께 넣는다:
  `python -m teleop.net.web_api.gen_certs --ip 10.42.0.1 192.168.1.58` (무선 IP는 DHCP라 바뀌면 재발급)
- 평문 http를 8443에 보내면 무응답 끊김(ERR_EMPTY_RESPONSE, 로그 없음) — 진단 시 최우선 의심
- Quest 필수 설정: chrome://flags "WebXR Experiments" + 설정→동작 추적→신체 추적
- 서버 로그: `logs/web_api.log`

## 구조 개편 + 웹 런타임 모드 전환 (2026-08-25)

- **teleop/ 재구성**: 파일명만 보고 역할이 보이도록 전면 개명. **이 문서의 이전 기록에 나오는
  옛 경로는 아래 표로 읽는다** (내용은 동일, 경로·이름만 변경):
    | 옛 경로 | 새 경로 |
    |---|---|
    | utils/config.py | 그대로 (한때 루트로 승격했다가 utils 정의에 부합해 복귀. **위치를 옮기면
      REPO_ROOT 역산 parents 깊이를 같이 고칠 것** — 실제로 밟은 함정) |
    | core/ik/solver.py, mapper.py | ik/ik_solver.py, pose_to_target.py |
    | core/ik/service.py (IK 자체 루프) | **robot_controller.py에 흡수** — 아래 "IK pull 구조" 항목 참고 |
    | core/ik/robot_model.py | robot/robot_model.py (IK 전용이 아니라 시각화 FK·카메라 외부 파라미터 공용) |
    | core/depth_to_3d/builder+packet.py | pointcloud/depth_to_pcd.py (변환+패킹 통합, `PointCloudConverter`·`pack_pointcloud`) |
    | core/depth_to_3d/service.py | pointcloud/pcd_streamer.py (`PointCloudStreamer`) |
    | net/web_api/server.py | net/web_server.py |
    | net/web_api/pose_hub.py | net/websocket.py (클래스 `PoseHub` → `WebSocketHub`) |
    | net/web_api/precompress.py | **web_server.py에 내장** (`_compress_static_assets` — 서버 기동 단계의 세부라 파일 분리 안 함) |
    | net/web_api/gen_certs.py | **tools/make_https_cert.py** (서버가 아닌 1회성 도구라 tools/로) |
    | robot_sdk/base.py | robot/robot_interface.py |
    | robot_sdk/control.py | robot/robot_controller.py (`RobotController` — 로봇에 명령 전송) |
    | robot_sdk/service.py | robot/robot_state_broadcaster.py (`RobotStateBroadcaster` — 상태 방송) |
    | tools/fake_quest.py (신설 당시 이름) | tools/fake_vr_quest3.py |
    | mode_manager.py (한때 분리) | **main.py에 통합** (`ModeManager` 클래스 — 최상단 파일은 main 하나) |
    | robot_sdk/rby1/robot·gripper.py, sim/robot·gripper.py | robot/rby1_robot·rby1_gripper·sim_robot·sim_gripper.py (평탄화) |
    | utils/camera/base·d455·synthetic.py | camera/camera_interface·d455_camera·synthetic_camera.py |
- **assets/ 통합**: `models/rby1a` + `web/assets/*` → 루트 `assets/` (rby1a/, avatar.glb, d455.glb).
  web/ 밖이므로 web_server가 `/assets/` 정적 라우트를 별도로 연다. URL은 기존과 동일해 프론트 수정 없음.
  precompress 대상은 [web/vendor, assets/] (assets는 재귀라 rby1a 메시 포함)
- **웹 런타임 모드 전환** (main.py의 `ModeManager`): TELEOP_MODE는 초기 모드일 뿐, 데스크톱 뷰 버튼으로 dev↔deploy 전환
    - 소유 구조: 모드 종속(백엔드·그리퍼·카메라 + 상태·제어·pcd 서비스)은 ModeManager가,
      모드 무관(포즈 허브·IK·웹 서버)은 main이 소유. 전환 시 종속만 재생성
    - **새 부품을 먼저 연결하고 성공했을 때만 기존을 해체** — deploy 전환 실패 시 dev가 무중단 유지
      (실측: 연결 실패 → "dev 모드 유지" 방송, 서비스 끊김 없음)
    - 안전: 제어 중(armed) 전환 거부 (B 해제 먼저), deploy 초기 기동 실패는 여전히 SystemExit,
      웹 deploy 전환은 confirm 대화상자
    - 프로토콜: 클라이언트 → `{"type":"set_mode","mode":"dev|deploy"}` (pose·monitor 소켓 양쪽 수신,
      "type" 필드 있는 텍스트는 포즈 샘플이 아니라 명령으로 분기) / 서버 → `mode_status` 방송
      {mode, switching, ok, message} (신규 접속에도 robot_config와 함께 1회)
    - UI: 데스크톱 좌상단 버튼 (dev 청록 / deploy 주황 / 전환 중 비활성), 결과는 상태 줄에 표시
    - 검증: dev 기동 → 웹 프로토콜로 deploy 요청 → 로봇 연결 실패 → dev 유지 + 오류 방송 확인,
      fake_quest 2사이클 제어 회귀(A→ready→추종→B→재-A), 스크린샷으로 버튼·씬 렌더링 확인

## 웹 수동 조작 패널 (web/src/manual_control.js, 2026-08-25)

- dev 모드 데스크톱 전용 (deploy 전환 시 패널 숨김 + 송신 정지 — 패널 입력이 실로봇으로 가는 일 방지)
- **입력 값의 출처만 다르고 코드 경로는 VR과 동일**: PoseReader와 같은 형식의 샘플을 같은
  PoseUplink(/ws/pose)로 30Hz 송신 — 서버는 Quest인지 패널인지 구분 불가
- 손 위치는 rest 기준 오프셋 슬라이더 (클러치 델타 매핑이라 grip 켠 채 움직인 거리 = TCP 이동 거리),
  A/B·주행 버튼은 누르는 동안만 활성(momentary, pointerleave에서도 해제)
- 송신 중지 시 별도 정지 명령 불필요 — 서버가 입력 두절(3초)로 자동 해제
- body 조인트는 fake_vr_quest3.py와 같은 근사로 합성. 디버깅 훅 `window.__teleop_manual`

## 웹 수동 조작 패널 + 모드 UI 정리 (2026-08-25)

- `web/src/manual_control.js`: dev 모드 데스크톱 전용 수동 조작 패널 (`index.html`의 #manualPanel)
    - **입력 출처만 다르고 코드 경로는 VR과 동일**: PoseReader와 같은 형식의 샘플을 만들어 같은
      `PoseUplink`(/ws/pose)로 보낸다. 서버는 Quest인지 패널인지 구분 불가 (body 조인트 합성은
      fake_vr_quest3.py와 동일 근사)
    - 조작: 활성 체크 → grip 체크 + 손 오프셋 슬라이더(rest 기준 ±0.3m, 클러치 델타라 슬라이더
      이동량 = TCP 이동량) + 트리거 슬라이더 + A/B 버튼(momentary) + 주행 화살표(홀드)
    - 앞(z) 슬라이더는 +가 전방 — WebXR -z로 변환해 싣는다
    - deploy 전환 시 패널 숨김 + 송신 정지 (패널 입력이 실로봇으로 가는 것 방지). 송신을 끄면
      서버 입력 두절 타임아웃이 자동 해제하므로 별도 정지 명령 불필요
    - 업링크 소켓은 첫 활성화 때 1회 접속, 다운링크 수신은 버림 (표시는 모니터 소켓 담당)
    - 디버깅 훅 `window.__teleop_manual` — CDP로 setEnabled/setGrip/setOffset/setButton/setStick을
      호출해 실기기 없이 종단 검증함 (클러치 앵커 → IK 추종 → 주행 → B 해제 확인)
    - **키보드 조작** (같은 setter를 키 이벤트가 호출 — 패널·키보드·CDP 모두 한 상태를 공유):
      M 활성 토글 / W·S 앞뒤, A·D 좌우, R·F 위아래 홀드 이동(0.15 m/s, Shift=왼손) /
      G grip 토글 / T 트리거 홀드(2.5/s 램프) / Enter=A·Backspace=B(momentary) / 방향키 주행(합성 허용).
      입력 요소에 포커스가 있으면 무시 (슬라이더와 충돌 방지), 홀드 적용은 송신 타이머 틱에서 dt 비례.
      방향키를 안 쓰는 동안은 스틱 값을 건드리지 않아 패널 버튼과 공존.
      검증: CDP 합성 키 이벤트로 M→Enter→G→W(1.5s 홀드 = 0.228m 이동, 슬라이더 동기화)→방향키→Backspace
      전 흐름 + 서버 로그(클러치 앵커·29Hz 전송·B 해제) 확인
    - **프로세스 정리 함정 (실제로 밟음)**: 컨테이너 안 정리에서 `python -*` 같은 느슨한 패턴은
      돌고 있던 서버(`python -m teleop.main`)까지 죽인다. 반드시 명령의 시작 단어로 정확히 매칭할 것
      (`chromium*` 접두 매칭 등). kill 명령 문자열의 자기 매칭 함정과 별개로 한 번 더 당한 것
- **실사용 버그 4종 수정 (2026-08-27, "손은 움직이는데 로봇이 안 따라옴 + 방향키가 카메라 회전")**
    - **momentary 버튼 소실**: 순간 클릭(수십 ms)이 33ms 송신 틱 사이에 끼면 눌린 샘플이 0개 전송 —
      A가 서버에 아예 도달하지 않아 제어가 시작되지 않았다 ("제어 시작" 로그 부재로 진단).
      → `setButton` 래치: 누름 시작 시 최소 6샘플(0.2초)의 눌린 송신 보장 (0ms 클릭으로 검증)
    - **패널 포커스 잔류**: 체크박스·슬라이더 클릭 후 포커스가 남아 키보드 입력이 전부 무시됨
      → 패널 pointerup 시 activeElement blur
    - **방향키 카메라 충돌**: ArcRotateCamera가 방향키를 기본 바인딩 → scene.js에서 `camera.keys* = []`
      (궤도 조작은 마우스 유지)
    - **주행이 안 보임**: 시뮬 베이스 적분이 로그에만 있고 시각화에 없었음 →
      `RobotBackend.base_pose()` (기본 None, 시뮬만 구현) → robot_state에 "base"(WebXR 기저) →
      robot_view가 로봇 루트를 통째로 이동. 포인트클라우드는 원점 유지 (합성 장면이라 정합 무의미,
      실로봇 오도메트리 연동 시 재검토). 실로봇은 base_pose None → 기존 동작 불변
    - **제어 상태 가시화**: robot_state에 "armed" 추가 (broadcaster의 control_source=RobotController)
      → HUD에 "제어 대기 — A를 눌러 시작 / 제어 중 (B=해제)" 표시. "왜 안 따라오지"의 답이 화면에 보임
- 모드 UI 정리: 상태 줄의 "모드: ..." 문구 삭제, 버튼은 전환 동작만 표시("deploy로 전환" —
  대상이 deploy면 주황 경고 스타일), 전환 실패만 alert. 현재 모드는 HUD 로봇 항목(시뮬/실로봇)으로 확인
- **CSS 함정 (실제로 밟음)**: `#ui button { display: block }`이 `hidden` 속성(UA의 display:none)을
  덮어써 숨긴 버튼이 보였다 → `#ui button[hidden] { display: none }`을 명시해 복원.
  hidden 속성을 쓰는 요소에 display를 주는 CSS를 추가할 때는 반드시 [hidden] 복원 규칙을 같이 둘 것

## IK pull 구조 — ik_loop 제거 (2026-08-25)

- **IK는 스스로 돌지 않는다**: ik/에는 순수 계산기(ik_solver, pose_to_target)만 남고,
  RobotController의 30Hz 루프가 매 주기 solve를 호출한다 (제어 중에만 — 실측 2.4-3.1 ms/solve)
- 제거 이유: IK 자체 루프는 로봇·제어 서비스가 없던 개발 단계의 잔재. 모드 구조 도입 후
  dev/deploy 모두 controller가 항상 존재하므로 30Hz 루프 두 개가 같은 상태를 공유할 이유가 없음.
  병목 문제 아님 (solve 3ms ≪ 33ms 예산)
- 함께 소멸한 것들:
    - **거울 모드(sync_to/mirroring)**: 두 루프 조율 장치였음 → 대기 상태에서 controller가
      `_seed_from_measured()`로 IK 상태(_last_q·솔버 warm start)를 직접 시드 (루프 하나라 경합 불가)
    - **A 버튼 에지 감지 중복**: ik_loop과 controller 양쪽에 있던 것이 controller 한곳으로
    - **팔/토르소 ready 보간 분리**: 팔(_step_toward_ready)도 controller로 — ready 복귀가 한 파일에
- 상태 리셋 규약: 모드 전환 시 controller가 재생성되며 IK 상태도 리셋되지만, 대기 첫 주기에
  측정값으로 재시드되고 클러치 앵커는 원래 grip 재체결 때 새로 잡히므로 리셋이 올바른 동작.
  비싼 객체(CasADi NLP를 든 solver, mapper)만 main이 1회 생성해 전환 시 새 controller에 재사용
- 그리퍼 열림(트리거 해석)도 controller 소유 — 시각화 폴백은 `RobotController.gripper_openness()`
  (RobotStateBroadcaster의 gripper_source, gripper_openness()만 있으면 되는 덕 타이핑)
- 부작용 없음 확인: dev 전체 시나리오 회귀 (A → ready 복귀 → 클러치 추종 29Hz → 주행 → B 해제 → 대기 시드)

## 실행 모드 (dev/deploy) — 하드웨어 없는 개발 체계 (2026-08-25)

- `config.TELEOP_MODE` (환경 변수, 기본 **dev**): main.py가 모드를 보고 부품만 갈아 끼운다.
  조립 구조·서비스 배선은 두 모드가 완전히 같다 — dev에서 검증한 코드 경로가 그대로 deploy에서 돈다
    - **deploy에서 로봇 연결 실패는 SystemExit** (시뮬 폴백 금지 — "연결 안 됐는데 조용히 떠 있는" 현장 사고 방지).
      기존의 "연결 실패 → IK 시뮬 시각화" 암묵 폴백은 제거됨. 카메라만 선택 사항 유지
    - 기본값이 dev인 이유: 실로봇을 움직이는 쪽이 명시적 선택이어야 함. 내장 PC 기동 명령에 `TELEOP_MODE=deploy` 필수
- `robot_sdk/sim/robot.py`: `SimRobotBackend` — RobotBackend 완전 구현 (Rby1Backend 자리에 그대로 꽂힘)
    - 명령 추종 1차 지연: state += (target-state)·(1-exp(-dt/τ)), τ=`SIM_TRACKING_TAU`(0.15s)
      → 명령-측정 오차가 실제로 생겨 추적 오차 해제·속도 클램프 경로까지 dev에서 자극됨
      (실측: 클러치 추종 중 최대 오차 0.27 rad < 해제 임계 0.5 — 오탐 해제 없음)
    - 초기 자세 전 관절 0 (부팅 직후 실로봇처럼 ready와 다른 자세) → 제어 시작 시 "측정값 연속 출발 → ready 복귀" 관찰 가능
    - 주행은 SE2 오일러 적분으로 가상 베이스 포즈에 누적 (통계 로그로 확인)
    - 관절 목록은 rby1/robot.py의 `COMMAND_JOINT_NAMES` 재사용 (단일 출처)
- `robot_sdk/sim/gripper.py`: `SimGripper` — 가짜 호밍 지연(`SIM_GRIPPER_HOMING_S` 1s) 후 ready.
  준비 전 openness None → 상태 서비스의 대체 공급자 폴백 경로까지 실기기와 같은 순서로 자극
- `utils/camera/synthetic.py`: `SyntheticCamera` — FrameSource 구현. 벽(3m, 체커보드)+바닥+공전 구를
  광학 프레임 z-깊이로 합성 (PointCloudBuilder 역투영 규약과 일치). 640x480@30 실측 29.8fps, 정적 장면을
  피해서 프레임마다 유효 깊이가 달라지는 경로 확인 가능
- `tools/fake_vr_quest3.py`: Quest 대체 합성 포즈 송신기 (wss://host:8443/ws/pose, pose_reader.js와 동일 샘플 형식)
    - 시나리오 24초/사이클: idle → A(제어 시작) → ready 대기 → grip+양손 원운동(좌우 반주기 위상차 —
      좌우 뒤바뀜 회귀 육안 구분용) → 트리거 삼각 램프 → 스틱 전진·회전 → B(해제)
    - 사용법: `python tools/fake_vr_quest3.py [--host localhost] [--rate 50] [--cycles 1]` (컨테이너 내부)
    - **다운링크를 반드시 소비한다** (_consume 태스크): 읽지 않으면 TCP 배압으로 서버 송신 큐 드롭 유발
    - 클러치 체결 샘플에서 손 위치가 rest→원 시작점으로 점프하지만 매퍼가 체결 순간을 앵커로 잡으므로 무해
- 검증 (2026-08-25, dev 모드 전체 파이프라인): 업링크 48.6Hz → A → 시뮬 준비·스트림 → ready 복귀 완료
  → 30Hz 명령 전송(클램프 동작) → 주행 w=0.37 반영 → B 해제 → 대기 복귀. IK 3-4ms/solve.
  헤드리스 크로미움 스크린샷으로 로봇 조립 상태·아바타·포인트클라우드(합성 구)·HUD(sqz=1.0) 렌더링 확인

## 개발 PC 환경 (Desktop/vr-teleop, 2026-08-25)

- 컨테이너 `airlab_hw_teleop`이 옛 사본(`Desktop/school/teleop`, init 커밋에 정체)을 마운트하고 있었음
  → 현 저장소의 docker/에서 `docker compose up -d`로 재생성 (마운트가 현 저장소로 교체됨)
- 로컬 이미지는 08-23 09:35 빌드(모델 미내장·ENV 없음) — 모델이 저장소로 이관된 뒤로는 그대로 사용 가능
- dev 모드 기동 (이 PC):
  `docker exec -d airlab_hw_teleop bash -c 'cd /workspace && TELEOP_MODE=dev python -m teleop.main >> logs/web_api.log 2>&1'`
- 이 PC 파일 소유자는 uid 1000 (내장 PC의 1001과 다름) — 컨테이너 산출물 chown 시 1000:1000
- 헤드리스 크로미움은 이 컨테이너에도 설치해 둠 (렌더링 체크용, 컨테이너 재생성 시 재설치 필요)
- 서버 종료 시 `/proc` 스캔 패턴 주의: kill 명령 문자열 자체가 `teleop.main`을 포함하면 자기 자신을 죽인다
  → `case "$c" in python*teleop.main*)` 처럼 python 프로세스만 매칭할 것

## 단계 진행 현황

0. RBY1 내장 PC 이전 — 완료 (2026-08-23)
    - 개발 PC → 로봇 내장 PC로 서버 이전. `oss/` 런타임 의존 제거(모델을 이미지에 내장), GPU 예약 제거,
      snap docker 소켓 그룹 문제 해결, Quest 직결 이더넷 `quest-link`(10.42.0.1) 구성
    - 검증: 이미지 빌드 → 모델 로드(nq=14, visual 34) → HTTPS 8443/443·HTTP 301 → `/models/` GLB 서빙
      → WSS 모니터에서 robot_config·robot_state 30Hz·포인트클라우드 바이너리 수신 → D455 30 fps
    - 이로써 5단계(실로봇 제어)가 열렸다: gRPC 192.168.30.1:50051 컨테이너에서 도달 확인
1. 환경 구축 — 완료 (2026-08-23)
2. 프론트 뼈대 — 완료 (2026-08-23): Quest 3 실기기 body-tracking 취득 확인
3. 포즈 파이프라인 — 완료 (2026-08-23): 실기기 업링크 45Hz·데스크톱 마커·EXIT 버튼 확인
4. IK + 시뮬레이션 시각화 — 완료에 근접 (2026-08-23), **클러치 조작 체계 실기기 확인 대기**
    - 개인 PC 작업 환경이라 실로봇 연결 불가 → 가상 로봇(GLB) 시각화로 검증하는 구조 (데모 겸용)
    - 렌더링 좌표 회귀 2건 해결: FORCE_RIGHT_HANDED로 인한 씬 RH 오염(빈 화면), GLB 기저 보정 누락(로봇 분해)
    - 리타게팅 시행착오 끝에 **클러치 델타 방식으로 확정** (절대 매핑의 자세 규약 오류·스케일링 회귀는
      독립 감사로 원인 규명 — mapper/solver 항목 참고)
    - **조작 체계 (custom-teleop 계승)**: grip 쥔 동안 델타 추종 + 팔꿈치 방향 트래킹(body-tracking),
      트리거 = 그리퍼 개폐(시각화: 손가락 6-106mm), 오른손 A = ready pose 복귀(속도 제한 보간)
7. 포인트클라우드 — 완료 (2026-08-23, 데스크톱 뷰 실측: 13k점 7-10 Hz 수신·렌더링 확인, VR 실기기 확인 대기)
    - 순서를 앞당긴 이유: 개인 PC라 실로봇(5단계) 불가, 카메라는 있음
5. 제어 통합 — **구현 완료 (2026-08-23), 실기기 A 버튼 제어 시험 대기**
    - 읽기: robot_sdk 추상 인터페이스 + Rby1Backend + RobotStateService. 실로봇 24관절 측정값을 30Hz로 읽어
      전체 모델 FK로 시각화 (연결 192.168.30.1:50051, 방송 29Hz, 머리 각도 → 카메라 외부 파라미터 연동 확인)
    - 명령: RobotControlService(A=전원·서보·CM·스트림·ready, B=해제, 안전장치 4종) + Rby1Gripper(툴 플랜지 12V → 호밍 → 트리거 추종)
      + 모바일 베이스 스틱. 서버 기동·그리퍼 호밍까지 실기기 확인, 팔 제어는 가짜 백엔드·명령 빌더 조립까지만 검증
    - **실기기 첫 A 시험 결과 (2026-08-23, 해결)**: `enable_control_manager` 거부 — 로봇 로그 "Joint 23 (head_1)
      position -0.66 is below the lower limit -0.35". 원인은 CM_UNLIMITED_MODE를 False로 바꾼 것 —
      제한 모드는 활성화 시 관절 한계 검사를 하고, unlimited(원본 설정)는 건너뛴다. **True로 되돌려 해결**
      (실기기에서 활성화 성공 확인). 실패 시 로봇 로그 사유를 서버 로그에 옮겨 찍는 `_log_robot_reason()` 추가
    - 실기기 첫 시험 체크리스트: A → CM 활성 로그 → 토르소·팔이 ready로 천천히 이동 → grip 쥐고 추종 →
      트리거 방향(GRIPPER_INVERT) → 스틱 부호(MOBILE_*_SIGN) → B 해제. 추적 오차 해제가 잦으면 CONTROL_MAX_TRACKING_ERR 상향
    - 미구현: 토르소 머리 추종(custom-teleop의 head → torso Cartesian), 왼손 X 전원 차단, HUD 제어 상태 표시

## 환경 구축 이력

- 기존 `t0mark251/hardware-sdk:rby1-vr-teleop` 이미지는 Unity/VisionPro 시절 구성이라 미사용 (보존)
- 2026-08-23 `docker image prune -f`로 dangling 레이어 131GB 정리 (태그 이미지·컨테이너 영향 없음)
- 2026-08-23 RBY1 내장 PC로 이전. 저장소를 새로 클론한 환경이라 gitignore 대상(`oss/`, `logs/`,
  `web/certs/`)이 하나도 없었다 — 모델 자산은 이미지 내장으로 해결해 다시 없어질 일이 없다
- 2026-08-23 **작업 손실 사고와 복구**: 에이전트가 `git checkout -- .`를 실행해 마지막 커밋(15:08) 이후
  추적 파일의 모든 변경이 날아갔다 (code.md 포함). 복구 방법 (재사용 가능):
  `~/.claude/projects/-home-airlab/*.jsonl` 세션 기록에서 Bash 도구 호출을 시간순으로 추출 → 파일을 바꾸는
  블록(`cat > … <<EOF`, `python3 - <<PY` 패치)만 골라 `sg`/`docker`/`sleep`을 스텁으로 막고 순서대로 재실행.
  살아남은 untracked 파일과 바이트 단위 일치로 검증했다. 이후 규칙: **사용자 요청 없이 git 명령 금지** (agent.md)
