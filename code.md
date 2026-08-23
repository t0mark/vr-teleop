# 작업 기록

## docker/Dockerfile

- 이미지: `t0mark251/rby1-web-teleop:latest` (1.89GB)
- 구성: debian:bookworm-slim + micromamba(/opt/bin/micromamba) + conda 환경(/opt/venv)
- 핵심 스택 설치 경로가 이원화되어 있음
    - conda-forge: python 3.11, pinocchio 4.1.0, casadi 3.7.2, numpy, scipy
    - pip: rby1-sdk(0.10.0 설치됨), aiortc, websockets, aiohttp, pyrealsense2
- 특이사항
    - **Pinocchio를 pip(pin 패키지)으로 설치하면 안 됨**: PyPI wheel은 전 버전(2.x-4.x)에서 casadi 지원 없이 빌드되어 `pinocchio.casadi`(cpin) import 불가. conda-forge 빌드만 포함. IK가 cpin에 의존하므로 conda 필수
    - librealsense 소스 빌드 대신 pyrealsense2 PyPI wheel 사용 (D455 취득에 충분). 문제 시 최소 소스 빌드로 회귀
    - `/opt/venv/bin`을 PATH 앞에 두어 컨테이너의 기본 `python` = conda 환경
    - GStreamer(NVENC)는 미포함. 영상 단계(6단계)에서 aiortc 소프트웨어 인코딩이 부족하면 추가

## docker/compose.yml

- 사용법: `cd docker && docker compose up -d`, 진입은 `docker exec -it airlab_hw_teleop bash`
- 컨테이너: `airlab_hw_teleop`, 상시 기동, 저장소 루트 → /workspace 마운트, network_mode: host
- D455: **`/dev:/dev` 마운트** + device_cgroup_rules(c 189/81). GPU: nvidia 예약 설정
    - pyrealsense2 PyPI wheel은 **V4L2 백엔드**라 `/dev/bus/usb`만으로는 장치가 0대로 잡힘 (libusb 백엔드 아님).
      `/dev/video*`가 필요하고 번호가 재연결마다 바뀌어 /dev 전체를 노출. 열 수 있는 장치는 cgroup 규칙으로 제한됨
- 특이사항
    - `name: rby1-web-teleop` 프로젝트 이름 고정 필수 (타 프로젝트 orphan 오인 방지)
    - 컨테이너(root)가 마운트 볼륨에 만든 파일은 `chown -R 1000:1000` 필요

## check/ 검증·디버그 스크립트 (gitignore됨)

- `env_check.py`: 의존성 5종 검증 (rby1a nq=30, rby1m nq=34 로드 확인)
- `transforms_check.py`: 좌표 변환 수치 검증 (축 매핑·회전 일관성·SE3 왕복)
- `pose_pipeline_check.py`: WSS 업링크 → 변환 → 모니터 중계 검증 (서버 기동 필요)
- `ik_check.py`: IK 단독 검증 — 연속 경로 추종 평균 1.2mm/최대 5.3mm, 팔꿈치 방향 정렬, 18ms/solve
- `mapper_check.py`: 리타게팅 yaw 불변성 검증 (사용자 방향 무관성)
- `ik_pipeline_check.py`: 포즈 입력 → IK → robot_state 방송 엔드투엔드 (서버 기동 필요)
    - 주의: 모니터 소켓은 계속 읽어야 함 (안 읽으면 큐에 과거 상태가 쌓여 오래된 자세를 읽음)
    - 로그의 "목표 잔차"는 컨트롤러 원시 좌표 대비 값 — 가슴 오프셋 매핑 때문에 0이 아닌 게 정상
- `cdp_probe.py`: **프론트 원격 디버깅 도구**. 헤드리스 크롬(remote-debugging-port=9333)에 CDP로 붙어
  JS 표현식 평가. 사용: 호스트에서 `google-chrome --headless=new --ignore-certificate-errors
  --enable-unsafe-swiftshader --remote-debugging-port=9333 <URL>` 실행 후
  `docker exec airlab_hw_teleop python /workspace/check/cdp_probe.py '<JS식>'`
    - 페이지의 `window.__teleop` 훅으로 engine/scene/robotView 접근 가능 (main.js에서 노출)
    - 스크린샷: `--shot <경로.png>` 옵션 (표현식으로 카메라를 옮긴 뒤 캡처 가능). 렌더링 디버깅에 매우 유효
    - 호스트 크롬은 `--user-data-dir`를 스크래치 경로로 주고, 끝나면 프로세스·프로필 삭제

## RBY1 URDF·메시 관련

- 위치: `oss/rby1-sdk/models/rby1a/urdf/model.urdf`, 메시는 같은 폴더 meshes/ (dae + 동명 glb 전부 존재)
- **표준 urdfdom으로 원본 로드 불가**: wheel 관절 limit에 effort 누락 → `robot_model.load_fixed_urdf_xml()`이 보정 후 `buildModelFromXML` 사용. collision capsule 에러 출력은 비치명(무시)
- **GLB 정점 데이터는 URDF 링크 프레임(z-up) 그대로** (bbox를 URDF collision과 대조해 확인).
  glb 내부 루트 노드는 전부 identity — 축 변환이 파일에 안 구워져 있음
- 팔 체인: arm_0-2(어깨 3축) → arm_3(팔꿈치) → arm_4-6(손목 3축), 손목 링크 = link_*_arm_6
- ready pose(기존 예제 Right-A 자세): 토르소 [0,45,-80,45,0,0]°(크라우치), 오른팔 [0,-15,0,-120,0,30,0]°, 왼팔 [0,15,0,-120,0,30,0]°,
  **머리 [0, 0.4] rad** (custom-teleop 기본 0.7은 바닥을 과하게 봐서 0.4≈23°로 완화)
- 카메라 장착점: URDF에 주석 처리된 `head_mount` 조인트 (link_head_2 + xyz 0.022 0 0.04)를 삼각대 나사
  마운트로 보고, depth 광학 중심 = 나사 + (0.0158, 0.0475, 0.0145) (realsense-ros `_d455.urdf.xacro` 값)

## teleop/ 백엔드 (4단계 시점)

- `main.py`: 조립 — RobotModel + ArmIkSolver + TargetMapper + PoseHub + IkService + WebAPIServer
    - 기동: `docker exec -d airlab_hw_teleop bash -c 'cd /workspace && python -m teleop.main >> /workspace/logs/web_api.log 2>&1'`
    - 종료: 컨테이너에 pgrep/pkill 없음 → `for p in /proc/[0-9]*; do grep -q teleop.main $p/cmdline 2>/dev/null && kill ${p#/proc/}; done`
- `utils/config.py`: 경로·포트·IK 파라미터(가중치·주기·스텝 제한)·카메라 장착·포인트클라우드 파라미터 일원화
- `utils/transforms.py`: WebXR↔로봇 좌표 변환 (양방향), 샘플 전체 변환
- `net/web_api/server.py`: HTTPS(8443/443) + HTTP 리다이렉트(80) + WSS 라우트 + `/models/` GLB 정적 서빙
    - **모든 응답 Cache-Control: no-cache** — 브라우저 휴리스틱 캐시가 옛 JS를 계속 서빙해
      수정이 반영 안 되는 문제가 있었음 (이번 렌더링 버그 디버깅을 크게 꼬이게 한 원인 중 하나)
- `net/web_api/pose_hub.py`: `PoseHub` — 업링크 수신·변환·보관, robot_config 접속 시 1회 전송,
  `broadcast_state()`(JSON)·`broadcast_bytes()`(바이너리)로 전 클라이언트(VR+모니터)에 방송
    - **송신 격리 구조 (2026-08-23 장애 후 도입)**: 클라이언트마다 큐(깊이 8) + 전용 writer 태스크.
      방송은 큐에 넣기만 하고 블로킹 없음. 큐가 차면 오래된 메시지 드롭(최신 우선), 드롭 수는 5초 로그에 누적 표시
    - 장애 내용: 한 클라이언트 소켓이 읽지 않아 `await send`가 영구 대기 → 같은 `_send_to`를 쓰던 IK 루프와
      포인트클라우드 루프가 **동시에 멈춤**(크래시 아님, 로그 없음). Quest는 robot_config만 받고 robot_state를
      못 받아 링크가 전부 원점에 겹친 "스폰 이상"으로 보였음. 증상: 송신 Hz가 서서히 떨어지다 통계 로그가 사라짐
    - 소켓 단절 시 예외는 `ConnectionResetError`가 아니라 일반 `ConnectionError("Connection lost")`로 올 수 있음
      → writer는 Exception 전체를 잡아 종료
- `core/ik/service.py`·`core/depth_to_3d/service.py`: 루프 태스크에 done 콜백으로 예외 로그 (조용한 정지 방지)
- `core/ik/robot_model.py`: `RobotModel` — URDF 보정 로드, 양팔 14 DOF 축소 모델(나머지 ready 고정),
  IK 프레임 FK, visual 링크 34개 FK, visual 메타 파싱 (origin은 WebXR 기저로 변환해 전달)
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
- `core/depth_to_3d/`: `PointCloudBuilder`(builder.py) → `VoxelAccumulator`(accumulator.py) → `pack_pointcloud`(packet.py)
  → `PointCloudService`(service.py)
    - **복셀 누적(서버)**: 0.02 m 복셀 키(int64 패킹)로 병합, 같은 복셀은 최신 프레임 점만 유지(중복 표면 제거),
      20프레임(PCD_ACCUM_FRAMES) 동안 미관측이면 소멸. 새 점을 앞에 두고 `np.unique(return_index)`의
      첫 등장 유지 성질로 "최신 우선"을 구현. 처리 0.7 ms. 패킷 1개 = 누적 전체 집합
    - 누적을 서버에 둔 이유: 클라이언트 링 버퍼 방식은 같은 표면이 N번 겹쳐 그려지고, 클라이언트에서
      복셀 중복을 풀려면 슬롯 만료 시 재출현 깜빡임 문제가 있음. 서버 병합이 의미·구현 모두 단순
    - 역투영: 정규화 좌표 격자 (u-ppx)/fx 를 1회 계산, 스트라이드 4 서브샘플(최대 19,200점), 깊이 0.2-4.0 m만 유지
    - 외부 파라미터는 `RobotModel.camera_pose()` — 머리 링크(config.CAMERA_MOUNT_LINK) FK × 장착 오프셋·광학 축 대응
      (config.CAMERA_MOUNT_POS/ROT). ready(머리 pitch 0.4) 기준 base 위치 약 [0.19, 0.05, 1.28], 광학축 23° 하향
    - 변환은 광학→로봇→WebXR(Cᵀ)을 3x3+오프셋 하나로 합성해 행렬곱 1회. 머리가 잠겨 있어 상수
      (머리 추종을 넣으면 매 프레임 camera_pose 재계산 필요)
    - 실기기 장착 후 오프셋 실측값으로 CAMERA_MOUNT_POS 보정할 것 (가상 로봇의 카메라 박스도 같은 값으로 이동)
    - `RobotModel.camera_visual_config()`가 장착 정보를 WebXR 기저로 바꿔 robot_config.camera로 1회 전송
    - 패킷: `PCD1` 매직 + count + float64 t + **int16 xyz(mm)** + uint8 rgb = 16 + 9N 바이트
      (float32 대비 절반, 13k점 기준 약 120 KB, 10 Hz → 1.2 MB/s 실측)
    - 처리 1.3 ms/프레임(numpy) → GPU 불필요 판단 (cupy/torch 미설치 유지). voxel 다운샘플 대신 스트라이드
      (균일하고 비용 0, 해상도 조절은 PCD_STRIDE)
    - 방송은 `PoseHub.broadcast_bytes()` (VR 업링크 소켓 + 모니터 소켓 모두에 바이너리 프레임)

## web/ 프론트엔드 — 렌더링 좌표 규약 (중요, 어기면 로봇이 분해되어 보임)

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
- `src/render/pose_markers.js`: 데스크톱 포즈 마커
- `src/net/pointcloud.js`: 패킷 디코더 (헤더 16바이트라 Int16Array 뷰를 복사 없이 생성)
- `src/render/pointcloud_view.js`: `PointCloudView` — 점 메시 1개, 버퍼는 (수신 점수 × 1.5) 용량으로 할당 후
  `updateVerticesData`로 덮어씀. 남는 슬롯은 NaN(그려지지 않음). z-거울만 적용하고
  `ROBOT_OFFSET`(robot_view.js에서 export) 아래 배치 → 로봇 base 기준 위치가 그대로 맞음
    - 누적·중복 제거는 서버 담당 (클라이언트 링 버퍼 누적은 중복 렌더링 때문에 폐기)
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
    - **Meta body 조인트 의미 (계층: Shoulder→Scapula→ArmUpper→ArmLower)**:
      `*-shoulder`=쇄골 관절(몸 안쪽), `*-arm-upper` 원점=어깨 볼(상완골 시작), `*-arm-lower` 원점=팔꿈치.
      어깨 기준점으로 *-shoulder를 쓰면 팔꿈치 방향이 안쪽으로 편향되어 robot arm_1이 한계(+1°)에
      고착되는 문제가 있었음 (매퍼는 arm-upper 사용, shoulder는 폴백)
    - 서버 진단 로그: 5초마다 한계 도달 관절 목록 + shoulder/arm-upper 좌우 간격 실측 (의미 검증용)
- 디버깅 훅: `window.__teleop` = { engine, scene, robotView }

## Quest 직결 네트워크 (확정)

- PC USB 이더넷을 NetworkManager "다른 컴퓨터에 공유" 모드 → PC 10.42.0.1 고정, Quest DHCP 자동
- 접속: Quest 브라우저에서 `10.42.0.1` 입력 (80→HTTPS 리다이렉트, 443 별칭도 동작)
- 평문 http를 8443에 보내면 무응답 끊김(ERR_EMPTY_RESPONSE, 로그 없음) — 진단 시 최우선 의심
- Quest 필수 설정: chrome://flags "WebXR Experiments" + 설정→동작 추적→신체 추적
- 서버 로그: `logs/web_api.log`

## 단계 진행 현황

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
5. 제어 통합 — 미착수. robot_sdk 추상 인터페이스(base.py) 작성 후 시뮬/rby1 백엔드 교체 구조 예정
    - 브리지 준비물: robot_state의 q(joint_names로 이름 매핑 필수)·gripper(0-1), ready 복귀는 서비스에 이미 있음

## 환경 구축 이력

- 기존 `t0mark251/hardware-sdk:rby1-vr-teleop` 이미지는 Unity/VisionPro 시절 구성이라 미사용 (보존)
- 2026-08-23 `docker image prune -f`로 dangling 레이어 131GB 정리 (태그 이미지·컨테이너 영향 없음)
