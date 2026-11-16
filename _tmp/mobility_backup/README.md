# 주행(모바일 베이스) 기능 제거 백업

제거 전 원본 파일. 경로는 저장소 루트 기준 상대 경로를 그대로 유지한다.
예외: 저장소 루트 `README.md` 원본은 이 문서와 이름이 겹쳐 `README.project.md`로 저장했다.
복원은 해당 파일을 저장소 같은 경로로 복사하면 된다 (파일 단위로 서로 맞물려 있으므로 전체 복원 권장).

## 파일별 주행 관련 내용

### 서버 (Python)
- `teleop/utils/config.py` — `MOBILE_LINEAR_MAX`(0.3 m/s), `MOBILE_ANGULAR_MAX`(0.5 rad/s), `MOBILE_LINEAR_SIGN`, `MOBILE_ANGULAR_SIGN`, `MOBILE_STICK_DEADZONE`(0.15), `MOBILE_SMOOTHING_TAU`(0.3 s) 상수.
- `teleop/utils/inputs.py` — `hand_thumbstick(sample, side)`: 업링크 샘플에서 스틱 (x, y) 추출.
- `teleop/robot/robot_controller.py` — `_mobility` 상태, `_update_mobility()`: 오른손 스틱 → 데드존 → (전진 m/s, yaw rad/s) 목표 → 1차 필터. `_control_step`에서 `send_targets(cmd, mobility, dt)`로 전달, `_arm`에서 0으로 초기화, 통계 로그에 `주행 v/w` 출력.
- `teleop/robot/robot_interface.py` — `RobotBackend.send_targets(named, mobility, dt)` 시그니처, `base_pose()` 기본 메서드(None 반환).
- `teleop/robot/rby1_robot.py` — `send_targets`에서 `rby.SE2VelocityCommandBuilder()`(hold time·velocity·minimum time)를 만들어 `ComponentBasedCommandBuilder.set_mobility_command()`로 실음.
- `teleop/robot/sim_robot.py` — 가상 베이스 포즈 `_base`(x, y, yaw), `send_targets`에서 오일러 평면 적분, `base_pose()`, 통계 로그에 베이스 포즈 출력.
- `teleop/robot/robot_state_broadcaster.py` — `base_pose()`를 WebXR 기저로 변환해 robot_state 메시지의 `base: {pos, quat}` 필드로 방송 (`import math`는 이 용도 전용).
- `teleop/utils/transforms.py` — docstring 문구("트리거·스틱")만 해당.

### 웹 (JS/HTML)
- `web/src/xr/pose_reader.js` — 컨트롤러 샘플의 `thumbstick: [axes[2], axes[3]]` 필드.
- `web/src/manual_control.js` — `STICK_HOLD`, `ARROW_KEYS`, `_state.stick`, `setStick()`, 주행 홀드 버튼(mpFwd/mpBack/mpTurnL/mpTurnR) 바인딩, 방향키 홀드(`_heldArrows`) 합성, 샘플의 `thumbstick` 필드.
- `web/index.html` — 주행 버튼 행(▲▼◀▶, `.mp-hold`)과 CSS, 키 도움말 "방향키 주행".
- `web/src/render/robot_view.js` — `applyState`에서 `state.base`로 로봇 루트(`_root`) 이동·회전 (주행 시각화).
- `web/src/render/hud.js` — 손 진단 줄의 `stick(...)` 표시.
- `web/src/render/scene.js` — 주행 키와 충돌해 ArcRotateCamera 방향키(`keysUp/Down/Left/Right`) 바인딩 해제.

### 도구·문서
- `tools/fake_vr_quest3.py` — 시나리오 `mobile_stick`(18-21초) 구간, `_stick()`, 샘플의 `thumbstick` 필드 (제거 후 사이클은 24초 → 21초).
- `README.md` (→ `README.project.md`) — 조작 설명의 "오른손 스틱 = 모바일 베이스".
