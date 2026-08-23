# RBY1 Web VR Teleop

Quest 브라우저(WebXR + Babylon.js)에서 RBY1 휴머노이드를 원격 조작하는 웹 기반 teleop.
PCVR 없이 Quest 온디바이스 렌더링, 서버는 Docker 컨테이너 하나로 동작한다.

## 의존성

### 하드웨어

| 항목 | 비고 |
|---|---|
| Meta Quest 3 | `chrome://flags` WebXR Experiments 활성화, 설정 → 동작 추적 → 신체 추적 켜기 |
| USB-C 이더넷 어댑터 | Quest - PC 유선 직결 (PC 측 NetworkManager "다른 컴퓨터에 공유" 모드, PC = 10.42.0.1) |
| Intel RealSense D455 | USB 3.x 포트 |
| NVIDIA GPU (선택) | compose에 예약만 되어 있음, 현재 단계에서는 미사용 |
| RBY1 | 내장 PC 이전 후 gRPC(192.168.30.1) 연결 예정 |

### 호스트

- Docker + Docker Compose v2
- NVIDIA Container Toolkit (GPU 예약용)

### 컨테이너 (`docker/Dockerfile`, 이미지 `t0mark251/rby1-web-teleop`)

- debian bookworm + micromamba
- conda-forge: Python 3.11, **pinocchio 4.1.0**, **casadi 3.7.2**, numpy, scipy
  - Pinocchio는 반드시 conda-forge 빌드 사용 (PyPI wheel에는 `pinocchio.casadi`가 없음)
- pip: rby1-sdk, aiortc, websockets, aiohttp, pyrealsense2
- 프론트엔드는 `web/vendor/`에 Babylon.js 9.22 + glTF 로더를 동봉 (오프라인 직결 환경 대비, 별도 설치 없음)

### 참조 저장소 (`oss/`, git 미포함)

- `rby1-sdk`: RBY1 URDF·메시 (`models/rby1a/`)
- `rby1-examples-vr-teleop`, `custom-teleop`: 기존 Unity 기반 teleop (제어 로직 이식 원본)

## 사용법

### 1. 서버 기동 (컨테이너)

```bash
# 이미지 빌드 + 컨테이너 기동 (최초 1회, 이후에는 up -d만)
cd docker && docker compose up -d

# 자체 서명 인증서 생성 (최초 1회, web/certs/ 출력, 직결 IP 10.42.0.1이 SAN 기본값)
docker exec airlab_hw_teleop python -m teleop.net.web_api.gen_certs

# 서버 기동 (HTTPS 8443/443 + HTTP 80 리다이렉트 + WSS + D455 포인트클라우드)
docker exec -d airlab_hw_teleop bash -c 'cd /workspace && python -m teleop.main >> /workspace/logs/web_api.log 2>&1'

# 로그 확인 / 종료
tail -f logs/web_api.log
docker exec airlab_hw_teleop bash -c 'for p in /proc/[0-9]*; do grep -q teleop.main $p/cmdline 2>/dev/null && kill ${p#/proc/}; done'
```

D455가 없으면 포인트클라우드만 비활성화되고 나머지는 정상 기동한다.

### 2. 클라이언트 접속

**Quest (조작)**
1. 이더넷 직결 후 Quest 브라우저에서 `10.42.0.1` 입력 (HTTPS로 자동 리다이렉트), 최초 1회 인증서 예외 허용
2. `Enter VR` → 눈앞의 가상 로봇이 IK 결과를 따라 움직임
3. 조작: grip을 쥔 동안 손 델타 추종 / 트리거 = 그리퍼 개폐 / 오른손 A = ready pose 복귀 / HUD EXIT 버튼 = 세션 종료

**PC 브라우저 (모니터링)**
- 같은 주소(`https://localhost:8443` 또는 `https://10.42.0.1`)에 접속하면 마우스 궤도 카메라의 데스크톱 뷰로 VR 포즈 마커·가상 로봇·포인트클라우드를 확인할 수 있다
