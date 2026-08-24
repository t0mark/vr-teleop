# RBY1 Web VR Teleop

Quest 브라우저(WebXR + Babylon.js)에서 RBY1 휴머노이드를 원격 조작하는 웹 기반 teleop.
PCVR 없이 Quest 온디바이스 렌더링, 서버는 RBY1 내장 PC의 Docker 컨테이너 하나로 동작한다.

## 의존성

### 하드웨어

| 항목 | 비고 |
|---|---|
| Meta Quest 3 | `chrome://flags` WebXR Experiments 활성화, 설정 → 동작 추적 → 신체 추적 켜기 |
| USB-C 이더넷 어댑터 | Quest - 내장 PC 유선 직결. 내장 PC 측 NetworkManager 공유 모드(`quest-link`, 10.42.0.1) |
| Intel RealSense D455 | USB 3.x 포트 |
| RBY1 | 서버가 내장 PC에서 동작. 로봇 제어는 gRPC(192.168.30.1:50051) |

### 호스트 (RBY1 내장 PC)

- Docker + Docker Compose v2
- GPU 없음(i5-1250PE 내장 그래픽) → 영상 인코딩은 소프트웨어 경로를 사용한다

### 컨테이너 (`docker/Dockerfile`, 이미지 `t0mark251/rby1-web-teleop`)

- debian bookworm + micromamba, 2단계 빌드(모델 자산 취득 → 런타임)
- conda-forge: Python 3.11, **pinocchio 4.1.0**, **casadi 3.7.2**, numpy, scipy
  - Pinocchio는 반드시 conda-forge 빌드 사용 (PyPI wheel에는 `pinocchio.casadi`가 없음)
- pip: rby1-sdk 0.9.1, aiortc, websockets, aiohttp, pyrealsense2
- **RBY1 URDF·GLB 메시는 이미지에 내장** (`/opt/rby1/models/rby1a`). pip 패키지에 모델이 없어
  빌드 시 rby1-sdk 저장소 태그 `v0.9.1`에서 rby1a URDF 경로만 sparse checkout 한다 (약 14 MB).
  경로는 `RBY1_MODEL_ROOT` 환경 변수로 덮어쓸 수 있다
- 프론트엔드는 `web/vendor/`에 Babylon.js 9.22 + glTF 로더를 동봉 (오프라인 직결 환경 대비, 별도 설치 없음)

### 참조 저장소 (`oss/`, git 미포함, 런타임 의존성 아님)

- `custom-teleop`: 기존 Unity 기반 teleop. 제어 로직 이식 원본으로만 참조하며, 이식이 끝나면 삭제한다

## 사용법

### 1. 서버 기동 (컨테이너)

```bash
# 이미지 빌드 + 컨테이너 기동 (최초 1회, 이후에는 up -d만)
cd docker && docker compose up -d --build

# 자체 서명 인증서 생성 (최초 1회, web/certs/ 출력, 직결 IP 10.42.0.1이 SAN 기본값)
docker exec airlab_hw_teleop python -m teleop.net.web_api.gen_certs

# 서버 기동 (HTTPS 8443/443 + HTTP 80 리다이렉트 + WSS + D455 포인트클라우드)
docker exec -d airlab_hw_teleop bash -c 'cd /workspace && python -m teleop.main >> /workspace/logs/web_api.log 2>&1'

# 컨테이너가 만든 파일은 root 소유이므로 호스트에서 다루려면 소유권을 넘긴다
docker exec airlab_hw_teleop chown -R 1001:1001 /workspace/logs /workspace/web/certs

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
- 같은 주소에 접속하면 마우스 궤도 카메라의 데스크톱 뷰로 VR 포즈 마커·가상 로봇·포인트클라우드를 확인할 수 있다
- 내장 PC 로컬은 `https://localhost:8443`, 같은 망의 다른 PC는 내장 PC의 무선 IP로 접속한다
