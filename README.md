# RBY1 Web VR Teleop

Quest 브라우저(WebXR + Babylon.js)에서 RBY1 휴머노이드를 원격 조작하는 웹 기반 teleop.

## 실행 방법

### 1. 컨테이너·인증서 준비 (최초 1회)

```bash
cd docker && docker compose up -d --build
docker exec airlab_hw_teleop python /workspace/tools/make_https_cert.py
```

### 2. 서버 기동

```bash
# 개발 (기본값): 시뮬 로봇 + 합성 카메라 — 하드웨어 불필요
docker exec -d airlab_hw_teleop bash -c 'cd /workspace && TELEOP_MODE=dev python -m teleop.main >> logs/web_api.log 2>&1'

# 운용 (로봇 내장 PC): 실로봇 + D455
docker exec -d airlab_hw_teleop bash -c 'cd /workspace && TELEOP_MODE=deploy python -m teleop.main >> logs/web_api.log 2>&1'
```

기동 후에는 PC 브라우저 데스크톱 뷰의 좌상단 모드 버튼으로 dev ↔ deploy 런타임 전환 가능.

```bash
# 로그 확인
tail -f logs/web_api.log

# 종료
docker exec airlab_hw_teleop bash -c 'for p in /proc/[0-9]*; do c=$(tr "\0" " " < $p/cmdline 2>/dev/null); case "$c" in python*teleop.main*) kill ${p#/proc/};; esac; done'
```

### 3. 접속

| 클라이언트 | 주소 | 비고 |
|---|---|---|
| Quest (조작) | `10.42.0.1` (유선 직결) | 최초 1회 인증서 예외 허용 → `Enter VR` |
| PC 브라우저 (모니터링) | `https://localhost:8443` 또는 내장 PC IP | 모드 전환 버튼(좌상단), dev 수동 조작 패널(우상단) |

### 4. 조작

- 오른손 **A** = 제어 시작 (ready pose 이동 후 추종) / 오른손 **B** = 제어 해제
- **grip**을 쥔 동안 손 델타 추종 / **트리거** = 그리퍼 개폐 / 오른손 **스틱** = 모바일 베이스
- dev 모드에서 VR 없이: 웹 수동 조작 패널 또는 합성 시나리오
  ```bash
  docker exec airlab_hw_teleop bash -c 'cd /workspace && python tools/fake_vr_quest3.py --cycles 1'
  ```
