# RBY1 Web VR Teleop

Quest 브라우저(WebXR + Babylon.js)에서 RBY1 휴머노이드를 원격 조작하는 웹 기반 teleop.

## 실행 방법

### 1. 최초 1회 준비

```bash
cd docker && docker compose up -d --build
docker exec airlab_hw_teleop python /workspace/tools/make_https_cert.py
```

부팅 시 자동으로 켜질 모드는 `docker/.env`(gitignore, 호스트별 파일)의 `TELEOP_MODE`로 정한다.
파일이 없으면 기본값은 `dev`(시뮬 로봇 + 합성 카메라, 하드웨어 불필요)이다.

```bash
# docker/.env (로봇 내장 PC 등 실물 하드웨어가 붙어 있는 호스트에서만 작성)
echo "TELEOP_MODE=deploy" > docker/.env
```

### 2. 서버 기동·재기동

`docker compose up -d`로 컨테이너를 띄우면 그 안에서 `teleop.main`이 곧바로 실행된다(추가 명령 불필요).
컨테이너에는 `restart: unless-stopped` 정책이 걸려 있어, PC를 재부팅하거나 서버 프로세스가
죽어도(예: 로봇 gRPC가 아직 준비되지 않아 초기 접속 실패) dockerd가 자동으로 다시 띄운다.
단, `docker stop`/`docker compose stop`으로 직접 멈춘 경우에는 재부팅해도 다시 켜지지 않는다.

기동 후에는 PC 브라우저 데스크톱 뷰의 좌상단 모드 버튼으로 dev ↔ deploy 런타임 전환도 가능하다
(이때는 `docker/.env`를 건드릴 필요 없이 그 세션에서만 즉시 바뀐다).

```bash
# 로그 확인
tail -f logs/web_api.log

# 재시작 (설정 변경 후 반영)
docker restart airlab_hw_teleop

# 종료 (재부팅해도 다시 켜지 않으려면)
cd docker && docker compose stop
```

### 3. 접속

| 클라이언트 | 주소 | 비고 |
|---|---|---|
| Quest (조작) | `10.42.0.1` (유선 직결) | 최초 1회 인증서 예외 허용 → `Enter VR` |
| PC 브라우저 (모니터링) | `https://localhost:8443` 또는 내장 PC IP | 모드 전환 버튼(좌상단), dev 수동 조작 패널(우상단) |

### 4. 조작

- 오른손 **A** = 제어 시작 (ready pose 이동 후 추종), 제어 중에 누르면 ready pose로 복귀 / 오른손 **B** = 제어 해제
- **grip**을 쥔 동안 손 델타 추종 / **트리거** = 그리퍼 개폐 / 오른손 **스틱** = 모바일 베이스
- dev 모드에서 VR 없이: 웹 수동 조작 패널 또는 합성 시나리오
  ```bash
  docker exec airlab_hw_teleop bash -c 'cd /workspace && python tools/fake_vr_quest3.py --cycles 1'
  ```
