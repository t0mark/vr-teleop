# RBY1 Web VR Teleop

Quest 브라우저(WebXR + Babylon.js)에서 RBY1 휴머노이드를 원격 조작하는 웹 기반 teleop.
조작자는 Quest 3를 쓰고, 손등에 부착한 Quest 3 컨트롤러로 손 위치를, Manus 장갑으로 손가락을 측정한다.

## 실행 방법

### 1. 최초 1회 준비

Manus SDK(MANUS 배포 패키지)를 `reference/manusSDK/`에 풀어 둔다. 이미지 빌드 시
`reference/manusSDK/Python/manus_sdk`가 이미지에 설치된다 (`docker/compose.yml`의 `additional_contexts`).

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

### 다지 손 (Inspire RH56E2, deploy 모드)

양손은 외부 24V 전원으로 구동되고, 내장 PC의 USB-LAN 어댑터를 통해 Modbus TCP로 접속한다
(`teleop/robot/inspire_hand.py`, 설정은 `config.py`의 `HAND_*`).

| 손 | 주소 |
|---|---|
| 오른손 | `192.168.11.211:6000` |
| 왼손 | `192.168.11.210:6000` |

호스트 IP `192.168.11.222`는 `/etc/NetworkManager/dispatcher.d/90-inspire-hand-ip`가 USB-LAN 어댑터에 추가한다.
손 접속이 실패해도 팔 제어는 그대로 동작하고, 손 스레드가 백그라운드에서 재접속을 반복한다.

### 3. 접속

| 클라이언트 | 주소 | 비고 |
|---|---|---|
| Quest (조작) | `10.42.0.1` (유선 직결) | 최초 1회 인증서 예외 허용 → `Enter VR` |
| PC 브라우저 (모니터링) | `https://localhost:8443` 또는 내장 PC IP | 모드 전환 버튼(좌상단), dev 수동 조작 패널(우상단) |

### 4. 조작

장비: Quest 3 헤드셋 + 손등에 부착한 Quest 3 컨트롤러(양손) + Manus 장갑(양손)
- 팔은 컨트롤러가 추적되는 동안 항상 컨트롤러 위치·자세 델타를 추종한다 (클러치 델타 방식, 1m = 1m)
- 로봇 손가락은 Manus 장갑의 손가락 굽힘·엄지 회전을 따라간다
- body-tracking으로 로봇 팔꿈치 자세를 유도한다
- **제어 시작**: `Enter VR` 후 2초 뒤 자동 시작 (ready pose 이동 후 추종)
- **제어 종료**: VR을 종료(Quest 시스템 메뉴)하면 로봇이 ready pose로 복귀한 뒤 제어를 해제한다
  - 브라우저 자체가 꺼지는 등 종료 신호를 못 보내면 업링크 두절(3초)로 그 자리에서 해제된다
- Quest 설정에서 손 추적이 켜져 있어도 이 페이지는 컨트롤러 입력만 쓴다

### 5. Manus 장갑

서버가 SDK Integrated 모드로 로봇 PC에 꽂힌 Manus 동글에 직접 접속한다 (MANUS Core 불필요,
`teleop/glove/manus_glove.py`, 설정은 `config.py`의 `MANUS_*`).
- 동글 라이선스에 Integrated 기능이 있어야 한다
- 페어링되지 않은 장갑이 보이면 서버가 자동으로 페어링한다 (장갑 전원을 켜 두면 된다)
- 장갑 접속이 실패해도 팔 제어는 그대로 동작한다 (손가락은 마지막 값 유지)
- 손가락 정규화 범위는 `MANUS_*_RANGE_DEG`로 보정한다. 로그에 손별 원시 각도가 5초마다 남는다:
  ```bash
  grep manus_glove logs/web_api.log | tail
  ```

dev 모드에서 VR 없이: 웹 수동 조작 패널 또는 합성 시나리오
```bash
docker exec airlab_hw_teleop bash -c 'cd /workspace && python tools/fake_vr_client.py --cycles 1'
```
