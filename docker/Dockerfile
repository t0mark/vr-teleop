# RBY1 웹 기반 VR Teleop 서버 이미지
# 구성: conda-forge Python 3.11 + Pinocchio/CasADi + Pink(팔 IK) + rby1-sdk + aiortc(WebRTC) + pyrealsense2(D455)
#       + Manus SDK(장갑, reference/manusSDK/Python을 추가 빌드 컨텍스트로 받는다) + dex-retargeting(손 리타게팅)
#       + Open3D(환경 메시 TSDF)
# Pinocchio(casadi 포함)는 conda-forge(micromamba) 배포판으로 설치한다.
# RBY1 모델(URDF·GLB)은 저장소 assets/rby1a를 마운트해 쓴다 (rby1-sdk와 같은 릴리스 태그 유지).

# 로봇 제어 API 버전 고정 (저장소 assets/rby1a와 같은 릴리스를 유지한다)
ARG RBY1_SDK_VERSION=0.9.1

FROM debian:bookworm-slim
ARG RBY1_SDK_VERSION

# curl·bzip2: micromamba 설치용 / libusb-1.0-0: pyrealsense2 wheel 런타임
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    bzip2 \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# micromamba 정적 바이너리 설치 (/opt/bin/micromamba)
RUN curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xj -C /opt bin/micromamba

# IK 핵심 스택 설치 (conda-forge: pinocchio 4.1.0·casadi 3.7.2, pinocchio.casadi 모듈 포함)
RUN /opt/bin/micromamba create -y -p /opt/venv -c conda-forge \
    python=3.11 \
    pinocchio=4.1.0 \
    casadi=3.7.2 \
    numpy \
    scipy \
    && /opt/bin/micromamba clean -a -y

# conda 환경을 기본 python으로 사용
# conda 환경의 libstdc++를 우선 로드 (torch가 먼저 import되면 시스템 libstdc++가 올라와 pinocchio가 깨진다)
ENV PATH=/opt/venv/bin:$PATH
ENV LD_LIBRARY_PATH=/opt/venv/lib

# 나머지 서버 의존성은 같은 환경에 pip으로 설치
# - rby1-sdk: 로봇 gRPC 제어 / aiortc: WebRTC DataChannel / aiohttp: HTTPS 정적 서빙·WebRTC 시그널링
# - pyrealsense2: D455 스트림 취득 (PyPI wheel)
# - pymodbus: Inspire RH56E2 손 Modbus TCP 통신 (3.x API)
RUN pip install --no-cache-dir \
    "rby1-sdk==${RBY1_SDK_VERSION}" \
    aiortc \
    aiohttp \
    pyrealsense2 \
    "pymodbus==3.16.2"

# Manus 장갑 SDK (Python, Integrated 모드로 동글 직접 접속)
# - libzmq5·libgomp1: libManusSDK 네이티브 런타임 / cffi: 바인딩 런타임
# - 패키지(네이티브 바이너리 포함)는 compose의 추가 빌드 컨텍스트(manus_sdk)에서 site-packages로 폴더째 복사한다
RUN apt-get update && apt-get install -y --no-install-recommends \
    libzmq5 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir cffi
COPY --from=manus_sdk manus_sdk /opt/venv/lib/python3.11/site-packages/manus_sdk

# 환경 메시 재구성 (TSDF, teleop/perception/tsdf_worker.py)
# - libegl1·libgl1: open3d import용 네이티브 런타임
# - open3d 0.20.0: 환경 메시 TSDF 적분·메시 추출
RUN apt-get update && apt-get install -y --no-install-recommends \
    libegl1 \
    libgl1 \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir "open3d==0.20.0"

# 팔·손 리타게팅
# - pin-pink·qpsolvers[daqp]: 팔 미분 IK (teleop/ik/ik_solver.py)
# - torch(CPU): dex-retargeting 최적화 의존성 / six: dex-retargeting 내장 URDF 파서 의존성 (패키지 미선언)
# - dex_retargeting: 손 리타게팅 DexPilot (teleop/glove/hand_retargeter.py)
RUN pip install --no-cache-dir "torch==2.15.0" --index-url https://download.pytorch.org/whl/cpu \
    && pip install --no-cache-dir \
    "pin-pink==4.4.0" \
    "qpsolvers[daqp]==4.13.0" \
    "dex_retargeting==0.5.0" \
    six

WORKDIR /workspace

CMD ["bash"]
