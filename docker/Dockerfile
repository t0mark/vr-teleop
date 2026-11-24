# RBY1 웹 기반 VR Teleop 서버 이미지
# 구성: conda-forge Python 3.11 + Pinocchio/CasADi(IK) + rby1-sdk + aiortc(WebRTC) + pyrealsense2(D455)
#       + Manus SDK(장갑, reference/manusSDK/Python을 추가 빌드 컨텍스트로 받는다) + Open3D(환경 메시 TSDF)
# Pinocchio는 PyPI wheel(pin)에 casadi 빌드가 없어 conda-forge(micromamba) 배포판을 사용한다.
# RBY1 모델(URDF·GLB)은 저장소 assets/rby1a에 포함되어 있어 이미지에는 넣지 않는다 —
# SDK를 올릴 때는 pip 버전과 같은 태그의 모델로 assets/rby1a를 함께 갱신할 것 (버전 불일치 주의).

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

# IK 핵심 스택은 conda-forge로 설치 (pinocchio.casadi 모듈은 conda-forge 빌드에만 포함)
# 검증된 버전 고정: pinocchio 4.1.0 / casadi 3.7.2
RUN /opt/bin/micromamba create -y -p /opt/venv -c conda-forge \
    python=3.11 \
    pinocchio=4.1.0 \
    casadi=3.7.2 \
    numpy \
    scipy \
    && /opt/bin/micromamba clean -a -y

# conda 환경을 기본 python으로 사용
ENV PATH=/opt/venv/bin:$PATH

# 나머지 서버 의존성은 같은 환경에 pip으로 설치
# - rby1-sdk: 로봇 gRPC 제어 / aiortc: WebRTC DataChannel / aiohttp: HTTPS 정적 서빙·WebRTC 시그널링
# - pyrealsense2: PyPI wheel 사용 (librealsense 소스 빌드 대신, D455 스트림 취득에 충분)
# - pymodbus: Inspire RH56E2 손 Modbus TCP 통신 (3.x API: device_id 키워드, 3.16.2에서 확인)
RUN pip install --no-cache-dir \
    "rby1-sdk==${RBY1_SDK_VERSION}" \
    aiortc \
    websockets \
    aiohttp \
    pyrealsense2 \
    "pymodbus==3.16.2"

# Manus 장갑 SDK (Python, Integrated 모드로 동글 직접 접속)
# - libzmq5·libgomp1: libManusSDK 네이티브 런타임 / cffi: 바인딩 런타임
# - 패키지는 compose의 추가 빌드 컨텍스트(manus_sdk = reference/manusSDK/Python)에서 site-packages로 복사한다
#   (setup.py는 linux 네이티브 바이너리를 package_data에 포함하지 않아 pip 설치 대신 폴더 그대로 복사)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libzmq5 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir cffi
COPY --from=manus_sdk manus_sdk /opt/venv/lib/python3.11/site-packages/manus_sdk

# 환경 메시 재구성 (TSDF, teleop/perception/tsdf_worker.py)
# - libegl1·libgl1: open3d import 시 필요한 네이티브 런타임 (렌더링은 하지 않음)
# - open3d 0.20.0: 기존 numpy·scipy 버전을 바꾸지 않는 것을 확인한 버전
RUN apt-get update && apt-get install -y --no-install-recommends \
    libegl1 \
    libgl1 \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir "open3d==0.20.0"

WORKDIR /workspace

CMD ["bash"]
