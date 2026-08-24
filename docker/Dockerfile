# RBY1 웹 기반 VR Teleop 서버 이미지
# 구성: conda-forge Python 3.11 + Pinocchio/CasADi(IK) + rby1-sdk + aiortc(WebRTC) + pyrealsense2(D455)
# Pinocchio는 PyPI wheel(pin)에 casadi 빌드가 없어 conda-forge(micromamba) 배포판을 사용한다.

# SDK와 모델 자산의 릴리스를 하나로 고정한다 (URDF와 제어 API 버전 불일치 방지)
ARG RBY1_SDK_VERSION=0.9.1

# ── 1단계: RBY1 모델 자산 취득 ────────────────────────────────────────────────
# URDF·메시는 pip 패키지(rby1_sdk)에 포함되지 않고 저장소에만 있다.
# 워크스페이스에 클론해 두는 대신 이미지에 구워, 런타임 자산을 코드 저장소에서 분리한다.
FROM debian:bookworm-slim AS models
ARG RBY1_SDK_VERSION

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    git \
    && rm -rf /var/lib/apt/lists/*

# blob 필터 + sparse checkout으로 models/ 전체(419MB) 대신 rby1a URDF 경로만 받는다.
# 추리는 기준: IK(pinocchio)는 URDF XML만 파싱하고 메시를 열지 않으며, 브라우저 시각화는
# GLB만 사용한다. 따라서 URDF가 참조하는 dae(47MB)는 이미지에 넣지 않는다.
RUN git clone --depth 1 --filter=blob:none --sparse --branch "v${RBY1_SDK_VERSION}" \
        https://github.com/RainbowRobotics/rby1-sdk.git /src \
    && git -C /src sparse-checkout set models/rby1a/urdf \
    && mkdir -p /models/rby1a/urdf/meshes \
    && cp /src/models/rby1a/urdf/model.urdf /models/rby1a/urdf/ \
    && cp /src/models/rby1a/urdf/meshes/*.glb /models/rby1a/urdf/meshes/ \
    && rm -rf /src

# ── 2단계: 런타임 이미지 ──────────────────────────────────────────────────────
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
# - rby1-sdk: 로봇 gRPC 제어 / aiortc: WebRTC 영상 / websockets·aiohttp: WSS 업링크·HTTPS 정적 서빙
# - pyrealsense2: PyPI wheel 사용 (librealsense 소스 빌드 대신, D455 스트림 취득에 충분)
RUN pip install --no-cache-dir \
    "rby1-sdk==${RBY1_SDK_VERSION}" \
    aiortc \
    websockets \
    aiohttp \
    pyrealsense2

# 1단계에서 추린 모델 자산. config.RBY1_MODEL_ROOT의 기본값과 같은 경로에 두고,
# 환경 변수로도 노출해 다른 위치에 마운트할 때 코드 수정 없이 바꿀 수 있게 한다.
COPY --from=models /models /opt/rby1/models
ENV RBY1_MODEL_ROOT=/opt/rby1/models/rby1a

WORKDIR /workspace

CMD ["bash"]
