# RBY1 웹 기반 VR Teleop 서버 이미지
# 구성: conda-forge Python 3.11 + Pinocchio/CasADi(IK) + rby1-sdk + aiortc(WebRTC) + pyrealsense2(D455)
# Pinocchio는 PyPI wheel(pin)에 casadi 빌드가 없어 conda-forge(micromamba) 배포판을 사용한다.
FROM debian:bookworm-slim

# curl·bzip2: micromamba 설치용 / libusb-1.0-0: pyrealsense2 wheel 런타임 / git: oss 참조용
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    bzip2 \
    git \
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
    "rby1-sdk>=0.7.0" \
    aiortc \
    websockets \
    aiohttp \
    pyrealsense2

WORKDIR /workspace

CMD ["bash"]
