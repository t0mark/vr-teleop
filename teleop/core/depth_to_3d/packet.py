"""포인트클라우드 바이너리 패킷 규약 (서버 → 브라우저, WSS binary).

레이아웃 (리틀엔디언):
    uint32 magic  = 0x31444350 ('PCD1' ASCII, 메시지 식별)
    uint32 count  = N
    float64 t     = 서버 송신 시각(초, time.time)
    int16  xyz[N][3]  WebXR 좌표, 단위 mm (±32 m 범위, float32 대비 절반 크기)
    uint8  rgb[N][3]
헤더 16바이트 + 9N 바이트. 프론트의 web/src/net/pointcloud.js 디코더와 일치해야 한다.
"""

import struct
import time

import numpy as np

MAGIC = 0x31444350
_HEADER = struct.Struct("<IId")


def pack_pointcloud(points: np.ndarray, colors: np.ndarray) -> bytes:
    """(N,3) float32 m 점과 (N,3) uint8 색을 패킷 바이트열로 직렬화해 반환한다."""
    n = points.shape[0]
    # m → mm 정수화: 1 mm 분해능이면 시각화에 충분하고 대역은 float32의 절반
    xyz_mm = np.clip(np.rint(points * 1000.0), -32767, 32767).astype("<i2")
    header = _HEADER.pack(MAGIC, n, time.time())
    return header + xyz_mm.tobytes() + np.ascontiguousarray(colors, dtype=np.uint8).tobytes()
