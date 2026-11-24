"""서버 → 브라우저 mesh DataChannel 바이너리 패킷 규약 (리틀엔디언).
프론트의 web/src/net/packets.js 디코더와 일치해야 한다. 좌표는 WebXR 기저, 단위 mm(int16, ±32 m).

MSH1 (메시 블록 묶음, 환경·작업 영역 두 층):
    uint32 magic = 'MSH1' | uint32 flags | uint32 block_count | uint32 reserved | float64 t   (24바이트)
    블록 × block_count:
        int32 layer | int32 key[3] | uint32 nv | uint32 nt                                    (24바이트)
        int16 xyz[nv][3] | uint8 rgb[nv][3] | 0-3바이트 패딩(4바이트 정렬) | uint32 tri[nt][3]
    layer: 0 = 환경, 1 = 작업 영역. 블록은 (layer, key)로 식별한다.
    flags bit0 = 스냅샷 시작: 클라이언트는 이 패킷을 적용하기 전에 기존 블록을 모두 지운다.
    nv = 0인 블록은 삭제를 뜻한다.
    모든 배열 시작 오프셋이 원소 크기에 정렬되므로 브라우저가 복사 없이 TypedArray 뷰를 만들 수 있다.
"""

import logging
import struct
import time

import numpy as np

from teleop.utils.transforms import robot_points_to_webxr_matrix

# MSH1 매직('MSH1' 리틀엔디언)과 스냅샷 시작 플래그
MESH_MAGIC = 0x3148534D
MESH_FLAG_SNAPSHOT = 0x1

logger = logging.getLogger("packets")

# 패킷 헤더 (magic, flags, block_count, reserved, t)와 블록 헤더 (layer, key[3], nv, nt)
_MESH_HEADER = struct.Struct("<IIIId")
_BLOCK_HEADER = struct.Struct("<iiiiII")

# 로봇 base → WebXR 기저 변환 (행 벡터 연산용으로 전치해 둔다)
_TO_WEBXR_T = robot_points_to_webxr_matrix().T.astype(np.float32)


def _to_webxr_mm(points: np.ndarray) -> np.ndarray:
    """base 프레임 점 (N,3) m를 WebXR 기저 int16 mm로 변환해 반환한다."""
    webxr = np.asarray(points, dtype=np.float32) @ _TO_WEBXR_T
    return np.clip(np.rint(webxr * 1000.0), -32767, 32767).astype("<i2")


def _pack_block(key, verts: np.ndarray | None, colors: np.ndarray | None, tris: np.ndarray | None) -> bytes:
    """블록 하나를 직렬화해 반환한다 (key = (layer, bx, by, bz), verts가 None이면 삭제 블록)."""
    if verts is None:
        return _BLOCK_HEADER.pack(*key, 0, 0)

    # 정점·색 → 4바이트 정렬 패딩 → 삼각형 인덱스
    xyz = _to_webxr_mm(verts).tobytes()
    rgb = np.ascontiguousarray(colors, dtype=np.uint8).tobytes()
    padding = b"\x00" * ((-(len(xyz) + len(rgb))) % 4)
    indices = np.ascontiguousarray(tris, dtype="<u4").tobytes()
    return _BLOCK_HEADER.pack(*key, len(verts), len(tris)) + xyz + rgb + padding + indices


def pack_mesh_blocks(blocks: list, snapshot: bool, max_bytes: int) -> list[bytes]:
    """블록 목록 [(key(layer, bx, by, bz), verts|None, colors|None, tris|None)]을 max_bytes 이하 MSH1 패킷들로 나눠 반환한다.

    snapshot이면 첫 패킷에 스냅샷 플래그를 단다 (블록이 없어도 빈 스냅샷 패킷 1개를 만든다 — 클라이언트 맵 초기화용).
    블록 하나만으로 max_bytes를 넘으면 보낼 수 없으므로(상대가 연결을 끊는다) 그 블록은 빼고 오류를 남긴다 —
    전송 블록 크기(MESH_SPLIT_SIZE·_WS)를 줄여야 한다는 신호다.
    """
    # 블록 본문을 max_bytes 이하 묶음으로 나누기
    chunks = []
    chunk = []
    chunk_bytes = _MESH_HEADER.size
    for key, verts, colors, tris in blocks:
        body = _pack_block(key, verts, colors, tris)
        if _MESH_HEADER.size + len(body) > max_bytes:
            logger.error(f"메시 블록 {key}가 메시지 상한 초과({len(body)} B) — 전송 제외, 전송 블록 크기를 줄여야 한다")
            continue
        if chunk and chunk_bytes + len(body) > max_bytes:
            chunks.append(chunk)
            chunk = []
            chunk_bytes = _MESH_HEADER.size
        chunk.append(body)
        chunk_bytes += len(body)
    if chunk or (snapshot and not chunks):
        chunks.append(chunk)

    # 묶음마다 헤더를 붙여 패킷 완성 (스냅샷이면 첫 패킷에 플래그)
    packets = []
    for chunk_index, bodies in enumerate(chunks):
        flags = MESH_FLAG_SNAPSHOT if (snapshot and chunk_index == 0) else 0
        packets.append(_MESH_HEADER.pack(MESH_MAGIC, flags, len(bodies), 0, time.time()) + b"".join(bodies))
    return packets
