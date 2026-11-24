// mesh DataChannel 바이너리 패킷 디코더. 서버 규약(teleop/perception/packets.py)과 일치해야 한다 (리틀엔디언).
// 좌표는 WebXR 기저 int16 mm.
//   MSH1: uint32 magic | uint32 flags | uint32 block_count | uint32 reserved | float64 t
//         블록마다 int32 layer | int32 key[3] | uint32 nv | uint32 nt | int16 xyz[nv][3] | uint8 rgb[nv][3] | 패딩(4정렬) | uint32 tri[nt][3]
//         layer: 0 = 환경, 1 = 작업 영역
//         flags bit0 = 스냅샷 시작(기존 블록 전부 삭제 후 적용), nv=0 블록 = 삭제

const MESH_MAGIC = 0x3148534D;
const MESH_FLAG_SNAPSHOT = 0x1;
const MESH_HEADER_BYTES = 24;
const BLOCK_HEADER_BYTES = 24;

/**
 * MSH1 패킷을 { snapshot, t, blocks: [{ key, nv, nt, xyz, rgb, tris }] } 로 디코드한다. 매직이 다르거나 깨졌으면 null.
 * key: "layer:x,y,z" 문자열 (블록 식별자), layer: 0 환경 / 1 작업 영역. nv=0이면 삭제 블록 (xyz·rgb·tris는 null).
 */
export function decodeMeshBlocks(buf) {
    if (buf.byteLength < MESH_HEADER_BYTES) return null;
    const view = new DataView(buf);
    if (view.getUint32(0, true) !== MESH_MAGIC) return null;

    const flags = view.getUint32(4, true);
    const blockCount = view.getUint32(8, true);
    const t = view.getFloat64(16, true);
    const blocks = [];
    let offset = MESH_HEADER_BYTES;

    for (let b = 0; b < blockCount; b++) {
        if (offset + BLOCK_HEADER_BYTES > buf.byteLength) {
            console.warn("[mesh] 블록 헤더 길이 부족");
            return null;
        }
        const layer = view.getInt32(offset, true);
        const key = `${layer}:${view.getInt32(offset + 4, true)},${view.getInt32(offset + 8, true)},${view.getInt32(offset + 12, true)}`;
        const nv = view.getUint32(offset + 16, true);
        const nt = view.getUint32(offset + 20, true);
        offset += BLOCK_HEADER_BYTES;
        if (nv === 0) {
            blocks.push({ key, layer, nv: 0, nt: 0, xyz: null, rgb: null, tris: null });
            continue;
        }

        // 정점 → 색 → 4바이트 정렬 패딩 → 삼각형 인덱스
        const xyzBytes = nv * 6;
        const rgbBytes = nv * 3;
        const pad = (4 - ((xyzBytes + rgbBytes) % 4)) % 4;
        const end = offset + xyzBytes + rgbBytes + pad + nt * 12;
        if (end > buf.byteLength) {
            console.warn("[mesh] 블록 본문 길이 부족");
            return null;
        }
        const xyz = new Int16Array(buf, offset, nv * 3);
        const rgb = new Uint8Array(buf, offset + xyzBytes, rgbBytes);
        const tris = new Uint32Array(buf, offset + xyzBytes + rgbBytes + pad, nt * 3);
        blocks.push({ key, layer, nv, nt, xyz, rgb, tris });
        offset = end;
    }
    return { snapshot: (flags & MESH_FLAG_SNAPSHOT) !== 0, t, blocks };
}
