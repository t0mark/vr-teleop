// 포인트클라우드 패킷 디코더. 서버 규약(teleop/core/depth_to_3d/packet.py)과 일치해야 한다.
//   uint32 magic('PCD1') | uint32 count | float64 t | int16 xyz[N][3] (mm, WebXR 좌표) | uint8 rgb[N][3]

const MAGIC = 0x31444350;
const HEADER_BYTES = 16;

/**
 * ArrayBuffer를 { count, t, xyz: Int16Array(3N), rgb: Uint8Array(3N) } 로 디코드한다.
 * 매직이 다르면 null (다른 바이너리 메시지로 간주).
 */
export function decodePointCloud(buf) {
    if (buf.byteLength < HEADER_BYTES) return null;
    const view = new DataView(buf);
    if (view.getUint32(0, true) !== MAGIC) return null;

    const count = view.getUint32(4, true);
    const t = view.getFloat64(8, true);
    const xyzBytes = count * 6;
    if (buf.byteLength < HEADER_BYTES + xyzBytes + count * 3) {
        console.warn("[pcd] 패킷 길이 부족");
        return null;
    }

    // 헤더 16바이트는 2바이트 정렬이므로 Int16Array 뷰를 복사 없이 만들 수 있다
    const xyz = new Int16Array(buf, HEADER_BYTES, count * 3);
    const rgb = new Uint8Array(buf, HEADER_BYTES + xyzBytes, count * 3);
    return { count, t, xyz, rgb };
}
