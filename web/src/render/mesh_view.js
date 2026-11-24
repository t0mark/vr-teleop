// 재구성 메시 렌더링: 서버 TSDF 재구성(환경 층·작업 영역 층)의 블록 단위 메시(MSH1)를 받아 블록별 Babylon 메시로
// 유지한다. VR·데스크톱 공용.
//
// 블록 하나 = 메시 하나. 바뀐 블록만 새로 만들고(기존 것은 폐기), nv=0 블록은 삭제, 스냅샷이면 전부 비운 뒤 적용한다.
// 좌표: 서버가 WebXR 기저 mm로 보내므로 로봇 링크와 같은 z-거울(x, y, -z)만 적용하고 ROBOT_OFFSET 아래에 둔다.
// z-거울은 삼각형 감김 방향을 뒤집으므로 인덱스 순서를 바꿔 되돌린다 — 법선이 바깥을 향해야 음영이 맞다.
// 머티리얼: 정점 색 × 씬 조명(위쪽 반구광) + 약한 자체 발광. 같은 색 물체끼리도 면 방향 차이가 음영으로 드러나고,
// 아래를 향한 면이 검게 죽지 않는다.

import { ROBOT_OFFSET } from "./robot_view.js";

// 자체 발광 밝기 (0-1): 조명을 덜 받는 면의 최소 밝기
const MESH_EMISSIVE = 0.35;

export class MeshView {
    /**
     * 블록 저장소와 공용 머티리얼을 초기화한다. scene: Babylon 씬.
     */
    constructor(scene) {
        this._scene = scene;
        // 블록 키("layer:x,y,z") → Babylon 메시
        this._blocks = new Map();
        this._triangles = 0;
        this._material = new BABYLON.StandardMaterial("reconMeshMat", scene);
        this._material.diffuseColor = BABYLON.Color3.White();
        this._material.specularColor = BABYLON.Color3.Black();
        this._material.emissiveColor = new BABYLON.Color3(MESH_EMISSIVE, MESH_EMISSIVE, MESH_EMISSIVE);
        this._material.backFaceCulling = false;
        // HUD·디버깅용 수신 통계
        this.rxHz = 0;
        this._rxCount = 0;
        setInterval(() => {
            this.rxHz = this._rxCount;
            this._rxCount = 0;
        }, 1000);
    }

    /** 현재 블록 수. */
    get blockCount() {
        return this._blocks.size;
    }

    /** 현재 삼각형 수. */
    get triangleCount() {
        return this._triangles;
    }

    /**
     * 디코드된 MSH1 패킷 { snapshot, blocks } 을 적용한다. 반환값 없음.
     */
    apply(msg) {
        if (msg.snapshot) {
            this._clear();
            console.log(`[mesh] 스냅샷 수신: 블록 ${msg.blocks.length}개`);
        }
        for (const block of msg.blocks) {
            this._removeBlock(block.key);
            if (block.nv > 0) this._addBlock(block);
        }
        this._rxCount += 1;
    }

    /**
     * 블록 하나의 메시를 만든다 (mm → m, z-거울, 감김 복원, 법선 계산, 색 0-255 → 0-1). 반환값 없음.
     */
    _addBlock(block) {
        const n = block.nv;
        const positions = new Float32Array(n * 3);
        const colors = new Float32Array(n * 4);
        for (let i = 0; i < n; i++) {
            positions[i * 3] = block.xyz[i * 3] * 0.001;
            positions[i * 3 + 1] = block.xyz[i * 3 + 1] * 0.001;
            positions[i * 3 + 2] = -block.xyz[i * 3 + 2] * 0.001;
            colors[i * 4] = block.rgb[i * 3] / 255;
            colors[i * 4 + 1] = block.rgb[i * 3 + 1] / 255;
            colors[i * 4 + 2] = block.rgb[i * 3 + 2] / 255;
            colors[i * 4 + 3] = 1;
        }

        // z-거울로 뒤집힌 감김 방향을 되돌린다 (삼각형마다 두 번째·세 번째 정점 교환)
        const indices = new Uint32Array(block.nt * 3);
        for (let t = 0; t < block.nt; t++) {
            indices[t * 3] = block.tris[t * 3];
            indices[t * 3 + 1] = block.tris[t * 3 + 2];
            indices[t * 3 + 2] = block.tris[t * 3 + 1];
        }
        const normals = new Float32Array(n * 3);
        BABYLON.VertexData.ComputeNormals(positions, indices, normals);

        const mesh = new BABYLON.Mesh(`reconBlock_${block.key}`, this._scene);
        const vd = new BABYLON.VertexData();
        vd.positions = positions;
        vd.colors = colors;
        vd.indices = indices;
        vd.normals = normals;
        vd.applyToMesh(mesh, false);
        mesh.material = this._material;
        mesh.isPickable = false;
        mesh.position.set(ROBOT_OFFSET.x, ROBOT_OFFSET.y, ROBOT_OFFSET.z);
        mesh.freezeWorldMatrix();
        this._blocks.set(block.key, mesh);
        this._triangles += block.nt;
    }

    /**
     * 블록 하나를 폐기한다 (없으면 무시). 반환값 없음.
     */
    _removeBlock(key) {
        const mesh = this._blocks.get(key);
        if (mesh === undefined) return;
        this._triangles -= mesh.getTotalIndices() / 3;
        mesh.dispose();
        this._blocks.delete(key);
    }

    /**
     * 모든 블록을 폐기한다. 반환값 없음.
     */
    _clear() {
        for (const mesh of this._blocks.values()) mesh.dispose();
        this._blocks.clear();
        this._triangles = 0;
    }
}
