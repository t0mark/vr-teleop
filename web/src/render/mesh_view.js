// 재구성 메시 렌더링: 서버 TSDF 재구성(환경 층·작업 영역 층)의 블록 단위 메시(MSH1)를 블록별 Babylon 메시로
// 유지한다. VR·데스크톱 공용.
//
// 블록 하나 = 메시 하나. 바뀐 블록은 다시 만들고, nv=0 블록은 삭제, 스냅샷이면 전부 비운 뒤 적용한다.
// 좌표: WebXR 기저 mm → m, z-거울(x, y, -z), ROBOT_OFFSET 아래 배치. z-거울로 뒤집힌 삼각형 감김은 인덱스 교환으로 복원한다.
// 머티리얼: 정점 색 × 씬 조명 + 약한 자체 발광.

import { ROBOT_OFFSET } from "./robot_view.js";

// 자체 발광 밝기 (0-1)
const MESH_EMISSIVE = 0.35;
// mm → m 변환 계수
const MM_TO_M = 0.001;
// 수신률 갱신 주기(ms)
const RX_RATE_INTERVAL_MS = 1000;

export class MeshView {
    /**
     * 블록 저장소와 공용 머티리얼을 초기화한다. scene: Babylon 씬.
     */
    constructor(scene) {
        this._scene = scene;
        // 블록 키("layer:x,y,z") → Babylon 메시
        this._blocks = new Map();
        this._triangles = 0;

        // 공용 머티리얼
        this._material = new BABYLON.StandardMaterial("reconMeshMat", scene);
        this._material.diffuseColor = BABYLON.Color3.White();
        this._material.specularColor = BABYLON.Color3.Black();
        this._material.emissiveColor = new BABYLON.Color3(MESH_EMISSIVE, MESH_EMISSIVE, MESH_EMISSIVE);
        this._material.backFaceCulling = false;

        // 수신 통계 (HUD 표시용)
        this._rxHz = 0;
        this._rxCount = 0;
        setInterval(() => this._updateRxRate(), RX_RATE_INTERVAL_MS);
    }

    /** 현재 블록 수. */
    get blockCount() {
        return this._blocks.size;
    }

    /** 현재 삼각형 수. */
    get triangleCount() {
        return this._triangles;
    }

    /** 최근 1초 패킷 수신률(Hz). */
    get rxHz() {
        return this._rxHz;
    }

    /**
     * 디코드된 MSH1 패킷 { snapshot, blocks } 을 적용한다. 반환값 없음.
     */
    apply(packet) {
        // 스냅샷이면 기존 블록 전부 삭제
        if (packet.snapshot) {
            this._clear();
            console.log(`[mesh] 스냅샷 수신: 블록 ${packet.blocks.length}개`);
        }

        // 블록 교체·삭제
        for (const block of packet.blocks) {
            this._removeBlock(block.key);
            if (block.nv > 0) this._addBlock(block);
        }
        this._rxCount += 1;
    }

    /**
     * 최근 주기 수신 횟수로 수신률(rxHz)을 갱신한다. 반환값 없음.
     */
    _updateRxRate() {
        this._rxHz = this._rxCount;
        this._rxCount = 0;
    }

    /**
     * 블록 하나의 메시를 만든다 (mm → m, z-거울, 감김 복원, 법선 계산, 색 0-255 → 0-1). 반환값 없음.
     */
    _addBlock(block) {
        // 정점 위치·색
        const vertexCount = block.nv;
        const positions = new Float32Array(vertexCount * 3);
        const colors = new Float32Array(vertexCount * 4);
        for (let vertex = 0; vertex < vertexCount; vertex++) {
            positions[vertex * 3] = block.xyz[vertex * 3] * MM_TO_M;
            positions[vertex * 3 + 1] = block.xyz[vertex * 3 + 1] * MM_TO_M;
            positions[vertex * 3 + 2] = -block.xyz[vertex * 3 + 2] * MM_TO_M;
            colors[vertex * 4] = block.rgb[vertex * 3] / 255;
            colors[vertex * 4 + 1] = block.rgb[vertex * 3 + 1] / 255;
            colors[vertex * 4 + 2] = block.rgb[vertex * 3 + 2] / 255;
            colors[vertex * 4 + 3] = 1;
        }

        // 삼각형마다 두 번째·세 번째 정점을 교환해 감김 방향 복원
        const indices = new Uint32Array(block.nt * 3);
        for (let triangle = 0; triangle < block.nt; triangle++) {
            indices[triangle * 3] = block.tris[triangle * 3];
            indices[triangle * 3 + 1] = block.tris[triangle * 3 + 2];
            indices[triangle * 3 + 2] = block.tris[triangle * 3 + 1];
        }

        // 법선 계산
        const normals = new Float32Array(vertexCount * 3);
        BABYLON.VertexData.ComputeNormals(positions, indices, normals);

        // 메시 생성·배치
        const mesh = new BABYLON.Mesh(`reconBlock_${block.key}`, this._scene);
        const vertexData = new BABYLON.VertexData();
        vertexData.positions = positions;
        vertexData.colors = colors;
        vertexData.indices = indices;
        vertexData.normals = normals;
        vertexData.applyToMesh(mesh, false);
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
