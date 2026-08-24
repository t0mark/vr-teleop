// 포인트클라우드 렌더링: 서버 패킷(WebXR 좌표, mm)을 Babylon 점 메시로 표시한다. VR·데스크톱 공용.
//
// 좌표: 서버가 WebXR 기저로 변환해 보내므로 로봇 링크와 같은 z-거울(x, y, -z)만 적용하고,
// 가상 로봇과 같은 오프셋(ROBOT_OFFSET) 아래에 두어 로봇 base 기준 배치가 그대로 맞는다.
// 서버는 단일 프레임만 보내므로 패킷 하나가 곧 이번에 표시할 전체 점집합이다 (시간 누적 없음).
// 성능: 정점 버퍼를 최대 용량으로 1회 할당하고 매 패킷마다 updateVerticesData로 덮어쓴다
// (메시 재생성 없음). 남는 슬롯은 NaN 위치로 두어 GPU가 그리지 않게 한다.

import { ROBOT_OFFSET } from "./robot_view.js";

// 점 크기(픽셀)
const POINT_SIZE = 3;
// 버퍼 재할당 시 여유 배율 (누적 점수가 프레임마다 조금씩 변해도 재할당을 피함)
const CAPACITY_MARGIN = 1.5;

export class PointCloudView {
    /**
     * 표시 상태를 초기화한다. scene: Babylon 씬.
     */
    constructor(scene) {
        this._scene = scene;
        this._mesh = null;
        this._capacity = 0;
        this._positions = null;
        this._colors = null;
        // HUD·디버깅용 최근 수신 정보
        this.lastCount = 0;
        this.rxHz = 0;
        this._rxCount = 0;
        setInterval(() => {
            this.rxHz = this._rxCount;
            this._rxCount = 0;
        }, 1000);
    }

    /**
     * 디코드된 패킷 { count, xyz, rgb } 로 점 메시를 갱신한다. 반환값 없음.
     */
    update(pcd) {
        const n = pcd.count;
        if (n === 0) return;
        if (this._mesh === null || n > this._capacity) this._allocate(Math.ceil(n * CAPACITY_MARGIN));

        const pos = this._positions;
        const col = this._colors;
        const xyz = pcd.xyz;
        const rgb = pcd.rgb;
        // mm → m, z-거울, 색 0-255 → 0-1
        for (let i = 0; i < n; i++) {
            pos[i * 3] = xyz[i * 3] * 0.001;
            pos[i * 3 + 1] = xyz[i * 3 + 1] * 0.001;
            pos[i * 3 + 2] = -xyz[i * 3 + 2] * 0.001;
            col[i * 4] = rgb[i * 3] / 255;
            col[i * 4 + 1] = rgb[i * 3 + 1] / 255;
            col[i * 4 + 2] = rgb[i * 3 + 2] / 255;
        }
        // 남는 슬롯은 NaN으로 비워 이전 프레임 잔상이 남지 않게 한다
        pos.fill(NaN, n * 3);

        this._mesh.updateVerticesData(BABYLON.VertexBuffer.PositionKind, pos, false, false);
        this._mesh.updateVerticesData(BABYLON.VertexBuffer.ColorKind, col, false, false);
        this.lastCount = n;
        this._rxCount += 1;
    }

    /**
     * 용량 n의 점 메시·정점 버퍼를 (재)할당한다. 반환값 없음.
     */
    _allocate(n) {
        if (this._mesh !== null) this._mesh.dispose();
        this._capacity = n;
        this._positions = new Float32Array(n * 3).fill(NaN);
        this._colors = new Float32Array(n * 4).fill(1);

        const mesh = new BABYLON.Mesh("pointCloud", this._scene);
        const vd = new BABYLON.VertexData();
        vd.positions = this._positions;
        vd.colors = this._colors;
        vd.applyToMesh(mesh, true);

        // 점 모드 머티리얼: 정점 색을 그대로 쓰고 조명 영향은 배제한다
        const mat = new BABYLON.StandardMaterial("pointCloudMat", this._scene);
        mat.pointsCloud = true;
        mat.pointSize = POINT_SIZE;
        mat.disableLighting = true;
        mat.emissiveColor = BABYLON.Color3.White();
        mesh.material = mat;
        mesh.isPickable = false;
        // 정점이 매 프레임 바뀌고 NaN도 섞이므로 경계 상자 컬링을 끄고 항상 그린다
        mesh.alwaysSelectAsActiveMesh = true;
        mesh.position.set(ROBOT_OFFSET.x, ROBOT_OFFSET.y, ROBOT_OFFSET.z);
        this._mesh = mesh;
        console.log(`[pcd] 점 버퍼 할당: ${n}`);
    }
}
