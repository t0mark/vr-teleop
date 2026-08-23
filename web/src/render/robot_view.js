// 가상 RBY1 렌더링: 서버의 robot_config(링크-메시 구성 + 카메라 장착)로 GLB를 로드하고
// robot_state(링크 포즈, WebXR 좌표)로 매 틱 자세를 갱신한다. VR·데스크톱 공용.
//
// 좌표 처리 (유도는 code.md 참고, 요약):
// - 링크 포즈·visual origin: 서버가 WebXR 기저로 변환해 보냄 → 왼손 씬 표시는
//   z-거울 켤레: pos (x,y,-z), quat (-x,-y,z,w)
// - GLB 정점은 URDF 링크 프레임(z-up) 데이터이므로 기저 재정의 보정 필요.
//   glTF 로더의 RH→LH 변환(__root__ = x-거울)까지 합쳐, 메시 래퍼에
//   W = M_z·Cᵀ·M_x = 순수 회전(축 (-1,1,1)/√3, 120°) = 쿼터니언 (-0.5, 0.5, 0.5, 0.5)를 적용
// - 주의: GLTFLoaderCoordinateSystemMode.FORCE_RIGHT_HANDED는 절대 사용 금지 —
//   씬 전체를 오른손 좌표계로 강제 전환해 카메라·HUD·컬링이 전부 깨진다 (겪은 회귀)
// - 머리 장착 카메라(D455 GLB, camera_link 프레임 = 링크와 같은 z-up 규약)도 같은 체인으로 붙인다

// 로봇 베이스 배치: 사용자 전방 1.5m (WebXR -z → 표시 +z), 로봇 정면은 사용자와 같은 방향
// 포인트클라우드 등 로봇 base 기준 표시물이 같은 오프셋을 공유하므로 export한다
export const ROBOT_OFFSET = { x: 0, y: 0, z: 1.5 };

export class RobotView {
    /**
     * 로봇 표시 상태를 초기화한다. scene: Babylon 씬.
     */
    constructor(scene) {
        this._scene = scene;
        this._root = null;
        // 링크명 → TransformNode (robot_state로 포즈 갱신 대상)
        this._linkNodes = new Map();
        this._loading = false;
    }

    /**
     * robot_config 메시지로 링크 노드 트리를 만들고 GLB를 비동기 로드한다. 반환값 없음.
     * 재접속으로 같은 메시지가 다시 와도 1회만 구축한다.
     */
    async applyConfig(cfg) {
        if (this._root !== null || this._loading) return;
        this._loading = true;

        this._root = new BABYLON.TransformNode("robotRoot", this._scene);
        this._root.position.set(ROBOT_OFFSET.x, ROBOT_OFFSET.y, ROBOT_OFFSET.z);

        const jobs = [];
        for (const [name, info] of Object.entries(cfg.links)) {
            jobs.push(this._buildLink(name, info, cfg.mesh_base));
        }
        if (cfg.camera) jobs.push(this._buildCamera(cfg.camera));
        await Promise.allSettled(jobs);
        this._loading = false;
        console.log(`[robot] 링크 ${this._linkNodes.size}개 메시 로드 완료`);
    }

    /**
     * 링크 1개의 노드를 만들고 GLB를 붙인다. 반환값 없음.
     */
    async _buildLink(name, info, meshBase) {
        const linkNode = new BABYLON.TransformNode(`link_${name}`, this._scene);
        linkNode.parent = this._root;
        linkNode.rotationQuaternion = BABYLON.Quaternion.Identity();
        this._linkNodes.set(name, linkNode);
        await this._attachMesh(linkNode, name, info.origin, meshBase, info.mesh);
    }

    /**
     * 머리 장착 카메라(D455 GLB)를 장착 링크 아래에 붙인다. 반환값 없음.
     * cam: {link, origin{pos,quat}(WebXR 기저, 링크 로컬), mesh_base, mesh}.
     */
    async _buildCamera(cam) {
        const linkNode = this._linkNodes.get(cam.link);
        if (!linkNode) {
            console.warn(`[robot] 카메라 장착 링크 없음: ${cam.link}`);
            return;
        }
        await this._attachMesh(linkNode, "camera", cam.origin, cam.mesh_base, cam.mesh);
    }

    /**
     * 부모 노드 아래에 (origin → 기저 보정 래퍼 → GLB) 체인을 만들어 메시를 붙인다. 반환값 없음.
     * origin: WebXR 기저의 부모 로컬 변환 {pos, quat} (표시용 z-거울 켤레를 여기서 적용).
     */
    async _attachMesh(parent, name, origin, meshBase, meshFile) {
        const originNode = new BABYLON.TransformNode(`origin_${name}`, this._scene);
        originNode.parent = parent;
        originNode.position.set(origin.pos[0], origin.pos[1], -origin.pos[2]);
        originNode.rotationQuaternion = new BABYLON.Quaternion(
            -origin.quat[0], -origin.quat[1], origin.quat[2], origin.quat[3],
        );

        // 기저 보정 래퍼: 링크 프레임(z-up) 정점 → 표시 좌표 정합 (파일 상단 유도 참고)
        const wrapper = new BABYLON.TransformNode(`meshfix_${name}`, this._scene);
        wrapper.parent = originNode;
        wrapper.rotationQuaternion = new BABYLON.Quaternion(-0.5, 0.5, 0.5, 0.5);

        try {
            const result = await BABYLON.SceneLoader.ImportMeshAsync(null, meshBase, meshFile, this._scene);
            for (const mesh of result.meshes) {
                if (mesh.parent === null) mesh.parent = wrapper;
                mesh.isPickable = false;
            }
        } catch (e) {
            console.warn(`[robot] 메시 로드 실패 (${meshFile}):`, e);
        }
    }

    /**
     * robot_state 메시지(WebXR 좌표 링크 포즈)로 전 링크 자세를 갱신한다. 반환값 없음.
     */
    applyState(state) {
        for (const [name, pose] of Object.entries(state.links)) {
            const node = this._linkNodes.get(name);
            if (!node) continue;
            const p = pose.pos, q = pose.quat;
            // WebXR 오른손 → 표시(왼손): z 거울 켤레
            node.position.set(p[0], p[1], -p[2]);
            node.rotationQuaternion.set(-q[0], -q[1], q[2], q[3]);
        }
    }
}
