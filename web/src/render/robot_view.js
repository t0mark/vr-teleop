// 가상 RBY1 렌더링: robot_config(링크-메시 구성 + 머리 카메라 장착)로 GLB를 로드하고
// robot_state(링크 포즈, WebXR 좌표)로 자세를 갱신한다. VR·데스크톱 공용.
//
// 좌표 처리:
// - 링크 포즈·visual origin(WebXR 기저) → 왼손 씬 표시: z-거울 켤레 pos (x,y,-z), quat (-x,-y,z,w)
// - GLB 정점(URDF 링크 프레임, z-up) → 표시 좌표: 메시 래퍼에 쿼터니언 (-0.5, 0.5, 0.5, 0.5) 적용
//   (glTF 로더의 RH→LH x-거울 포함, 축 (-1,1,1)/√3 120° 회전)
// - glTF 로더 좌표계 모드는 기본값을 쓴다 (FORCE_RIGHT_HANDED 사용 금지)
// - 머리 장착 카메라(D455 GLB, camera_link 프레임)도 같은 체인으로 붙인다
//
// 표시 모드
// - "all": 전 링크 표시
// - "body": 머리 링크·카메라만 숨김 (VR 로봇 시점)
// - "none": 메시 숨김 (링크 변환과 머리 카메라 앵커는 그대로 갱신)

// 로봇 base 위치 (씬 원점, 로봇 기준 표시물 공통)
export const ROBOT_OFFSET = { x: 0, y: 0, z: 0 };
// 머리 링크 이름 접두사 ("body" 표시 모드에서 숨기는 대상)
const HEAD_LINK_PREFIX = "link_head_";
// GLB 정점 기저 보정 쿼터니언 (x, y, z, w)
const MESH_BASIS_FIX_QUAT = [-0.5, 0.5, 0.5, 0.5];

export class RobotView {
    /**
     * 로봇 표시 상태를 초기화한다. scene: Babylon 씬.
     */
    constructor(scene) {
        this._scene = scene;
        this._root = null;
        // 링크명 → TransformNode (robot_state로 포즈 갱신 대상)
        this._linkNodes = new Map();
        // 표시 대상명("링크명" 또는 "camera") → 메시 origin 노드 (표시 전환 단위)
        this._meshNodes = new Map();
        // 머리 장착 카메라 프레임 노드 (VR 시점 앵커, 표시 전환과 무관)
        this._cameraAnchor = null;
        // 구성 로드 진행 여부와 현재 표시 모드
        this._loading = false;
        this._mode = "all";
    }

    /**
     * robot_config 메시지로 링크 노드 트리를 만들고 GLB를 비동기 로드한다. 반환값 없음.
     * 재접속으로 같은 메시지가 다시 와도 1회만 구축한다.
     */
    async applyConfig(cfg) {
        if (this._root !== null || this._loading) return;
        this._loading = true;

        try {
            // 로봇 루트 노드
            this._root = new BABYLON.TransformNode("robotRoot", this._scene);
            this._root.position.set(ROBOT_OFFSET.x, ROBOT_OFFSET.y, ROBOT_OFFSET.z);

            // 링크·카메라 구축 (링크별 mesh_base가 있으면 그것을 쓴다)
            const jobs = [];
            for (const [name, info] of Object.entries(cfg.links)) {
                jobs.push(this._buildLink(name, info, info.mesh_base ?? cfg.mesh_base));
            }
            if (cfg.camera) jobs.push(this._buildCamera(cfg.camera));
            await Promise.allSettled(jobs);

            // 현재 표시 모드를 로드된 메시에 반영
            this.setVisibleParts(this._mode);
            console.log(`[robot] 링크 ${this._linkNodes.size}개 메시 로드 완료`);
        } catch (e) {
            console.error("[robot] robot_config 적용 실패 (다음 robot_config에서 재구축):", e);
            this._disposeModel();
        } finally {
            this._loading = false;
        }
    }

    /**
     * 구축 중이던 로봇 노드·메시를 모두 폐기하고 초기 상태로 되돌린다. 반환값 없음.
     */
    _disposeModel() {
        if (this._root !== null) {
            this._root.dispose();
            this._root = null;
        }
        this._linkNodes.clear();
        this._meshNodes.clear();
        this._cameraAnchor = null;
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
     * 머리 장착 카메라(D455 GLB)를 장착 링크 아래에 붙이고 시점 앵커를 만든다. 반환값 없음.
     * cam: {link, origin{pos,quat}(WebXR 기저, 링크 로컬), mesh_base, mesh}.
     */
    async _buildCamera(cam) {
        const linkNode = this._linkNodes.get(cam.link);
        if (!linkNode) {
            console.warn(`[robot] 카메라 장착 링크 없음: ${cam.link}`);
            return;
        }
        // 시점 앵커 노드 (메시 노드와 같은 프레임의 별도 노드)
        this._cameraAnchor = this._makeOriginNode(linkNode, "cameraAnchor", cam.origin);
        await this._attachMesh(linkNode, "camera", cam.origin, cam.mesh_base, cam.mesh);
    }

    /**
     * 부모 노드 아래에 origin 변환만 적용한 TransformNode를 만들어 반환한다.
     * origin: WebXR 기저의 부모 로컬 변환 {pos, quat} (표시용 z-거울 켤레를 여기서 적용).
     */
    _makeOriginNode(parent, name, origin) {
        const node = new BABYLON.TransformNode(`origin_${name}`, this._scene);
        node.parent = parent;
        node.position.set(origin.pos[0], origin.pos[1], -origin.pos[2]);
        node.rotationQuaternion = new BABYLON.Quaternion(
            -origin.quat[0], -origin.quat[1], origin.quat[2], origin.quat[3],
        );
        return node;
    }

    /**
     * 부모 노드 아래에 (origin → 기저 보정 래퍼 → GLB) 체인을 만들어 메시를 붙인다. 반환값 없음.
     */
    async _attachMesh(parent, name, origin, meshBase, meshFile) {
        const originNode = this._makeOriginNode(parent, name, origin);
        this._meshNodes.set(name, originNode);

        // 기저 보정 래퍼: 링크 프레임(z-up) 정점 → 표시 좌표
        const wrapper = new BABYLON.TransformNode(`meshfix_${name}`, this._scene);
        wrapper.parent = originNode;
        wrapper.rotationQuaternion = new BABYLON.Quaternion(...MESH_BASIS_FIX_QUAT);

        // GLB 로드 후 래퍼 아래에 붙인다
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
     * 표시 모드를 전환한다. mode: "all"(전체), "body"(머리 링크·카메라 제외), "none"(숨김). 반환값 없음.
     * 메시 노드만 끄고 링크 변환 노드·시점 앵커는 유지한다.
     */
    setVisibleParts(mode) {
        this._mode = mode;
        for (const [name, node] of this._meshNodes) {
            const isHead = name === "camera" || name.startsWith(HEAD_LINK_PREFIX);
            node.setEnabled(mode === "all" || (mode === "body" && !isHead));
        }
    }

    /**
     * 머리 장착 카메라 프레임의 월드 위치를 반환한다 (VR 시점 앵커용). 없으면 null.
     */
    cameraAnchorPosition() {
        if (this._cameraAnchor === null) return null;
        // 부모 체인을 포함한 월드 행렬 재계산
        this._cameraAnchor.computeWorldMatrix(true);
        return this._cameraAnchor.getAbsolutePosition();
    }

    /**
     * robot_state 메시지(WebXR 좌표 링크 포즈)로 전 링크 자세를 갱신한다. 반환값 없음.
     */
    applyState(state) {
        for (const [name, pose] of Object.entries(state.links)) {
            const node = this._linkNodes.get(name);
            if (!node) continue;

            // WebXR 오른손 → 표시(왼손): z 거울 켤레
            const position = pose.pos;
            const quat = pose.quat;
            node.position.set(position[0], position[1], -position[2]);
            node.rotationQuaternion.set(-quat[0], -quat[1], quat[2], quat[3]);
        }
    }
}
