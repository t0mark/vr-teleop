// 사람 아바타 렌더링: body-tracking 조인트로 리깅 아바타의 본을 돌려 조작자 자세를 재현한다.
// 데스크톱 모니터링·데모 뷰 전용 (VR에서는 사용자 자신이므로 그리지 않는다).
//
// 리타게팅 방식: **방향 정합(position-based)**
//   본마다 rest 자세의 방향 d0(월드)와 추적 조인트로 만든 목표 방향 d1을 구하고,
//   최단호 회전 q = rot(d0 → d1)를 rest 월드 회전에 좌곱해 본의 월드 회전으로 넣는다.
//   조인트 쿼터니언을 직접 쓰지 않는 이유: 트래킹 조인트와 리그 본은 로컬 축 규약이 서로 달라
//   상사변환 규약을 잘못 잡으면 팔이 통째로 틀어진다 (매퍼에서 겪은 함정과 같은 종류).
//   방향 정합은 축 규약에 의존하지 않는 대신 본 축 회전(roll)을 잃는데, 자세 확인 용도로는 충분하다.
//
// **본 조작은 스켈레톤 공간에서 한다 (메시 월드 행렬을 끼우지 않는다)**
//   `bone.getRotationQuaternion(WORLD, mesh)` / `setRotationQuaternion(..., WORLD, mesh)`는 메시 월드
//   행렬을 함께 분해하는데, 이 아바타는 Armature에 0.01 배율이 있고 glTF 로더가 __root__에 z-거울을
//   넣어 둬서(det<0) 분해가 어긋나 본 로컬 행렬에 배율이 섞여 들어간다 → 메시가 100배로 부푼다 (겪은 증상).
//   그래서 rest 자세와 목표 방향을 모두 스켈레톤 공간에서 잡고 mesh 인자 없이 회전을 넣는다.
//   목표 방향은 메시 월드 행렬의 역행렬로 선형 변환해 스켈레톤 공간으로 가져온다.
//
// 리그 해석: 본 이름 목록을 박아 두지 않고 **접미사 패턴 + 계층 탐색**으로 찾는다.
//   Mixamo 계열(mixamorig:LeftForeArm 등) 표준 humanoid 리그를 기준으로 하며, 아바타를 바꿔도
//   같은 관례를 따르면 코드 수정 없이 동작한다. 좌우는 이름이 아니라 rest 자세의 x 부호로 가른다.
//
// 좌표: 추적 조인트는 WebXR(오른손) → 표시(왼손)로 z 거울(x, y, -z)만 적용한다.
// 배치: 어깨선으로 사람의 yaw를 구해 로봇과 같은 방향(+z)을 보도록 정규화한 뒤, 엉덩이를
//   HUMAN_OFFSET 위치로 옮긴다. 조작자가 방을 어느 방향으로 서 있든 로봇 옆에 나란히 서 보인다.
//
// 에셋: web/assets/avatar.glb = Mixamo "X Bot" (three.js 예제 배포본, 회색 마네킹 + 표준 humanoid 리그)

import { ROBOT_OFFSET } from "./robot_view.js";

// 로봇 옆 배치(m): 표시 좌표 +x = 로봇의 오른쪽이므로 -x는 로봇의 왼쪽 옆자리.
// 양쪽 다 팔을 벌려도 닿지 않게 1.5 m 띄운다
export const HUMAN_OFFSET = { x: ROBOT_OFFSET.x - 1.5, z: ROBOT_OFFSET.z };

// 본 슬롯 판별 패턴 (본 이름 접미사 기준, 대소문자 무시).
// ForeArm/UpLeg가 Arm/Leg 패턴에 걸리지 않도록 후방탐색으로 배제한다
const BONE_PATTERNS = {
    hips: /(hips|pelvis)$/i,
    spine: /spine$/i,
    spine1: /spine1$/i,
    spine2: /(spine2|chest)$/i,
    neck: /neck$/i,
    head: /head$/i,
    shoulder: /(shoulder|clavicle)$/i,
    upperArm: /(?<!fore)arm$/i,
    foreArm: /forearm$/i,
    hand: /hand$/i,
    upLeg: /(upleg|thigh)$/i,
    lowerLeg: /(?<!up)leg$/i,
    foot: /foot$/i,
    toe: /(toebase|toe)$/i,
};

// 척추 체인: 본 슬롯과 대응 추적 조인트 (본 i는 조인트 i → i+1 방향을 향한다)
const SPINE_SLOTS = ["hips", "spine", "spine1", "spine2", "neck", "head"];
const SPINE_JOINTS = [
    ["hips", "root"],
    ["spine-lower", "spine-middle"],
    ["spine-middle", "spine-upper"],
    ["spine-upper", "chest"],
    ["neck"],
    ["head"],
];

// 팔·다리 체인: 어깨(쇄골)부터 손까지, 엉덩관절부터 발끝까지
const ARM_SLOTS = ["shoulder", "upperArm", "foreArm", "hand"];
const armJoints = (side) => [
    [`${side}-shoulder`, `${side}-scapula`],
    [`${side}-arm-upper`],
    [`${side}-arm-lower`],
    [`${side}-hand-wrist`, `${side}-wrist`],
];
const LEG_SLOTS = ["upLeg", "lowerLeg", "foot", "toe"];
const legJoints = (side) => [
    [`${side}-upper-leg`, `${side}-leg-upper`],
    [`${side}-lower-leg`, `${side}-leg-lower`],
    [`${side}-foot-ankle`, `${side}-ankle`],
    [`${side}-foot-ball`, `${side}-foot-toe`],
];

// 디버그 마커 지름(m)
const JOINT_MARKER_DIAMETER = 0.04;

export class HumanView {
    /**
     * 아바타 표시 상태를 초기화한다. scene: Babylon 씬.
     */
    constructor(scene) {
        this._scene = scene;
        this._container = null;
        this._mesh = null;
        this._skeleton = null;
        // 본 → { bone, restPos, restRot }
        this._rest = new Map();
        // 체인 정의: [{ bones: [Bone|null], joints: [[후보 이름]] }]
        this._chains = [];
        this._restSpan = 1.0;
        this._scaled = false;
        this._markers = new Map();
        this._showJoints = false;
        this.ready = false;
    }

    /**
     * 아바타 GLB를 로드하고 리그를 해석해 rest 자세를 기록한다. 반환값 없음.
     * base·file: 정적 서빙 경로와 파일명.
     */
    async load(base, file) {
        this._container = new BABYLON.TransformNode("humanRoot", this._scene);

        let result;
        try {
            result = await BABYLON.SceneLoader.ImportMeshAsync(null, base, file, this._scene);
        } catch (e) {
            console.warn("[human] 아바타 로드 실패:", e);
            return;
        }

        // GLB에 딸린 애니메이션이 자동 재생되면 본을 덮어써 리타게팅이 무효가 된다
        for (const group of result.animationGroups ?? []) group.stop();

        this._skeleton = result.skeletons[0] ?? null;
        this._mesh = result.meshes.find((m) => m.skeleton) ?? result.meshes[0];
        for (const mesh of result.meshes) {
            if (mesh.parent === null) mesh.parent = this._container;
            mesh.isPickable = false;
            // 본을 월드 기준으로 직접 돌리므로 경계 상자가 갱신되지 않아 컬링으로 사라질 수 있다
            mesh.alwaysSelectAsActiveMesh = true;
        }
        if (this._skeleton === null) {
            console.warn("[human] 아바타에 스켈레톤이 없어 자세 재현 불가");
            return;
        }

        this._detachTransformNodes();
        this._captureRest();
        this._buildChains();
        this.ready = this._chains.length > 0;
        // 자세 샘플이 없어도 기본 자리(로봇 옆)에 rest 자세로 서 있게 한다.
        // 원점에 두면 로봇과 겹친다 (겪은 증상) — 숨기는 대신 자리를 잡아 주는 게 맞다
        this._placeAt(this._restHipsWorld());
        console.log(`[human] 아바타 로드 완료: 본 ${this._skeleton.bones.length}개, 체인 ${this._chains.length}개`);
    }

    /**
     * 본에 연결된 TransformNode 링크를 끊는다. 반환값 없음.
     *
     * glTF 로더는 본마다 대응 TransformNode를 만들어 linkTransformNode로 묶어 두는데, 이 상태에서는
     * 매 프레임 노드 → 본 방향으로 행렬이 덮어써져 본을 직접 돌린 결과가 사라진다.
     * 리타게팅이 본을 직접 다루므로 링크를 끊어야 한다 (노드는 스키닝에 쓰이지 않는다).
     */
    _detachTransformNodes() {
        for (const bone of this._skeleton.bones) {
            if (typeof bone.linkTransformNode !== "function") continue;
            if (bone.getTransformNode?.() != null) bone.linkTransformNode(null);
        }
    }

    /**
     * 모든 본의 rest(초기) 자세를 기록한다. 반환값 없음.
     * 리타게팅의 기준이므로 본을 한 번이라도 돌리기 전에 호출해야 한다.
     *
     * pos·rot은 **스켈레톤 공간**(mesh 인자 없음) — 회전 계산에 쓴다.
     * worldPos만 메시 월드 행렬을 거친 값으로, 좌우 판별에 쓴다 (스켈레톤 공간은 거울 때문에
     * x 부호가 뒤집혀 있어 좌우 판단에 못 쓴다). 위치 변환은 분해가 없어 안전하다.
     */
    _captureRest() {
        for (const bone of this._skeleton.bones) {
            this._rest.set(bone, {
                pos: bone.getAbsolutePosition().clone(),
                rot: bone.getRotationQuaternion(BABYLON.Space.WORLD).clone(),
                worldPos: bone.getAbsolutePosition(this._mesh).clone(),
            });
        }
    }

    /**
     * 리그를 해석해 척추·양팔·양다리 체인을 구성한다. 반환값 없음.
     *
     * 팔·다리는 슬롯별로 따로 찾지 않고, 좌우로 가른 뿌리 본에서 계층을 타고 내려가 짝을 맞춘다
     * (슬롯마다 독립적으로 좌우를 정하면 리그에 따라 좌우가 섞일 수 있다).
     */
    _buildChains() {
        const spine = SPINE_SLOTS.map((slot) => this._findBones(BONE_PATTERNS[slot])[0] ?? null);
        this._chains = [{ bones: spine, joints: SPINE_JOINTS }];

        // 아바타 배율 기준: rest 자세의 엉덩이-머리 거리 (사람 몸통 길이에 맞춘다)
        const hips = spine[0], head = spine[SPINE_SLOTS.indexOf("head")];
        if (hips && head) {
            this._restSpan = BABYLON.Vector3.Distance(
                this._rest.get(hips).worldPos, this._rest.get(head).worldPos,
            );
        }

        for (const [slots, jointsOf] of [[ARM_SLOTS, armJoints], [LEG_SLOTS, legJoints]]) {
            const roots = this._splitSides(this._findBones(BONE_PATTERNS[slots[0]]));
            for (const side of ["left", "right"]) {
                if (roots[side] === null) continue;
                const bones = [roots[side]];
                for (const slot of slots.slice(1)) {
                    const from = bones[bones.length - 1];
                    bones.push(from === null ? null : this._findDescendant(from, BONE_PATTERNS[slot]));
                }
                this._chains.push({ bones, joints: jointsOf(side) });
            }
        }
    }

    /**
     * 패턴에 맞는 본을 전부 찾아 반환한다 (없으면 빈 배열).
     */
    _findBones(pattern) {
        return this._skeleton.bones.filter((b) => pattern.test(b.name));
    }

    /**
     * 본 두 개를 rest 자세의 x 부호로 좌우에 배정해 { left, right }로 반환한다.
     * 사람은 +z를 보도록 정규화되므로 표시 좌표에서 왼쪽은 -x 쪽이다.
     */
    _splitSides(bones) {
        if (bones.length < 2) return { left: null, right: null };
        const sorted = [...bones].sort((a, b) => this._rest.get(a).worldPos.x - this._rest.get(b).worldPos.x);
        return { left: sorted[0], right: sorted[sorted.length - 1] };
    }

    /**
     * 본의 자손 중 패턴에 맞는 첫 본을 너비 우선으로 찾아 반환한다 (없으면 null).
     * 중간에 twist·roll 보조 본이 낀 리그도 통과하도록 자손 전체를 훑는다.
     */
    _findDescendant(bone, pattern) {
        const queue = [...bone.children];
        while (queue.length > 0) {
            const b = queue.shift();
            if (pattern.test(b.name)) return b;
            queue.push(...b.children);
        }
        return null;
    }

    /**
     * 추적 조인트 마커 표시를 켜고 끈다 (리타게팅 디버깅용). 반환값 없음.
     */
    setDebugJoints(on) {
        this._showJoints = on;
        if (!on) for (const mesh of this._markers.values()) mesh.isVisible = false;
    }

    /**
     * 샘플 1개로 아바타 자세를 갱신한다. 반환값 없음.
     * sample: { body: { joints: {이름: {pos, quat}} } } (WebXR 좌표).
     * 어깨를 못 찾으면 아바타를 숨긴다 (자세를 만들 근거가 없으므로).
     */
    update(sample) {
        if (!this.ready) return;

        const placed = this._placeJoints(sample.body?.joints ?? {});
        // 자세를 만들 근거가 없으면 마지막 자세 그대로 둔다 (사라지게 하지 않는다)
        if (placed === null) return;

        this._applyScale(placed);
        this._placeAt(placed.__hips);
        // 월드 → 스켈레톤 공간 변환 (본 회전을 메시 행렬 없이 계산하기 위한 다리)
        const toSkeleton = BABYLON.Matrix.Invert(this._mesh.getWorldMatrix());
        for (const chain of this._chains) this._alignChain(chain, placed, toSkeleton);
        this._updateMarkers(placed);
    }

    /**
     * 추적 조인트를 표시 좌표로 옮기고 로봇 옆자리에 세운 위치 사전을 반환한다 (불가하면 null).
     *
     * 처리: WebXR→표시 z 거울 → 어깨선 yaw를 +z 정면으로 정규화 → 엉덩이를 HUMAN_OFFSET으로 평행이동.
     * 높이(y)는 그대로 두어 실제 키·자세 높이가 보존된다.
     */
    _placeJoints(joints) {
        const raw = {};
        for (const [name, j] of Object.entries(joints)) {
            raw[name] = new BABYLON.Vector3(j.pos[0], j.pos[1], -j.pos[2]);
        }
        const pickRaw = (candidates) => {
            for (const name of candidates) if (raw[name]) return raw[name];
            return null;
        };

        const leftSh = pickRaw(["left-arm-upper", "left-shoulder"]);
        const rightSh = pickRaw(["right-arm-upper", "right-shoulder"]);
        if (leftSh === null || rightSh === null) return null;
        // 엉덩이 조인트가 없는 트래킹을 대비해 어깨 중점에서 내린 지점을 대용으로 쓴다
        const hips = pickRaw(["hips", "root"])
            ?? BABYLON.Vector3.Center(leftSh, rightSh).subtract(new BABYLON.Vector3(0, 0.45, 0));

        // 어깨선(오른쪽→왼쪽)이 -x를 향하도록 하는 y축 회전각.
        // Babylon 왼손계에서 RotationY(θ): x' = x cosθ + z sinθ, z' = -x sinθ + z cosθ.
        // (sx, sz) → (-1, 0)을 만족하는 해가 θ = atan2(-sz, -sx)이다.
        const s = leftSh.subtract(rightSh);
        const len = Math.hypot(s.x, s.z);
        const yaw = len < 1e-6 ? 0 : Math.atan2(-s.z / len, -s.x / len);
        const c = Math.cos(yaw), sn = Math.sin(yaw);

        const out = {};
        for (const [name, p] of Object.entries(raw)) {
            const dx = p.x - hips.x, dz = p.z - hips.z;
            out[name] = new BABYLON.Vector3(
                HUMAN_OFFSET.x + dx * c + dz * sn,
                p.y,
                HUMAN_OFFSET.z - dx * sn + dz * c,
            );
        }
        out.__hips = new BABYLON.Vector3(HUMAN_OFFSET.x, hips.y, HUMAN_OFFSET.z);
        // 이후 조회는 배치가 끝난 좌표에서 해야 하므로 변환된 사전 기준으로 다시 만든다
        out.__pick = (candidates) => {
            for (const name of candidates) if (out[name]) return out[name];
            return null;
        };
        return out;
    }

    /**
     * 사람의 몸통 길이에 맞춰 아바타 균일 배율을 1회 정한다. 반환값 없음.
     */
    _applyScale(placed) {
        if (this._scaled) return;
        const head = placed.__pick(["head"]);
        if (head === null || this._restSpan < 1e-6) return;
        const span = BABYLON.Vector3.Distance(placed.__hips, head);
        const scale = Math.max(0.5, Math.min(2.0, span / this._restSpan));
        this._container.scaling.setAll(scale);
        this._scaled = true;
        console.log(`[human] 아바타 배율 ${scale.toFixed(3)} (사람 몸통 ${span.toFixed(2)}m)`);
    }

    /**
     * rest 자세의 엉덩이가 로봇 옆 기본 자리에 오도록 하는 목표 위치를 반환한다.
     * 높이는 아바타 자신의 rest 높이를 써서 발이 바닥에 닿는다.
     */
    _restHipsWorld() {
        const hips = this._chains[0].bones[0];
        const y = hips ? this._rest.get(hips).worldPos.y : 1.0;
        return new BABYLON.Vector3(HUMAN_OFFSET.x, y, HUMAN_OFFSET.z);
    }

    /**
     * 엉덩이 본이 target(월드)에 오도록 컨테이너를 평행이동한다. 반환값 없음.
     *
     * 현재 엉덩이 월드 위치와의 차이만큼 미는 방식이라 배율·거울에 영향을 받지 않는다
     * (엉덩이는 루트 본이라 회전해도 제자리다).
     * 이동 후 월드 행렬을 즉시 갱신한다 — 이어지는 본 회전이 메시 월드 행렬을 참조하기 때문이다.
     */
    _placeAt(target) {
        const hips = this._chains[0].bones[0];
        if (!hips) return;
        const current = BABYLON.Vector3.TransformCoordinates(
            this._rest.get(hips).pos, this._mesh.getWorldMatrix(),
        );
        this._container.position.addInPlace(target.subtract(current));
        this._container.computeWorldMatrix(true);
        this._mesh.computeWorldMatrix(true);
    }

    /**
     * 체인 하나를 추적 조인트 방향에 맞춘다. 반환값 없음.
     * bones[i]는 joints[i] → joints[i+1] 방향을 향하게 되고, 마지막 본은 회전시키지 않는다.
     * 부모부터 처리해야 자식의 월드 회전 계산이 갱신된 부모 변환을 쓴다.
     */
    _alignChain(chain, placed, toSkeleton) {
        const { bones, joints } = chain;
        for (let i = 0; i < bones.length - 1; i++) {
            const bone = bones[i], next = bones[i + 1];
            if (bone === null || next === null) continue;

            const from = placed.__pick(joints[i]);
            const to = placed.__pick(joints[i + 1]);
            if (from === null || to === null) continue;

            // 목표 방향(월드)을 스켈레톤 공간으로 옮겨 rest 방향과 같은 기준에서 비교한다
            const target = BABYLON.Vector3.TransformNormal(to.subtract(from), toSkeleton);
            const rest = this._rest.get(next).pos.subtract(this._rest.get(bone).pos);
            if (target.lengthSquared() < 1e-8 || rest.lengthSquared() < 1e-8) continue;

            const q = _shortestArc(rest.normalize(), target.normalize());
            bone.setRotationQuaternion(q.multiply(this._rest.get(bone).rot), BABYLON.Space.WORLD);
        }
    }

    /**
     * 디버그 마커 위치를 갱신한다. 반환값 없음.
     */
    _updateMarkers(placed) {
        if (!this._showJoints) return;
        for (const [name, p] of Object.entries(placed)) {
            if (name.startsWith("__")) continue;
            let mesh = this._markers.get(name);
            if (!mesh) {
                mesh = BABYLON.MeshBuilder.CreateSphere(
                    `humanJoint_${name}`, { diameter: JOINT_MARKER_DIAMETER, segments: 6 }, this._scene,
                );
                const mat = new BABYLON.StandardMaterial(`humanJointMat_${name}`, this._scene);
                mat.emissiveColor = new BABYLON.Color3(1.0, 0.55, 0.15);
                mat.disableLighting = true;
                mesh.material = mat;
                mesh.isPickable = false;
                this._markers.set(name, mesh);
            }
            mesh.position.copyFrom(p);
            mesh.isVisible = true;
        }
    }
}

/**
 * 단위 벡터 a를 b로 보내는 최단호 회전 쿼터니언을 반환한다.
 * 정반대 방향(내적 -1)은 축이 정의되지 않으므로 a에 수직인 임의 축으로 180° 회전을 쓴다.
 */
function _shortestArc(a, b) {
    const dot = BABYLON.Vector3.Dot(a, b);
    if (dot > 0.999999) return BABYLON.Quaternion.Identity();
    if (dot < -0.999999) {
        const axis = Math.abs(a.x) < 0.9
            ? BABYLON.Vector3.Cross(a, BABYLON.Axis.X)
            : BABYLON.Vector3.Cross(a, BABYLON.Axis.Y);
        return BABYLON.Quaternion.RotationAxis(axis.normalize(), Math.PI);
    }
    const axis = BABYLON.Vector3.Cross(a, b);
    return new BABYLON.Quaternion(axis.x, axis.y, axis.z, 1 + dot).normalize();
}
