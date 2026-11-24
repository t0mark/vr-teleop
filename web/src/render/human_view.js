// 사람 아바타 렌더링: body-tracking 조인트로 리깅 아바타의 본을 돌려 조작자 자세를 재현한다 (데스크톱 뷰 전용).
//
// 리타게팅: 방향 정합. 본마다 rest 방향 d0와 추적 조인트로 만든 목표 방향 d1 사이의 최단호 회전을
//   rest 회전에 좌곱해 본 회전으로 넣는다 (본 축 회전(roll)은 재현하지 않는다).
// 본 조작은 스켈레톤 공간에서 한다: rest 자세와 목표 방향을 스켈레톤 공간에서 잡고 mesh 인자 없이 회전을 넣는다.
//   목표 방향은 메시 월드 행렬의 역행렬로 스켈레톤 공간으로 옮긴다.
// 리그 해석: 본 이름 접미사 패턴 + 계층 탐색 (Mixamo 계열 humanoid 리그 기준). 좌우는 rest 자세의 x 부호로 가른다.
// 좌표: 추적 조인트 WebXR(오른손) → 표시(왼손) z 거울(x, y, -z).
// 배치: 어깨선 yaw를 로봇과 같은 방향(+z)으로 정규화하고 엉덩이를 HUMAN_OFFSET 위치로 옮긴다.
// 에셋: web/assets/avatar.glb = Mixamo "X Bot".

import { ROBOT_OFFSET } from "./robot_view.js";

// 로봇 옆 배치 위치(m): 로봇 왼쪽 옆자리
export const HUMAN_OFFSET = { x: ROBOT_OFFSET.x - 1.5, z: ROBOT_OFFSET.z };

// 본 슬롯 판별 패턴 (본 이름 접미사 기준, 대소문자 무시, ForeArm/UpLeg는 후방탐색으로 배제)
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

// 척추 체인: 본 슬롯과 대응 추적 조인트 후보 (본 i는 조인트 i → i+1 방향을 향한다)
const SPINE_SLOTS = ["hips", "spine", "spine1", "spine2", "neck", "head"];
const SPINE_JOINTS = [
    ["hips", "root"],
    ["spine-lower", "spine-middle"],
    ["spine-middle", "spine-upper"],
    ["spine-upper", "chest"],
    ["neck"],
    ["head"],
];

// 팔·다리 체인 슬롯 (어깨(쇄골)부터 손까지, 엉덩관절부터 발끝까지)
const ARM_SLOTS = ["shoulder", "upperArm", "foreArm", "hand"];
const LEG_SLOTS = ["upLeg", "lowerLeg", "foot", "toe"];

// 어깨 조인트 후보와 엉덩이 조인트 후보
const LEFT_SHOULDER_JOINTS = ["left-arm-upper", "left-shoulder"];
const RIGHT_SHOULDER_JOINTS = ["right-arm-upper", "right-shoulder"];
const HIPS_JOINTS = ["hips", "root"];
// 엉덩이 조인트가 없을 때 어깨 중점에서 내리는 높이(m)
const HIPS_FALLBACK_DROP_M = 0.45;
// 아바타 배율 한계
const MIN_AVATAR_SCALE = 0.5;
const MAX_AVATAR_SCALE = 2.0;
// 디버그 마커 지름(m)
const JOINT_MARKER_DIAMETER = 0.04;

/**
 * 팔 체인의 슬롯별 추적 조인트 후보를 반환한다. side: left/right.
 */
function _armJoints(side) {
    return [
        [`${side}-shoulder`, `${side}-scapula`],
        [`${side}-arm-upper`],
        [`${side}-arm-lower`],
        [`${side}-hand-wrist`, `${side}-wrist`],
    ];
}

/**
 * 다리 체인의 슬롯별 추적 조인트 후보를 반환한다. side: left/right.
 */
function _legJoints(side) {
    return [
        [`${side}-upper-leg`, `${side}-leg-upper`],
        [`${side}-lower-leg`, `${side}-leg-lower`],
        [`${side}-foot-ankle`, `${side}-ankle`],
        [`${side}-foot-ball`, `${side}-foot-toe`],
    ];
}

/**
 * 조인트 사전에서 후보 이름 중 처음 있는 조인트 위치를 반환한다 (없으면 null).
 */
function _pickJoint(joints, candidates) {
    for (const name of candidates) {
        if (joints[name]) return joints[name];
    }
    return null;
}

export class HumanView {
    /**
     * 아바타 표시 상태를 초기화한다. scene: Babylon 씬.
     */
    constructor(scene) {
        this._scene = scene;
        this._container = null;
        this._mesh = null;
        this._skeleton = null;
        // 본 → { pos, rot, worldPos } (rest 자세)
        this._rest = new Map();
        // 체인 정의: [{ bones: [Bone|null], joints: [[후보 이름]] }]
        this._chains = [];
        this._restSpan = 1.0;
        this._scaled = false;
        this._markers = new Map();
        this._showJoints = false;
        // 리그 해석 완료 여부 (자세 갱신 가능 여부)
        this._ready = false;
    }

    /**
     * 아바타 GLB를 로드하고 리그를 해석해 rest 자세를 기록한다. 반환값 없음.
     * base·file: 정적 서빙 경로와 파일명.
     */
    async load(base, file) {
        this._container = new BABYLON.TransformNode("humanRoot", this._scene);

        // GLB 로드
        let result;
        try {
            result = await BABYLON.SceneLoader.ImportMeshAsync(null, base, file, this._scene);
        } catch (e) {
            console.warn("[human] 아바타 로드 실패:", e);
            return;
        }

        // 딸린 애니메이션 정지
        for (const group of result.animationGroups ?? []) group.stop();

        // 스켈레톤·메시 연결 (경계 상자 컬링 제외)
        this._skeleton = result.skeletons[0] ?? null;
        this._mesh = result.meshes.find((mesh) => mesh.skeleton) ?? result.meshes[0];
        for (const mesh of result.meshes) {
            if (mesh.parent === null) mesh.parent = this._container;
            mesh.isPickable = false;
            mesh.alwaysSelectAsActiveMesh = true;
        }
        if (this._skeleton === null) {
            console.warn("[human] 아바타에 스켈레톤이 없어 자세 재현 불가");
            return;
        }

        // 리그 해석
        this._detachTransformNodes();
        this._captureRest();
        this._buildChains();
        this._ready = this._chains.length > 0;

        // 자세 샘플 전에는 로봇 옆 기본 자리에 rest 자세로 세운다
        this._placeAt(this._restHipsWorld());
        console.log(`[human] 아바타 로드 완료: 본 ${this._skeleton.bones.length}개, 체인 ${this._chains.length}개`);
    }

    /**
     * 본에 연결된 TransformNode 링크를 끊어 본을 직접 회전할 수 있게 한다. 반환값 없음.
     */
    _detachTransformNodes() {
        for (const bone of this._skeleton.bones) {
            if (typeof bone.linkTransformNode !== "function") continue;
            if (bone.getTransformNode?.() != null) bone.linkTransformNode(null);
        }
    }

    /**
     * 모든 본의 rest(초기) 자세를 기록한다 (본을 돌리기 전에 호출). 반환값 없음.
     * pos·rot: 스켈레톤 공간 (회전 계산용), worldPos: 메시 월드 좌표 (좌우 판별·배율 계산용).
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
     * 팔·다리는 좌우로 가른 뿌리 본에서 계층을 타고 내려가 슬롯을 채운다.
     */
    _buildChains() {
        // 척추 체인
        const spine = SPINE_SLOTS.map((slot) => this._findBones(BONE_PATTERNS[slot])[0] ?? null);
        this._chains = [{ bones: spine, joints: SPINE_JOINTS }];

        // 아바타 배율 기준: rest 자세의 엉덩이-머리 거리
        const hipsBone = spine[0];
        const headBone = spine[SPINE_SLOTS.indexOf("head")];
        if (hipsBone && headBone) {
            this._restSpan = BABYLON.Vector3.Distance(
                this._rest.get(hipsBone).worldPos, this._rest.get(headBone).worldPos,
            );
        }

        // 팔·다리 체인
        for (const [slots, jointsOf] of [[ARM_SLOTS, _armJoints], [LEG_SLOTS, _legJoints]]) {
            const roots = this._splitSides(this._findBones(BONE_PATTERNS[slots[0]]));
            for (const side of ["left", "right"]) {
                if (roots[side] === null) continue;
                const bones = [roots[side]];
                for (const slot of slots.slice(1)) {
                    const parentBone = bones[bones.length - 1];
                    bones.push(parentBone === null ? null : this._findDescendant(parentBone, BONE_PATTERNS[slot]));
                }
                this._chains.push({ bones, joints: jointsOf(side) });
            }
        }
    }

    /**
     * 패턴에 맞는 본을 전부 찾아 반환한다 (없으면 빈 배열).
     */
    _findBones(pattern) {
        return this._skeleton.bones.filter((bone) => pattern.test(bone.name));
    }

    /**
     * 본들을 rest 자세의 x 부호로 좌우에 배정해 { left, right }로 반환한다 (표시 좌표 왼쪽 = -x).
     */
    _splitSides(bones) {
        if (bones.length < 2) return { left: null, right: null };
        const sorted = [...bones].sort((boneA, boneB) => this._rest.get(boneA).worldPos.x - this._rest.get(boneB).worldPos.x);
        return { left: sorted[0], right: sorted[sorted.length - 1] };
    }

    /**
     * 본의 자손 중 패턴에 맞는 첫 본을 너비 우선으로 찾아 반환한다 (없으면 null).
     */
    _findDescendant(bone, pattern) {
        const queue = [...bone.children];
        while (queue.length > 0) {
            const candidate = queue.shift();
            if (pattern.test(candidate.name)) return candidate;
            queue.push(...candidate.children);
        }
        return null;
    }

    /**
     * 추적 조인트 마커 표시를 켜고 끈다 (리타게팅 디버깅용). 반환값 없음.
     */
    setDebugJoints(on) {
        this._showJoints = on;
        if (!on) {
            for (const marker of this._markers.values()) marker.isVisible = false;
        }
    }

    /**
     * 샘플 1개로 아바타 자세를 갱신한다 (어깨를 못 찾으면 마지막 자세 유지). 반환값 없음.
     * sample: { body: { joints: {이름: {pos, quat}} } } (WebXR 좌표).
     */
    update(sample) {
        if (!this._ready) return;

        // 조인트를 표시 좌표로 배치
        const placed = this._placeJoints(sample.body?.joints ?? {});
        if (placed === null) return;

        // 배율·위치 반영
        this._applyScale(placed);
        this._placeAt(placed.hips);

        // 월드 → 스켈레톤 공간 변환으로 체인별 본 회전
        const toSkeleton = BABYLON.Matrix.Invert(this._mesh.getWorldMatrix());
        for (const chain of this._chains) this._alignChain(chain, placed.joints, toSkeleton);
        this._updateMarkers(placed.joints);
    }

    /**
     * 추적 조인트를 표시 좌표로 옮겨 로봇 옆자리에 세운 { joints, hips }를 반환한다 (어깨가 없으면 null).
     * 처리: z 거울 → 어깨선 yaw를 +z 정면으로 정규화 → 엉덩이를 HUMAN_OFFSET으로 평행이동 (높이 유지).
     */
    _placeJoints(sampleJoints) {
        // WebXR → 표시 좌표 z 거울
        const mirrored = {};
        for (const [name, joint] of Object.entries(sampleJoints)) {
            mirrored[name] = new BABYLON.Vector3(joint.pos[0], joint.pos[1], -joint.pos[2]);
        }

        // 어깨·엉덩이 (엉덩이가 없으면 어깨 중점에서 내린 지점)
        const leftShoulder = _pickJoint(mirrored, LEFT_SHOULDER_JOINTS);
        const rightShoulder = _pickJoint(mirrored, RIGHT_SHOULDER_JOINTS);
        if (leftShoulder === null || rightShoulder === null) return null;
        const hips = _pickJoint(mirrored, HIPS_JOINTS)
            ?? BABYLON.Vector3.Center(leftShoulder, rightShoulder).subtract(new BABYLON.Vector3(0, HIPS_FALLBACK_DROP_M, 0));

        // 어깨선(오른쪽→왼쪽)이 -x를 향하도록 하는 y축 회전각: θ = atan2(-sz, -sx)
        const shoulderLine = leftShoulder.subtract(rightShoulder);
        const shoulderLength = Math.hypot(shoulderLine.x, shoulderLine.z);
        const yaw = shoulderLength < 1e-6 ? 0 : Math.atan2(-shoulderLine.z / shoulderLength, -shoulderLine.x / shoulderLength);
        const cosYaw = Math.cos(yaw);
        const sinYaw = Math.sin(yaw);

        // 엉덩이 기준 yaw 회전 후 HUMAN_OFFSET으로 평행이동
        const joints = {};
        for (const [name, position] of Object.entries(mirrored)) {
            const dx = position.x - hips.x;
            const dz = position.z - hips.z;
            joints[name] = new BABYLON.Vector3(
                HUMAN_OFFSET.x + dx * cosYaw + dz * sinYaw,
                position.y,
                HUMAN_OFFSET.z - dx * sinYaw + dz * cosYaw,
            );
        }
        return { joints, hips: new BABYLON.Vector3(HUMAN_OFFSET.x, hips.y, HUMAN_OFFSET.z) };
    }

    /**
     * 사람의 몸통 길이에 맞춰 아바타 균일 배율을 1회 정한다. 반환값 없음.
     */
    _applyScale(placed) {
        if (this._scaled) return;
        const head = _pickJoint(placed.joints, ["head"]);
        if (head === null || this._restSpan < 1e-6) return;

        // 배율 = 사람 엉덩이-머리 거리 / 아바타 rest 거리
        const span = BABYLON.Vector3.Distance(placed.hips, head);
        const scale = Math.max(MIN_AVATAR_SCALE, Math.min(MAX_AVATAR_SCALE, span / this._restSpan));
        this._container.scaling.setAll(scale);
        this._scaled = true;
        console.log(`[human] 아바타 배율 ${scale.toFixed(3)} (사람 몸통 ${span.toFixed(2)}m)`);
    }

    /**
     * rest 자세의 엉덩이가 로봇 옆 기본 자리에 오도록 하는 목표 위치를 반환한다 (높이는 아바타 rest 높이).
     */
    _restHipsWorld() {
        const hipsBone = this._chains[0].bones[0];
        const hipsY = hipsBone ? this._rest.get(hipsBone).worldPos.y : 1.0;
        return new BABYLON.Vector3(HUMAN_OFFSET.x, hipsY, HUMAN_OFFSET.z);
    }

    /**
     * 엉덩이 본이 target(월드)에 오도록 컨테이너를 평행이동하고 월드 행렬을 즉시 갱신한다. 반환값 없음.
     */
    _placeAt(target) {
        const hipsBone = this._chains[0].bones[0];
        if (!hipsBone) return;

        // 현재 엉덩이 월드 위치와의 차이만큼 이동
        const current = BABYLON.Vector3.TransformCoordinates(
            this._rest.get(hipsBone).pos, this._mesh.getWorldMatrix(),
        );
        this._container.position.addInPlace(target.subtract(current));

        // 이어지는 본 회전을 위한 월드 행렬 갱신
        this._container.computeWorldMatrix(true);
        this._mesh.computeWorldMatrix(true);
    }

    /**
     * 체인 하나를 추적 조인트 방향에 맞춘다 (부모부터, 마지막 본은 회전하지 않음). 반환값 없음.
     * bones[i]는 joints[i] → joints[i+1] 방향을 향하게 된다.
     */
    _alignChain(chain, placedJoints, toSkeleton) {
        const { bones, joints } = chain;
        for (let boneIndex = 0; boneIndex < bones.length - 1; boneIndex++) {
            const bone = bones[boneIndex];
            const nextBone = bones[boneIndex + 1];
            if (bone === null || nextBone === null) continue;

            // 목표 구간 조인트
            const fromJoint = _pickJoint(placedJoints, joints[boneIndex]);
            const toJoint = _pickJoint(placedJoints, joints[boneIndex + 1]);
            if (fromJoint === null || toJoint === null) continue;

            // 목표 방향(스켈레톤 공간)과 rest 방향
            const targetDirection = BABYLON.Vector3.TransformNormal(toJoint.subtract(fromJoint), toSkeleton);
            const restDirection = this._rest.get(nextBone).pos.subtract(this._rest.get(bone).pos);
            if (targetDirection.lengthSquared() < 1e-8 || restDirection.lengthSquared() < 1e-8) continue;

            // rest 회전에 최단호 회전을 좌곱해 적용
            const arcRotation = _shortestArc(restDirection.normalize(), targetDirection.normalize());
            bone.setRotationQuaternion(arcRotation.multiply(this._rest.get(bone).rot), BABYLON.Space.WORLD);
        }
    }

    /**
     * 디버그 마커 위치를 갱신한다 (표시가 꺼져 있으면 생략). 반환값 없음.
     */
    _updateMarkers(placedJoints) {
        if (!this._showJoints) return;
        for (const [name, position] of Object.entries(placedJoints)) {
            // 마커가 없으면 생성
            let marker = this._markers.get(name);
            if (!marker) {
                marker = BABYLON.MeshBuilder.CreateSphere(
                    `humanJoint_${name}`, { diameter: JOINT_MARKER_DIAMETER, segments: 6 }, this._scene,
                );
                const markerMaterial = new BABYLON.StandardMaterial(`humanJointMat_${name}`, this._scene);
                markerMaterial.emissiveColor = new BABYLON.Color3(1.0, 0.55, 0.15);
                markerMaterial.disableLighting = true;
                marker.material = markerMaterial;
                marker.isPickable = false;
                this._markers.set(name, marker);
            }

            // 위치 갱신
            marker.position.copyFrom(position);
            marker.isVisible = true;
        }
    }
}

/**
 * 단위 벡터 fromDir을 toDir로 보내는 최단호 회전 쿼터니언을 반환한다.
 * 정반대 방향이면 fromDir에 수직인 축으로 180° 회전한다.
 */
function _shortestArc(fromDir, toDir) {
    const dot = BABYLON.Vector3.Dot(fromDir, toDir);
    if (dot > 0.999999) return BABYLON.Quaternion.Identity();

    // 정반대 방향
    if (dot < -0.999999) {
        const perpendicularAxis = Math.abs(fromDir.x) < 0.9
            ? BABYLON.Vector3.Cross(fromDir, BABYLON.Axis.X)
            : BABYLON.Vector3.Cross(fromDir, BABYLON.Axis.Y);
        return BABYLON.Quaternion.RotationAxis(perpendicularAxis.normalize(), Math.PI);
    }

    // 일반: 축 = from × to, w = 1 + dot
    const axis = BABYLON.Vector3.Cross(fromDir, toDir);
    return new BABYLON.Quaternion(axis.x, axis.y, axis.z, 1 + dot).normalize();
}
