// 포즈 마커 렌더링: 수신한 샘플의 body 조인트·컨트롤러 위치를 구체로 표시한다.
// 데스크톱 모니터링 뷰 전용.
// 좌표 변환: WebXR(오른손, -z 전방) → Babylon 왼손 씬은 z 부호 반전으로 대응
// (x·y는 동일, Babylon 자체의 XR 카메라 변환과 같은 규약).

// 마커 크기(m): body 조인트는 작게, 컨트롤러(손목)는 크게 구분
const JOINT_DIAMETER = 0.035;
const HAND_DIAMETER = 0.06;

export class PoseMarkers {
    /**
     * 마커 관리 상태를 초기화한다. scene: Babylon 씬.
     */
    constructor(scene) {
        this._scene = scene;
        // 이름 → 메시 매핑: 조인트 개수가 프레임마다 흔들려도 재사용하도록 캐시
        this._markers = new Map();
        this._materials = this._buildMaterials();
    }

    /**
     * 좌/우/기타 구분용 재질 3종을 만들어 반환한다.
     */
    _buildMaterials() {
        const make = (name, color) => {
            const mat = new BABYLON.StandardMaterial(name, this._scene);
            mat.emissiveColor = color;
            mat.disableLighting = true;
            return mat;
        };
        return {
            left: make("markerLeft", new BABYLON.Color3(1.0, 0.55, 0.15)),
            right: make("markerRight", new BABYLON.Color3(0.2, 0.8, 1.0)),
            other: make("markerOther", new BABYLON.Color3(0.7, 0.7, 0.75)),
        };
    }

    /**
     * 이름에 해당하는 마커 메시를 반환한다(없으면 생성). diameter: 구체 지름(m).
     */
    _getMarker(name, diameter) {
        let mesh = this._markers.get(name);
        if (!mesh) {
            mesh = BABYLON.MeshBuilder.CreateSphere(`marker_${name}`, { diameter, segments: 8 }, this._scene);
            const side = name.startsWith("left") ? "left" : name.startsWith("right") ? "right" : "other";
            mesh.material = this._materials[side];
            mesh.isPickable = false;
            this._markers.set(name, mesh);
        }
        return mesh;
    }

    /**
     * 샘플 1개로 모든 마커 위치를 갱신한다. 반환값 없음.
     * 이번 샘플에 없는 마커는 숨겨서 잔상을 남기지 않는다.
     */
    update(sample) {
        const seen = new Set();

        for (const [name, j] of Object.entries(sample.body?.joints ?? {})) {
            const mesh = this._getMarker(name, JOINT_DIAMETER);
            // WebXR 오른손 → Babylon 왼손: z 반전
            mesh.position.set(j.pos[0], j.pos[1], -j.pos[2]);
            mesh.isVisible = true;
            seen.add(name);
        }

        for (const hand of ["left", "right"]) {
            const c = sample.hands?.[hand];
            if (!c) continue;
            const key = `${hand}-controller`;
            const mesh = this._getMarker(key, HAND_DIAMETER);
            mesh.position.set(c.pos[0], c.pos[1], -c.pos[2]);
            mesh.isVisible = true;
            seen.add(key);
        }

        for (const [name, mesh] of this._markers) {
            if (!seen.has(name)) mesh.isVisible = false;
        }
    }
}
