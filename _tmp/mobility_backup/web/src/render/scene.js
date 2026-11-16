// Babylon.js 기본 씬 구성: 엔진, 데스크톱용 궤도 카메라, 바닥 그리드, 원점 축.
// 좌표계: Babylon 기본(왼손, y-up)을 유지한다. 오른손 모드로 바꾸면 카메라 전방이 -z가 되어
// 카메라에 고정한 HUD(+z 배치)가 뒤로 가는 문제가 있음. WebXR 포즈(오른손)의 표시 변환은
// 표시 단계에서 z 반전으로 처리한다 (robot_view·human_view·pointcloud_view 공통 규약).

/**
 * 엔진과 씬을 생성해 { engine, scene } 을 반환한다.
 * canvas: 렌더링 대상 <canvas> 요소.
 */
export function createScene(canvas) {
    const engine = new BABYLON.Engine(canvas, true, { stencil: true });
    const scene = new BABYLON.Scene(engine);
    scene.clearColor = new BABYLON.Color4(0.05, 0.06, 0.09, 1.0);

    // 데스크톱 모니터링용 궤도 카메라 (VR 진입 시 Babylon이 XR 카메라로 자동 전환).
    // 로봇 정면(+z 쪽)에서 로봇과 그 옆 사람 아바타를 함께 담도록 초기 시점을 잡는다
    const camera = new BABYLON.ArcRotateCamera(
        "desktopCam", Math.PI / 2, Math.PI / 2.4, 3.5,
        new BABYLON.Vector3(-0.5, 1.0, 0), scene,
    );
    camera.attachControl(canvas, true);
    camera.wheelDeltaPercentage = 0.01;
    // 방향키 기본 바인딩 해제 — 수동 조작(주행 키)과 충돌한다. 궤도 조작은 마우스로 충분
    camera.keysUp = [];
    camera.keysDown = [];
    camera.keysLeft = [];
    camera.keysRight = [];

    // 조명은 방향성 없는 반구광 하나로 충분 (표시물은 라인·HUD 위주)
    new BABYLON.HemisphericLight("light", new BABYLON.Vector3(0, 1, 0), scene);

    _createFloorGrid(scene);
    _createOriginAxes(scene);

    return { engine, scene };
}

/**
 * 바닥 그리드(10m x 10m, 0.5m 간격)를 라인 메시로 생성한다. 반환값 없음.
 * GridMaterial 등 추가 라이브러리 의존을 피하기 위해 라인 시스템을 사용한다.
 */
function _createFloorGrid(scene) {
    const half = 5.0;
    const step = 0.5;
    const lines = [];
    for (let v = -half; v <= half + 1e-6; v += step) {
        lines.push([new BABYLON.Vector3(v, 0, -half), new BABYLON.Vector3(v, 0, half)]);
        lines.push([new BABYLON.Vector3(-half, 0, v), new BABYLON.Vector3(half, 0, v)]);
    }
    const grid = BABYLON.MeshBuilder.CreateLineSystem("floorGrid", { lines }, scene);
    grid.color = new BABYLON.Color3(0.25, 0.28, 0.35);
    grid.isPickable = false;
}

/**
 * 원점 기준 축 표시(x=빨강, y=초록, z=파랑, 길이 0.5m)를 생성한다. 반환값 없음.
 */
function _createOriginAxes(scene) {
    const len = 0.5;
    const axes = [
        { name: "axisX", dir: new BABYLON.Vector3(len, 0, 0), color: new BABYLON.Color3(1, 0.2, 0.2) },
        { name: "axisY", dir: new BABYLON.Vector3(0, len, 0), color: new BABYLON.Color3(0.2, 1, 0.2) },
        { name: "axisZ", dir: new BABYLON.Vector3(0, 0, len), color: new BABYLON.Color3(0.3, 0.5, 1) },
    ];
    for (const a of axes) {
        const line = BABYLON.MeshBuilder.CreateLines(a.name, {
            points: [BABYLON.Vector3.Zero(), a.dir],
        }, scene);
        line.color = a.color;
        line.isPickable = false;
    }
}
