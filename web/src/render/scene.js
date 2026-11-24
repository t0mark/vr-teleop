// Babylon.js 기본 씬 구성: 엔진, 데스크톱용 궤도 카메라, 조명, 바닥 그리드, 원점 축.
// 좌표계: Babylon 기본(왼손, y-up). WebXR 포즈(오른손)는 각 표시 모듈에서 z 반전으로 변환한다
// (robot_view·human_view·mesh_view 공통 규약).

/**
 * 엔진과 씬을 생성해 { engine, scene } 을 반환한다.
 * canvas: 렌더링 대상 <canvas> 요소.
 */
export function createScene(canvas) {
    const engine = new BABYLON.Engine(canvas, true, { stencil: true });
    const scene = new BABYLON.Scene(engine);
    scene.clearColor = new BABYLON.Color4(0.05, 0.06, 0.09, 1.0);

    // 데스크톱 궤도 카메라: 로봇 정면(+z 쪽)에서 로봇과 사람 아바타를 함께 담는 초기 시점
    const camera = new BABYLON.ArcRotateCamera(
        "desktopCam", Math.PI / 2, Math.PI / 2.4, 3.5,
        new BABYLON.Vector3(-0.5, 1.0, 0), scene,
    );
    camera.attachControl(canvas, true);
    camera.wheelDeltaPercentage = 0.01;

    // 반구광
    new BABYLON.HemisphericLight("light", new BABYLON.Vector3(0, 1, 0), scene);

    // 바닥 그리드·원점 축
    _createFloorGrid(scene);
    _createOriginAxes(scene);

    return { engine, scene };
}

/**
 * 바닥 그리드(10m x 10m, 0.5m 간격)를 라인 메시로 생성한다. 반환값 없음.
 */
function _createFloorGrid(scene) {
    const halfSize = 5.0;
    const spacing = 0.5;

    // x·z 방향 격자선
    const lines = [];
    for (let coord = -halfSize; coord <= halfSize + 1e-6; coord += spacing) {
        lines.push([new BABYLON.Vector3(coord, 0, -halfSize), new BABYLON.Vector3(coord, 0, halfSize)]);
        lines.push([new BABYLON.Vector3(-halfSize, 0, coord), new BABYLON.Vector3(halfSize, 0, coord)]);
    }

    // 라인 메시 생성
    const grid = BABYLON.MeshBuilder.CreateLineSystem("floorGrid", { lines }, scene);
    grid.color = new BABYLON.Color3(0.25, 0.28, 0.35);
    grid.isPickable = false;
}

/**
 * 원점 기준 축 표시(x=빨강, y=초록, z=파랑, 길이 0.5m)를 생성한다. 반환값 없음.
 */
function _createOriginAxes(scene) {
    const axisLength = 0.5;
    const axes = [
        { name: "axisX", direction: new BABYLON.Vector3(axisLength, 0, 0), color: new BABYLON.Color3(1, 0.2, 0.2) },
        { name: "axisY", direction: new BABYLON.Vector3(0, axisLength, 0), color: new BABYLON.Color3(0.2, 1, 0.2) },
        { name: "axisZ", direction: new BABYLON.Vector3(0, 0, axisLength), color: new BABYLON.Color3(0.3, 0.5, 1) },
    ];

    // 축마다 라인 메시 생성
    for (const axis of axes) {
        const line = BABYLON.MeshBuilder.CreateLines(axis.name, {
            points: [BABYLON.Vector3.Zero(), axis.direction],
        }, scene);
        line.color = axis.color;
        line.isPickable = false;
    }
}
