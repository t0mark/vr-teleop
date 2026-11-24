// WebXR 세션 관리: Babylon 기본 XR 경험 생성과 immersive-vr 진입.
// 요청할 선택 기능(optionalFeatures)은 입력 모듈이 정한다 (input/controller_input.js).
// 선택 기능이라 미지원이어도 세션은 열린다.

/**
 * Babylon WebXR 경험을 생성해 반환한다 (자동 UI·텔레포트·포인터 선택 비활성).
 * WebXR 미지원 환경이면 null을 반환해 데스크톱 뷰만 동작하게 한다.
 * 컨트롤러는 손등에 부착된 위치 추적기로만 쓰므로 3D 모델·레이(포인터 선택)·근접 상호작용을 모두 끈다.
 * 시점은 main.js가 기준 공간 오프셋으로 로봇 머리에 고정하므로, 진입 시 첫 프레임 보정과 데스크톱 카메라 위치·방향 상속을 모두 제거한다.
 */
export async function createXR(scene) {
    try {
        const xr = await scene.createDefaultXRExperienceAsync({
            disableDefaultUI: true,
            disableTeleportation: true,
            disableNearInteraction: true,
            disablePointerSelection: true,
            inputOptions: { doNotLoadControllerMeshes: true },
        });
        xr.baseExperience.camera.compensateOnFirstFrame = false;

        // 진입 직후 XR 카메라에 복사된 데스크톱 카메라 위치·방향을 원점·정면으로 초기화
        xr.baseExperience.onInitialXRPoseSetObservable.add((camera) => {
            camera.position.set(0, 0, 0);
            camera.rotationQuaternion.set(0, 0, 0, 1);
            console.log("[xr] 진입 시 데스크톱 카메라 자세 상속 제거");
        });

        return xr;
    } catch (e) {
        console.warn("WebXR 초기화 불가 (데스크톱 뷰로 진행):", e);
        return null;
    }
}

/**
 * immersive-vr 세션에 진입한다. 반환: 진입 성공 여부(boolean).
 * optionalFeatures: 입력 모듈이 요청하는 WebXR 선택 기능 목록.
 * referenceSpace는 바닥 기준(local-floor)을 사용해 로봇 좌표 변환 시 높이 원점을 명확히 한다.
 */
export async function enterVR(xr, optionalFeatures = []) {
    try {
        await xr.baseExperience.enterXRAsync("immersive-vr", "local-floor", undefined, {
            optionalFeatures,
        });
        return true;
    } catch (e) {
        console.error("VR 진입 실패:", e);
        return false;
    }
}

/**
 * immersive-vr 지원 여부를 반환한다 (버튼 노출 판단용).
 */
export async function isVRSupported() {
    if (!navigator.xr) return false;
    try {
        return await navigator.xr.isSessionSupported("immersive-vr");
    } catch {
        return false;
    }
}
