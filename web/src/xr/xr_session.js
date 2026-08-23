// WebXR 세션 관리: Babylon 기본 XR 경험 생성과 immersive-vr 진입.
// body-tracking은 Quest 브라우저 실험 기능이라 optionalFeatures로 요청한다 (미지원이어도 세션은 열림).

/**
 * Babylon WebXR 경험을 생성해 반환한다 (자동 UI·텔레포트 비활성).
 * WebXR 미지원 환경이면 null을 반환해 데스크톱 뷰만 동작하게 한다.
 * 오프라인 직결 환경이므로 컨트롤러 3D 모델(CDN 로드)을 받지 않는다.
 */
export async function createXR(scene) {
    try {
        const xr = await scene.createDefaultXRExperienceAsync({
            disableDefaultUI: true,
            disableTeleportation: true,
            inputOptions: { doNotLoadControllerMeshes: true },
        });
        return xr;
    } catch (e) {
        console.warn("WebXR 초기화 불가 (데스크톱 뷰로 진행):", e);
        return null;
    }
}

/**
 * immersive-vr 세션에 진입한다. 반환: 진입 성공 여부(boolean).
 * referenceSpace는 바닥 기준(local-floor)을 사용해 로봇 좌표 변환 시 높이 원점을 명확히 한다.
 */
export async function enterVR(xr) {
    try {
        await xr.baseExperience.enterXRAsync("immersive-vr", "local-floor", undefined, {
            optionalFeatures: ["body-tracking"],
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
