// 포즈·입력 상태 표시 HUD: VR에서는 월드 고정 패널, 데스크톱에서는 페이지 오버레이(<pre>)에 같은 텍스트를 그린다.
//
// VR 패널 배치: 진입 후 첫 유효 머리 포즈에서 머리 위치·수평 시선 기준 왼쪽 아래에 한 번 놓고 머리를 향하게 기울인다.
// 이후 방향은 고정한 채 위치만 머리 + 배치 오프셋으로 따라간다.

// 앵커 오프셋(m): 머리 기준 수평 전방·왼쪽·아래
const ANCHOR_OFFSET = { forward: 0.9, left: 0.8, down: 0.6 };

// VR 패널 크기(m)·텍스처 크기(px)·줄 높이(px)
const PANEL_W = 0.48;
const PANEL_H = 0.30;
const TEX_W = 768;
const TEX_H = 480;
const LINE_H = 22;
// 배치 전 앵커 위치 (바닥 아래)
const HIDDEN_ANCHOR_Y = -100;
// 배치를 시작하는 최소 머리 높이(m)
const MIN_HEAD_HEIGHT_M = 0.3;

/**
 * 캔버스에 라운드 사각형 경로를 그린다. 반환값 없음.
 */
function _roundRectPath(ctx, x, y, width, height, radius) {
    ctx.beginPath();
    ctx.moveTo(x + radius, y);
    ctx.arcTo(x + width, y, x + width, y + height, radius);
    ctx.arcTo(x + width, y + height, x, y + height, radius);
    ctx.arcTo(x, y + height, x, y, radius);
    ctx.arcTo(x, y, x + width, y, radius);
    ctx.closePath();
}

/**
 * 벡터를 소수 3자리 문자열 "x, y, z"로 반환한다.
 */
function _formatVector(vec) {
    return vec.map((value) => value.toFixed(3)).join(", ");
}

/**
 * 손가락 굽힘 배열을 0-9 숫자 문자열로 반환한다 (배열이 아니면 "-").
 */
function _formatFingers(fingers) {
    if (!Array.isArray(fingers)) return "-";
    return fingers.map((value) => Math.min(9, Math.round(value * 10))).join("");
}

export class Hud {
    /**
     * HUD를 초기화한다. scene: Babylon 씬, overlayEl: 데스크톱용 <pre> 요소.
     */
    constructor(scene, overlayEl) {
        this._scene = scene;
        this._overlayEl = overlayEl;
        this._anchor = null;
        this._panel = null;
        this._texture = null;
        this._xrCamera = null;
        this._placed = false;
        // 배치 시 저장한 머리 → 앵커 오프셋 (월드)
        this._anchorOffset = null;
        // 배치·추종 렌더 옵저버
        this._followObserver = null;
    }

    /**
     * VR 패널을 생성한다(앵커 노드 아래). 실제 월드 배치는 첫 유효 머리 포즈에서 수행. 반환값 없음.
     */
    attachToCamera(xrCamera) {
        this._xrCamera = xrCamera;
        this._placed = false;

        // 앵커 노드 (배치 전에는 바닥 아래)
        this._anchor = new BABYLON.TransformNode("hudAnchor", this._scene);
        this._anchor.position.set(0, HIDDEN_ANCHOR_Y, 0);

        // 텍스트 텍스처
        this._texture = new BABYLON.DynamicTexture("hudTex", { width: TEX_W, height: TEX_H }, this._scene, false);
        this._texture.hasAlpha = true;

        // 조명 무관 발광 텍스처 머티리얼
        const panelMaterial = new BABYLON.StandardMaterial("hudMat", this._scene);
        panelMaterial.emissiveTexture = this._texture;
        panelMaterial.opacityTexture = this._texture;
        panelMaterial.disableLighting = true;
        panelMaterial.backFaceCulling = false;

        // 패널 메시
        this._panel = BABYLON.MeshBuilder.CreatePlane("hudPanel", { width: PANEL_W, height: PANEL_H }, this._scene);
        this._panel.material = panelMaterial;
        this._panel.parent = this._anchor;
        this._panel.isPickable = false;

        // 매 프레임 배치(첫 유효 머리 포즈까지) 및 머리 위치 추종
        this._followObserver = this._scene.onBeforeRenderObservable.add(() => this._placeAnchor());
    }

    /**
     * 첫 유효 머리 포즈에서 앵커를 머리 왼쪽 아래에 놓고 머리를 향하게 기울인다. 반환값 없음.
     * 머리 높이가 바닥 근처(포즈 미수신)면 다음 호출로 미룬다.
     * 배치 후에는 매 프레임 저장한 오프셋으로 머리 위치를 따라간다 (방향 고정).
     */
    _placeAnchor() {
        if (!this._xrCamera || !this._anchor) return;
        const head = this._xrCamera.globalPosition;

        // 배치 후: 머리 위치 + 저장 오프셋으로 추종
        if (this._placed) {
            head.addToRef(this._anchorOffset, this._anchor.position);
            return;
        }
        if (head.y < MIN_HEAD_HEIGHT_M) return;

        // 수평 시선: 카메라 전방 벡터에서 상하 성분 제거
        const forward = this._xrCamera.getDirection(BABYLON.Axis.Z);
        forward.y = 0;
        if (forward.lengthSquared() < 1e-6) return;
        forward.normalize();

        // 왼쪽 = 전방 × 위쪽
        const left = BABYLON.Vector3.Cross(forward, BABYLON.Axis.Y).normalize();

        // 앵커 위치: 머리 + 전방·왼쪽·아래 오프셋
        const anchorPosition = head
            .add(forward.scale(ANCHOR_OFFSET.forward))
            .add(left.scale(ANCHOR_OFFSET.left))
            .add(new BABYLON.Vector3(0, -ANCHOR_OFFSET.down, 0));
        this._anchor.position.copyFrom(anchorPosition);

        // 패널의 읽히는 면(-z)이 머리를 향하도록 +z를 머리 반대편 점으로 향하게 한다
        const awayPoint = anchorPosition.scale(2).subtract(head);
        this._anchor.lookAt(awayPoint);
        this._anchorOffset = anchorPosition.subtract(head);
        this._placed = true;
        console.log(`[hud] 패널 배치: (${anchorPosition.x.toFixed(2)}, ${anchorPosition.y.toFixed(2)}, ${anchorPosition.z.toFixed(2)})`);
    }

    /**
     * VR 패널을 제거한다(세션 종료 시). 반환값 없음.
     */
    detach() {
        // 추종 옵저버 해제
        if (this._followObserver) {
            this._scene.onBeforeRenderObservable.remove(this._followObserver);
            this._followObserver = null;
        }

        // 패널·머티리얼·텍스처·앵커 폐기
        if (this._panel) {
            this._panel.material.dispose();
            this._panel.dispose();
            this._texture.dispose();
            this._panel = null;
            this._texture = null;
        }
        if (this._anchor) {
            this._anchor.dispose();
            this._anchor = null;
        }
        this._xrCamera = null;
        this._placed = false;
        this._anchorOffset = null;
    }

    /**
     * 텍스트 라인 배열을 VR 패널·데스크톱 오버레이에 반영한다. 반환값 없음.
     */
    update(lines) {
        // 데스크톱 오버레이
        this._overlayEl.textContent = lines.join("\n");
        if (!this._texture) return;

        // VR 패널: 라운드 반투명 배경
        const ctx = this._texture.getContext();
        ctx.clearRect(0, 0, TEX_W, TEX_H);
        _roundRectPath(ctx, 0, 0, TEX_W, TEX_H, 18);
        ctx.fillStyle = "rgba(12, 14, 20, 0.78)";
        ctx.fill();

        // VR 패널: 텍스트 줄
        ctx.font = "17px monospace";
        ctx.fillStyle = "#9fe8a0";
        ctx.textBaseline = "alphabetic";
        for (let lineIndex = 0; lineIndex < lines.length; lineIndex++) {
            ctx.fillText(lines[lineIndex], 14, LINE_H * (lineIndex + 1) + 6);
        }
        this._texture.update();
    }
}

/**
 * 공통 형식 샘플을 HUD 텍스트 라인 배열로 변환한다.
 * sample: 입력 모듈이 만든 공통 샘플 (null 허용, body는 null일 수 있음), extra: 상단 상태 문자열 배열.
 * showJointList: body 조인트별 위치 목록 표시 여부 (false면 조인트 수만 표시).
 */
export function formatSample(sample, extra = [], showJointList = true) {
    const lines = [...extra];
    if (!sample) {
        lines.push("(포즈 샘플 없음 - VR 세션 대기)");
        return lines;
    }

    // 손: 위치·추종 여부·손가락 굽힘(x10 표기: 새끼 약지 중지 검지 엄지굽힘 엄지회전)
    for (const handedness of ["left", "right"]) {
        const hand = sample.hands?.[handedness];
        if (!hand) {
            lines.push(`[${handedness}] (미검출)`);
            continue;
        }
        lines.push(`[${handedness}] p(${_formatVector(hand.pos)}) follow=${hand.follow ? "on" : "off"} fingers=${_formatFingers(hand.fingers)}`);
    }

    // body-tracking 조인트 (body가 null이면 없음 표시)
    if (sample.body) {
        lines.push(`[body] supported=${sample.body.supported} joints=${sample.body.jointCount}`);
        if (!showJointList) return lines;
        for (const [name, joint] of Object.entries(sample.body.joints ?? {})) {
            lines.push(`  ${name}: p(${_formatVector(joint.pos)})`);
        }
    } else {
        lines.push("[body] (없음)");
    }
    return lines;
}
