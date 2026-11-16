// 포즈·입력 상태 표시 HUD: VR 안에서는 사용자 왼쪽 아래 구석에 월드 고정된 작은 패널 + EXIT 버튼,
// 데스크톱에서는 페이지 오버레이(<pre>)에 같은 텍스트를 그린다.
// 디자인은 외부 에셋 없이 캔버스 2D 드로잉 (오프라인 직결 환경 유지).
//
// 배치 방식: 카메라에 붙이면 고개를 어디로 돌려도 화면의 같은 자리에 따라오므로 "구석에 두고
// 필요할 때 고개를 돌려 본다"가 불가능하다. 대신 VR 진입 시 머리 위치·수평 시선 기준으로
// 왼쪽 아래 월드 좌표에 한 번 고정하고(anchor), 패널이 머리를 향하도록 기울인다.
// 정면(로봇)을 볼 때는 시야 밖이고, 왼쪽 아래로 고개를 돌리면 보인다.

// 앵커 오프셋(m): 머리 기준 수평 전방·왼쪽·아래 (약 42° 좌, 34° 하 — 중심 시야 밖)
const ANCHOR_OFFSET = { forward: 0.9, left: 0.8, down: 0.6 };

// VR 패널: 0.48m x 0.30m (텍스처 768x480px), 앵커 원점에 배치
const PANEL_W = 0.48;
const PANEL_H = 0.30;
const TEX_W = 768;
const TEX_H = 480;
const LINE_H = 22;

// EXIT 버튼: 패널 왼쪽 아래 정렬, 0.18m x 0.07m (텍스처 384x150px), 앵커 기준 오프셋
const BTN_W = 0.18;
const BTN_H = 0.07;
const BTN_LOCAL = { x: -0.15, y: -0.23 };

/**
 * 캔버스에 라운드 사각형 경로를 그린다 (roundRect 미지원 브라우저 대비 수동 구현).
 */
function _roundRectPath(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
}

/**
 * EXIT 버튼 텍스처를 그린다. hover: 호버 하이라이트 여부.
 */
function _drawExitButton(ctx, w, h, hover) {
    ctx.clearRect(0, 0, w, h);

    // 라운드 필 배경 + 붉은 테두리 (호버 시 채도·밝기 상승)
    _roundRectPath(ctx, 4, 4, w - 8, h - 8, (h - 8) / 2);
    ctx.fillStyle = hover ? "rgba(80, 28, 32, 0.95)" : "rgba(28, 30, 38, 0.88)";
    ctx.fill();
    ctx.lineWidth = hover ? 5 : 3;
    ctx.strokeStyle = hover ? "#ff6b6b" : "#e05555";
    ctx.stroke();

    // 로그아웃 아이콘: 문틀([ 모양) + 밖으로 나가는 화살표
    const cy = h / 2;
    const ix = 62;
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 7;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.beginPath();
    ctx.moveTo(ix + 22, cy - 32);
    ctx.lineTo(ix, cy - 32);
    ctx.lineTo(ix, cy + 32);
    ctx.lineTo(ix + 22, cy + 32);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(ix + 14, cy);
    ctx.lineTo(ix + 62, cy);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(ix + 46, cy - 14);
    ctx.lineTo(ix + 62, cy);
    ctx.lineTo(ix + 46, cy + 14);
    ctx.stroke();

    // 라벨
    ctx.font = "600 44px system-ui, sans-serif";
    ctx.fillStyle = "#ffffff";
    ctx.textBaseline = "middle";
    ctx.fillText("EXIT", ix + 92, cy + 2);
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
        this._exitBtn = null;
        this._exitTex = null;
        this._xrCamera = null;
        this._placed = false;
    }

    /**
     * VR 패널·EXIT 버튼을 생성한다(앵커 노드 아래). 실제 월드 배치는 첫 유효 머리 포즈에서 수행.
     * onExit: EXIT 버튼 클릭 시 호출할 콜백 (세션 종료 처리). 반환값 없음.
     */
    attachToCamera(xrCamera, onExit) {
        this._xrCamera = xrCamera;
        this._placed = false;
        this._anchor = new BABYLON.TransformNode("hudAnchor", this._scene);
        // 배치 전에는 보이지 않게 멀리 둔다 (첫 프레임 원점 번쩍임 방지)
        this._anchor.position.set(0, -100, 0);

        this._texture = new BABYLON.DynamicTexture("hudTex", { width: TEX_W, height: TEX_H }, this._scene, false);
        this._texture.hasAlpha = true;

        const mat = new BABYLON.StandardMaterial("hudMat", this._scene);
        // 조명 무관하게 항상 읽히도록 발광 텍스처로만 표시
        mat.emissiveTexture = this._texture;
        mat.opacityTexture = this._texture;
        mat.disableLighting = true;
        mat.backFaceCulling = false;

        this._panel = BABYLON.MeshBuilder.CreatePlane("hudPanel", { width: PANEL_W, height: PANEL_H }, this._scene);
        this._panel.material = mat;
        this._panel.parent = this._anchor;
        this._panel.isPickable = false;

        this._createExitButton(onExit);
    }

    /**
     * EXIT 버튼(라운드 필 + 로그아웃 아이콘, 호버 하이라이트)을 앵커 아래에 생성한다. 반환값 없음.
     */
    _createExitButton(onExit) {
        this._exitTex = new BABYLON.DynamicTexture("exitTex", { width: 384, height: 150 }, this._scene, false);
        this._exitTex.hasAlpha = true;
        _drawExitButton(this._exitTex.getContext(), 384, 150, false);
        this._exitTex.update();

        const mat = new BABYLON.StandardMaterial("exitMat", this._scene);
        mat.emissiveTexture = this._exitTex;
        mat.opacityTexture = this._exitTex;
        mat.disableLighting = true;
        mat.backFaceCulling = false;

        this._exitBtn = BABYLON.MeshBuilder.CreatePlane("exitBtn", { width: BTN_W, height: BTN_H }, this._scene);
        this._exitBtn.material = mat;
        this._exitBtn.parent = this._anchor;
        this._exitBtn.position = new BABYLON.Vector3(BTN_LOCAL.x, BTN_LOCAL.y, 0);
        this._exitBtn.isPickable = true;

        // 컨트롤러 레이 픽: 클릭 = 종료, 호버 = 하이라이트 다시 그리기
        const redraw = (hover) => {
            _drawExitButton(this._exitTex.getContext(), 384, 150, hover);
            this._exitTex.update();
        };
        this._exitBtn.actionManager = new BABYLON.ActionManager(this._scene);
        this._exitBtn.actionManager.registerAction(
            new BABYLON.ExecuteCodeAction(BABYLON.ActionManager.OnPickTrigger, () => onExit()),
        );
        this._exitBtn.actionManager.registerAction(
            new BABYLON.ExecuteCodeAction(BABYLON.ActionManager.OnPointerOverTrigger, () => redraw(true)),
        );
        this._exitBtn.actionManager.registerAction(
            new BABYLON.ExecuteCodeAction(BABYLON.ActionManager.OnPointerOutTrigger, () => redraw(false)),
        );
    }

    /**
     * 첫 유효 머리 포즈에서 앵커를 왼쪽 아래 월드 좌표에 고정하고 머리를 향하게 기울인다. 반환값 없음.
     * 머리 높이가 바닥 근처(포즈 미수신)면 다음 호출로 미룬다.
     */
    _placeAnchor() {
        if (this._placed || !this._xrCamera) return;
        const head = this._xrCamera.globalPosition;
        if (head.y < 0.3) return;

        // 수평 시선: 카메라 전방 벡터에서 상하 성분을 제거 (고개를 숙인 채 진입해도 배치 일정)
        const fwd = this._xrCamera.getDirection(BABYLON.Axis.Z);
        fwd.y = 0;
        if (fwd.lengthSquared() < 1e-6) return;
        fwd.normalize();
        // 왼쪽 = 전방 × 위쪽 (검산: fwd=(0,0,1), up=(0,1,0) → (-1,0,0) = -x = 왼쪽)
        const left = BABYLON.Vector3.Cross(fwd, BABYLON.Axis.Y).normalize();

        const pos = head
            .add(fwd.scale(ANCHOR_OFFSET.forward))
            .add(left.scale(ANCHOR_OFFSET.left))
            .add(new BABYLON.Vector3(0, -ANCHOR_OFFSET.down, 0));
        this._anchor.position.copyFrom(pos);

        // 패널의 읽히는 면(-z)이 머리를 향하도록: +z를 머리 반대편 점으로 향하게 한다
        const away = pos.scale(2).subtract(head);
        this._anchor.lookAt(away);
        this._placed = true;
    }

    /**
     * VR 패널·EXIT 버튼을 제거한다(세션 종료 시). 반환값 없음.
     */
    detach() {
        if (this._panel) {
            this._panel.material.dispose();
            this._panel.dispose();
            this._texture.dispose();
            this._panel = null;
            this._texture = null;
        }
        if (this._exitBtn) {
            this._exitBtn.material.dispose();
            this._exitBtn.dispose();
            this._exitTex.dispose();
            this._exitBtn = null;
            this._exitTex = null;
        }
        if (this._anchor) {
            this._anchor.dispose();
            this._anchor = null;
        }
        this._xrCamera = null;
        this._placed = false;
    }

    /**
     * 텍스트 라인 배열을 VR 패널·데스크톱 오버레이에 반영한다. 반환값 없음.
     */
    update(lines) {
        this._overlayEl.textContent = lines.join("\n");

        if (this._texture) {
            this._placeAnchor();
            const ctx = this._texture.getContext();
            ctx.clearRect(0, 0, TEX_W, TEX_H);
            // 라운드 반투명 배경으로 가독성 확보 (시야를 과하게 가리지 않게)
            _roundRectPath(ctx, 0, 0, TEX_W, TEX_H, 18);
            ctx.fillStyle = "rgba(12, 14, 20, 0.78)";
            ctx.fill();
            ctx.font = "17px monospace";
            ctx.fillStyle = "#9fe8a0";
            ctx.textBaseline = "alphabetic";
            lines.forEach((line, i) => {
                ctx.fillText(line, 14, LINE_H * (i + 1) + 6);
            });
            this._texture.update();
        }
    }
}

/**
 * 포즈 샘플을 HUD 텍스트 라인 배열로 변환한다.
 * sample: PoseReader.read() 반환값 (null 허용), extra: 상단 상태 문자열 배열.
 */
export function formatSample(sample, extra = []) {
    const lines = [...extra];
    if (!sample) {
        lines.push("(포즈 샘플 없음 - VR 세션 대기)");
        return lines;
    }

    const fmtVec = (v) => v.map((x) => x.toFixed(3)).join(", ");

    lines.push(`[body] supported=${sample.body.supported} joints=${sample.body.jointCount}`);
    for (const [name, j] of Object.entries(sample.body.joints)) {
        lines.push(`  ${name}: p(${fmtVec(j.pos)})`);
    }

    for (const hand of ["left", "right"]) {
        const c = sample.hands[hand];
        if (!c) {
            lines.push(`[${hand}] (미검출)`);
            continue;
        }
        lines.push(
            `[${hand}] p(${fmtVec(c.pos)}) trig=${c.trigger.toFixed(2)} ` +
            `sqz=${c.squeeze.toFixed(2)} stick(${fmtVec(c.thumbstick)})`,
        );
    }
    return lines;
}
