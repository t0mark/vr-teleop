// 포즈·입력 상태 표시 HUD: VR 안에서는 사용자 왼쪽 아래 구석에 월드 고정된 작은 패널,
// 데스크톱에서는 페이지 오버레이(<pre>)에 같은 텍스트를 그린다.
// 디자인은 외부 에셋 없이 캔버스 2D 드로잉 (오프라인 직결 환경 유지).
//
// 배치 방식: 카메라에 붙이면 고개를 어디로 돌려도 화면의 같은 자리에 따라오므로 "구석에 두고
// 필요할 때 고개를 돌려 본다"가 불가능하다. 대신 VR 진입 시 머리 위치·수평 시선 기준으로
// 왼쪽 아래에 한 번 배치하고(anchor), 패널이 머리를 향하도록 기울인다.
// 이후 방향은 고정한 채 위치만 머리 + 배치 오프셋으로 따라간다 (로봇 머리 시점 이동 대응).
// 정면(로봇)을 볼 때는 시야 밖이고, 왼쪽 아래로 고개를 돌리면 보인다.
// VR 종료는 Quest 시스템 메뉴로 한다 (컨트롤러가 손등에 부착되어 레이 버튼을 쓸 수 없다).

// 앵커 오프셋(m): 머리 기준 수평 전방·왼쪽·아래 (약 42° 좌, 34° 하 — 중심 시야 밖)
const ANCHOR_OFFSET = { forward: 0.9, left: 0.8, down: 0.6 };

// VR 패널: 0.48m x 0.30m (텍스처 768x480px), 앵커 원점에 배치
const PANEL_W = 0.48;
const PANEL_H = 0.30;
const TEX_W = 768;
const TEX_H = 480;
const LINE_H = 22;

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
            this._anchor.position.copyFrom(head.add(this._anchorOffset));
            return;
        }
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
        this._anchorOffset = pos.subtract(head);
        this._placed = true;
        console.log(`[hud] 패널 배치: (${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, ${pos.z.toFixed(2)})`);
    }

    /**
     * VR 패널을 제거한다(세션 종료 시). 반환값 없음.
     */
    detach() {
        if (this._followObserver) {
            this._scene.onBeforeRenderObservable.remove(this._followObserver);
            this._followObserver = null;
        }
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
        this._overlayEl.textContent = lines.join("\n");

        if (this._texture) {
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
 * 공통 형식 샘플을 HUD 텍스트 라인 배열로 변환한다.
 * sample: 입력 모듈이 만든 공통 샘플 (null 허용, body는 null일 수 있음), extra: 상단 상태 문자열 배열.
 */
export function formatSample(sample, extra = []) {
    const lines = [...extra];
    if (!sample) {
        lines.push("(포즈 샘플 없음 - VR 세션 대기)");
        return lines;
    }

    const fmtVec = (v) => v.map((x) => x.toFixed(3)).join(", ");

    // 손: 위치·추종 여부·손가락 굽힘(0=펼침, 1=굽힘, 1자리 x10 표기: 새끼 약지 중지 검지 엄지굽힘 엄지회전)
    for (const hand of ["left", "right"]) {
        const c = sample.hands?.[hand];
        if (!c) {
            lines.push(`[${hand}] (미검출)`);
            continue;
        }
        const fingers = Array.isArray(c.fingers)
            ? c.fingers.map((x) => Math.min(9, Math.round(x * 10))).join("")
            : "-";
        lines.push(`[${hand}] p(${fmtVec(c.pos)}) follow=${c.follow ? "on" : "off"} fingers=${fingers}`);
    }

    // body-tracking 조인트 (미지원 기기는 body가 null)
    if (sample.body) {
        lines.push(`[body] supported=${sample.body.supported} joints=${sample.body.jointCount}`);
        for (const [name, j] of Object.entries(sample.body.joints ?? {})) {
            lines.push(`  ${name}: p(${fmtVec(j.pos)})`);
        }
    } else {
        lines.push("[body] (없음)");
    }
    return lines;
}
