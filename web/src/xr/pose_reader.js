// WebXR 프레임에서 포즈·입력을 수집한다: body-tracking 조인트(어깨·팔꿈치 등),
// 컨트롤러 손목(grip) 포즈, 트리거·그립 입력.
// 좌표계: WebXR reference space(오른손, y-up), 단위 m. 쿼터니언은 (x, y, z, w).

// 손가락 조인트(손마다 20여 개)만 제외하고 나머지 전신 조인트를 보낸다.
// 몸통·머리·다리는 IK에 쓰지 않지만 데스크톱 뷰의 사람 아바타 자세에 필요하고,
// 제외 후 20여 개 수준이라 대역폭도 문제되지 않는다.
// (이름은 Meta WebXR Body Tracking 규약, 실기기 검증에서 확인)
const FINGER_JOINT_PATTERN = /-(thumb|index|middle|ring|little|pinky)-/;

export class PoseReader {
    /**
     * 수집 상태를 초기화한다. bodyJointNames는 최초 body 프레임에서 1회 채운다.
     */
    constructor() {
        // 전체 조인트 이름 목록: 실기기에서 실제 명칭을 확인하기 위한 1회성 덤프용
        this.bodyJointNames = null;
        this.lastSample = null;
    }

    /**
     * XRFrame에서 샘플 1개를 수집해 반환한다 (수집 불가 항목은 null 필드).
     * frame: XRFrame, refSpace: XRReferenceSpace, session: XRSession.
     * 반환: { t, body: {지원 여부, 조인트 수, 상체 조인트 포즈}, hands: {left/right 컨트롤러} }
     */
    read(frame, refSpace, session) {
        const sample = {
            t: performance.now(),
            body: this._readBody(frame, refSpace),
            hands: this._readControllers(frame, refSpace, session),
        };
        this.lastSample = sample;
        return sample;
    }

    /**
     * body-tracking 조인트를 읽는다. 미지원/미제공 프레임이면 supported=false.
     * 반환: { supported, jointCount, joints: {이름: {pos, quat}} } (상체 필터 적용)
     */
    _readBody(frame, refSpace) {
        const body = frame.body;
        if (!body) {
            return { supported: false, jointCount: 0, joints: {} };
        }

        // 최초 1회 전체 조인트 이름을 기록해 콘솔로 확인 가능하게 한다 (명칭 규약 검증 목적)
        if (this.bodyJointNames === null) {
            this.bodyJointNames = [...body.keys()];
            console.log(`[body-tracking] 조인트 ${this.bodyJointNames.length}개:`, this.bodyJointNames);
        }

        const joints = {};
        for (const [name, jointSpace] of body) {
            // 손가락 마디는 버린다 (사용처가 없고 조인트 수가 몇 배로 늘어난다)
            if (FINGER_JOINT_PATTERN.test(name)) continue;
            const pose = frame.getPose(jointSpace, refSpace);
            if (!pose) continue;
            const p = pose.transform.position;
            const q = pose.transform.orientation;
            joints[name] = { pos: [p.x, p.y, p.z], quat: [q.x, q.y, q.z, q.w] };
        }
        return { supported: true, jointCount: body.size ?? this.bodyJointNames.length, joints };
    }

    /**
     * 컨트롤러 손목 포즈(gripSpace)와 버튼·축 입력을 읽는다.
     * 반환: { left, right } — 각 항목 { pos, quat, trigger, squeeze, buttons } 또는 null.
     * 버튼 인덱스는 xr-standard 게임패드 매핑(0=trigger, 1=squeeze).
     */
    _readControllers(frame, refSpace, session) {
        const hands = { left: null, right: null };
        for (const source of session.inputSources) {
            if (!source.gripSpace || !hands.hasOwnProperty(source.handedness)) continue;
            const pose = frame.getPose(source.gripSpace, refSpace);
            if (!pose) continue;

            const p = pose.transform.position;
            const q = pose.transform.orientation;
            const gp = source.gamepad;
            hands[source.handedness] = {
                pos: [p.x, p.y, p.z],
                quat: [q.x, q.y, q.z, q.w],
                trigger: gp ? gp.buttons[0]?.value ?? 0 : 0,
                squeeze: gp ? gp.buttons[1]?.value ?? 0 : 0,
                buttons: gp ? gp.buttons.map((b) => (b.pressed ? 1 : 0)) : [],
            };
        }
        return hands;
    }
}
