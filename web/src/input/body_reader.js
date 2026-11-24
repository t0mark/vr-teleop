// body-tracking 읽기: WebXR 프레임의 전신 조인트를 공통 샘플 body 항목으로 변환한다 (Quest 팔꿈치 유도용).
//
// 반환 형식: { supported, jointCount, joints: { 이름: { pos[3], quat[x,y,z,w] } } }
// 손가락 조인트(손마다 20여 개)만 제외하고 나머지 전신 조인트를 보낸다.
// 몸통·머리·다리는 IK에 쓰지 않지만 데스크톱 뷰의 사람 아바타 자세에 필요하고,
// 제외 후 20여 개 수준이라 대역폭도 문제되지 않는다.
// (이름은 Meta WebXR Body Tracking 규약, 실기기 검증에서 확인)
// 좌표계: WebXR reference space(오른손, y-up), 단위 m.

// 제외할 손가락 조인트 이름 패턴
const FINGER_JOINT_PATTERN = /-(thumb|index|middle|ring|little|pinky)-/;

export class BodyReader {
    /**
     * 읽기 상태를 초기화한다. 조인트 이름 목록은 최초 body 프레임에서 1회 채운다.
     */
    constructor() {
        // 전체 조인트 이름 목록: 실기기에서 실제 명칭을 확인하기 위한 1회성 덤프용
        this._jointNames = null;
    }

    /** 최초 body 프레임의 전체 조인트 이름 목록 (미수신이면 null). */
    get jointNames() {
        return this._jointNames;
    }

    /** body-tracking 상태 문자열 (HUD 표시용). */
    get status() {
        return `body-tracking: ${this._jointNames === null ? "대기/미지원" : "활성"}`;
    }

    /**
     * body-tracking 조인트를 읽는다. 미지원/미제공 프레임이면 supported=false.
     * frame: XRFrame, refSpace: XRReferenceSpace.
     */
    read(frame, refSpace) {
        const body = frame.body;
        if (!body) {
            return { supported: false, jointCount: 0, joints: {} };
        }

        // 최초 1회 전체 조인트 이름을 콘솔에 기록한다
        if (this._jointNames === null) {
            this._jointNames = [...body.keys()];
            console.log(`[body-tracking] 조인트 ${this._jointNames.length}개:`, this._jointNames);
        }

        // 손가락 조인트를 제외한 조인트 포즈 수집
        const joints = {};
        for (const [name, jointSpace] of body) {
            if (FINGER_JOINT_PATTERN.test(name)) continue;
            const pose = frame.getPose(jointSpace, refSpace);
            if (!pose) continue;
            const p = pose.transform.position;
            const q = pose.transform.orientation;
            joints[name] = { pos: [p.x, p.y, p.z], quat: [q.x, q.y, q.z, q.w] };
        }
        return { supported: true, jointCount: body.size ?? this._jointNames.length, joints };
    }
}
