"""인식 서비스: D455 프레임마다 포즈를 붙여 TSDF 워커로 넘기고, 재구성 메시 블록(환경 층·고해상도 작업 영역 층)을
받아 mesh DataChannel로 방송한다. 장면은 전부 메시로 보이고, 로봇 자신은 관절 측정값으로 움직이는 로봇 모델이 보여 준다.

데이터 흐름 (PERCEPTION_RATE_HZ):
    camera.latest() → PosedFrameBuilder(촬영 시각 관절 보간 → 카메라 포즈)
        → (머리 속도 ≤ TSDF_MAX_HEAD_SPEED) 팔·손 메시 투영으로 로봇 몸 픽셀 제거 + 격자 역투영으로 작업 영역 픽셀 판정
        → TsdfWorker.submit (워커가 작업 영역 픽셀은 고해상도 층, 나머지는 환경 층에 블록을 만든다)
    TsdfWorker.poll() → 블록 저장소 갱신 → MSH1 → broadcast_mesh (드롭 없음)

메인 프로세스가 전체 블록 저장소를 들고 있어, 새로 접속한 클라이언트(mesh 채널 열림)에는 스냅샷을 1회 보낸다.
서비스가 시작될 때(모드 전환 포함)와 워커를 재시작할 때는 빈 스냅샷을 방송해 클라이언트 맵을 비운다.
"""

import asyncio
import contextlib
import logging
import time

import numpy as np

from teleop.camera.camera_interface import FrameSource
from teleop.perception.packets import pack_mesh_blocks
from teleop.perception.posed_frame import PosedFrame, PosedFrameBuilder
from teleop.perception.projection import DepthProjector, box_mask, upsample_mask
from teleop.perception.self_mask import SelfMask
from teleop.perception.tsdf_worker import TsdfWorker
from teleop.utils import config

logger = logging.getLogger("perception")


def _log_task_exception(task: asyncio.Task):
    """루프 태스크가 예외로 끝나면 로그에 남긴다. 반환값 없음."""
    if task.cancelled() or task.exception() is None:
        return
    logger.error(f"인식 루프 종료: {task.exception()!r}", exc_info=task.exception())


class PerceptionService:
    """처리 루프, 프레임 포즈·자기 몸 마스크·TSDF 워커, 메시 블록 저장소를 소유한다."""

    def __init__(self, camera: FrameSource, rtc_hub, robot_model, joint_source):
        """robot_model: 카메라 외부 파라미터·자기 몸 FK 제공자. joint_source: 관절·손가락 측정 공급자."""
        self._camera = camera
        self._hub = rtc_hub
        self._robot_model = robot_model
        self._joint_source = joint_source
        self._posed_builder = PosedFrameBuilder(robot_model, joint_source)
        self._mask_projector: DepthProjector | None = None
        self._self_mask: SelfMask | None = None
        self._worker: TsdfWorker | None = None
        self._task: asyncio.Task | None = None

        # 메시 블록 저장소 {(layer, bx, by, bz): (verts, colors, tris)}
        self._blocks: dict[tuple, tuple] = {}
        self._last_seq = -1

        # 통계 누적기·워커 통계·워커 재시작 시각
        self._stats = self._empty_stats()
        self._stats_start = 0.0
        self._worker_stats: dict | None = None
        self._worker_restart_time = 0.0

    @staticmethod
    def _empty_stats() -> dict:
        """통계 누적기를 새로 만들어 반환한다."""
        return {"frames": 0, "no_pose": 0, "proc_ms": 0.0, "tsdf_sent": 0, "tsdf_skipped_motion": 0, "mesh_bytes": 0}

    def start(self):
        """역투영기·워커를 준비하고 처리 루프를 시작한다 (카메라 start 이후 호출). 반환값 없음."""
        # 역투영기·자기 몸 마스크·TSDF 워커·관절 이력 준비
        intrinsics = self._camera.intrinsics
        self._mask_projector = DepthProjector(intrinsics, config.PIXEL_MASK_STRIDE)
        self._self_mask = SelfMask(self._robot_model, intrinsics)
        self._worker = TsdfWorker(intrinsics)
        self._worker.start()
        self._posed_builder.start()

        # 클라이언트 맵 초기화 + 신규 접속 스냅샷 연결
        self._reset_client_maps()
        self._hub.on_mesh_open = self._send_snapshot

        # 처리 루프 시작
        self._stats_start = time.monotonic()
        self._task = asyncio.create_task(self._loop())
        self._task.add_done_callback(_log_task_exception)
        logger.info(
            f"인식 서비스 시작: {config.PERCEPTION_RATE_HZ:.0f} Hz, 작업 영역 {config.WORKSPACE_BOX}, "
            f"메시 TSDF 환경 {config.TSDF_VOXEL * 1000:.0f} mm / 작업 영역 {config.TSDF_WS_VOXEL * 1000:.0f} mm"
        )

    async def stop(self):
        """루프·폴링·워커를 정지한다. 반환값 없음."""
        # 처리 루프 정지 (루프 예외는 done 콜백이 기록)
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self._task
            self._task = None

        # 관절 폴링 정지 + 스냅샷 콜백 해제
        await self._posed_builder.stop()
        if self._hub.on_mesh_open == self._send_snapshot:
            self._hub.on_mesh_open = None

        # 워커 정지 (executor에서 join)
        if self._worker is not None:
            await asyncio.get_running_loop().run_in_executor(None, self._worker.stop)
            self._worker = None
        logger.info("인식 서비스 정지")

    # ── 루프 ────────────────────────────────────────────────────────────────

    async def _loop(self):
        """고정 주기로 새 프레임 처리 + 워커 결과 방송을 수행한다 (루프 본체)."""
        event_loop = asyncio.get_running_loop()
        period = 1.0 / config.PERCEPTION_RATE_HZ
        while True:
            cycle_start = time.monotonic()

            # 새 프레임에 포즈를 붙여 처리
            frame = self._camera.latest()
            if frame is not None and frame.seq != self._last_seq:
                self._last_seq = frame.seq
                posed = self._posed_builder.build(frame)
                if posed is None:
                    self._stats["no_pose"] += 1
                else:
                    await self._process_frame(event_loop, posed)

            # 죽은 워커는 재시작 간격마다 다시 띄우고, 살아 있으면 결과를 방송
            if self._worker is not None and not self._worker.alive:
                await self._restart_worker_if_due(event_loop)
            self._broadcast_worker_results()
            self._log_stats()

            # 다음 주기까지 대기
            await asyncio.sleep(max(0.0, period - (time.monotonic() - cycle_start)))

    async def _process_frame(self, event_loop, posed: PosedFrame):
        """한 프레임: (머리가 느리면) 로봇 몸을 지운 depth와 작업 영역 픽셀을 TSDF 워커로 넘긴다. 반환값 없음."""
        process_start = time.perf_counter()

        # 머리 속도가 상한을 넘는 프레임은 TSDF 입력에서 제외
        if posed.head_speed > config.TSDF_MAX_HEAD_SPEED:
            self._stats["tsdf_skipped_motion"] += 1
        else:
            # 링크 FK는 루프 스레드에서, 투영·픽셀 판정은 executor에서 수행
            link_poses = self._self_mask.link_poses(posed.joints, self._joint_source.hand_fingers())
            depth_mm, workspace = await event_loop.run_in_executor(None, self._make_tsdf_input, posed, link_poses)

            # 카메라 → base 4x4 포즈를 붙여 워커로 전달
            cam_to_base = np.eye(4)
            cam_to_base[:3, :3] = posed.cam_rot
            cam_to_base[:3, 3] = posed.cam_pos
            if self._worker.submit(depth_mm, posed.frame.color, cam_to_base, workspace):
                self._stats["tsdf_sent"] += 1

        # 처리 통계 누적
        self._stats["frames"] += 1
        self._stats["proc_ms"] += (time.perf_counter() - process_start) * 1000

    def _make_tsdf_input(self, posed: PosedFrame, link_poses) -> tuple[np.ndarray, np.ndarray]:
        """(로봇 몸 픽셀을 지운 depth (H, W) uint16 mm, 작업 영역 픽셀 (H, W) bool)을 반환한다 (executor에서 실행).

        로봇 몸은 팔·손 메시 투영으로, 작업 영역은 격자 역투영 점의 상자 판정으로 정하고 원 해상도로 확장한다.
        """
        # 로봇 몸 픽셀·작업 영역 픽셀 판정
        depth = posed.frame.depth
        body = self._self_mask.pixel_mask(depth, posed.cam_pos, posed.cam_rot, link_poses)
        grid_points = self._mask_projector.project_grid(depth, posed.cam_pos, posed.cam_rot)
        workspace = upsample_mask(box_mask(grid_points, config.WORKSPACE_BOX), self._mask_projector.stride, depth.shape)

        # depth를 mm 정수로 바꾸고 로봇 몸 픽셀을 무효(0)로
        depth_mm = np.rint(depth * 1000.0).astype(np.uint16)
        depth_mm[body] = 0
        return depth_mm, workspace

    # ── 메시 블록 ───────────────────────────────────────────────────────

    def _broadcast_worker_results(self):
        """워커 결과로 블록 저장소를 갱신하고 바뀐 블록을 방송한다. 반환값 없음."""
        if self._worker is None or not self._worker.alive:
            return

        for result in self._worker.poll():
            # 바뀐 블록·삭제 블록을 저장소에 반영하고 전송 목록 작성
            blocks = []
            for key, verts, colors, tris in result["changed"]:
                self._blocks[key] = (verts, colors, tris)
                blocks.append((key, verts, colors, tris))
            for key in result["removed"]:
                self._blocks.pop(key, None)
                blocks.append((key, None, None, None))

            # MSH1 패킷으로 묶어 방송
            for payload in pack_mesh_blocks(blocks, snapshot=False, max_bytes=config.MESH_MAX_MESSAGE_BYTES):
                self._hub.broadcast_mesh(payload)
                self._stats["mesh_bytes"] += len(payload)
            self._worker_stats = result["stats"]

    def _send_snapshot(self, channel):
        """새로 열린 mesh 채널 하나에 전체 블록 스냅샷을 보낸다 (허브 콜백). 반환값 없음."""
        blocks = [(key, *data) for key, data in self._blocks.items()]
        payloads = pack_mesh_blocks(blocks, snapshot=True, max_bytes=config.MESH_MAX_MESSAGE_BYTES)

        # 스냅샷 패킷 송신 (실패 시 경고 로그)
        try:
            for payload in payloads:
                channel.send(payload)
        except Exception as error:
            logger.warning(f"메시 스냅샷 송신 실패: {type(error).__name__}: {error}")
            return
        logger.info(f"메시 스냅샷 전송: 블록 {len(blocks)}개, 패킷 {len(payloads)}개, "
                    f"{sum(len(payload) for payload in payloads) / 1e6:.2f} MB")

    def _reset_client_maps(self):
        """블록 저장소를 비우고 모든 클라이언트에 빈 스냅샷을 방송해 맵을 비운다. 반환값 없음."""
        self._blocks.clear()
        for payload in pack_mesh_blocks([], snapshot=True, max_bytes=config.MESH_MAX_MESSAGE_BYTES):
            self._hub.broadcast_mesh(payload)

    async def _restart_worker_if_due(self, event_loop):
        """죽은 TSDF 워커를 TSDF_WORKER_RESTART_S 간격으로 다시 띄우고 클라이언트 맵을 비운다. 반환값 없음."""
        now = time.monotonic()
        if now - self._worker_restart_time < config.TSDF_WORKER_RESTART_S:
            return
        self._worker_restart_time = now
        logger.error(f"TSDF 워커 프로세스 종료 감지 (종료 코드 {self._worker.exit_code}) — 워커 재시작, 메시 맵 초기화")

        # 죽은 워커 정리 후 새 워커 시작
        await event_loop.run_in_executor(None, self._worker.stop)
        self._worker = TsdfWorker(self._camera.intrinsics)
        self._worker.start()
        self._reset_client_maps()

    # ── 통계 ────────────────────────────────────────────────────────────────

    def _log_stats(self):
        """주기적으로 프레임 처리·메시 현황을 로그로 남긴다. 반환값 없음."""
        now = time.monotonic()
        if now - self._stats_start < config.CAMERA_STATS_INTERVAL_S:
            return

        # 프레임 처리·TSDF 입력·메시 송신 현황
        elapsed_s = now - self._stats_start
        stats = self._stats
        frame_count = max(stats["frames"], 1)
        dropped = self._worker.take_dropped() if self._worker else 0
        logger.info(
            f"프레임 {stats['frames'] / elapsed_s:.1f} Hz, 처리 {stats['proc_ms'] / frame_count:.1f} ms/프레임, "
            f"포즈 없음 {stats['no_pose']}회 | TSDF 입력 {stats['tsdf_sent'] / elapsed_s:.1f} Hz "
            f"(머리 이동 제외 {stats['tsdf_skipped_motion']}, 큐 드롭 {dropped}), "
            f"메시 송신 {stats['mesh_bytes'] / elapsed_s / 1e6:.2f} MB/s"
        )

        # 층별 TSDF 워커 현황
        if self._worker_stats is not None:
            for layer_name, layer_stats in self._worker_stats.items():
                logger.info(
                    f"메시 {layer_name}: 블록 {layer_stats['blocks']}개, 삼각형 {layer_stats['tris']}, "
                    f"최근 추출 변경 {layer_stats['changed']}·삭제 {layer_stats['removed']}, "
                    f"적분 {layer_stats['frames']}프레임·평균 {layer_stats['integrate_ms']:.1f} ms, "
                    f"추출 {layer_stats['extract_ms']:.0f} ms, 분할 {layer_stats['split_ms']:.0f} ms"
                )

        # 누적기 초기화
        self._stats = self._empty_stats()
        self._stats_start = now
