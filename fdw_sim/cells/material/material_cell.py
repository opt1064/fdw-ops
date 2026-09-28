"""Material / Transport Cell.

소재 입고 → 보관(Smart Rack) → 피킹 → AMR 이송 → 공정 투입 → 버퍼 관리.

PoC-1에서는 다음을 단순화한다:
    - 실제 AMR 동역학 대신 "이동 시간" 모델 사용 (거리 / 속도)
    - Smart Rack은 dict 기반 재고 테이블
    - 이송 도중에는 부품이 'in_transit' 상태로 별도 관리
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import logging
import math

from fdw_sim.cells.base.cell_base import (
    CellConfig,
    DistributedIntelligenceCell,
)
from fdw_sim.messaging.bus import MessageBus, Topics
from fdw_sim.messaging.schemas import (
    CellState,
    CellType,
    DispatchCommand,
    MaterialTransferCommand,
)

logger = logging.getLogger(__name__)


@dataclass
class AMR:
    """단일 AMR 차량 모델 (저충실도)."""
    amr_id: str
    speed_mps: float = 1.0          # 평균 주행속도(m/s)
    position: tuple = (0.0, 0.0)
    busy: bool = False
    payload_part_id: Optional[str] = None
    from_cell: Optional[str] = None
    target_cell: Optional[str] = None
    remaining_distance_m: float = 0.0
    transfer_command_id: Optional[str] = None
    total_distance_m: float = 0.0
    # 배송 완료 시점의 스냅샷 — 시각화(WorkshopVisualizer)가 이 셀의 step()
    # 이후에 폴링하므로, payload_part_id/target_cell이 이미 None으로 지워진
    # 뒤에도 "방금 뭘 어디로 배송했는지" 알 수 있도록 남겨둔다.
    # transfer_command_id는 매 이송마다 고유하므로, 시각화 쪽에서 이 값이
    # 바뀌었는지만 보고 "새로 배송된 건인지" edge-detect 한다.
    last_delivered_part_id: Optional[str] = None
    last_delivered_to: Optional[str] = None
    last_delivered_command_id: Optional[str] = None
    # require_pickup_confirmation=True일 때, 도착은 했지만 로봇의
    # pick-and-place가 끝날 때까지(WorkshopVisualizer.confirm_pickup 호출
    # 전까지) 재배차되지 않고 도크에서 대기 중임을 표시. busy는 True로
    # 유지되어 _dispatch_amrs()가 이 AMR을 건드리지 않는다.
    awaiting_pickup: bool = False
    pickup_wait_elapsed: float = 0.0


@dataclass
class CellLocation:
    """공정 셀의 좌표 (이송 시간 계산용)."""
    cell_id: str
    position: tuple   # (x, y) in meters


class MaterialCell(DistributedIntelligenceCell):
    """소재·이송 분산지능 셀.

    이 셀은 다른 일반 공정 셀과 달리 항상 PROCESSING 상태로 유지되는 게 아니라,
    이송 명령(MaterialTransferCommand)을 큐에 받아 비동기적으로 수행한다.
    """

    def __init__(self, config: CellConfig, bus: MessageBus,
                 cell_locations: Optional[Dict[str, CellLocation]] = None,
                 num_amrs: int = 2) -> None:
        super().__init__(config, bus)
        self.smart_rack: Dict[str, dict] = {}            # part_id -> {"part_type": ..., ...}
        self.cell_locations: Dict[str, CellLocation] = cell_locations or {}
        self.amrs: List[AMR] = [
            AMR(amr_id=f"AMR_{i+1:02d}", speed_mps=1.0,
                position=self.cell_locations.get(self.cell_id, CellLocation(self.cell_id, (0.0, 0.0))).position)
            for i in range(num_amrs)
        ]
        self.transfer_queue: List[MaterialTransferCommand] = []

        # WorkshopVisualizer.attach_material_cell()이 로봇이 있는 시각화
        # 환경에서만 True로 켠다 — 켜지면 도착한 AMR이 confirm_pickup()
        # 호출(또는 max_pickup_wait_sec 타임아웃) 전까지 재배차되지 않는다.
        # 기본 False: discrete 모드/테스트/시각화 없는 headless 실행은
        # 예전처럼 도착 즉시 AMR이 자유로워진다.
        self.require_pickup_confirmation: bool = False
        self.max_pickup_wait_sec: float = 15.0

        # 이송 중인 부품: part_id -> 현재 보유 AMR
        self.in_transit: Dict[str, AMR] = {}

        # 다른 셀의 입출력 버퍼 핸들 (시뮬레이션 매니저가 주입)
        self.cell_registry: Dict[str, DistributedIntelligenceCell] = {}

        # 이송 명령 토픽 구독
        self.bus.subscribe(Topics.MATERIAL_TRANSFER, self._on_transfer_command)

    # ------------------------------------------------------------------ setup
    def configure(self) -> None:
        # Isaac Sim 자산 로드는 SimulationManager에서 별도로 진행.
        # 여기서는 논리적 초기화만 수행.
        logger.info("[%s] MaterialCell configured with %d AMRs", self.cell_id, len(self.amrs))

    def register_cell(self, cell: DistributedIntelligenceCell, location: CellLocation) -> None:
        """다른 공정 셀을 이송 대상으로 등록."""
        self.cell_registry[cell.cell_id] = cell
        self.cell_locations[cell.cell_id] = location
        logger.debug("[%s] registered target cell %s @ %s", self.cell_id, cell.cell_id, location.position)

    def stock_part(self, part_id: str, part_type: str) -> None:
        """입고: Smart Rack에 부품 등록."""
        self.smart_rack[part_id] = {"part_type": part_type}
        self.log_kpi("rack_stock_count", float(len(self.smart_rack)), unit="parts")
        logger.info("[%s] stocked %s (type=%s, total=%d)",
                    self.cell_id, part_id, part_type, len(self.smart_rack))

    # =========================================================================
    # Command 처리
    # =========================================================================
    def on_command(self, command: DispatchCommand) -> bool:
        """MaterialCell은 일반 DispatchCommand로는 거의 동작하지 않는다.
        대신 _on_transfer_command 콜백으로 이송 명령을 받는다."""
        # 호환성을 위해 일부 명령(예: PURGE, RESET) 처리만 유지
        op = command.operation
        if op == "RESET":
            self.transfer_queue.clear()
            for amr in self.amrs:
                amr.busy = False
                amr.payload_part_id = None
                amr.from_cell = None
                amr.target_cell = None
                amr.awaiting_pickup = False
                amr.pickup_wait_elapsed = 0.0
            return True
        return False

    def _on_transfer_command(self, cmd: MaterialTransferCommand) -> None:
        if cmd.from_cell != self.cell_id and cmd.from_cell not in self.cell_registry:
            return  # 우리 책임 아님
        self.transfer_queue.append(cmd)
        logger.info("[%s] queued transfer %s: %s -> %s (part=%s)",
                    self.cell_id, cmd.command_id, cmd.from_cell, cmd.to_cell, cmd.part_id)

    # =========================================================================
    # 메인 step 로직 (PROCESSING 상태가 아니어도 항상 실행)
    # =========================================================================
    def step(self, dt: float, sim_time: float) -> None:
        # 큐에서 idle AMR에 작업 할당
        self._dispatch_amrs()

        # 각 AMR 진행
        for amr in self.amrs:
            if amr.awaiting_pickup:
                self._step_awaiting_pickup(amr, dt)
            elif amr.busy:
                self._step_amr(amr, dt)

        # 부모 step (상태 발행 등)
        super().step(dt, sim_time)

    def _dispatch_amrs(self) -> None:
        if not self.transfer_queue:
            return
        for amr in self.amrs:
            if amr.busy:
                continue
            if not self.transfer_queue:
                break
            cmd = self.transfer_queue.pop(0)
            self._assign_amr(amr, cmd)

    def _assign_amr(self, amr: AMR, cmd: MaterialTransferCommand) -> None:
        # 1) From cell에서 부품 takeout 시도
        from_cell = self.cell_registry.get(cmd.from_cell)
        if from_cell is None and cmd.from_cell == self.cell_id:
            # MaterialCell 자체에서 출고 (스마트랙)
            if cmd.part_id not in self.smart_rack:
                logger.warning("[%s] cannot takeout %s from rack (not stocked)",
                               self.cell_id, cmd.part_id)
                return
            del self.smart_rack[cmd.part_id]
            taken = cmd.part_id
        elif from_cell is not None:
            taken = from_cell.takeout_part()
            if taken is None or taken != cmd.part_id:
                # 아직 출력버퍼에 없음 → 큐 뒤로 보냄
                self.transfer_queue.append(cmd)
                return
        else:
            logger.warning("[%s] unknown source cell %s", self.cell_id, cmd.from_cell)
            return

        amr.busy = True
        amr.payload_part_id = taken
        amr.from_cell = cmd.from_cell
        amr.target_cell = cmd.to_cell
        amr.transfer_command_id = cmd.command_id

        # 거리 계산
        from_loc = self.cell_locations.get(cmd.from_cell)
        to_loc = self.cell_locations.get(cmd.to_cell)
        if from_loc and to_loc:
            amr.remaining_distance_m = math.dist(from_loc.position, to_loc.position)
            amr.position = from_loc.position
        else:
            amr.remaining_distance_m = 5.0  # 기본 5m
        amr.total_distance_m = amr.remaining_distance_m

        logger.info("[%s] %s assigned: %s (%.1fm)",
                    self.cell_id, amr.amr_id, cmd.part_id, amr.remaining_distance_m)

    def _step_amr(self, amr: AMR, dt: float) -> None:
        amr.remaining_distance_m -= amr.speed_mps * dt
        if amr.remaining_distance_m > 0:
            return

        # 도착: 타겟 셀에 부품 전달
        target = self.cell_registry.get(amr.target_cell)
        delivered_to = amr.target_cell
        delivered_part = amr.payload_part_id

        if target is not None:
            ok = target.receive_part(amr.payload_part_id)
            if not ok:
                # 타겟 입력버퍼가 차있음 → 잠시 대기 후 재시도
                amr.remaining_distance_m = 0.5
                return
        # else: 타겟이 외부 시스템(예: 출하)인 경우 그냥 소멸

        self.log_kpi("amr_delivery_count", 1.0, tags={"amr_id": amr.amr_id})
        amr.last_delivered_part_id = delivered_part
        amr.last_delivered_to = delivered_to
        amr.last_delivered_command_id = amr.transfer_command_id
        amr.payload_part_id = None
        amr.from_cell = None
        amr.target_cell = None
        amr.transfer_command_id = None
        if self.require_pickup_confirmation:
            # busy=True 유지 — WorkshopVisualizer가 pick-and-place를 끝내고
            # confirm_pickup()을 호출할 때까지(또는 타임아웃까지) 도크에서
            # 대기, _dispatch_amrs()의 재배차 대상에서 제외된다.
            amr.awaiting_pickup = True
            amr.pickup_wait_elapsed = 0.0
        else:
            amr.busy = False
        logger.info("[%s] %s delivered %s to %s",
                    self.cell_id, amr.amr_id, delivered_part, delivered_to)

    def _step_awaiting_pickup(self, amr: AMR, dt: float) -> None:
        """도착 후 로봇의 pick-and-place 완료(confirm_pickup) 대기.

        WorkshopVisualizer가 pick-and-place를 시작조차 못 했거나(로봇 없음/
        busy 경합) 알림을 깜빡 누락하는 경우에도 AMR이 영구히 묶이지 않도록
        max_pickup_wait_sec 타임아웃 시 강제로 풀어준다.
        """
        amr.pickup_wait_elapsed += dt
        if amr.pickup_wait_elapsed >= self.max_pickup_wait_sec:
            logger.warning(
                "[%s] %s pickup confirmation timeout (%.1fs) — force-releasing "
                "(WorkshopVisualizer가 confirm_pickup()을 호출 안 했을 가능성)",
                self.cell_id, amr.amr_id, amr.pickup_wait_elapsed)
            amr.awaiting_pickup = False
            amr.busy = False
            amr.pickup_wait_elapsed = 0.0

    def confirm_pickup(self, amr_id: str) -> None:
        """WorkshopVisualizer가 pick-and-place(또는 그 폴백)를 끝낸 뒤 호출 —
        도크에서 대기 중이던 AMR을 재배차 가능 상태로 풀어준다."""
        for amr in self.amrs:
            if amr.amr_id == amr_id and amr.awaiting_pickup:
                amr.awaiting_pickup = False
                amr.busy = False
                amr.pickup_wait_elapsed = 0.0
                logger.info("[%s] %s released (pickup confirmed)",
                            self.cell_id, amr.amr_id)
                return

    # MaterialCell은 자체 PROCESSING 작업이 없음
    def step_processing(self, dt: float) -> None:  # pragma: no cover
        pass
