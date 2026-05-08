"""WorkshopVisualizer — 추상 셀들의 상태를 USD 스테이지에 매핑.

PoC-1의 셀(MaterialCell/WeldingCell/InspectionCell)에서 발생하는
이벤트(부품 입고/이송/가공/완료)를 시각적으로 반영한다.

설계 원칙:
- 시각화는 시뮬레이션 로직과 완전히 분리됨 (옵저버 패턴)
- mode="discrete"에서는 절대 인스턴스화되지 않음
- 부품 이동은 lerp 보간으로 부드럽게 표현
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import logging

from fdw_sim.cells.base.cell_base import DistributedIntelligenceCell
from fdw_sim.cells.material.material_cell import MaterialCell
from fdw_sim.messaging.bus import MessageBus, Topics
from fdw_sim.visualization.scene_builder import SceneBuilder, SceneConfig

logger = logging.getLogger(__name__)


# 셀 종류별 기본 색상
CELL_COLORS = {
    "material":   (0.50, 0.55, 0.65),
    "welding":    (0.75, 0.40, 0.35),
    "inspection": (0.40, 0.70, 0.50),
    "forming":    (0.65, 0.55, 0.30),
    "default":    (0.55, 0.55, 0.60),
}

# 부품 타입별 색상
PART_COLORS = {
    "tubular_frame_A": (0.20, 0.60, 1.00),   # 파랑
    "tubular_frame_B": (1.00, 0.60, 0.20),   # 주황
    "default":         (0.50, 0.50, 0.50),
}


@dataclass
class PartTween:
    """부품 위치 보간 상태."""
    part_id: str
    start: Tuple[float, float, float]
    end: Tuple[float, float, float]
    duration: float
    elapsed: float = 0.0

    def progress(self) -> float:
        if self.duration <= 0:
            return 1.0
        return min(1.0, self.elapsed / self.duration)

    def current(self) -> Tuple[float, float, float]:
        t = self.progress()
        # smoothstep
        s = t * t * (3.0 - 2.0 * t)
        return (
            self.start[0] + (self.end[0] - self.start[0]) * s,
            self.start[1] + (self.end[1] - self.start[1]) * s,
            self.start[2] + (self.end[2] - self.start[2]) * s,
        )


@dataclass
class WorkshopVizConfig:
    """워크숍 시각화 옵션."""
    cell_size: Tuple[float, float, float] = (1.8, 1.8, 0.8)
    part_height_above_bench: float = 0.2     # 부품이 작업대 위에 떠있는 높이
    transfer_duration_sec: float = 2.0       # 셀 간 이동 애니메이션 길이
    amr_speed_mps: float = 1.0               # AMR 이동 속도
    show_smart_rack: bool = True
    show_robot_arm: bool = True
    show_camera: bool = True


class WorkshopVisualizer:
    """추상 셀을 USD 스테이지에 매핑하고 step마다 부품 위치를 갱신.

    사용 예:

        viz = WorkshopVisualizer(bus=bus, config=WorkshopVizConfig())
        viz.register_cell("MAT_01",  cell_type="material",   position=(0, 0, 0))
        viz.register_cell("WELD_01", cell_type="welding",    position=(5, 0, 0))
        viz.register_cell("INSP_01", cell_type="inspection", position=(10, 0, 0))
        viz.register_amr("AMR_01")
        viz.build_scene()

        # 매 step:
        viz.update(dt, sim_time)
    """

    def __init__(self, bus: MessageBus,
                 config: Optional[WorkshopVizConfig] = None,
                 scene_config: Optional[SceneConfig] = None) -> None:
        self.bus = bus
        self.config = config or WorkshopVizConfig()
        self.scene_config = scene_config or SceneConfig()
        self.scene: Optional[SceneBuilder] = None  # build_scene()에서 생성

        # 등록 정보 (build_scene 전에 수집)
        self._pending_cells: List[Dict] = []
        self._pending_amrs: List[Dict] = []

        # 셀 위치 저장 (cell_id -> (x, y, z))
        self.cell_positions: Dict[str, Tuple[float, float, float]] = {}
        self.cell_types: Dict[str, str] = {}

        # 부품 추적
        self._part_tweens: Dict[str, PartTween] = {}
        self._part_locations: Dict[str, str] = {}   # part_id -> cell_id (정적)
        self._part_types: Dict[str, str] = {}

        # MaterialCell 참조 (rack 입출고 추적용)
        self._material_cell: Optional[MaterialCell] = None
        self._known_rack_parts: set = set()

        # bus 구독 (셀 상태 변화 감지)
        self.bus.subscribe(Topics.CELL_STATUS, self._on_cell_status)

    # ========================================================================
    # 등록 (build_scene 전에 호출)
    # ========================================================================
    def register_cell(self, cell_id: str,
                      cell_type: str,
                      position: Tuple[float, float, float]) -> None:
        self._pending_cells.append({
            "cell_id": cell_id,
            "cell_type": cell_type.lower(),
            "position": position,
        })
        self.cell_positions[cell_id] = position
        self.cell_types[cell_id] = cell_type.lower()

    def register_amr(self, amr_id: str,
                     position: Tuple[float, float, float] = (0.0, -2.0, 0.0)) -> None:
        self._pending_amrs.append({
            "amr_id": amr_id,
            "position": position,
        })

    def attach_material_cell(self, cell: MaterialCell) -> None:
        """MaterialCell 참조 — 랙 부품 자동 spawn에 사용."""
        self._material_cell = cell

    # ========================================================================
    # 빌드
    # ========================================================================
    def build_scene(self) -> None:
        """SceneBuilder를 생성하고 등록된 셀/AMR을 USD에 추가.

        반드시 SimulationApp 인스턴스화 이후에 호출되어야 한다.
        """
        self.scene = SceneBuilder(self.scene_config)
        self.scene.add_ground_plane(size=30.0)

        # 셀 배치
        for c in self._pending_cells:
            cid = c["cell_id"]
            ctype = c["cell_type"]
            pos = c["position"]

            color = CELL_COLORS.get(ctype, CELL_COLORS["default"])
            self.scene.add_cell_workbench(cid, position=pos,
                                           size=self.config.cell_size,
                                           color=color)

            # 셀 종류별 부속 시각요소
            if ctype == "material" and self.config.show_smart_rack:
                self.scene.add_smart_rack(cid, capacity=8)
            elif ctype == "welding" and self.config.show_robot_arm:
                self.scene.add_robot_arm_placeholder(cid, color=(1.0, 0.45, 0.1))
            elif ctype == "inspection" and self.config.show_camera:
                self.scene.add_camera_placeholder(cid)
            elif ctype == "forming" and self.config.show_robot_arm:
                self.scene.add_robot_arm_placeholder(cid, color=(0.7, 0.5, 0.1))

        # AMR 배치
        for a in self._pending_amrs:
            self.scene.add_amr(a["amr_id"], position=a["position"])

        logger.info("[VIS] workshop scene built: %d cells, %d AMRs",
                    len(self._pending_cells), len(self._pending_amrs))

    # ========================================================================
    # 좌표 헬퍼
    # ========================================================================
    def _bench_top(self, cell_id: str) -> Tuple[float, float, float]:
        x, y, z = self.cell_positions[cell_id]
        bz = self.config.cell_size[2] + self.config.part_height_above_bench
        return (x, y, z + bz)

    def _input_buffer_pos(self, cell_id: str) -> Tuple[float, float, float]:
        x, y, z = self.cell_positions[cell_id]
        sx = self.config.cell_size[0]
        sz = self.config.cell_size[2]
        return (x - sx * 0.35, y, z + sz + 0.15)

    def _output_buffer_pos(self, cell_id: str) -> Tuple[float, float, float]:
        x, y, z = self.cell_positions[cell_id]
        sx = self.config.cell_size[0]
        sz = self.config.cell_size[2]
        return (x + sx * 0.35, y, z + sz + 0.15)

    # ========================================================================
    # 부품 시각화 API (외부에서 호출 가능)
    # ========================================================================
    def spawn_part(self, part_id: str, part_type: str, cell_id: str) -> None:
        """부품을 특정 셀의 input buffer 위치에 시각적으로 생성."""
        if self.scene is None:
            return
        color = PART_COLORS.get(part_type, PART_COLORS["default"])
        pos = self._input_buffer_pos(cell_id)
        self.scene.add_part(part_id, position=pos, color=color)
        self._part_locations[part_id] = cell_id
        self._part_types[part_id] = part_type

    def transfer_part(self, part_id: str, from_cell: str, to_cell: str,
                      duration: Optional[float] = None) -> None:
        """부품을 한 셀 → 다른 셀로 부드럽게 이동시키는 tween 등록."""
        if self.scene is None:
            return
        if part_id not in self._part_types:
            return

        start = self._output_buffer_pos(from_cell) if from_cell in self.cell_positions \
                else self._bench_top(from_cell)
        end = self._input_buffer_pos(to_cell)
        dur = duration if duration is not None else self.config.transfer_duration_sec

        self._part_tweens[part_id] = PartTween(part_id=part_id,
                                                start=start, end=end,
                                                duration=dur)
        self._part_locations[part_id] = to_cell

    def remove_part(self, part_id: str) -> None:
        if self.scene is None:
            return
        self.scene.remove_part(part_id)
        self._part_tweens.pop(part_id, None)
        self._part_locations.pop(part_id, None)
        self._part_types.pop(part_id, None)

    # ========================================================================
    # 매 step 업데이트
    # ========================================================================
    def update(self, dt: float, sim_time: float) -> None:
        """SimulationManager가 매 step마다 호출."""
        if self.scene is None:
            return

        # 1) tween 진행
        finished: List[str] = []
        for pid, tw in self._part_tweens.items():
            tw.elapsed += dt
            self.scene.move_part(pid, tw.current())
            if tw.progress() >= 1.0:
                finished.append(pid)
        for pid in finished:
            del self._part_tweens[pid]

        # 2) MaterialCell 랙 자동 sync (새 부품 입고 감지)
        if self._material_cell is not None:
            current = set()
            for slot in self._material_cell.smart_rack.slots:
                if slot.part_id is not None:
                    current.add((slot.part_id, slot.part_type))
            new_parts = current - {(pid, self._part_types.get(pid, ""))
                                    for pid in self._known_rack_parts}
            for pid, ptype in new_parts:
                if pid not in self._part_types:
                    self.spawn_part(pid, ptype, self._material_cell.cell_id)
                    self._known_rack_parts.add(pid)

    # ========================================================================
    # 이벤트 핸들러
    # ========================================================================
    def _on_cell_status(self, msg) -> None:
        """셀 상태 변경 이벤트 (현재는 로그용; 향후 시각 효과 추가)."""
        # 추후: 로봇팔 회전/조명 색 변화 등 추가 가능
        pass

    # ========================================================================
    # 디버깅
    # ========================================================================
    def get_part_position(self, part_id: str) -> Optional[Tuple[float, float, float]]:
        if self.scene is None:
            return None
        path = self.scene.get_part_path(part_id)
        return path
