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
# IK 컨트롤러는 lazy import (Isaac Sim 없을 때 부담 줄이기 위함)

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

    # Level 2.1: 실 로봇팔 사용 옵션
    use_real_robot: bool = False             # True면 Franka Panda USD 로드
    robot_name: str = "franka_panda"         # "franka_panda" | "ur10"
    enable_ik: bool = True                   # 용접 시 IK로 토치가 부품 추적
    weld_path_offset_y: float = 0.25         # 용접 경로의 y 방향 범위 (±)
    weld_path_height: float = 0.05           # 용접 경로의 부품 표면 위 높이

    # Level 2.1: GPU device-lost 회피용 안전 토글
    skip_auto_camera: bool = False           # True면 _auto_frame_camera 건너뜀
                                              # (AGX Thor에서 SceneCamera 생성이 GPU crash trigger인 경우)

    # ---------------------------------------------------------------- Level 2.2
    # 모션 모드: "auto" | "rmpflow" | "ik" | "heuristic"
    # - auto : RMPflow → IK → heuristic 순으로 자동 fallback
    # - ik   : 기존 Level 2.1 IK 컨트롤러 (RMPflow 비활성)
    # - rmpflow : RMPflow 강제 (실패 시 RMPflowController 내부에서 fallback)
    # - heuristic : 의존성 없는 휴리스틱 모션
    motion_mode: str = "auto"

    # 용접 스파크 파티클
    enable_sparks: bool = True
    spark_rate: float = 30.0                 # sparks / sec
    spark_lifetime_sec: float = 0.4

    # RMPflow 장애물 등록
    rmpflow_register_obstacles: bool = True  # 부품/작업대를 Sphere 장애물로 등록

    # ---------------------------------------------------------------- Level 2.3
    # 셀별 실 USD 자산 사용 옵션 (placeholder fallback 자동)
    use_real_inspection_cam: bool = True     # inspection 셀에 실 카메라 prim
    use_real_amr: bool = True                # AMR을 USD(NovaCarter 등)로 로드
    amr_asset_name: str = "nova_carter"      # asset_catalog 엔트리 이름
    use_real_smart_rack: bool = True         # material 셀에 KLT bin USD rack
    smart_rack_asset_name: str = "klt_bin"   # asset_catalog 엔트리 이름
    use_real_forming_arm: bool = True        # forming 셀에 실 UR 로봇팔
    forming_robot_name: str = "ur10"         # robot_loader.ROBOT_CATALOG 키

    # 환경/배경 디테일 — 순수 시각 요소(물리/충돌 없음), 실패해도 씬 빌드는 계속됨
    show_factory_walls: bool = True          # 작업장을 감싸는 4면 벽
    show_ceiling_lights: bool = True         # 셀 위 천장 조명 피팅


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

        # Level 2.1/2.2: 실 로봇팔 + (RMPflow 또는 IK) 컨트롤러
        # cell_id -> {
        #     "articulation": ..., "spec": RobotSpec,
        #     "ik":   IKController | None,         (motion_mode=="ik"인 경우)
        #     "rmp":  RMPflowController | None,    (motion_mode in {"auto","rmpflow","heuristic"}인 경우)
        #     "sparks": WeldingSparkEmitter | None, (enable_sparks=True인 경우)
        # }
        self._robots: Dict[str, Dict] = {}
        # 현재 가공 중인 셀의 부품 위치 (용접 경로 계산)
        self._cell_processing_part: Dict[str, str] = {}   # cell_id -> part_id

        # Level 2.2: 모션 모드 정규화
        self._motion_mode = (self.config.motion_mode or "auto").lower()
        if self._motion_mode not in ("auto", "rmpflow", "ik", "heuristic"):
            logger.warning("[VIS] unknown motion_mode=%s, falling back to 'auto'",
                           self._motion_mode)
            self._motion_mode = "auto"

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
    # 로봇팔 spawn (placeholder vs 실 모델)
    # ========================================================================
    def _spawn_robot_arm(self, cell_id: str,
                          color: Tuple[float, float, float] = (1.0, 0.45, 0.1)) -> None:
        """use_real_robot 설정에 따라 placeholder 또는 실 USD 로봇팔을 생성.

        실 로봇팔이 성공적으로 로드되면 IKController를 함께 등록한다.
        로드 실패(자산 누락 등) 시 placeholder로 자동 fallback.
        """
        if not self.config.use_real_robot:
            self.scene.add_robot_arm_placeholder(cell_id, color=color)
            return

        try:
            articulation = self.scene.add_real_robot_arm(
                cell_id,
                robot_name=self.config.robot_name,
                offset=(0.0, -0.3, self.config.cell_size[2] + 0.05),
            )
        except Exception as e:
            logger.warning("[VIS] real robot load failed for %s: %s "
                           "— falling back to placeholder", cell_id, e)
            self.scene.add_robot_arm_placeholder(cell_id, color=color)
            return

        # 모션 컨트롤러 준비 — Level 2.2: RMPflow vs IK 선택
        ik_ctrl = None
        rmp_ctrl = None
        spec = None

        if self.config.enable_ik and articulation is not None:
            try:
                from fdw_sim.visualization.robot_loader import ROBOT_CATALOG
                spec = ROBOT_CATALOG.get(self.config.robot_name)
            except Exception as e:
                logger.warning("[VIS] robot spec lookup failed: %s", e)
                spec = None

            if spec is not None:
                if self._motion_mode == "ik":
                    # 명시적 IK 모드
                    try:
                        from fdw_sim.visualization.ik_controller import (
                            IKConfig, IKController,
                        )
                        ik_ctrl = IKController(articulation, spec, config=IKConfig())
                        ik_ctrl.go_home()
                        logger.info("[VIS] IK controller attached to %s (robot=%s)",
                                    cell_id, self.config.robot_name)
                    except Exception as e:
                        logger.warning("[VIS] IK controller setup failed for %s: %s",
                                       cell_id, e)
                else:
                    # auto / rmpflow / heuristic → RMPflowController로 통합
                    try:
                        from fdw_sim.visualization.rmpflow_controller import (
                            RMPflowConfig, RMPflowController,
                        )
                        backend = ("auto" if self._motion_mode == "auto"
                                   else self._motion_mode)
                        rmp_ctrl = RMPflowController(
                            articulation,
                            spec,
                            config=RMPflowConfig(preferred_backend=backend),
                        )
                        rmp_ctrl.go_home()
                        logger.info("[VIS] RMPflow controller attached to %s "
                                    "(robot=%s, mode=%s)",
                                    cell_id, self.config.robot_name,
                                    self._motion_mode)
                    except Exception as e:
                        logger.warning("[VIS] RMPflow controller setup failed for %s: %s "
                                       "— falling back to IK", cell_id, e)
                        try:
                            from fdw_sim.visualization.ik_controller import (
                                IKConfig, IKController,
                            )
                            ik_ctrl = IKController(articulation, spec, config=IKConfig())
                            ik_ctrl.go_home()
                        except Exception as e2:
                            logger.warning("[VIS] IK fallback also failed for %s: %s",
                                           cell_id, e2)

        # 스파크 emitter — RMPflow와 IK 양쪽 모드에서 동작 가능
        sparks = None
        if self.config.enable_sparks and articulation is not None:
            try:
                from fdw_sim.visualization.spark_emitter import (
                    SparkEmitterConfig, WeldingSparkEmitter,
                )
                cell_root = self.scene.get_cell_path(cell_id)
                if cell_root is not None:
                    sparks = WeldingSparkEmitter(
                        self.scene._stage,
                        cell_root,
                        SparkEmitterConfig(
                            spark_rate=self.config.spark_rate,
                            lifetime_sec=self.config.spark_lifetime_sec,
                        ),
                    )
                    logger.info("[VIS] spark emitter attached to %s", cell_id)
            except Exception as e:
                logger.warning("[VIS] spark emitter setup failed for %s: %s",
                               cell_id, e)
                sparks = None

        self._robots[cell_id] = {
            "articulation": articulation,
            "ik": ik_ctrl,
            "rmp": rmp_ctrl,
            "spec": spec,
            "sparks": sparks,
        }

    # ========================================================================
    # 빌드
    # ========================================================================
    def build_scene(self) -> None:
        """SceneBuilder를 생성하고 등록된 셀/AMR을 USD에 추가.

        반드시 SimulationApp 인스턴스화 이후에 호출되어야 한다.
        """
        self.scene = SceneBuilder(self.scene_config)
        self.scene.add_ground_plane(size=30.0)

        if self.config.show_factory_walls:
            try:
                self.scene.add_factory_walls()
            except Exception:
                logger.exception("[VIS] add_factory_walls failed — continuing without walls")

        if self.config.show_ceiling_lights:
            try:
                self.scene.add_ceiling_lights()
            except Exception:
                logger.exception("[VIS] add_ceiling_lights failed — continuing without them")

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
                placed = None
                if self.config.use_real_smart_rack:
                    try:
                        placed = self.scene.add_smart_rack_real(
                            cid,
                            capacity=6,
                            prop_asset=self.config.smart_rack_asset_name,
                        )
                    except Exception:
                        logger.exception("[VIS] add_smart_rack_real failed for %s", cid)
                        placed = None
                if placed is None:
                    # placeholder fallback (cube rack)
                    self.scene.add_smart_rack(cid, capacity=8)
            elif ctype == "welding" and self.config.show_robot_arm:
                self._spawn_robot_arm(cid, color=(1.0, 0.45, 0.1))
            elif ctype == "inspection" and self.config.show_camera:
                placed = None
                if self.config.use_real_inspection_cam:
                    try:
                        placed = self.scene.add_inspection_camera_real(cid)
                    except Exception:
                        logger.exception("[VIS] add_inspection_camera_real failed for %s", cid)
                        placed = None
                if placed is None:
                    self.scene.add_camera_placeholder(cid)
            elif ctype == "forming" and self.config.show_robot_arm:
                # forming 셀: 실 UR 로봇팔 우선, 실패 시 placeholder
                spawned = False
                if self.config.use_real_forming_arm:
                    try:
                        # _spawn_robot_arm은 self.config.robot_name을 사용하므로
                        # 임시로 forming_robot_name으로 스왑한다.
                        saved_name = self.config.robot_name
                        saved_use_real = self.config.use_real_robot
                        self.config.robot_name = self.config.forming_robot_name
                        self.config.use_real_robot = True
                        try:
                            self._spawn_robot_arm(cid, color=(0.7, 0.5, 0.1))
                            spawned = cid in self._robots
                        finally:
                            self.config.robot_name = saved_name
                            self.config.use_real_robot = saved_use_real
                    except Exception:
                        logger.exception("[VIS] forming arm USD load failed for %s", cid)
                        spawned = False
                if not spawned:
                    self.scene.add_robot_arm_placeholder(cid, color=(0.7, 0.5, 0.1))

        # AMR 배치 — 실 USD(NovaCarter 등) 우선, 실패 시 cube placeholder
        for a in self._pending_amrs:
            placed = None
            if self.config.use_real_amr:
                try:
                    placed = self.scene.add_amr_usd(
                        a["amr_id"],
                        asset_name=self.config.amr_asset_name,
                        position=a["position"],
                    )
                except Exception:
                    logger.exception("[VIS] add_amr_usd failed for %s", a["amr_id"])
                    placed = None
            if placed is None:
                self.scene.add_amr(a["amr_id"], position=a["position"])

        logger.info("[VIS] workshop scene built: %d cells, %d AMRs",
                    len(self._pending_cells), len(self._pending_amrs))

        # Level 2.3 진단 — 어떤 cell/AMR이 실 USD reference를 갖고 있고
        # 어떤 것이 placeholder fallback 으로 떨어졌는지 명시적으로 출력.
        # 환경변수 FDW_DISABLE_STAGE_DIAGNOSE=1 로 끌 수 있다.
        import os as _os
        if _os.environ.get("FDW_DISABLE_STAGE_DIAGNOSE", "") != "1":
            try:
                self.scene.diagnose_stage(verbose=True)
            except Exception:
                logger.exception("[VIS] diagnose_stage failed")

        # 카메라 자동 framing — 모든 셀이 한눈에 보이도록 viewport 이동
        # skip_auto_camera=True인 경우 (GPU device-lost 회피용) 건너뜀
        if self.config.skip_auto_camera:
            logger.warning("[VIS] auto camera framing SKIPPED "
                           "(skip_auto_camera=True — GPU device-lost 회피 모드)")
        else:
            self._auto_frame_camera()

    def _auto_frame_camera(self) -> None:
        """등록된 셀들의 bounding box를 기반으로 viewport 카메라 자동 배치."""
        if not self.cell_positions:
            return
        xs = [p[0] for p in self.cell_positions.values()]
        ys = [p[1] for p in self.cell_positions.values()]
        zs = [p[2] for p in self.cell_positions.values()]
        cx = (min(xs) + max(xs)) / 2.0
        cy = (min(ys) + max(ys)) / 2.0
        cz = (min(zs) + max(zs)) / 2.0 + self.config.cell_size[2] / 2.0

        # 셀 간격을 기반으로 카메라 거리 결정
        spread = max(max(xs) - min(xs), max(ys) - min(ys), 4.0)
        distance = max(spread * 1.8, 8.0)
        height = max(spread * 0.7, 4.0)

        try:
            self.scene.frame_viewport_to_scene(
                center=(cx, cy, cz),
                distance=distance,
                height=height,
                # viewport active-camera 변경은 위험할 수 있어 분리됨
                # skip_auto_camera=False여도 viewport switch는 환경변수로 추가 제어 가능
                set_viewport_camera=self._should_set_viewport_camera(),
            )
        except Exception:
            logger.exception("[VIS] auto frame camera failed")

    def _should_set_viewport_camera(self) -> bool:
        """viewport active-camera 변경 허용 여부.

        환경변수 FDW_DISABLE_VIEWPORT_SWITCH=1 이면 카메라 prim은 만들되
        viewport 변경(SetActiveCamera)은 건너뜀. AGX Thor에서 이 호출이
        SetLightingMenuModeCommand를 트리거하고 GPU device-lost를
        일으키는 경우의 회피책.
        """
        import os
        if os.environ.get("FDW_DISABLE_VIEWPORT_SWITCH", "") == "1":
            return False
        return True

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
        # smart_rack 구조: Dict[part_id, {"part_type": ..., ...}]
        if self._material_cell is not None:
            try:
                rack = self._material_cell.smart_rack
                # dict 형태 (Dict[str, dict]) — 정식 스키마
                if isinstance(rack, dict):
                    for pid, meta in rack.items():
                        if pid in self._known_rack_parts:
                            continue
                        ptype = (meta or {}).get("part_type", "default") \
                            if isinstance(meta, dict) else "default"
                        if pid not in self._part_types:
                            self.spawn_part(pid, ptype, self._material_cell.cell_id)
                        self._known_rack_parts.add(pid)
                # 객체 형태 fallback (.slots 인터페이스가 있을 경우)
                elif hasattr(rack, "slots"):
                    for slot in rack.slots:
                        pid = getattr(slot, "part_id", None)
                        if pid is None or pid in self._known_rack_parts:
                            continue
                        ptype = getattr(slot, "part_type", "default")
                        if pid not in self._part_types:
                            self.spawn_part(pid, ptype, self._material_cell.cell_id)
                        self._known_rack_parts.add(pid)
            except Exception:
                logger.exception("smart_rack sync failed")

        # 3) 모션 컨트롤러 + 스파크 emitter 업데이트
        #    (용접 셀 로봇팔 → 부품 추적 + TCP 위치 기반 스파크 방출)
        for cell_id, rob in self._robots.items():
            ik = rob.get("ik")
            rmp = rob.get("rmp")
            sparks = rob.get("sparks")
            ctrl = rmp if rmp is not None else ik

            if ctrl is not None:
                try:
                    ctrl.update(dt)
                except Exception:
                    logger.exception("[VIS] controller update failed for %s", cell_id)

            # 스파크: TCP 위치 갱신 + weld 단계일 때만 활성화
            if sparks is not None:
                tcp = None
                if rmp is not None and hasattr(rmp, "get_tcp_position"):
                    try:
                        tcp = rmp.get_tcp_position()
                    except Exception:
                        tcp = None
                if tcp is None and ik is not None and hasattr(ik, "get_tcp_position"):
                    try:
                        tcp = ik.get_tcp_position()
                    except Exception:
                        tcp = None
                if tcp is not None:
                    sparks.set_tcp_position(tcp)

                # 활성 조건: 현재 컨트롤러 phase == "weld"
                active = False
                if ctrl is not None and hasattr(ctrl, "get_phase"):
                    try:
                        active = (str(ctrl.get_phase()).lower() == "weld")
                    except Exception:
                        active = False
                sparks.set_active(active)

                try:
                    sparks.update(dt)
                except Exception:
                    logger.exception("[VIS] spark emitter update failed for %s",
                                     cell_id)

    # ========================================================================
    # 이벤트 핸들러
    # ========================================================================
    def _on_cell_status(self, msg) -> None:
        """셀 상태 변경 이벤트 — 용접 셀이 PROCESSING이면 모션 경로 시작."""
        try:
            cell_id = getattr(msg, "cell_id", None)
            state = getattr(msg, "state", None)
            state_str = state.value if hasattr(state, "value") else str(state)
        except Exception:
            return

        if cell_id is None or cell_id not in self._robots:
            return

        rob = self._robots[cell_id]
        ctrl = rob.get("rmp") or rob.get("ik")
        if ctrl is None:
            return

        # PROCESSING 상태로 전환되면 용접 경로 시작
        if state_str.upper() == "PROCESSING" and ctrl.is_idle():
            self._start_welding_motion(cell_id)
        elif state_str.upper() in ("IDLE", "READY") and not ctrl.is_idle():
            # 가공 종료 → home으로
            try:
                ctrl.go_home()
            except Exception:
                logger.exception("[VIS] go_home failed for %s", cell_id)

    def _start_welding_motion(self, cell_id: str) -> None:
        """용접 셀의 입력 버퍼 부품 위로 토치를 이동시키는 경로 시작.

        RMPflow 컨트롤러가 있으면 RMPflow API (start_path(start, end, ...)) 사용,
        없으면 IK 컨트롤러 (start_path(WeldingPath))로 fallback.
        장애물(작업대, 부품)을 가능하면 RMPflow에 등록.
        """
        if cell_id not in self.cell_positions:
            return
        rob = self._robots.get(cell_id)
        if rob is None:
            return

        # 부품의 위치 (입력 버퍼 상단)
        bx, by, bz = self._input_buffer_pos(cell_id)
        oy = self.config.weld_path_offset_y
        h = self.config.weld_path_height
        start = (bx, by - oy, bz + h)
        end = (bx, by + oy, bz + h)
        travel_time_sec = 8.0
        approach_height = 0.1

        rmp = rob.get("rmp")
        if rmp is not None:
            # 장애물 등록: 작업대 상판 + 부품 (Sphere 근사)
            if self.config.rmpflow_register_obstacles:
                try:
                    self._register_cell_obstacles(cell_id, rmp)
                except Exception:
                    logger.exception("[VIS] obstacle registration failed for %s",
                                     cell_id)
            try:
                rmp.start_path(
                    start=start,
                    end=end,
                    travel_time_sec=travel_time_sec,
                    approach_height=approach_height,
                )
                logger.info("[VIS] RMPflow welding motion started @ %s "
                            "(%s -> %s)", cell_id, start, end)
            except Exception:
                logger.exception("[VIS] RMPflow start_path failed for %s", cell_id)
            return

        ik = rob.get("ik")
        if ik is not None:
            try:
                from fdw_sim.visualization.ik_controller import WeldingPath
            except Exception:
                return
            path = WeldingPath(
                start=start,
                end=end,
                travel_time_sec=travel_time_sec,
                approach_height=approach_height,
            )
            try:
                ik.start_path(path)
                logger.info("[VIS] IK welding motion started @ %s (path=%s -> %s)",
                            cell_id, path.start, path.end)
            except Exception:
                logger.exception("[VIS] IK start_path failed for %s", cell_id)

    def _register_cell_obstacles(self, cell_id: str, rmp) -> None:
        """용접 셀의 작업대/부품을 RMPflow CollisionSphere로 등록."""
        try:
            from fdw_sim.visualization.rmpflow_controller import CollisionSphere
        except Exception:
            return

        if not hasattr(rmp, "clear_obstacles") or not hasattr(rmp, "add_obstacle"):
            return

        rmp.clear_obstacles()

        # 작업대 상판 — 셀 중앙, sx*sy 영역을 큰 Sphere 1개로 근사
        cx, cy, cz = self.cell_positions[cell_id]
        sx, sy, sz = self.config.cell_size
        bench_top_z = cz + sz
        # 가로/세로 중 큰 변의 절반을 반지름으로
        bench_radius = max(sx, sy) * 0.6
        rmp.add_obstacle(CollisionSphere(
            name=f"{cell_id}_bench",
            center=(cx, cy, bench_top_z - bench_radius * 0.5),
            radius=bench_radius,
            static=True,
        ))

        # 입력 버퍼 위 부품 (있다면) — 작은 Sphere
        bx, by, bz = self._input_buffer_pos(cell_id)
        rmp.add_obstacle(CollisionSphere(
            name=f"{cell_id}_part",
            center=(bx, by, bz),
            radius=0.08,
            static=True,
        ))
        logger.info("[VIS] registered %d obstacles for RMPflow @ %s", 2, cell_id)

    # ========================================================================
    # 디버깅
    # ========================================================================
    def get_part_position(self, part_id: str) -> Optional[Tuple[float, float, float]]:
        if self.scene is None:
            return None
        path = self.scene.get_part_path(part_id)
        return path
