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
import math

from fdw_sim.cells.base.cell_base import DistributedIntelligenceCell
from fdw_sim.cells.material.material_cell import MaterialCell
from fdw_sim.messaging.bus import MessageBus, Topics
from fdw_sim.visualization.scene_builder import (
    CELL_SIGNAL_COLORS, SceneBuilder, SceneConfig,
)
# IK 컨트롤러는 lazy import (Isaac Sim 없을 때 부담 줄이기 위함)

logger = logging.getLogger(__name__)


# 셀 종류별 기본 색상 — 실사 공장 사진(흰색/회색 CNC 설비) 레퍼런스에 맞춰
# 전부 흰색/밝은 회색 계열로 통일(2026-09-28). 예전엔 이 색으로 구역을
# 구분했지만, 그 역할은 이제 scene_builder.add_signal_tower()의 경광등
# 색(CELL_SIGNAL_COLORS)이 대신한다.
CELL_COLORS = {
    "material":   (0.88, 0.88, 0.89),
    "welding":    (0.85, 0.85, 0.87),
    "inspection": (0.90, 0.90, 0.91),
    "forming":    (0.85, 0.85, 0.86),
    "default":    (0.86, 0.86, 0.87),
}

# 부품 타입별 색상 — 실사 아연도금 강관(용접 전 원자재) 레퍼런스에 맞춰
# 채도 높은 디버그용 파랑/주황 대신 금속 회색 계열로 통일(2026-09-29,
# 사용자 제공 USDA 스펙 color=(0.75,0.78,0.80) 반영). 부품 타입 구분은
# 색이 아니라 part_id/로그로 확인한다 — add_part()가 이 색에 metallic
# PBR 재질(GalvanizedSteel)도 같이 입힌다.
PART_COLORS = {
    "tubular_frame_A": (0.75, 0.78, 0.80),
    "tubular_frame_B": (0.73, 0.75, 0.77),   # 아주 약간 어둡게 — 타입 구분용
    "default":         (0.75, 0.78, 0.80),
}


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
    use_real_robot: bool = False             # True면 CRX-10iA USD 로드
    robot_name: str = "fanuc_crx10ia"        # "fanuc_crx10ia" | "franka_panda" | "ur10"
    enable_ik: bool = True                   # 용접 시 IK로 토치가 부품 추적
    weld_path_offset_y: float = 0.25         # 용접 경로의 y 방향 범위 (±)
    weld_path_height: float = 0.05           # 용접 경로의 부품 표면 위 높이

    # Level 2.1: GPU device-lost 회피용 안전 토글
    skip_auto_camera: bool = False           # True면 _auto_frame_camera 건너뜀
                                              # (AGX Thor에서 SceneCamera 생성이 GPU crash trigger인 경우)

    # Level 2.4: AMR 실제 주행 + 로봇팔 pick-and-place
    amr_dock_offset: Tuple[float, float] = (0.0, -1.3)  # 셀 중심 기준 AMR 정차 위치 오프셋
    amr_deck_height: float = 0.28            # 부품이 AMR 적재함 위에 놓이는 높이(m)
    pick_place_travel_time_sec: float = 3.0  # AMR -> 작업대 운반 구간 소요 시간
    pick_place_approach_height: float = 0.3  # pick/place 시 위아래로 여유를 두는 높이

    # ---------------------------------------------------------------- Level 2.2
    # 모션 모드: "auto" | "rmpflow" | "ik" | "heuristic"
    # - auto : RMPflow → IK → heuristic 순으로 자동 fallback
    # - ik   : 기존 Level 2.1 IK 컨트롤러 (RMPflow 비활성)
    # - rmpflow : RMPflow 강제 (실패 시 RMPflowController 내부에서 fallback)
    # - heuristic : 의존성 없는 휴리스틱 모션
    motion_execution: str = "verified"  # verified | schematic (explicit logical demo)
    amr_footprint_size: Tuple[float, float] = (1.2, 0.8)
    amr_clearance_m: float = 0.15
    # Maximum centered carried-load dimensions reserved even on empty trips.
    # The default visible 0.525m pipe is smaller. Oversized parts fail closed.
    amr_payload_footprint_size: Tuple[float, float] = (1.2, 0.8)
    amr_payload_offset: Tuple[float, float] = (0.0, 0.0)
    amr_payload_height_m: float = 1.0
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
    use_real_amr: bool = True                # AMR을 USD(MiR100 등)로 로드
    amr_asset_name: str = "mir100"           # asset_catalog 엔트리 이름
    use_real_smart_rack: bool = True         # material 셀에 KLT bin USD rack
    smart_rack_asset_name: str = "klt_bin"   # asset_catalog 엔트리 이름
    use_real_forming_arm: bool = True        # forming 셀에 실 UR 로봇팔
    forming_robot_name: str = "ur10"         # robot_loader.ROBOT_CATALOG 키

    # 환경/배경 디테일 — 순수 시각 요소(물리/충돌 없음), 실패해도 씬 빌드는 계속됨
    show_factory_walls: bool = True          # 작업장을 감싸는 4면 벽
    show_ceiling_lights: bool = True         # 셀 위 천장 조명 피팅
    show_workshop_layout: bool = True        # FDW 배치도(1안) 구역/랙/펜스/placeholder


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

        # Level 2.4: AMR 실 주행 + pick-and-place
        # amr_id -> 마지막으로 처리한 last_delivered_command_id (중복 처리 방지)
        self._amr_last_seen_delivery: Dict[str, str] = {}
        # cell_id -> part awaiting explicitly schematic logical placement
        self._active_pick_place: Dict[str, str] = {}
        # cell_id -> 그 part를 배송한 amr_id — pick-and-place가 끝나면
        # material_cell.confirm_pickup(amr_id)으로 AMR을 재배차 가능하게 푼다
        self._active_pick_place_amr: Dict[str, str] = {}
        self._pick_context: Dict[str, dict] = {}
        self._weld_context: Dict[str, dict] = {}
        self._blocked_deliveries: set = set()
        if self.config.motion_execution not in ("verified", "schematic"):
            raise ValueError("motion_execution must be verified or schematic")

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
                     position: Tuple[float, float, float] = (0.0, -2.0, 0.0),
                     heading: float = 0.0) -> None:
        self._pending_amrs.append({
            "amr_id": amr_id,
            "position": position,
            "heading": heading,
        })

    def attach_material_cell(self, cell: MaterialCell) -> None:
        """Bind physical transport geometry and explicitly select motion gates."""
        self._material_cell = cell
        cell.require_pickup_confirmation = True
        docks = {cid: self._amr_dock_pos(cid) for cid in self.cell_positions}
        cell.configure_transport_geometry(
            docks, footprint_size=self.config.amr_footprint_size,
            clearance=self.config.amr_clearance_m,
            payload_footprint_size=self.config.amr_payload_footprint_size,
            payload_offset=self.config.amr_payload_offset,
        )
        for cid, target in cell.cell_registry.items():
            if self.cell_types.get(cid) == "welding":
                target.set_motion_required(self.config.use_real_robot)
        mode = self.config.motion_execution if self.config.use_real_robot else "logical-no-robot"
        logger.warning("[VIS] execution=%s; schematic/logical transfers are not physical grasps", mode)

    # ========================================================================
    # 로봇팔 spawn (placeholder vs 실 모델)
    # ========================================================================
    def _spawn_robot_arm(self, cell_id: str,
                          color: Tuple[float, float, float] = (0.85, 0.85, 0.86)) -> None:
        """use_real_robot 설정에 따라 placeholder 또는 실 USD 로봇팔을 생성.

        실 로봇팔이 성공적으로 로드되면 IKController를 함께 등록한다.
        로드 실패(자산 누락 등) 시 placeholder로 자동 fallback.
        """
        if not self.config.use_real_robot:
            self.scene.add_robot_arm_placeholder(cell_id, color=color)
            return

        robot_offset = (0.0, -0.3, self.config.cell_size[2] + 0.05)
        try:
            articulation = self.scene.add_real_robot_arm(
                cell_id,
                robot_name=self.config.robot_name,
                offset=robot_offset,
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
                from fdw_sim.visualization.robot_base_pose import (
                    RobotBasePose, read_robot_base_pose,
                )
                # The loader's offset is parent-local. Prefer the composed USD
                # or articulation world pose; use registered cell coordinates
                # only before either live source is available.
                cell_pos = self.cell_positions[cell_id]
                fallback_pos = (
                    cell_pos[0] + robot_offset[0],
                    cell_pos[1] + robot_offset[1],
                    cell_pos[2] + robot_offset[2] + spec.base_offset_z,
                )
                cell_root = self.scene.get_cell_path(cell_id)
                base_pose = read_robot_base_pose(
                    articulation, RobotBasePose(fallback_pos),
                    stage=getattr(self.scene, "_stage", None),
                    prim_path=f"{cell_root}/RobotArm" if cell_root else None,
                )
                base_kwargs = {
                    "base_position": base_pose.position,
                    "base_orientation": base_pose.orientation,
                }
                if self._motion_mode == "ik":
                    # 명시적 IK 모드
                    try:
                        from fdw_sim.visualization.ik_controller import (
                            IKConfig, IKController,
                        )
                        ik_ctrl = IKController(articulation, spec, config=IKConfig(),
                                               **base_kwargs)
                        # Preserve measured USD rest joints; no forced startup home.
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
                            **base_kwargs,
                        )
                        # Preserve measured USD rest joints; no forced startup home.
                        logger.info("[VIS] motion controller attached to %s "
                                    "(robot=%s, requested=%s, actual_backend=%s, grasp=unverified)",
                                    cell_id, self.config.robot_name,
                                    self._motion_mode, rmp_ctrl.backend_name)
                    except Exception as e:
                        logger.warning("[VIS] RMPflow controller setup failed for %s: %s "
                                       "— falling back to IK", cell_id, e)
                        try:
                            from fdw_sim.visualization.ik_controller import (
                                IKConfig, IKController,
                            )
                            ik_ctrl = IKController(articulation, spec, config=IKConfig(),
                                                   **base_kwargs)
                            # Preserve measured USD rest joints; no forced startup home.
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
        # WORKSHOP_BOUNDS(0~36.1, 0~12.4)는 원점에서 멀리 떨어져 있어
        # 원점 중심 plane으로는 절반이 빈 공간을 덮게 된다 — 건물 중심으로 이동.
        from fdw_sim.visualization.scene_builder import WORKSHOP_BOUNDS
        x_min, x_max, y_min, y_max = WORKSHOP_BOUNDS
        ground_center = ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0, 0.0)
        ground_size = max(x_max - x_min, y_max - y_min) + 8.0
        self.scene.add_ground_plane(size=ground_size, center=ground_center)

        if self.config.show_factory_walls:
            try:
                # 2026-09-29 사용자 요청: 지붕과 남쪽 벽은 안 그린다 — 카메라가
                # 작업장 내부를 위/앞에서 볼 수 있도록. add_factory_walls()의
                # skip_walls 기본값이 이미 South를 빼므로 별도 인자 불필요.
                self.scene.add_factory_walls()
                # forming_cell 안전펜스 반입구(x 약 28.2~33.0) 구간엔 기둥을
                # 안 놓는다 — scene_builder.WORKSHOP_ZONES["forming_cell"] +
                # FORMING_GATE_WIDTH_M 기준 계산(반입구를 막아 보이는 것 방지).
                from fdw_sim.visualization.scene_builder import (
                    FORMING_GATE_WIDTH_M, WORKSHOP_ZONES,
                )
                fx0, _fy0, fw, _fd, _c, _l = WORKSHOP_ZONES["forming_cell"]
                gate_cx = fx0 + fw / 2.0
                gate_half = FORMING_GATE_WIDTH_M / 2.0 + 0.5
                self.scene.add_structural_pillars(
                    skip_x_ranges=[(gate_cx - gate_half, gate_cx + gate_half)])
                self.scene.add_overhead_crane()
            except Exception:
                logger.exception("[VIS] add_factory_walls/pillars/crane failed — continuing without them")

        if self.config.show_ceiling_lights:
            try:
                self.scene.add_ceiling_lights()
            except Exception:
                logger.exception("[VIS] add_ceiling_lights failed — continuing without them")

        if self.config.show_workshop_layout:
            try:
                self.scene.add_workshop_layout()
            except Exception:
                logger.exception("[VIS] add_workshop_layout failed — continuing without it")

        # 셀 배치
        for c in self._pending_cells:
            cid = c["cell_id"]
            ctype = c["cell_type"]
            pos = c["position"]

            color = CELL_COLORS.get(ctype, CELL_COLORS["default"])
            self.scene.add_cell_workbench(cid, position=pos,
                                           size=self.config.cell_size,
                                           color=color)
            try:
                signal_color = CELL_SIGNAL_COLORS.get(ctype, CELL_SIGNAL_COLORS["default"])
                self.scene.add_signal_tower(cid, color=signal_color)
            except Exception:
                logger.exception("[VIS] add_signal_tower failed for %s", cid)

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
                self._spawn_robot_arm(cid, color=(0.88, 0.88, 0.89))
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
                            self._spawn_robot_arm(cid, color=(0.85, 0.85, 0.86))
                            spawned = cid in self._robots
                        finally:
                            self.config.robot_name = saved_name
                            self.config.use_real_robot = saved_use_real
                    except Exception:
                        logger.exception("[VIS] forming arm USD load failed for %s", cid)
                        spawned = False
                if not spawned:
                    self.scene.add_robot_arm_placeholder(cid, color=(0.85, 0.85, 0.86))

        # AMR 배치 — 실 USD(NovaCarter 등) 우선, 실패 시 cube placeholder
        measured_footprints = []
        measured_heights = []
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
            initial_world = self.scene.local_point_to_world_meters(a["position"])
            self.scene.move_amr(a["amr_id"], initial_world, heading=a["heading"])
            footprint = self.scene.get_amr_footprint(a["amr_id"], a["position"])
            if footprint is None:
                raise RuntimeError("Cannot measure loaded AMR footprint; refusing unbounded transport")
            measured_footprints.append(footprint)
            height = self.scene.get_amr_height(a["amr_id"])
            if height is None or not math.isfinite(height) or height <= 0:
                raise RuntimeError("Cannot measure loaded AMR height; refusing transport")
            measured_heights.append(height)

        if measured_footprints and self._material_cell is not None:
            footprint = tuple(max(self.config.amr_footprint_size[i],
                                  *(size[i] for size in measured_footprints)) for i in range(2))
            if (not math.isfinite(self.config.amr_payload_height_m) or self.config.amr_payload_height_m <= 0
                    or not math.isfinite(self.config.amr_deck_height) or self.config.amr_deck_height < 0):
                raise ValueError("invalid configured payload/deck height")
            # The scene's composed USD geometry is authoritative. Never fall
            # back to a rack-free map if references/bounds cannot be measured.
            navigation = self.scene.get_static_navigation_map(robot_height_m=max(
                *measured_heights, self.config.amr_deck_height + self.config.amr_payload_height_m))
            if navigation is None:
                raise RuntimeError("Cannot measure static navigation map; refusing transport")
            from fdw_sim.cells.material.navigation_setup import select_transport_positions
            cell_centers = {cid: self.scene.get_cell_world_position(cid)[:2]
                            for cid in self.cell_positions}
            requested_docks = {cid: self.scene.local_point_to_world_meters(
                (*self._amr_dock_pos(cid), self.cell_positions[cid][2]))[:2]
                for cid in self.cell_positions}
            radius = max(math.hypot(*footprint) / 2, math.hypot(*(
                abs(self.config.amr_payload_offset[i]) + self.config.amr_payload_footprint_size[i] / 2
                for i in range(2))))
            docks, bays = select_transport_positions(requested_docks, cell_centers,
                navigation.obstacles, navigation.bounds, radius,
                self.config.amr_clearance_m, len(self._material_cell.amrs),
                boundary_polygon=navigation.boundary_polygon)
            self._material_cell.configure_transport_geometry(
                docks, parking_positions=bays, footprint_size=footprint,
                clearance=self.config.amr_clearance_m,
                payload_footprint_size=self.config.amr_payload_footprint_size,
                payload_offset=self.config.amr_payload_offset,
                static_obstacles=navigation.obstacles, navigation_bounds=navigation.bounds,
                navigation_boundary=navigation.boundary_polygon,
                navigation_source=navigation.source)
            self._navigation_docks = docks
            self._navigation_floor_height = navigation.floor_height_m
            self.scene.add_navigation_markers(docks, bays)
            logger.warning("[VIS] static navigation source=%s obstacles=%d docks=%s bays=%s; "
                           "service poses do not certify arm reach or physical grasp",
                           navigation.source, len(navigation.obstacles), docks, bays)
            for amr in self._material_cell.amrs:
                self.scene.move_amr(amr.amr_id, (*amr.position, getattr(self, "_navigation_floor_height", 0.0)), heading=amr.heading)
            logger.info("[VIS] measured fleet envelope=%s with %.3fm clearance", footprint,
                        self.config.amr_clearance_m)

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

    def _amr_dock_pos(self, cell_id: str) -> Optional[Tuple[float, float]]:
        """AMR이 이 셀 옆에 정차하는 지점(바닥, 2D) — 작업대 중앙을 그대로
        관통하지 않도록 셀 중심에서 고정 오프셋만큼 떨어뜨린다."""
        if cell_id in getattr(self, "_navigation_docks", {}):
            return self._navigation_docks[cell_id]
        pos = self.cell_positions.get(cell_id)
        if pos is None:
            return None
        ox, oy = self.config.amr_dock_offset
        return (pos[0] + ox, pos[1] + oy)

    # ========================================================================
    # 부품 시각화 API (외부에서 호출 가능)
    # ========================================================================
    def spawn_part(self, part_id: str, part_type: str, cell_id: str) -> None:
        """부품을 특정 셀의 input buffer 위치에 시각적으로 생성.

        이미 스폰된 part_id면 아무것도 하지 않는다(idempotent) — SimulationManager
        .submit_job()이 재고 등록 시점에 이 메서드를 직접 호출해 즉시 스폰하고,
        MaterialCell 랙 폴링 루프(update())도 동일한 part_id를 나중에 다시 볼 수
        있으므로 두 경로 모두에서 안전하게 호출 가능해야 한다.
        """
        if self.scene is None:
            return
        if part_id in self._part_types:
            return
        color = PART_COLORS.get(part_type, PART_COLORS["default"])
        pos = self._input_buffer_pos(cell_id)
        self.scene.add_part(part_id, position=pos, color=color)
        if self._material_cell is not None and callable(getattr(self.scene, "get_part_footprint", None)):
            size = self.scene.get_part_footprint(part_id)
            height = self.scene.get_part_height(part_id)
            if (height is None or not math.isfinite(height) or height <= 0
                    or height > self.config.amr_payload_height_m + 1e-8
                    or size is None or len(size) != 2 or not all(math.isfinite(v) and v > 0 for v in size)
                    or any(size[i] > self.config.amr_payload_footprint_size[i] + 1e-8 for i in range(2))):
                self._material_cell.payload_geometry_issues[part_id] = (
                    f"unmeasured or oversized payload {part_id}: measured={size}, height={height}, "
                    f"permitted={self.config.amr_payload_footprint_size}")
            else:
                self._material_cell.payload_geometry_issues.pop(part_id, None)
                self._material_cell.validated_payloads.add(part_id)
        self._part_locations[part_id] = cell_id
        self._part_types[part_id] = part_type
        self._known_rack_parts.add(part_id)

    def remove_part(self, part_id: str) -> None:
        if self.scene is None:
            return
        self.scene.remove_part(part_id)
        self._part_locations.pop(part_id, None)
        self._part_types.pop(part_id, None)

    # ========================================================================
    # 매 step 업데이트
    # ========================================================================
    def update(self, dt: float, sim_time: float) -> None:
        """SimulationManager가 매 step마다 호출."""
        if self.scene is None:
            return

        # 1) MaterialCell 랙 자동 sync (새 부품 입고 감지)
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

        # A schematic demo never commands the arm or pretends its target is a
        # measured grasp. Verified manipulation requires a backend capability
        # that the currently shipped controllers deliberately do not claim.
        for cell_id, rob in self._robots.items():
            ctrl = rob.get("rmp") or rob.get("ik")
            if ctrl is not None and self.config.motion_execution == "verified":
                try:
                    ctrl.update(dt)
                except Exception:
                    logger.exception("[VIS] controller update failed for %s", cell_id)
                    self._fail_motion(cell_id)
            sparks = rob.get("sparks")
            if sparks is not None:
                welding = self._weld_context.get(cell_id)
                phase = ctrl.get_phase() if ctrl is not None else None
                tcp = ctrl.get_measured_tcp_position() if ctrl is not None else None
                sparks.set_active(bool(welding and not welding["schematic"]
                                       and phase == "weld" and tcp is not None))
                if tcp is not None:
                    sparks.set_tcp_position(tcp)
                sparks.update(dt)

        for cell_id, ctx in list(self._pick_context.items()):
            ctx["elapsed"] += dt
            if ctx["schematic"] and ctx["elapsed"] >= self.config.pick_place_travel_time_sec:
                # Explicit logical relocation, not a independently flying payload
                # or a successful physical gripper/contact acknowledgement.
                if self._material_cell.confirm_pickup(
                        ctx["amr_id"], ctx["part_id"], ctx["transfer_id"]):
                    self.scene.move_part(ctx["part_id"], self._input_buffer_pos(cell_id))
                    logger.info("[VIS] SCHEMATIC placement complete %s part=%s transfer=%s",
                                cell_id, ctx["part_id"], ctx["transfer_id"])
                self._pick_context.pop(cell_id, None)
                self._active_pick_place.pop(cell_id, None)
                self._active_pick_place_amr.pop(cell_id, None)

        for cell_id, ctx in list(self._weld_context.items()):
            ctx["elapsed"] += dt
            target = self._material_cell.cell_registry[cell_id]
            ctrl = self._robots.get(cell_id, {}).get("rmp") or self._robots.get(cell_id, {}).get("ik")
            if ctx["schematic"]:
                done = ctx["elapsed"] >= ctx["travel_time_sec"]
                failed = False
            else:
                done = bool(ctrl and ctrl.motion_succeeded())
                failed = not ctrl or ctrl.get_motion_status() in ("failed", "unverified")
            if done or failed:
                target.confirm_process_motion(ctx["command_id"], ctx["part_id"], success=done)
                self._weld_context.pop(cell_id, None)

        self._update_amr_positions(dt)

    def _fail_motion(self, cell_id: str) -> None:
        ctx = self._weld_context.pop(cell_id, None)
        if ctx and self._material_cell:
            self._material_cell.cell_registry[cell_id].confirm_process_motion(
                ctx["command_id"], ctx["part_id"], success=False)

    def _on_cell_status(self, msg) -> None:
        cell_id = getattr(msg, "cell_id", None)
        state = getattr(msg, "state", None)
        state_str = state.value if hasattr(state, "value") else str(state)
        if (state_str == "PROCESSING" and cell_id not in self._weld_context
                and self._material_cell is not None
                and self.cell_types.get(cell_id) == "welding"):
            self._start_welding_motion(cell_id)

    def _start_welding_motion(self, cell_id: str) -> None:
        target = self._material_cell.cell_registry.get(cell_id) if self._material_cell else None
        if target is None or not self.config.use_real_robot:
            return  # explicitly logical process with no robot motion dependency
        spec = target.get_process_motion_spec()
        if spec is None or spec["motion_complete"]:
            return
        if self.config.motion_execution == "schematic":
            self._weld_context[cell_id] = dict(spec, elapsed=0.0, schematic=True)
            logger.info("[VIS] SCHEMATIC process timer %s part=%s duration=%.3fs (no robot weld)",
                        cell_id, spec["part_id"], spec["travel_time_sec"])
            return
        rob = self._robots.get(cell_id, {})
        ctrl = rob.get("rmp") or rob.get("ik")
        capabilities = ctrl.get_capabilities() if ctrl is not None else {}
        if not capabilities.get("supports_physical_manipulation", False):
            logger.error("[VIS] process blocked: backend=%s has no verified manipulation",
                         capabilities.get("backend", "none"))
            target.confirm_process_motion(spec["command_id"], spec["part_id"], success=False)
            return
        # Future validated adapter: current shipped controllers do not assert
        # physical manipulation capability and cannot enter this path.
        bx, by, bz = self._input_buffer_pos(cell_id)
        start = (bx, by - self.config.weld_path_offset_y, bz + self.config.weld_path_height)
        end = (bx, by + self.config.weld_path_offset_y, bz + self.config.weld_path_height)
        try:
            if rob.get("rmp"):
                ctrl.start_path(start=start, end=end, travel_time_sec=spec["travel_time_sec"],
                                approach_height=0.1)
            else:
                from fdw_sim.visualization.ik_controller import WeldingPath
                ctrl.start_path(WeldingPath(start=start, end=end,
                    travel_time_sec=spec["travel_time_sec"], approach_height=0.1))
            self._weld_context[cell_id] = dict(spec, elapsed=0.0, schematic=False)
            logger.info("[VIS] welding motion backend=%s @ %s", capabilities["backend"], cell_id)
        except Exception:
            logger.exception("[VIS] start_path failed for %s", cell_id)
            target.confirm_process_motion(spec["command_id"], spec["part_id"], success=False)

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
    # Level 2.4: AMR 실 주행 + 화물 부품 추종 + pick-and-place
    # ========================================================================
    def _amr_current_world_pos(self, amr) -> Tuple[float, float, float]:
        """Render the single authoritative pose, including idle and endpoints."""
        return (float(amr.position[0]), float(amr.position[1]),
                getattr(self, "_navigation_floor_height", 0.0))

    def _update_amr_positions(self, dt: float) -> None:
        if self._material_cell is None or self.scene is None:
            return
        for amr in self._material_cell.amrs:
            pos = self._amr_current_world_pos(amr)
            self.scene.move_amr(amr.amr_id, pos, heading=amr.heading)
            if (amr.payload_part_id and amr.payload_part_id not in self._active_pick_place.values()):
                getattr(self.scene, "move_part_world", self.scene.move_part)(amr.payload_part_id,
                                     (pos[0], pos[1], pos[2] + self.config.amr_deck_height))
            cmd_id = amr.last_delivered_command_id
            if cmd_id and self._amr_last_seen_delivery.get(amr.amr_id) != cmd_id:
                self._amr_last_seen_delivery[amr.amr_id] = cmd_id
                if amr.last_delivered_part_id and amr.last_delivered_to:
                    self._on_amr_delivered(amr.last_delivered_part_id,
                        amr.last_delivered_to, pos, amr.amr_id, cmd_id)

    def _on_amr_delivered(self, part_id: str, to_cell: str,
                          amr_pos: Tuple[float, float, float], amr_id: str,
                          transfer_id: str) -> None:
        robot_requested = self.config.use_real_robot and self.cell_types.get(to_cell) == "welding"
        if not robot_requested:
            # Explicit no-robot/non-welding logical handoff. This does not claim
            # that a missing robot physically moved the part.
            if self._material_cell.confirm_pickup(amr_id, part_id, transfer_id):
                self.scene.move_part(part_id, self._input_buffer_pos(to_cell))
                logger.info("[VIS] LOGICAL handoff %s part=%s (no robot manipulation)", to_cell, part_id)
            return
        self._start_pick_and_place(to_cell, part_id, amr_pos, amr_id, transfer_id)

    def _start_pick_and_place(self, cell_id: str, part_id: str,
                              amr_pos: Tuple[float, float, float], amr_id: str,
                              transfer_id: str) -> bool:
        if cell_id in self._pick_context:
            return False
        if self.config.motion_execution != "schematic":
            # Neither commanded TCP nor an arm USD proves grasp/contact/reach.
            # Keep the payload at its actual dock until a validated manipulation
            # backend is implemented; never acknowledge a timer as success.
            rob = self._robots.get(cell_id, {})
            ctrl = rob.get("rmp") or rob.get("ik")
            caps = ctrl.get_capabilities() if ctrl is not None else {}
            key = (part_id, transfer_id)
            if key not in self._blocked_deliveries:
                self._blocked_deliveries.add(key)
                logger.error("[VIS] VERIFIED pickup blocked @ %s part=%s backend=%s: "
                    "no validated grasp/contact adapter; use --motion-execution schematic "
                    "only for a labeled operations demo", cell_id, part_id, caps.get("backend", "none"))
            return False
        self._active_pick_place[cell_id] = part_id
        self._active_pick_place_amr[cell_id] = amr_id
        self._pick_context[cell_id] = dict(part_id=part_id, transfer_id=transfer_id,
                                          amr_id=amr_id, elapsed=0.0, schematic=True)
        logger.info("[VIS] SCHEMATIC transfer wait @ %s part=%s (no arm grasp)", cell_id, part_id)
        return True

    # ========================================================================
    # 디버깅
    # ========================================================================
    def get_part_position(self, part_id: str) -> Optional[Tuple[float, float, float]]:
        if self.scene is None:
            return None
        path = self.scene.get_part_path(part_id)
        return path
