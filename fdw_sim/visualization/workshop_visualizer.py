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

# 부품 타입별 색상
PART_COLORS = {
    "tubular_frame_A": (0.20, 0.60, 1.00),   # 파랑
    "tubular_frame_B": (1.00, 0.60, 0.20),   # 주황
    "default":         (0.50, 0.50, 0.50),
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
        # cell_id -> 그 셀 로봇이 지금 나르고 있는 part_id (carry 단계에서
        # 매 프레임 TCP 위치로 부품을 따라가게 함)
        self._active_pick_place: Dict[str, str] = {}
        # cell_id -> 그 part를 배송한 amr_id — pick-and-place가 끝나면
        # material_cell.confirm_pickup(amr_id)으로 AMR을 재배차 가능하게 푼다
        self._active_pick_place_amr: Dict[str, str] = {}
        # PROCESSING 이벤트가 pick-and-place 도중에 도착해 용접 모션을
        # 바로 시작 못 한 셀들 — pick-and-place 끝나는 대로 시작해준다
        self._pending_weld_after_pick: set = set()

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
        """MaterialCell 참조 — 랙 부품 자동 spawn에 사용.

        시각화가 붙는 순간부터는 AMR이 도착 즉시 재배차되지 않고, 이
        visualizer가 pick-and-place(또는 그 폴백)를 끝내고 confirm_pickup()을
        호출할 때까지 도크에서 대기하도록 MaterialCell에 알린다 — 로봇이
        집어가기도 전에 AMR이 다음 배송으로 가버리던 문제(2026-09-28 DGX
        Spark 실측) 수정."""
        self._material_cell = cell
        cell.require_pickup_confirmation = True

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
        # WORKSHOP_BOUNDS(0~36.1, 0~12.4)는 원점에서 멀리 떨어져 있어
        # 원점 중심 plane으로는 절반이 빈 공간을 덮게 된다 — 건물 중심으로 이동.
        from fdw_sim.visualization.scene_builder import WORKSHOP_BOUNDS
        x_min, x_max, y_min, y_max = WORKSHOP_BOUNDS
        ground_center = ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0, 0.0)
        ground_size = max(x_max - x_min, y_max - y_min) + 8.0
        self.scene.add_ground_plane(size=ground_size, center=ground_center)

        if self.config.show_factory_walls:
            try:
                self.scene.add_factory_walls()
                self.scene.add_roof()
            except Exception:
                logger.exception("[VIS] add_factory_walls/add_roof failed — continuing without them")

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

        # 2) 모션 컨트롤러 + 스파크 emitter + pick-and-place 운반 업데이트
        #    (용접 셀 로봇팔 → 부품 추적 + TCP 위치 기반 스파크 방출 /
        #    AMR에서 집어온 부품을 carry 단계 동안 TCP를 따라가게 함)
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

            phase = None
            if ctrl is not None and hasattr(ctrl, "get_phase"):
                try:
                    phase = str(ctrl.get_phase()).lower()
                except Exception:
                    phase = None

            tcp = None
            if ctrl is not None and hasattr(ctrl, "get_tcp_position"):
                try:
                    tcp = ctrl.get_tcp_position()
                except Exception:
                    tcp = None

            # 스파크: TCP 위치 갱신 + weld 단계일 때만 활성화
            if sparks is not None:
                if tcp is not None:
                    sparks.set_tcp_position(tcp)
                sparks.set_active(phase == "weld")
                try:
                    sparks.update(dt)
                except Exception:
                    logger.exception("[VIS] spark emitter update failed for %s",
                                     cell_id)

            # pick-and-place: carry(=weld phase 재사용) 단계 동안 부품이
            # TCP를 따라가고, 경로가 끝나면 작업대에 내려놓은 뒤 밀려있던
            # 용접 모션이 있으면 바로 시작한다.
            part_id = self._active_pick_place.get(cell_id)
            if part_id is not None:
                if phase == "weld" and tcp is not None:
                    self.scene.move_part(part_id, tcp)
                if ctrl is not None and ctrl.is_idle():
                    self.scene.move_part(part_id, self._input_buffer_pos(cell_id))
                    del self._active_pick_place[cell_id]
                    logger.info("[VIS] pick-and-place finished @ %s: %s placed on bench",
                                cell_id, part_id)
                    amr_id = self._active_pick_place_amr.pop(cell_id, None)
                    if amr_id and self._material_cell is not None:
                        self._material_cell.confirm_pickup(amr_id)
                    if cell_id in self._pending_weld_after_pick:
                        self._pending_weld_after_pick.discard(cell_id)
                        self._start_welding_motion(cell_id)

        # 3) AMR 실주행 + 화물 부품 추종
        self._update_amr_positions(dt)

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

        # 로봇이 AMR에서 부품을 집어 작업대로 옮기는 pick-and-place 도중이면
        # 이 이벤트로 끼어들지 않는다 — 모션이 끝나면 update() 루프가
        # _pending_weld_after_pick을 보고 알아서 용접을 시작/go_home 한다.
        if cell_id in self._active_pick_place:
            if state_str.upper() == "PROCESSING":
                self._pending_weld_after_pick.add(cell_id)
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
    # Level 2.4: AMR 실 주행 + 화물 부품 추종 + pick-and-place
    # ========================================================================
    def _amr_current_world_pos(self, amr) -> Optional[Tuple[float, float, float]]:
        """AMR의 현재 월드 좌표(바닥, z=0) — from/to 정차 지점 사이를
        remaining_distance_m/total_distance_m 진행률로 선형보간."""
        from_dock = self._amr_dock_pos(amr.from_cell) if amr.from_cell else None
        to_dock = self._amr_dock_pos(amr.target_cell) if amr.target_cell else None
        if from_dock is None or to_dock is None:
            return None
        total = max(amr.total_distance_m, 1e-6)
        remaining = max(0.0, min(total, amr.remaining_distance_m))
        progress = max(0.0, min(1.0, 1.0 - remaining / total))
        x = from_dock[0] + (to_dock[0] - from_dock[0]) * progress
        y = from_dock[1] + (to_dock[1] - from_dock[1]) * progress
        return (x, y, 0.0)

    def _update_amr_positions(self, dt: float) -> None:
        """MaterialCell.amrs를 폴링해서 (1) 이동 중인 AMR을 실제로 굴리고
        그 위 화물을 따라가게 하고, (2) 방금 배송 완료된 AMR을 감지해서
        로봇팔 pick-and-place를 트리거한다."""
        if self._material_cell is None or self.scene is None:
            return

        for amr in self._material_cell.amrs:
            # 배송 완료 감지 (edge-detect — MaterialCell.step()이 이 update()
            # 보다 먼저 실행되므로, 이 시점엔 이미 payload/target이 지워져
            # 있다. last_delivered_* 스냅샷으로 판단한다)
            cmd_id = amr.last_delivered_command_id
            if cmd_id and self._amr_last_seen_delivery.get(amr.amr_id) != cmd_id:
                self._amr_last_seen_delivery[amr.amr_id] = cmd_id
                dock = self._amr_dock_pos(amr.last_delivered_to) if amr.last_delivered_to else None
                if amr.last_delivered_part_id and amr.last_delivered_to and dock is not None:
                    self._on_amr_delivered(
                        amr.last_delivered_part_id, amr.last_delivered_to,
                        (dock[0], dock[1], 0.0), amr.amr_id,
                    )

            if not amr.busy:
                continue

            pos = self._amr_current_world_pos(amr)
            if pos is None:
                continue
            self.scene.move_amr(amr.amr_id, pos)

            # 화물이 있고, 아직 로봇이 안 집어간 상태면 AMR 위에 얹혀서 이동
            if amr.payload_part_id and amr.payload_part_id not in self._active_pick_place.values():
                deck_pos = (pos[0], pos[1], pos[2] + self.config.amr_deck_height)
                try:
                    self.scene.move_part(amr.payload_part_id, deck_pos)
                except Exception:
                    logger.exception("[VIS] move_part (AMR cargo) failed for %s",
                                      amr.payload_part_id)

    def _on_amr_delivered(self, part_id: str, to_cell: str,
                          amr_pos: Tuple[float, float, float],
                          amr_id: str) -> None:
        """AMR이 부품을 셀에 배송한 순간 — 그 셀에 로봇팔이 있으면
        pick-and-place(AMR -> 작업대) 모션을 시작한다. 로봇이 이미 다른
        동작 중이라 pick-and-place를 못 시작했을 때만 예전처럼 바로 입력
        버퍼 위치로 스냅한다(로봇이 있는데 안 집어가면 부품이 AMR 위에
        영원히 남아있는 것도 이상하므로).

        그 셀에 로봇팔 자체가 없는 경우(--real-robot 미지정 등, 2026-09-28
        AMR 단독 검증 시나리오)는 스냅하지 않는다 — 넘겨줄 로봇이 없는데
        입력 버퍼로 순간이동시키면 "AMR이 실어나른다"는 걸 확인하려는
        목적과 반대로 부품이 AMR과 무관하게 텔레포트하는 것처럼 보인다.
        이 경우 부품은 _update_amr_positions()가 마지막으로 놓아둔 위치
        (AMR 적재함, 도착 지점)에 그대로 남는다.

        MaterialCell.require_pickup_confirmation이 켜져 있으면(=이
        visualizer가 attach_material_cell로 붙어있으면 항상 켜짐) 이 AMR은
        도착 시점에 이미 도크에서 대기 상태(awaiting_pickup)다 —
        pick-and-place가 실제로 시작된 경우에만 나중에(update() 루프에서
        모션이 끝날 때) confirm_pickup을 호출해 풀어주고, 그 외의 모든
        경우(로봇 없음/경합으로 시작 실패)는 기다릴 대상이 없으므로 여기서
        바로 풀어준다 — 안 그러면 max_pickup_wait_sec 타임아웃까지
        불필요하게 도크를 막아버린다."""
        started = False
        if to_cell in self._robots:
            started = self._start_pick_and_place(to_cell, part_id, amr_pos, amr_id)
        if started:
            return
        if self._material_cell is not None:
            self._material_cell.confirm_pickup(amr_id)
        if to_cell not in self._robots:
            return
        if self.scene is not None:
            try:
                self.scene.move_part(part_id, self._input_buffer_pos(to_cell))
            except Exception:
                logger.exception("[VIS] fallback move_part failed for %s", part_id)

    def _start_pick_and_place(self, cell_id: str, part_id: str,
                              amr_pos: Tuple[float, float, float],
                              amr_id: str) -> bool:
        """AMR 위 부품을 로봇팔로 집어 작업대(입력 버퍼)로 옮기는 모션 시작.

        기존 _start_welding_motion과 동일한 3단계 경로(approach/weld/retreat)
        컨트롤러 메커니즘을 그대로 재사용한다 — approach 단계가 AMR 위로
        내려가는 "집기" 동작, weld 단계가 AMR -> 작업대로 나르는 "운반"
        동작(그래서 update() 루프에서 이 구간에 TCP를 따라 부품을 이동시킨다),
        retreat 단계가 내려놓고 올라오는 "놓기" 동작이 된다.

        로봇이 이미 다른 모션(이전 용접 등) 중이면 시작하지 않고 False를
        반환한다 — 흔치 않은 경합 상황이라 호출자가 즉시 작업대로 스냅하는
        단순 폴백으로 처리한다.
        """
        rob = self._robots.get(cell_id)
        if rob is None:
            return False
        rmp = rob.get("rmp")
        ik = rob.get("ik")
        ctrl = rmp if rmp is not None else ik
        if ctrl is None or not ctrl.is_idle():
            return False

        pick_h = self.config.amr_deck_height
        start = (amr_pos[0], amr_pos[1], amr_pos[2] + pick_h)
        end = self._input_buffer_pos(cell_id)
        travel_time_sec = self.config.pick_place_travel_time_sec
        approach_height = self.config.pick_place_approach_height

        try:
            if rmp is not None:
                # RMPflowController — kwarg 기반 API
                rmp.start_path(start=start, end=end,
                               travel_time_sec=travel_time_sec,
                               approach_height=approach_height)
            else:
                # IKController — WeldingPath 객체를 넘기는 API
                # (_start_welding_motion과 동일한 패턴)
                from fdw_sim.visualization.ik_controller import WeldingPath
                path = WeldingPath(start=start, end=end,
                                   travel_time_sec=travel_time_sec,
                                   approach_height=approach_height)
                ik.start_path(path)
        except Exception:
            logger.exception("[VIS] pick-and-place start_path failed for %s", cell_id)
            return False

        self._active_pick_place[cell_id] = part_id
        self._active_pick_place_amr[cell_id] = amr_id
        logger.info("[VIS] pick-and-place started @ %s: %s (%s -> %s)",
                    cell_id, part_id, start, end)
        return True

    # ========================================================================
    # 디버깅
    # ========================================================================
    def get_part_position(self, part_id: str) -> Optional[Tuple[float, float, float]]:
        if self.scene is None:
            return None
        path = self.scene.get_part_path(part_id)
        return path
