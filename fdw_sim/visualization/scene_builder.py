"""SceneBuilder — Isaac Sim USD 스테이지에 FDW-OPS 셀들을 배치.

이 모듈은 Isaac Sim 5.x의 stage utility를 lazy import 하므로,
mode="discrete"인 경우 절대 import되지 않는다.

사용 예:

    builder = SceneBuilder(SceneConfig())
    builder.add_ground_plane()
    builder.add_cell_workbench("MAT_01",  position=(0, 0, 0),  size=(2, 2, 0.8), color=(0.6, 0.6, 0.7))
    builder.add_cell_workbench("WELD_01", position=(5, 0, 0),  size=(2, 2, 0.8), color=(0.7, 0.4, 0.4))
    builder.add_cell_workbench("INSP_01", position=(10, 0, 0), size=(2, 2, 0.8), color=(0.4, 0.7, 0.4))

    # 부품 표현
    cube_path = builder.add_part("PART_A_001", position=(0, 0, 1.0), color=(0.2, 0.6, 1.0))
    builder.move_part("PART_A_001", target=(5, 0, 1.0))   # 다음 frame에서 이동
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple
import logging

logger = logging.getLogger(__name__)

# ============================================================================
# FDW 유연생산 작업장 배치도 - 1안 좌표계
# ----------------------------------------------------------------------------
# 도면 기준(원점 = 건물 좌하단, 단위 m): 전체 36.1 x 12.4.
# name: (x0, y0, w, d, color, label) — x0,y0은 구역 좌하단, w/d는 폭/깊이.
# ============================================================================
WORKSHOP_BOUNDS: Tuple[float, float, float, float] = (0.0, 36.1, 0.0, 12.4)

WORKSHOP_ZONES: Dict[str, Tuple[float, float, float, float,
                                 Tuple[float, float, float], str]] = {
    # 6세부 소재관리 존 — 좌측 전체 (서버실/AMR충전 포함하는 큰 사각형이므로
    # 겹치는 하위 구역보다 z를 낮게 그린다, z-fighting 방지)
    "material_mgmt":  (0.0, 0.0, 20.6, 12.4, (0.55, 0.62, 0.70), "6-material-mgmt"),
    "server_room":    (0.0, 9.4, 3.3, 3.0, (0.35, 0.45, 0.60), "1-server-room"),
    "amr_charge":     (3.3, 9.4, 7.6, 3.0, (0.45, 0.65, 0.50), "amr-charge"),
    "amr_aisle":      (20.6, 0.0, 3.0, 12.4, (0.85, 0.85, 0.80), "amr-main-aisle"),
    "welding_cell":   (23.6, 9.1, 3.0, 3.3, (0.80, 0.45, 0.30), "2-welding"),
    "additive_cell":  (28.6, 9.9, 2.5, 2.5, (0.35, 0.60, 0.55), "4-additive"),
    "machining_cell": (33.1, 8.4, 3.0, 4.0, (0.50, 0.50, 0.60), "5-machining"),
    "forming_cell":   (25.1, 0.0, 11.0, 4.5, (0.75, 0.65, 0.35), "3-forming"),
}
# material_mgmt 위에 얹히는 하위 구역들 — 같은 z에 그리면 깜빡임(z-fighting).
_WORKSHOP_ZONE_NESTED = {"server_room", "amr_charge"}

# 3세부 소성가공 셀 안전펜스 반입구 폭(도면 명기)
FORMING_GATE_WIDTH_M = 3.8
FORMING_FENCE_HEIGHT_M = 1.47


@dataclass
class SceneConfig:
    """SceneBuilder 동작 옵션."""
    root_prim_path: str = "/World/FDW"
    ground_color: Tuple[float, float, float] = (0.25, 0.25, 0.28)
    add_default_lighting: bool = True
    cell_label_above: bool = True


class SceneBuilder:
    """Isaac Sim 스테이지에 FDW 셀/부품을 배치하는 헬퍼.

    중요: 이 클래스의 모든 메서드는 SimulationApp이 이미 인스턴스화된 후에
    호출되어야 한다 (Carbonite 요구사항).
    """

    def __init__(self, config: Optional[SceneConfig] = None) -> None:
        self.config = config or SceneConfig()

        # lazy import: Isaac Sim이 시작된 뒤에만 가능
        from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf  # type: ignore
        import omni.usd  # type: ignore

        self._Usd = Usd
        self._UsdGeom = UsdGeom
        self._UsdLux = UsdLux
        self._Gf = Gf
        self._Sdf = Sdf
        self._omni_usd = omni.usd

        self._stage = omni.usd.get_context().get_stage()
        if self._stage is None:
            raise RuntimeError("USD stage is None. Did you start SimulationApp?")

        # 루트 Xform 생성
        UsdGeom.Xform.Define(self._stage, self.config.root_prim_path)

        # prim 추적
        self._cell_prims: Dict[str, str] = {}     # cell_id -> prim path
        self._part_prims: Dict[str, str] = {}     # part_id -> prim path
        self._amr_prims: Dict[str, str] = {}      # amr_id -> prim path

        if self.config.add_default_lighting:
            self._add_default_lighting()

        logger.info("[VIS] SceneBuilder initialized at %s", self.config.root_prim_path)

    # ========================================================================
    # 환경
    # ========================================================================
    def add_ground_plane(self, size: float = 50.0,
                         center: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> str:
        """간단한 그라운드 플레인 (Isaac Lab의 기본 ground와는 별개).

        Args:
            center: 플레인 중심 world 좌표. 기본은 원점이지만, 셀 배치가
                원점에서 먼 곳(예: 이 프로젝트의 WORKSHOP_BOUNDS처럼
                x:[0,36.1])에 몰려 있으면 plane 절반이 빈 공간을 덮게
                되므로 건물 중심으로 옮겨줘야 한다.
        """
        path = f"{self.config.root_prim_path}/Ground"
        plane = self._UsdGeom.Plane.Define(self._stage, path)
        plane.CreateAxisAttr("Z")
        plane.CreateLengthAttr(size)
        plane.CreateWidthAttr(size)
        if center != (0.0, 0.0, 0.0):
            self._set_translate(path, center)
        self._set_color(path, self.config.ground_color)
        return path

    def add_factory_walls(self,
                          bounds: Tuple[float, float, float, float] = WORKSHOP_BOUNDS,
                          height: float = 4.5,
                          thickness: float = 0.2,
                          color: Tuple[float, float, float] = (0.55, 0.58, 0.62)) -> str:
        """작업장을 감싸는 4면 벽 — 배경 디테일용 (충돌/물리 없이 시각 전용).

        Args:
            bounds: (x_min, x_max, y_min, y_max) — 벽으로 둘러쌀 바닥 영역.
                기본값은 이 프로젝트의 셀 배치(material@x=0, welding@x=5,
                inspection@x=10, AMR은 x≈-1.5)를 여유 있게 감싸도록 잡았다.
            height: 벽 높이(m)
            thickness: 벽 두께(m)
        """
        x_min, x_max, y_min, y_max = bounds
        walls_root = f"{self.config.root_prim_path}/Environment/Walls"
        self._UsdGeom.Xform.Define(self._stage, walls_root)

        width_x = x_max - x_min
        width_y = y_max - y_min
        cx = (x_min + x_max) / 2.0
        cy = (y_min + y_max) / 2.0
        hz = height / 2.0

        # (이름, 중심, half-extent) — North/South는 X축을 따라 긴 벽, East/West는 Y축
        specs = [
            ("North", (cx, y_max, hz), (width_x / 2.0 + thickness, thickness, hz)),
            ("South", (cx, y_min, hz), (width_x / 2.0 + thickness, thickness, hz)),
            ("East", (x_max, cy, hz), (thickness, width_y / 2.0 + thickness, hz)),
            ("West", (x_min, cy, hz), (thickness, width_y / 2.0 + thickness, hz)),
        ]
        for name, center, half_extent in specs:
            wall_path = f"{walls_root}/{name}"
            wall = self._UsdGeom.Cube.Define(self._stage, wall_path)
            wall.CreateSizeAttr(1.0)
            self._set_scale(wall_path, half_extent)
            self._set_translate(wall_path, center)
            self._set_color(wall_path, color)

        logger.info("[VIS] factory walls added (bounds=%s, height=%.1fm)",
                    bounds, height)
        return walls_root

    def add_ceiling_lights(self,
                           positions: Optional[list] = None,
                           intensity: float = 8000.0) -> str:
        """작업 구역 위에 사각 천장 조명(RectLight) + 피팅(박스)을 배치.

        RectLight는 로컬 -Z 방향으로 발광하므로(USD 기본값), 회전 없이
        그대로 두면 천장에서 바닥 쪽(world -Z)을 자연스럽게 비춘다.

        Args:
            positions: 조명을 둘 (x, y, z) 목록. None이면 작업장 배치도(1안)의
                주요 구역 중심 위쪽에 기본 5개를 놓는다.
        """
        if positions is None:
            positions = [
                (10.3, 6.2, 4.0),   # 6세부 소재관리 존
                (25.1, 10.75, 4.0),  # 2세부 용접 셀
                (29.85, 11.15, 4.0),  # 4세부 적층 셀
                (34.6, 10.4, 4.0),   # 5세부 정밀가공 셀
                (30.6, 2.25, 4.0),   # 3세부 소성가공 셀
            ]

        lights_root = f"{self.config.root_prim_path}/Environment/CeilingLights"
        self._UsdGeom.Xform.Define(self._stage, lights_root)

        for i, pos in enumerate(positions):
            # 조명 피팅(박스) — 발광체 자체는 안 보여도 시각적 앵커 역할
            fixture_path = f"{lights_root}/Fixture_{i:02d}"
            fixture = self._UsdGeom.Cube.Define(self._stage, fixture_path)
            fixture.CreateSizeAttr(1.0)
            self._set_scale(fixture_path, (0.6, 0.15, 0.05))
            self._set_translate(fixture_path, pos)
            self._set_color(fixture_path, (0.85, 0.85, 0.8))

            # 실제 발광 — RectLight (피팅보다 살짝 아래, 천장 쪽에서 아래를 향해 발광)
            light_path = f"{lights_root}/Fixture_{i:02d}_Light"
            light = self._UsdLux.RectLight.Define(self._stage, light_path)
            light.CreateWidthAttr(1.1)
            light.CreateHeightAttr(0.25)
            light.CreateIntensityAttr(intensity)
            light.CreateColorAttr(self._Gf.Vec3f(1.0, 0.97, 0.9))
            self._set_translate(light_path, (pos[0], pos[1], pos[2] - 0.06))

        logger.info("[VIS] %d ceiling light fixtures added", len(positions))
        return lights_root

    # ========================================================================
    # FDW 유연생산 작업장 배치도 - 1안 전체 레이아웃
    # ========================================================================
    def add_zone_floor(self, name: str,
                       x0: float, y0: float, w: float, d: float,
                       color: Tuple[float, float, float],
                       z: float = 0.011) -> str:
        """구역 바닥 표시(색상 타일). z를 살짝 높여 겹치는 상위 구역과의
        z-fighting을 피할 수 있다 (예: material_mgmt 위에 얹힌
        server_room/amr_charge)."""
        path = f"{self.config.root_prim_path}/Zones/{name}"
        cube = self._UsdGeom.Cube.Define(self._stage, path)
        cube.CreateSizeAttr(1.0)
        self._set_scale(path, (w / 2.0, d / 2.0, 0.01))
        self._set_translate(path, (x0 + w / 2.0, y0 + d / 2.0, z))
        self._set_color(path, color)
        return path

    def add_safety_fence(self, root_name: str,
                         x0: float, y0: float, w: float, d: float,
                         gate_w: float = FORMING_GATE_WIDTH_M,
                         height: float = FORMING_FENCE_HEIGHT_M,
                         color: Tuple[float, float, float] = (0.95, 0.8, 0.1)
                         ) -> str:
        """구역 둘레를 감싸는 안전펜스 — 남쪽(y0) 벽 중앙에 폭 gate_w
        반입구만 비운다 (3세부 소성가공 셀 도면 기준)."""
        t = 0.05
        h = height
        fence_root = f"{self.config.root_prim_path}/Fences/{root_name}"
        self._UsdGeom.Xform.Define(self._stage, fence_root)

        half_gap = (w - gate_w) / 2.0
        segments = [
            ("N", (x0 + w / 2.0, y0 + d - t / 2.0, h / 2.0), (w / 2.0 + t, t, h / 2.0)),
            ("W", (x0 + t / 2.0, y0 + d / 2.0, h / 2.0), (t, d / 2.0, h / 2.0)),
            ("E", (x0 + w - t / 2.0, y0 + d / 2.0, h / 2.0), (t, d / 2.0, h / 2.0)),
            ("S_L", (x0 + half_gap / 2.0, y0 + t / 2.0, h / 2.0), (half_gap / 2.0, t, h / 2.0)),
            ("S_R", (x0 + w - half_gap / 2.0, y0 + t / 2.0, h / 2.0), (half_gap / 2.0, t, h / 2.0)),
        ]
        for seg_name, center, half_extent in segments:
            if half_extent[0] <= 0 or half_extent[1] <= 0:
                continue
            seg_path = f"{fence_root}/{seg_name}"
            seg = self._UsdGeom.Cube.Define(self._stage, seg_path)
            seg.CreateSizeAttr(1.0)
            self._set_scale(seg_path, half_extent)
            self._set_translate(seg_path, center)
            self._set_color(seg_path, color)

        logger.info("[VIS] safety fence '%s' added (gate=%.1fm)", root_name, gate_w)
        return fence_root

    def add_unimplemented_cell_marker(self, zone_name: str,
                                       x0: float, y0: float, w: float, d: float,
                                       label: str,
                                       color: Tuple[float, float, float] = (0.5, 0.5, 0.5),
                                       ) -> str:
        """아직 셀 로직(DistributedIntelligenceCell)이 구현되지 않은 구역용
        시각 placeholder — 반투명하지 않은 무채색 상자 + 경고색 테두리
        스트립으로 "미구현" 임을 눈에 띄게 표시한다 (적층/정밀가공/소성가공)."""
        root = f"{self.config.root_prim_path}/UnimplementedCells/{zone_name}"
        self._UsdGeom.Xform.Define(self._stage, root)
        cx, cy = x0 + w / 2.0, y0 + d / 2.0
        h = 1.2

        body_path = f"{root}/Placeholder"
        body = self._UsdGeom.Cube.Define(self._stage, body_path)
        body.CreateSizeAttr(1.0)
        self._set_scale(body_path, (w * 0.35, d * 0.35, h / 2.0))
        self._set_translate(body_path, (cx, cy, h / 2.0))
        self._set_color(body_path, color)

        # 경고색 스트라이프 상단 테두리 — "이 셀은 아직 시뮬레이션 로직 없음"
        stripe_path = f"{root}/WarningStripe"
        stripe = self._UsdGeom.Cube.Define(self._stage, stripe_path)
        stripe.CreateSizeAttr(1.0)
        self._set_scale(stripe_path, (w * 0.35, d * 0.35, 0.03))
        self._set_translate(stripe_path, (cx, cy, h + 0.03))
        self._set_color(stripe_path, (0.95, 0.75, 0.1))

        logger.info("[VIS] unimplemented-cell placeholder '%s' (%s) @ (%.1f, %.1f)",
                    zone_name, label, cx, cy)
        return root

    def add_workshop_layout(self) -> None:
        """FDW 유연생산 작업장 배치도 - 1안을 그대로 반영한 정적 배경 요소
        일괄 생성: 구역 바닥 색상 타일, 소재관리 존 보관 랙 + AMR 충전 도크,
        서버실 가벽, 소성가공 셀 안전펜스, 그리고 아직 셀 로직이 없는
        적층/정밀가공/소성가공 구역의 placeholder 마커.

        실제 시뮬레이션 셀(material/welding/inspection)은 이 메서드가 그리지
        않는다 — 그건 WorkshopVisualizer.build_scene()이 poc1.yaml의
        cells.*.location 좌표로 add_cell_workbench()를 호출해서 그린다.
        이 좌표들은 이 배치도의 material_mgmt/welding_cell 구역 중심과
        일치하도록 poc1.yaml에서 맞춰뒀다.
        """
        for name, (x0, y0, w, d, color, _label) in WORKSHOP_ZONES.items():
            z = 0.013 if name in _WORKSHOP_ZONE_NESTED else 0.011
            self.add_zone_floor(name, x0, y0, w, d, color, z=z)

        # 소재관리 존 — 보관 랙 3열
        for i, (x0, y0, w, d, h) in enumerate([
            (4.0, 3.0, 8.0, 1.0, 2.2),
            (4.0, 5.0, 8.0, 1.0, 2.2),
            (4.0, 7.0, 8.0, 1.0, 2.2),
        ]):
            rack_path = f"{self.config.root_prim_path}/Layout/MaterialRacks/rack_{i}"
            rack = self._UsdGeom.Cube.Define(self._stage, rack_path)
            rack.CreateSizeAttr(1.0)
            self._set_scale(rack_path, (w / 2.0, d / 2.0, h / 2.0))
            self._set_translate(rack_path, (x0 + w / 2.0, y0 + d / 2.0, h / 2.0))
            self._set_color(rack_path, (0.25, 0.45, 0.65))

        # AMR 충전 도크 표시 (충전소 구역 안, 바닥 마커)
        for i, (x, y) in enumerate([(4.5, 10.9), (6.5, 10.9)]):
            dock_path = f"{self.config.root_prim_path}/Layout/ChargeDocks/dock_{i}"
            dock = self._UsdGeom.Cube.Define(self._stage, dock_path)
            dock.CreateSizeAttr(1.0)
            self._set_scale(dock_path, (0.6, 0.45, 0.025))
            self._set_translate(dock_path, (x, y, 0.02))
            self._set_color(dock_path, (0.3, 0.7, 0.3))

        # 서버실 가벽 (동쪽 + 남쪽 — 복도 쪽으로 열려 있는 나머지 2면은 건물 외벽이 대신함)
        sx0, sy0, sw, sd, _c, _l = WORKSHOP_ZONES["server_room"]
        t, h = 0.1, 3.0
        wall_e = f"{self.config.root_prim_path}/Layout/ServerRoom/wall_e"
        we = self._UsdGeom.Cube.Define(self._stage, wall_e)
        we.CreateSizeAttr(1.0)
        self._set_scale(wall_e, (t / 2.0, sd / 2.0, h / 2.0))
        self._set_translate(wall_e, (sx0 + sw - t / 2.0, sy0 + sd / 2.0, h / 2.0))
        self._set_color(wall_e, (0.7, 0.7, 0.72))

        wall_s = f"{self.config.root_prim_path}/Layout/ServerRoom/wall_s"
        ws = self._UsdGeom.Cube.Define(self._stage, wall_s)
        ws.CreateSizeAttr(1.0)
        self._set_scale(wall_s, (sw / 2.0, t / 2.0, h / 2.0))
        self._set_translate(wall_s, (sx0 + sw / 2.0, sy0 + t / 2.0, h / 2.0))
        self._set_color(wall_s, (0.7, 0.7, 0.72))

        # 3세부 소성가공 셀 — 안전펜스 (반입구 3.8m)
        fx0, fy0, fw, fd, _c, _l = WORKSHOP_ZONES["forming_cell"]
        self.add_safety_fence("forming_cell", fx0, fy0, fw, fd)

        # 아직 셀 로직이 없는 구역 — 눈에 띄는 placeholder만 배치
        for zone_name in ("additive_cell", "machining_cell", "forming_cell"):
            zx0, zy0, zw, zd, _c, label = WORKSHOP_ZONES[zone_name]
            self.add_unimplemented_cell_marker(zone_name, zx0, zy0, zw, zd, label)

        logger.info("[VIS] FDW workshop layout (1안) applied — %d zones",
                    len(WORKSHOP_ZONES))

    def _add_default_lighting(self) -> None:
        light_path = f"{self.config.root_prim_path}/Lighting/Distant"
        light = self._UsdLux.DistantLight.Define(self._stage, light_path)
        light.CreateIntensityAttr(3000.0)
        light.CreateAngleAttr(0.5)

        dome_path = f"{self.config.root_prim_path}/Lighting/Dome"
        dome = self._UsdLux.DomeLight.Define(self._stage, dome_path)
        dome.CreateIntensityAttr(500.0)

    # ========================================================================
    # 카메라 framing — viewport를 셀 전체가 보이는 위치로 자동 이동
    # ========================================================================
    def frame_viewport_to_scene(self,
                                  center: Tuple[float, float, float] = (5.0, 0.0, 1.0),
                                  distance: float = 12.0,
                                  height: float = 6.0,
                                  set_viewport_camera: bool = True) -> None:
        """Isaac Sim viewport의 perspective 카메라를 셀 전체가 보이도록 이동.

        viewport에서 직접 perspective 카메라를 조작하기 어려우므로,
        새 카메라 prim을 만들어 viewport active camera로 설정한다.

        Args:
            set_viewport_camera: False면 카메라 prim은 만들되 viewport active camera
                는 바꾸지 않음 (omni.kit.viewport.utility 호출이 AGX Thor에서
                SetLightingMenuModeCommand를 트리거하고 GPU device-lost를
                유발하는 경우의 회피 모드).
        """
        # 1) 카메라 prim 생성 (USD 레벨만 — GPU 자원 업로드 없음)
        cam_path = f"{self.config.root_prim_path}/SceneCamera"
        try:
            UsdGeom = self._UsdGeom
            Gf = self._Gf

            cam = UsdGeom.Camera.Define(self._stage, cam_path)
            cam.CreateFocalLengthAttr(24.0)
            cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 10000.0))

            cx, cy, cz = center
            # 대각선 위에서 셀을 내려다보는 시점
            cam_pos = (cx - distance * 0.6, cy - distance * 0.7, cz + height)
            target = (cx, cy, cz)

            # look-at 행렬 계산 — forward/right/up 기저벡터를 직접 구해 4x4
            # transform 하나로 설정한다 (RotateZ+RotateX 오일러각을 손으로
            # 조합하던 이전 버전은 합성 순서/부호 실수가 나기 쉬웠고, 실측
            # (Thor RDP GUI)에서 카메라가 씬을 완전히 빗나가는 것으로 확인됨 —
            # 그라운드 플레인을 거의 옆에서 보는 각도로 렌더링됨).
            import math

            def _sub(a, b):
                return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

            def _cross(a, b):
                return (a[1] * b[2] - a[2] * b[1],
                        a[2] * b[0] - a[0] * b[2],
                        a[0] * b[1] - a[1] * b[0])

            def _normalize(v, fallback=(0.0, 0.0, 1.0)):
                length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
                if length < 1e-9:
                    return fallback
                return (v[0] / length, v[1] / length, v[2] / length)

            # 이 프로젝트의 모든 위치는 Z를 높이로 취급하므로(Isaac Sim
            # 기본 stage도 Z-up) world up은 항상 (0,0,1).
            world_up = (0.0, 0.0, 1.0)
            forward = _normalize(_sub(target, cam_pos))
            right_raw = _cross(forward, world_up)
            right_len = math.sqrt(sum(c * c for c in right_raw))
            if right_len < 1e-6:
                # forward가 world_up과 거의 평행(카메라가 거의 수직으로
                # 내려다보는 경우) — 임의의 right 축으로 폴백.
                right = (1.0, 0.0, 0.0)
            else:
                right = tuple(c / right_len for c in right_raw)
            up = _cross(right, forward)  # right, forward 모두 단위벡터라 이미 정규화됨

            # USD 카메라는 로컬 -Z 방향을 바라보고 +Y가 up, +X가 right이다.
            # Gf.Matrix4d는 row-vector 규약(v' = v * M)이라 회전 성분은
            # 행(row)에 기저벡터를, 4번째 행에 translation을 넣는다.
            rx, ry, rz = right
            ux, uy, uz = up
            fx, fy, fz = forward
            ex, ey, ez = cam_pos
            look_at_matrix = Gf.Matrix4d(
                rx, ry, rz, 0.0,
                ux, uy, uz, 0.0,
                -fx, -fy, -fz, 0.0,
                ex, ey, ez, 1.0,
            )

            xformable = UsdGeom.Xformable(cam.GetPrim())
            xformable.ClearXformOpOrder()
            xformable.AddTransformOp().Set(look_at_matrix)

            logger.info("[VIS] scene camera prim created @ %s looking at %s",
                        cam_pos, target)
        except Exception:
            logger.exception("[VIS] scene camera prim creation failed — skipping")
            return

        # 2) viewport active camera 설정 — 분리된 try/except
        # 이 부분이 AGX Thor에서 SetLightingMenuModeCommand를 트리거하므로
        # set_viewport_camera=False로 disable 가능
        if not set_viewport_camera:
            logger.warning("[VIS] viewport active-camera switch SKIPPED "
                           "(set_viewport_camera=False — GPU device-lost 회피)")
            return

        try:
            import omni.kit.viewport.utility as vp_util  # type: ignore
            vp = vp_util.get_active_viewport()
            if vp is not None:
                vp.set_active_camera(cam_path)
                logger.info("[VIS] viewport camera set to %s", cam_path)
        except Exception as e:
            # viewport util 자체가 lighting mode를 건드릴 수 있으니
            # 실패하더라도 카메라 prim은 이미 만들어졌으니 안전하게 계속
            logger.warning("[VIS] could not set viewport camera (continuing): %s", e)

    # ========================================================================
    # 셀 (work bench)
    # ========================================================================
    def add_cell_workbench(self, cell_id: str,
                           position: Tuple[float, float, float],
                           size: Tuple[float, float, float] = (2.0, 2.0, 0.8),
                           color: Tuple[float, float, float] = (0.6, 0.6, 0.6)) -> str:
        """셀의 작업대(직육면체)를 생성."""
        cell_root = f"{self.config.root_prim_path}/Cells/{cell_id}"
        self._UsdGeom.Xform.Define(self._stage, cell_root)
        self._set_translate(cell_root, position)

        # workbench
        bench_path = f"{cell_root}/Workbench"
        bench = self._UsdGeom.Cube.Define(self._stage, bench_path)
        bench.CreateSizeAttr(1.0)
        # scale + half-height translate
        sx, sy, sz = size
        self._set_scale(bench_path, (sx / 2.0, sy / 2.0, sz / 2.0))
        self._set_translate(bench_path, (0.0, 0.0, sz / 2.0))
        self._set_color(bench_path, color)

        # 입력 버퍼 표시 (앞쪽)
        in_path = f"{cell_root}/InputBuffer"
        in_buf = self._UsdGeom.Cube.Define(self._stage, in_path)
        in_buf.CreateSizeAttr(1.0)
        self._set_scale(in_path, (0.3, 0.3, 0.05))
        self._set_translate(in_path, (-sx * 0.35, 0.0, sz + 0.05))
        self._set_color(in_path, (0.2, 0.5, 0.9))   # 파랑

        # 출력 버퍼 표시 (뒤쪽)
        out_path = f"{cell_root}/OutputBuffer"
        out_buf = self._UsdGeom.Cube.Define(self._stage, out_path)
        out_buf.CreateSizeAttr(1.0)
        self._set_scale(out_path, (0.3, 0.3, 0.05))
        self._set_translate(out_path, (sx * 0.35, 0.0, sz + 0.05))
        self._set_color(out_path, (0.9, 0.5, 0.2))  # 주황

        self._cell_prims[cell_id] = cell_root
        logger.info("[VIS] cell %s placed @ %s", cell_id, position)
        return cell_root

    # ========================================================================
    # 셀별 시각요소
    # ========================================================================
    def add_smart_rack(self, cell_id: str,
                       capacity: int = 8,
                       slot_size: float = 0.2) -> str:
        """Material Cell용 Smart Rack — 격자 형태의 부품 슬롯."""
        cell_root = self._cell_prims.get(cell_id)
        if cell_root is None:
            raise ValueError(f"cell {cell_id} not registered; call add_cell_workbench first")

        rack_path = f"{cell_root}/SmartRack"
        self._UsdGeom.Xform.Define(self._stage, rack_path)
        self._set_translate(rack_path, (0.0, 0.8, 0.85))

        # 4 columns × N rows
        cols = 4
        rows = max(1, capacity // cols + (1 if capacity % cols else 0))
        for i in range(capacity):
            r = i // cols
            c = i % cols
            slot = f"{rack_path}/Slot_{i:02d}"
            cube = self._UsdGeom.Cube.Define(self._stage, slot)
            cube.CreateSizeAttr(1.0)
            self._set_scale(slot, (slot_size / 2, slot_size / 2, slot_size / 4))
            self._set_translate(slot, (
                (c - cols / 2 + 0.5) * (slot_size + 0.05),
                0.0,
                r * (slot_size + 0.05),
            ))
            self._set_color(slot, (0.4, 0.4, 0.45))

        logger.info("[VIS] smart rack added @ %s (capacity=%d)", cell_id, capacity)
        return rack_path

    def add_amr(self, amr_id: str,
                position: Tuple[float, float, float] = (0.0, 0.0, 0.05),
                color: Tuple[float, float, float] = (0.95, 0.85, 0.1)) -> str:
        """간이 AMR — 노란 박스로 표현 (실제 Carter/Jetbot USD는 Level 2.5+)."""
        amr_path = f"{self.config.root_prim_path}/AMRs/{amr_id}"
        self._UsdGeom.Xform.Define(self._stage, amr_path)
        self._set_translate(amr_path, position)

        body = f"{amr_path}/Body"
        cube = self._UsdGeom.Cube.Define(self._stage, body)
        cube.CreateSizeAttr(1.0)
        self._set_scale(body, (0.4, 0.3, 0.1))   # 0.8 x 0.6 x 0.2 m
        self._set_translate(body, (0.0, 0.0, 0.1))
        self._set_color(body, color)

        # 4 wheels (시각용)
        for i, (dx, dy) in enumerate([(-0.3, -0.2), (0.3, -0.2), (-0.3, 0.2), (0.3, 0.2)]):
            wheel = f"{amr_path}/Wheel_{i}"
            cyl = self._UsdGeom.Cylinder.Define(self._stage, wheel)
            cyl.CreateRadiusAttr(0.05)
            cyl.CreateHeightAttr(0.05)
            cyl.CreateAxisAttr("X")
            self._set_translate(wheel, (dx, dy, 0.05))
            self._set_color(wheel, (0.1, 0.1, 0.1))

        self._amr_prims[amr_id] = amr_path
        logger.info("[VIS] AMR %s placed @ %s", amr_id, position)
        return amr_path

    def add_real_robot_arm(self, cell_id: str,
                            robot_name: str = "franka_panda",
                            offset: Tuple[float, float, float] = (0.0, -0.3, 0.85),
                            ) -> Optional[object]:
        """Isaac Sim 기본 제공 USD 로봇팔(Franka Panda 등)을 셀 위에 로드.

        Args:
            cell_id: 이미 add_cell_workbench로 등록된 셀
            robot_name: ROBOT_CATALOG의 키 (franka_panda, ur10 등)
            offset: 셀 작업대 기준 로봇 베이스 위치

        Returns:
            Articulation 핸들 (SingleArticulation) 또는 None
        """
        cell_root = self._cell_prims.get(cell_id)
        if cell_root is None:
            raise ValueError(f"cell {cell_id} not registered")

        # 셀의 월드 좌표 + offset
        from fdw_sim.visualization.robot_loader import (
            RobotLoader, ROBOT_CATALOG, resolve_robot_usd_path,
        )

        cell_prim = self._stage.GetPrimAtPath(cell_root)
        xf_cache = self._UsdGeom.XformCache()
        world_xform = xf_cache.GetLocalToWorldTransform(cell_prim)
        cell_pos = world_xform.ExtractTranslation()

        world_pos = (
            cell_pos[0] + offset[0],
            cell_pos[1] + offset[1],
            cell_pos[2] + offset[2],
        )

        # 진단 로그 — USD 경로/위치를 명확히 출력
        spec = ROBOT_CATALOG.get(robot_name)
        if spec is not None:
            usd_path = resolve_robot_usd_path(spec)
            is_remote = (usd_path.startswith("omniverse://")
                         or usd_path.startswith("http://")
                         or usd_path.startswith("https://"))
            logger.info("[VIS] >>> attempting to load real robot %s for cell %s",
                        robot_name, cell_id)
            logger.info("[VIS]     USD path: %s", usd_path)
            logger.info("[VIS]     source   : %s",
                        "REMOTE (Nucleus/S3)" if is_remote else "LOCAL disk")
            logger.info("[VIS]     base position (world): %s", world_pos)
            if is_remote:
                logger.warning("[VIS]     >> remote USD — AGX Thor에서 fetch 실패 가능. "
                               "권장: scripts/download_franka_usd.sh 실행 후 "
                               "ISAAC_NUCLEUS_DIR_LOCAL=~/isaac_assets 설정")
        else:
            logger.error("[VIS] unknown robot_name: %s", robot_name)
            return None

        loader = RobotLoader()
        prim_path = f"{cell_root}/RobotArm"
        articulation = loader.load_robot(
            robot_name=robot_name,
            prim_path=prim_path,
            position=world_pos,
        )

        # 로드 결과 진단 — prim이 실제로 생성됐고 자식 prim이 있는지 확인
        prim = self._stage.GetPrimAtPath(prim_path)
        child_count = 0
        if prim and prim.IsValid():
            child_count = len(list(prim.GetChildren()))
        if child_count == 0:
            logger.warning("[VIS] *** robot prim %s has 0 children — "
                           "USD likely failed to resolve (check Nucleus connection / "
                           "ISAAC_NUCLEUS_DIR / path: %s)",
                           prim_path, usd_path)
        else:
            logger.info("[VIS] <<< robot %s loaded successfully "
                        "(prim=%s, children=%d, articulation=%s)",
                        robot_name, prim_path, child_count,
                        articulation is not None)

        return articulation

    def add_robot_arm_placeholder(self, cell_id: str,
                                   color: Tuple[float, float, float] = (1.0, 0.5, 0.0)) -> str:
        """간이 로봇팔 — 3 segment articulated 박스 (USD 모델 미존재 시 placeholder)."""
        cell_root = self._cell_prims.get(cell_id)
        if cell_root is None:
            raise ValueError(f"cell {cell_id} not registered")

        arm_root = f"{cell_root}/RobotArm"
        self._UsdGeom.Xform.Define(self._stage, arm_root)
        self._set_translate(arm_root, (0.0, -0.3, 0.85))

        # Base
        base = f"{arm_root}/Base"
        cube = self._UsdGeom.Cube.Define(self._stage, base)
        cube.CreateSizeAttr(1.0)
        self._set_scale(base, (0.2, 0.2, 0.1))
        self._set_translate(base, (0.0, 0.0, 0.1))
        self._set_color(base, (0.3, 0.3, 0.35))

        # Link 1 (수직)
        l1 = f"{arm_root}/Link1"
        c1 = self._UsdGeom.Cylinder.Define(self._stage, l1)
        c1.CreateRadiusAttr(0.06)
        c1.CreateHeightAttr(0.4)
        c1.CreateAxisAttr("Z")
        self._set_translate(l1, (0.0, 0.0, 0.4))
        self._set_color(l1, color)

        # Link 2 (전방 사선)
        l2 = f"{arm_root}/Link2"
        c2 = self._UsdGeom.Cylinder.Define(self._stage, l2)
        c2.CreateRadiusAttr(0.05)
        c2.CreateHeightAttr(0.4)
        c2.CreateAxisAttr("Y")
        self._set_translate(l2, (0.0, 0.2, 0.65))
        self._set_color(l2, color)

        # Tool (welding torch)
        tool = f"{arm_root}/Tool"
        ct = self._UsdGeom.Cone.Define(self._stage, tool)
        ct.CreateRadiusAttr(0.04)
        ct.CreateHeightAttr(0.15)
        self._set_translate(tool, (0.0, 0.4, 0.55))
        self._set_color(tool, (0.9, 0.9, 0.2))

        logger.info("[VIS] robot-arm placeholder added @ %s", cell_id)
        return arm_root

    def add_camera_placeholder(self, cell_id: str,
                                color: Tuple[float, float, float] = (0.1, 0.3, 0.6)) -> str:
        """Inspection Cell용 카메라 placeholder."""
        cell_root = self._cell_prims.get(cell_id)
        if cell_root is None:
            raise ValueError(f"cell {cell_id} not registered")

        cam_root = f"{cell_root}/Camera"
        self._UsdGeom.Xform.Define(self._stage, cam_root)
        self._set_translate(cam_root, (0.0, -0.4, 1.4))

        body = f"{cam_root}/Body"
        cube = self._UsdGeom.Cube.Define(self._stage, body)
        cube.CreateSizeAttr(1.0)
        self._set_scale(body, (0.08, 0.05, 0.05))
        self._set_translate(body, (0.0, 0.0, 0.0))
        self._set_color(body, color)

        lens = f"{cam_root}/Lens"
        cone = self._UsdGeom.Cone.Define(self._stage, lens)
        cone.CreateRadiusAttr(0.04)
        cone.CreateHeightAttr(0.05)
        self._set_translate(lens, (0.0, 0.07, 0.0))
        self._set_color(lens, (0.05, 0.05, 0.05))

        logger.info("[VIS] camera placeholder added @ %s", cell_id)
        return cam_root

    # ========================================================================
    # Level 2.3: 일반 USD 자산 reference (AMR / sensor / rack / etc.)
    # ========================================================================
    def add_usd_reference(self,
                          prim_path: str,
                          usd_path: str,
                          translation: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                          scale: Tuple[float, float, float] = (1.0, 1.0, 1.0),
                          rotate_z_deg: float = 0.0,
                          variant_selection: Optional[Dict[str, str]] = None,
                          ) -> Optional[object]:
        """USD 파일을 prim에 reference로 attach.

        실 자산이 로컬/원격에서 안 잡힐 수 있으므로 호출자 측에서 사전 검증
        (find_local_asset 등) 후 호출하는 것을 권장.

        variant_selection:
            ``{variant_set_name: variant_name}`` 형태의 dict.
            AddReference 직후 ``prim.GetVariantSets().GetVariantSet(k)
            .SetVariantSelection(v)`` 가 적용된다. NovaCarter처럼 USD가
            variant-set 컨테이너인 경우, 이 값이 없으면 Physics/Configuration
            variant payload가 일관되지 않은 기본값으로 골라져
            "PhysicsUSD::CreateJoint - no bodies defined at body0 and body1"
            경고가 발생한다. 자세한 내용은 `asset_catalog.UsdAssetSpec` 참고.
        """
        UsdGeom = self._UsdGeom
        Gf = self._Gf

        # 부모 prim 보장
        parent_path = "/".join(prim_path.rstrip("/").split("/")[:-1]) or "/World"
        if not self._stage.GetPrimAtPath(parent_path):
            UsdGeom.Xform.Define(self._stage, parent_path)

        UsdGeom.Xform.Define(self._stage, prim_path)
        prim = self._stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            logger.warning("[VIS] add_usd_reference: prim define failed @ %s",
                           prim_path)
            return None

        # 1) reference attach — Isaac/USD에서는 즉시 효과가 stage에 compose 되지만
        #    GetChildren()은 reference payload가 컴포지션을 끝낸 후 평가된다.
        #    AddReference()가 raise 없이 끝나면 일단 attach는 성공.
        try:
            refs = prim.GetReferences()
            refs.AddReference(usd_path)
        except Exception as e:
            logger.warning("[VIS] add_usd_reference: AddReference failed for "
                           "%s (%s): %s", prim_path, usd_path, e)
            return None

        # 1.5) variant selection — NovaCarter 등 variant-set 컨테이너용.
        #      AddReference 직후, transform 적용 이전에 반드시 호출해야
        #      compose 그래프가 올바른 variant payload(rigid body / joint
        #      target prim)를 stage에 포함시킨다.
        if variant_selection:
            self._apply_variant_selection(prim, variant_selection, usd_path)

        # 2) transform — reference 다음에 설정하면 reference 안의 root xform을
        #    덮어쓰게 된다. 정확한 순서: SetReference → AddTranslate/Scale/Rotate.
        xformable = UsdGeom.Xformable(prim)
        xformable.ClearXformOpOrder()
        t = xformable.AddTranslateOp()
        t.Set(Gf.Vec3d(*translation))
        if abs(rotate_z_deg) > 1e-6:
            r = xformable.AddRotateZOp()
            r.Set(float(rotate_z_deg))
        if scale != (1.0, 1.0, 1.0):
            s = xformable.AddScaleOp()
            s.Set(Gf.Vec3f(*scale))

        # 3) 진단 — children, prim type, reference 목록을 명시적으로 로그
        child_count = len(list(prim.GetChildren()))
        try:
            ref_list = prim.GetMetadata("references")
            ref_count = (len(ref_list.prependedItems) + len(ref_list.appendedItems)
                         if ref_list is not None else 0)
        except Exception:
            ref_count = -1  # 못 읽음 (=라이브러리 버전 차이)

        logger.info("[VIS] add_usd_reference: %s ← %s "
                    "(children=%d, refs=%s, type=%s)",
                    prim_path, usd_path, child_count,
                    ref_count if ref_count >= 0 else "n/a",
                    prim.GetTypeName())

        if child_count == 0 and ref_count <= 0:
            # 진짜로 reference가 안 붙은 경우만 실패 신호.
            # children=0 이지만 ref_count>0 이면 USD compose가 다음 frame에 일어나는
            # 정상 케이스 — None 반환하지 않는다.
            logger.warning("[VIS] add_usd_reference: %s — reference attach FAILED "
                           "(no children, no refs, usd=%s)", prim_path, usd_path)
            return None
        return prim

    def add_amr_usd(self, amr_id: str,
                    asset_name: str = "nova_carter",
                    position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                    rotate_z_deg: float = 0.0,
                    ) -> Optional[str]:
        """ASSET_CATALOG의 AMR USD를 로드 (실패 시 None — 호출자가 placeholder fallback)."""
        from fdw_sim.visualization.asset_catalog import (
            ASSET_CATALOG, find_local_asset, resolve_asset_usd_path,
        )

        spec = ASSET_CATALOG.get(asset_name)
        if spec is None:
            logger.warning("[VIS] add_amr_usd: unknown asset %s", asset_name)
            return None

        local = find_local_asset(spec)
        usd_path = local if local is not None else resolve_asset_usd_path(spec)
        if local is None:
            is_remote = (usd_path.startswith("omniverse://")
                         or usd_path.startswith("http://")
                         or usd_path.startswith("https://"))
            if is_remote:
                logger.warning("[VIS] add_amr_usd: %s only available remotely "
                               "(%s) — may fail without Nucleus",
                               asset_name, usd_path)

        amr_path = f"{self.config.root_prim_path}/AMRs/{amr_id}_USD"
        prim = self.add_usd_reference(
            amr_path, usd_path,
            translation=position,
            scale=spec.scale,
            rotate_z_deg=rotate_z_deg,
            variant_selection=spec.variant_selection,
        )
        # add_usd_reference는 이미 reference attach 성공 여부를 검증한다.
        # children==0 이라도 reference만 붙었으면 다음 frame에 compose 된다.
        if prim is None:
            logger.warning("[VIS] add_amr_usd: %s — reference attach failed", amr_id)
            return None

        self._amr_prims[amr_id] = amr_path
        logger.info("[VIS] AMR %s placed via USD (%s) @ %s",
                    amr_id, asset_name, position)
        return amr_path

    def add_cell_prop(self, cell_id: str,
                      asset_name: str,
                      offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                      rotate_z_deg: float = 0.0,
                      ) -> Optional[str]:
        """셀 위에 일반 USD prop(rack/box 등)을 reference로 부착."""
        from fdw_sim.visualization.asset_catalog import (
            ASSET_CATALOG, find_local_asset, resolve_asset_usd_path,
        )

        spec = ASSET_CATALOG.get(asset_name)
        if spec is None:
            return None

        cell_root = self._cell_prims.get(cell_id)
        if cell_root is None:
            return None

        local = find_local_asset(spec)
        usd_path = local if local is not None else resolve_asset_usd_path(spec)

        prop_path = f"{cell_root}/Prop_{asset_name}"
        prim = self.add_usd_reference(
            prop_path, usd_path,
            translation=offset,
            scale=spec.scale,
            rotate_z_deg=rotate_z_deg,
            variant_selection=spec.variant_selection,
        )
        if prim is None:
            return None
        return prop_path

    def add_inspection_camera_real(self, cell_id: str,
                                    height: float = 1.4,
                                    use_usd_camera: bool = True,
                                    ) -> Optional[str]:
        """Inspection Cell용 — 사실적 카메라(housing + lens + pole) + UsdGeom.Camera prim."""
        cell_root = self._cell_prims.get(cell_id)
        if cell_root is None:
            return None

        cam_root = f"{cell_root}/InspectionCam"
        self._UsdGeom.Xform.Define(self._stage, cam_root)
        self._set_translate(cam_root, (0.0, -0.4, height))

        # housing
        housing = f"{cam_root}/Housing"
        cube = self._UsdGeom.Cube.Define(self._stage, housing)
        cube.CreateSizeAttr(1.0)
        self._set_scale(housing, (0.06, 0.08, 0.06))
        self._set_translate(housing, (0.0, 0.0, 0.0))
        self._set_color(housing, (0.15, 0.15, 0.2))

        # lens (실린더)
        lens = f"{cam_root}/Lens"
        cyl = self._UsdGeom.Cylinder.Define(self._stage, lens)
        cyl.CreateRadiusAttr(0.035)
        cyl.CreateHeightAttr(0.08)
        cyl.CreateAxisAttr("Y")
        self._set_translate(lens, (0.0, 0.10, 0.0))
        self._set_color(lens, (0.02, 0.02, 0.02))

        # ring (포커스 링)
        ring = f"{cam_root}/Ring"
        ring_cyl = self._UsdGeom.Cylinder.Define(self._stage, ring)
        ring_cyl.CreateRadiusAttr(0.045)
        ring_cyl.CreateHeightAttr(0.01)
        ring_cyl.CreateAxisAttr("Y")
        self._set_translate(ring, (0.0, 0.14, 0.0))
        self._set_color(ring, (0.7, 0.6, 0.1))

        # 마운트 폴
        pole = f"{cam_root}/Pole"
        pole_cyl = self._UsdGeom.Cylinder.Define(self._stage, pole)
        pole_cyl.CreateRadiusAttr(0.02)
        pole_cyl.CreateHeightAttr(height - 0.1)
        pole_cyl.CreateAxisAttr("Z")
        self._set_translate(pole, (0.0, -0.05, -(height - 0.1) / 2))
        self._set_color(pole, (0.4, 0.4, 0.45))

        # 실 USD Camera prim
        if use_usd_camera:
            cam_prim_path = f"{cam_root}/Camera"
            try:
                cam = self._UsdGeom.Camera.Define(self._stage, cam_prim_path)
                cam.CreateFocalLengthAttr(35.0)
                cam.CreateClippingRangeAttr(self._Gf.Vec2f(0.05, 100.0))
                cam_x = self._UsdGeom.Xformable(cam.GetPrim())
                cam_x.ClearXformOpOrder()
                rop = cam_x.AddRotateXOp()
                rop.Set(-60.0)
            except Exception:
                logger.warning("[VIS] inspection USD camera prim failed",
                               exc_info=True)

        logger.info("[VIS] inspection camera (real-style) added @ %s", cell_id)
        return cam_root

    def add_smart_rack_real(self, cell_id: str,
                             capacity: int = 6,
                             prop_asset: str = "klt_bin",
                             ) -> Optional[str]:
        """Material Cell — 실 KLT bin USD를 격자로 배치.

        prop_asset이 로컬에 없으면 None → 호출자가 add_smart_rack() 큐브로 fallback.
        """
        from fdw_sim.visualization.asset_catalog import (
            ASSET_CATALOG, find_local_asset,
        )

        spec = ASSET_CATALOG.get(prop_asset)
        if spec is None:
            return None

        cell_root = self._cell_prims.get(cell_id)
        if cell_root is None:
            return None

        local = find_local_asset(spec)
        if local is None:
            logger.info("[VIS] add_smart_rack_real: %s not local — fallback",
                        prop_asset)
            return None
        usd_path = local

        rack_path = f"{cell_root}/SmartRack_USD"
        self._UsdGeom.Xform.Define(self._stage, rack_path)
        self._set_translate(rack_path, (0.0, 0.8, 0.85))

        cols = 3
        slot_w = 0.25
        slot_h = 0.20

        success = 0
        for i in range(capacity):
            r = i // cols
            c = i % cols
            slot_path = f"{rack_path}/Slot_{i:02d}"
            tx = (c - cols / 2 + 0.5) * slot_w
            ty = 0.0
            tz = r * slot_h
            prim = self.add_usd_reference(
                slot_path, usd_path,
                translation=(tx, ty, tz),
                scale=spec.scale,
                variant_selection=spec.variant_selection,
            )
            # add_usd_reference 가 reference attach 성공을 이미 검증함.
            # children==0 이어도 다음 frame에 compose 되므로 prim is not None 으로 충분.
            if prim is not None:
                success += 1

        if success == 0:
            logger.warning("[VIS] add_smart_rack_real: 0 slots succeeded "
                           "— removing %s", rack_path)
            self._stage.RemovePrim(rack_path)
            return None

        logger.info("[VIS] smart rack (USD) added @ %s (slots=%d/%d, asset=%s)",
                    cell_id, success, capacity, prop_asset)
        return rack_path

    # ========================================================================
    # 부품 (Part)
    # ========================================================================
    def add_part(self, part_id: str,
                 position: Tuple[float, float, float],
                 color: Tuple[float, float, float] = (0.2, 0.6, 1.0),
                 size: float = 0.15,
                 shape: str = "pipe") -> str:
        """부품 — 기본은 파이프(원통) 모양 (part_type이 tubular_frame_* 인 것과
        일치). shape="cube"로 예전 큐브 표현도 여전히 가능.

        파이프는 로컬 Y축을 따라 눕혀서 만든다 — 용접 경로가 정확히 Y축을
        따라 부품 위를 지나가도록 설계돼 있으므로(_start_welding_motion의
        start=(bx, by-oy, ..), end=(bx, by+oy, ..)), 토치가 파이프의 길이
        방향을 따라 이음매를 훑는 것처럼 자연스럽게 보인다.
        """
        part_path = f"{self.config.root_prim_path}/Parts/{part_id}"

        if shape == "pipe":
            cyl = self._UsdGeom.Cylinder.Define(self._stage, part_path)
            cyl.CreateAxisAttr("Y")
            cyl.CreateRadiusAttr(size * 0.35)
            cyl.CreateHeightAttr(size * 3.5)
        else:
            cube = self._UsdGeom.Cube.Define(self._stage, part_path)
            cube.CreateSizeAttr(1.0)
            self._set_scale(part_path, (size / 2, size / 2, size / 2))

        self._set_translate(part_path, position)
        self._set_color(part_path, color)

        self._part_prims[part_id] = part_path
        logger.info("[VIS] part %s spawned @ %s (shape=%s)",
                    part_id, position, shape)
        return part_path

    def move_part(self, part_id: str, target: Tuple[float, float, float]) -> None:
        """부품 즉시 이동 (애니메이션은 호출자가 step 단위로 갱신)."""
        path = self._part_prims.get(part_id)
        if path is None:
            logger.warning("[VIS] move_part: unknown part %s", part_id)
            return
        self._set_translate(path, target)

    def remove_part(self, part_id: str) -> None:
        path = self._part_prims.pop(part_id, None)
        if path is None:
            return
        self._stage.RemovePrim(path)

    def move_amr(self, amr_id: str, target: Tuple[float, float, float]) -> None:
        path = self._amr_prims.get(amr_id)
        if path is None:
            logger.warning("[VIS] move_amr: unknown amr %s", amr_id)
            return
        self._set_translate(path, target)

    # ========================================================================
    # 조회
    # ========================================================================
    def get_cell_path(self, cell_id: str) -> Optional[str]:
        return self._cell_prims.get(cell_id)

    def get_part_path(self, part_id: str) -> Optional[str]:
        return self._part_prims.get(part_id)

    # ========================================================================
    # 진단 — USD 자산이 실제로 stage에 붙었는지 확인
    # ========================================================================
    def diagnose_stage(self, verbose: bool = True) -> Dict[str, Dict]:
        """등록된 모든 cell/AMR prim의 USD reference 상태를 dict로 반환.

        각 항목:
          - path        : prim 경로
          - exists      : prim이 stage에 존재하는가
          - type        : prim 타입 (Xform, Mesh, ...)
          - children    : 자식 prim 수
          - has_ref     : USD reference가 메타데이터로 잡혀 있는가
          - ref_targets : reference target USD 파일 목록
        """
        Sdf = self._Sdf
        report: Dict[str, Dict] = {}

        def _inspect(label: str, prim_path: str) -> Dict:
            prim = self._stage.GetPrimAtPath(prim_path)
            info = {
                "path": prim_path,
                "exists": bool(prim and prim.IsValid()),
                "type": "",
                "children": 0,
                "has_ref": False,
                "ref_targets": [],
            }
            if not info["exists"]:
                return info
            info["type"] = str(prim.GetTypeName())
            info["children"] = len(list(prim.GetChildren()))
            try:
                meta = prim.GetMetadata("references")
                if meta is not None:
                    targets = []
                    for item in list(meta.prependedItems) + list(meta.appendedItems):
                        # Sdf.Reference 의 assetPath 추출
                        try:
                            targets.append(item.assetPath)
                        except Exception:
                            targets.append(str(item))
                    info["ref_targets"] = targets
                    info["has_ref"] = len(targets) > 0
            except Exception:
                pass
            return info

        # cells
        for cell_id, cell_path in self._cell_prims.items():
            report[f"cell:{cell_id}"] = _inspect(cell_id, cell_path)
            # cell 하위에 SmartRack/RobotArm/InspectionCam 가 있을 수 있음
            for sub in ("SmartRack", "SmartRack_USD", "RobotArm",
                        "InspectionCam", "Camera"):
                sub_path = f"{cell_path}/{sub}"
                sub_prim = self._stage.GetPrimAtPath(sub_path)
                if sub_prim and sub_prim.IsValid():
                    report[f"cell:{cell_id}/{sub}"] = _inspect(
                        f"{cell_id}/{sub}", sub_path)

        # AMRs
        for amr_id, amr_path in self._amr_prims.items():
            report[f"amr:{amr_id}"] = _inspect(amr_id, amr_path)

        if verbose:
            logger.info("[VIS] === stage diagnose: %d entries ===", len(report))
            for key, info in report.items():
                if info["has_ref"]:
                    logger.info("[VIS] %-40s children=%d ref=%s",
                                key, info["children"],
                                info["ref_targets"][0] if info["ref_targets"]
                                else "?")
                else:
                    logger.info("[VIS] %-40s children=%d type=%s (no-ref)",
                                key, info["children"], info["type"])
        return report

    # ========================================================================
    # 내부 helper
    # ========================================================================
    def _apply_variant_selection(self, prim, variant_selection: Dict[str, str],
                                  usd_path: str) -> None:
        """USD variant set 선택을 prim에 적용 (NovaCarter 등 variant 컨테이너용).

        Args:
            prim:               variant를 적용할 USD prim (이미 AddReference 후)
            variant_selection:  ``{variant_set_name: variant_name}``
            usd_path:           로그용 — 어떤 USD에 대한 적용인지 표시

        실패 모드:
          * variant set 자체가 USD에 없음    → 경고 + 사용 가능 set 목록 출력
          * 지정한 variant name이 set에 없음 → 경고 + 사용 가능 variant 출력
                                                + 기본값으로 폴백 (변경 없음)
          * pxr API 예외                      → 경고 + 다음 set 시도

        모든 실패는 경고 레벨로만 처리하고 reference 자체는 살려둔다.
        그렇게 해야 variant 이름 spelling이 약간 어긋난 경우라도 기본 variant로
        장면이 보여지고, 사용자가 로그에 출력된 사용 가능 목록을 보고
        ``asset_catalog.py``의 spelling을 보정할 수 있다.
        """
        try:
            vsets = prim.GetVariantSets()
        except Exception as e:
            logger.warning("[VIS] variant: GetVariantSets failed on %s (%s): %s",
                           prim.GetPath(), usd_path, e)
            return

        try:
            available_sets = list(vsets.GetNames())
        except Exception:
            available_sets = []

        for set_name, variant_name in variant_selection.items():
            if set_name not in available_sets:
                logger.warning(
                    "[VIS] variant: set '%s' NOT FOUND on %s "
                    "(usd=%s). Available variant sets: %s — "
                    "skipping this selection (default variant used).",
                    set_name, prim.GetPath(), usd_path,
                    available_sets if available_sets else "(none)")
                continue

            try:
                vset = vsets.GetVariantSet(set_name)
                available_variants = list(vset.GetVariantNames())
            except Exception as e:
                logger.warning("[VIS] variant: GetVariantSet('%s') failed on %s: %s",
                               set_name, prim.GetPath(), e)
                continue

            if variant_name not in available_variants:
                logger.warning(
                    "[VIS] variant: '%s' NOT in set '%s' on %s "
                    "(usd=%s). Available variants in this set: %s — "
                    "leaving default. Please fix asset_catalog.UsdAssetSpec.variant_selection.",
                    variant_name, set_name, prim.GetPath(), usd_path,
                    available_variants)
                continue

            try:
                ok = vset.SetVariantSelection(variant_name)
                logger.info("[VIS] variant: %s.%s = %s (ok=%s) on %s",
                            set_name, variant_name, variant_name, ok,
                            prim.GetPath())
            except Exception as e:
                logger.warning("[VIS] variant: SetVariantSelection(%s=%s) failed: %s",
                               set_name, variant_name, e)

    def _enforce_xform_order(self, xformable) -> None:
        """USD 표준 xform op 순서로 재정렬: translate → rotate → scale.

        USD/Omniverse가 권장하는 op 순서는 [translate, rotate, scale]
        (수학적으로 scale이 먼저 적용됨 — point' = T·R·S·point).
        op order가 어긋나면 [omni.usd._impl.utils] Incompatible
        xformOpOrder, rotation or translation applied before scale.
        경고가 매 프레임 출력된다.
        """
        UsdGeom = self._UsdGeom
        ops = xformable.GetOrderedXformOps()
        if not ops:
            return

        # 우선순위: translate(0) < rotate*(1) < orient(2) < scale(3)
        def _rank(op) -> int:
            t = op.GetOpType()
            if t == UsdGeom.XformOp.TypeTranslate:
                return 0
            if t in (UsdGeom.XformOp.TypeRotateX,
                     UsdGeom.XformOp.TypeRotateY,
                     UsdGeom.XformOp.TypeRotateZ,
                     UsdGeom.XformOp.TypeRotateXYZ,
                     UsdGeom.XformOp.TypeRotateXZY,
                     UsdGeom.XformOp.TypeRotateYXZ,
                     UsdGeom.XformOp.TypeRotateYZX,
                     UsdGeom.XformOp.TypeRotateZXY,
                     UsdGeom.XformOp.TypeRotateZYX):
                return 1
            if t == UsdGeom.XformOp.TypeOrient:
                return 2
            if t == UsdGeom.XformOp.TypeScale:
                return 3
            return 4

        sorted_ops = sorted(ops, key=_rank)
        # 변경이 없으면 set 호출도 생략 (불필요한 USD 알림 방지)
        if [op.GetOpName() for op in sorted_ops] != [op.GetOpName() for op in ops]:
            xformable.SetXformOpOrder(sorted_ops)

    def _set_translate(self, prim_path: str,
                       translation: Tuple[float, float, float]) -> None:
        prim = self._stage.GetPrimAtPath(prim_path)
        if not prim:
            return
        xformable = self._UsdGeom.Xformable(prim)
        ops = xformable.GetOrderedXformOps()
        translate_op = None
        for op in ops:
            if op.GetOpType() == self._UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
            # 새로 추가한 경우 표준 순서 강제
            self._enforce_xform_order(xformable)
        translate_op.Set(self._Gf.Vec3d(*translation))

    def _set_scale(self, prim_path: str,
                   scale: Tuple[float, float, float]) -> None:
        prim = self._stage.GetPrimAtPath(prim_path)
        if not prim:
            return
        xformable = self._UsdGeom.Xformable(prim)
        ops = xformable.GetOrderedXformOps()
        scale_op = None
        for op in ops:
            if op.GetOpType() == self._UsdGeom.XformOp.TypeScale:
                scale_op = op
                break
        if scale_op is None:
            scale_op = xformable.AddScaleOp()
            # 새로 추가한 경우 표준 순서 강제
            self._enforce_xform_order(xformable)
        scale_op.Set(self._Gf.Vec3f(*scale))

    def _set_color(self, prim_path: str,
                   color: Tuple[float, float, float]) -> None:
        prim = self._stage.GetPrimAtPath(prim_path)
        if not prim:
            return
        gprim = self._UsdGeom.Gprim(prim)
        attr = gprim.GetDisplayColorAttr()
        attr.Set([self._Gf.Vec3f(*color)])
