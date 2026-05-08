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
    def add_ground_plane(self, size: float = 50.0) -> str:
        """간단한 그라운드 플레인 (Isaac Lab의 기본 ground와는 별개)."""
        path = f"{self.config.root_prim_path}/Ground"
        plane = self._UsdGeom.Plane.Define(self._stage, path)
        plane.CreateAxisAttr("Z")
        plane.CreateLengthAttr(size)
        plane.CreateWidthAttr(size)
        self._set_color(path, self.config.ground_color)
        return path

    def _add_default_lighting(self) -> None:
        light_path = f"{self.config.root_prim_path}/Lighting/Distant"
        light = self._UsdLux.DistantLight.Define(self._stage, light_path)
        light.CreateIntensityAttr(3000.0)
        light.CreateAngleAttr(0.5)

        dome_path = f"{self.config.root_prim_path}/Lighting/Dome"
        dome = self._UsdLux.DomeLight.Define(self._stage, dome_path)
        dome.CreateIntensityAttr(500.0)

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
    # 부품 (Part)
    # ========================================================================
    def add_part(self, part_id: str,
                 position: Tuple[float, float, float],
                 color: Tuple[float, float, float] = (0.2, 0.6, 1.0),
                 size: float = 0.15) -> str:
        """부품 — 작은 컬러 큐브."""
        part_path = f"{self.config.root_prim_path}/Parts/{part_id}"
        cube = self._UsdGeom.Cube.Define(self._stage, part_path)
        cube.CreateSizeAttr(1.0)
        self._set_scale(part_path, (size / 2, size / 2, size / 2))
        self._set_translate(part_path, position)
        self._set_color(part_path, color)

        self._part_prims[part_id] = part_path
        logger.info("[VIS] part %s spawned @ %s", part_id, position)
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
    # 내부 helper
    # ========================================================================
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
        scale_op.Set(self._Gf.Vec3f(*scale))

    def _set_color(self, prim_path: str,
                   color: Tuple[float, float, float]) -> None:
        prim = self._stage.GetPrimAtPath(prim_path)
        if not prim:
            return
        gprim = self._UsdGeom.Gprim(prim)
        attr = gprim.GetDisplayColorAttr()
        attr.Set([self._Gf.Vec3f(*color)])
