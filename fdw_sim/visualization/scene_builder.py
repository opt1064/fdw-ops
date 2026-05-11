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

            # look-at 행렬 계산 (단순 버전)
            import math
            dx = target[0] - cam_pos[0]
            dy = target[1] - cam_pos[1]
            dz = target[2] - cam_pos[2]
            yaw_deg = math.degrees(math.atan2(dx, -dy))
            horiz = math.sqrt(dx * dx + dy * dy)
            pitch_deg = -math.degrees(math.atan2(dz, horiz))

            xformable = UsdGeom.Xformable(cam.GetPrim())
            xformable.ClearXformOpOrder()
            t_op = xformable.AddTranslateOp()
            t_op.Set(Gf.Vec3d(*cam_pos))
            rz_op = xformable.AddRotateZOp()
            rz_op.Set(yaw_deg)
            rx_op = xformable.AddRotateXOp()
            rx_op.Set(90.0 + pitch_deg)  # Z-up → Y-forward 보정

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
                          ) -> Optional[object]:
        """USD 파일을 prim에 reference로 attach.

        실 자산이 로컬/원격에서 안 잡힐 수 있으므로 호출자 측에서 사전 검증
        (find_local_asset 등) 후 호출하는 것을 권장.
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

        try:
            refs = prim.GetReferences()
            refs.AddReference(usd_path)
        except Exception as e:
            logger.warning("[VIS] add_usd_reference: AddReference failed for "
                           "%s (%s): %s", prim_path, usd_path, e)
            return None

        # transform
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

        child_count = len(list(prim.GetChildren()))
        if child_count == 0:
            logger.warning("[VIS] add_usd_reference: %s loaded but has 0 children "
                           "(USD likely failed to resolve: %s)", prim_path, usd_path)
        else:
            logger.info("[VIS] add_usd_reference: %s @ %s (children=%d)",
                        prim_path, translation, child_count)
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
        )
        if prim is None or len(list(prim.GetChildren())) == 0:
            return None

        self._amr_prims[amr_id] = amr_path
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
        )
        if prim is None or len(list(prim.GetChildren())) == 0:
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
            )
            if prim is not None and len(list(prim.GetChildren())) > 0:
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
