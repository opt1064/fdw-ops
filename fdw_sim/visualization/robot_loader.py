"""RobotLoader — Isaac Sim에 실제 로봇팔 USD 에셋을 로드.

Isaac Sim 5.x에서 기본 제공되는 로봇 모델(Franka Panda, UR10 등)을
NVIDIA Nucleus 경로에서 가져와 USD 스테이지에 배치한다.

USD 경로:
    Franka Panda  : /Isaac/Robots/Franka/franka.usd
    UR10          : /Isaac/Robots/UR10/ur10.usd
    Carter v1     : /Isaac/Robots/Carter/carter_v1.usd
    Jetbot        : /Isaac/Robots/Jetbot/jetbot.usd

Isaac Sim의 자산 서버는 기본적으로 `omniverse://localhost/NVIDIA/Assets/Isaac`
또는 로컬 캐시(`~/Documents/Kit/shared/exts/...`)에 마운트된다.
환경 변수 `ISAAC_NUCLEUS_DIR`로 재정의 가능.

이 모듈은 모든 import를 lazy하게 처리하여,
Isaac Sim이 시작되지 않은 환경에서도 모듈 자체는 import 가능하다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import logging
import os

logger = logging.getLogger(__name__)


# =============================================================================
# 로봇 카탈로그
# =============================================================================
@dataclass
class RobotSpec:
    """로봇 USD 에셋 메타데이터."""
    name: str                                       # "franka_panda"
    usd_subpath: str                                # "Isaac/Robots/Franka/franka.usd"
    end_effector_frame: str                         # IK 타겟 프레임 이름
    home_joint_positions: List[float] = field(default_factory=list)
    base_offset_z: float = 0.0                      # 작업대 위에 올릴 때 보정값
    description: str = ""


# Isaac Sim 5.x 기본 카탈로그
ROBOT_CATALOG: Dict[str, RobotSpec] = {
    "franka_panda": RobotSpec(
        name="franka_panda",
        usd_subpath="Isaac/Robots/Franka/franka.usd",
        end_effector_frame="panda_hand",
        home_joint_positions=[0.012, -0.57, 0.0, -2.81, 0.0, 3.04, 0.741, 0.04, 0.04],
        base_offset_z=0.0,
        description="Franka Emika Panda 7-DOF arm with parallel gripper",
    ),
    "ur10": RobotSpec(
        name="ur10",
        usd_subpath="Isaac/Robots/UR10/ur10.usd",
        end_effector_frame="ee_link",
        home_joint_positions=[0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        base_offset_z=0.0,
        description="Universal Robots UR10 6-DOF arm",
    ),
    # 5.x 신규 경로 (혹시 기본 경로가 다를 경우 fallback)
    "franka_alt": RobotSpec(
        name="franka_alt",
        usd_subpath="Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
        end_effector_frame="panda_hand",
        home_joint_positions=[0.012, -0.57, 0.0, -2.81, 0.0, 3.04, 0.741, 0.04, 0.04],
    ),
}


# =============================================================================
# Nucleus 경로 해석
# =============================================================================
def get_isaac_assets_root() -> str:
    """Isaac Sim 5.x 자산 루트 경로 — 환경에 따라 자동 선택.

    우선순위:
      1) 환경변수 ISAAC_NUCLEUS_DIR
      2) Isaac Sim 5.x의 get_assets_root_path()
      3) 기본값 omniverse://localhost/NVIDIA/Assets/Isaac
    """
    # 1) 환경변수
    if os.environ.get("ISAAC_NUCLEUS_DIR"):
        return os.environ["ISAAC_NUCLEUS_DIR"].rstrip("/")

    # 2) Isaac Sim 표준 API
    try:
        from isaacsim.storage.native import get_assets_root_path  # type: ignore
        path = get_assets_root_path()
        if path:
            return path.rstrip("/")
    except Exception:
        pass

    # 2b) Isaac Sim 4.x fallback
    try:
        from omni.isaac.nucleus import get_assets_root_path  # type: ignore
        path = get_assets_root_path()
        if path:
            return path.rstrip("/")
    except Exception:
        pass

    # 3) hard-coded default
    return "omniverse://localhost/NVIDIA/Assets/Isaac"


def resolve_robot_usd_path(spec: RobotSpec) -> str:
    """RobotSpec → 절대 USD 경로."""
    root = get_isaac_assets_root()
    return f"{root}/{spec.usd_subpath}"


# =============================================================================
# RobotLoader
# =============================================================================
class RobotLoader:
    """USD 스테이지에 로봇팔을 reference로 추가하고 Articulation을 노출.

    사용 예 (SimulationApp 인스턴스화 후):

        loader = RobotLoader()
        franka = loader.load_robot(
            robot_name="franka_panda",
            prim_path="/World/FDW/Cells/WELD_01/Franka",
            position=(5.0, -0.3, 0.85),
        )

        # Articulation API
        franka.set_joint_positions([0.0, -0.5, 0.0, -2.0, 0.0, 2.5, 0.7])
    """

    def __init__(self) -> None:
        # lazy import 확인
        try:
            from pxr import Usd, UsdGeom, Sdf, Gf  # noqa: F401
            import omni.usd  # noqa: F401
        except ImportError as e:
            raise RuntimeError(
                "RobotLoader requires Isaac Sim runtime. "
                "Did you start SimulationApp first?"
            ) from e

        self._loaded: Dict[str, dict] = {}   # prim_path -> {spec, articulation?}

    # ------------------------------------------------------------------
    def load_robot(self,
                   robot_name: str,
                   prim_path: str,
                   position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                   orientation_deg_z: float = 0.0,
                   ) -> Optional[object]:
        """USD reference로 로봇팔을 로드하고 (선택적으로) Articulation을 반환.

        Args:
            robot_name: ROBOT_CATALOG의 key
            prim_path: USD 스테이지 내 절대 prim 경로
            position: 월드 좌표 (x, y, z)
            orientation_deg_z: Z축 회전 (도)

        Returns:
            isaacsim.core.prims.SingleArticulation 객체 (5.x) 또는 None
        """
        spec = ROBOT_CATALOG.get(robot_name)
        if spec is None:
            logger.error("[RobotLoader] unknown robot: %s", robot_name)
            return None

        usd_path = resolve_robot_usd_path(spec)
        logger.info("[RobotLoader] loading %s from %s", robot_name, usd_path)

        # 1) USD reference 추가
        from pxr import Usd, UsdGeom, Sdf, Gf
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            logger.error("[RobotLoader] no USD stage")
            return None

        # Xform 부모 prim 생성
        UsdGeom.Xform.Define(stage, prim_path)
        prim = stage.GetPrimAtPath(prim_path)

        # reference 추가
        refs = prim.GetReferences()
        refs.AddReference(usd_path)

        # 위치 / 회전 적용
        xformable = UsdGeom.Xformable(prim)
        # 기존 op 정리 후 새로 설정
        xformable.ClearXformOpOrder()
        t_op = xformable.AddTranslateOp()
        t_op.Set(Gf.Vec3d(position[0], position[1],
                          position[2] + spec.base_offset_z))
        if abs(orientation_deg_z) > 1e-3:
            r_op = xformable.AddRotateZOp()
            r_op.Set(orientation_deg_z)

        # 2) Articulation wrapper 시도
        articulation = self._wrap_articulation(prim_path, robot_name)

        self._loaded[prim_path] = {
            "spec": spec,
            "articulation": articulation,
        }

        logger.info("[RobotLoader] %s placed @ %s (articulation=%s)",
                    robot_name, position, articulation is not None)
        return articulation

    # ------------------------------------------------------------------
    def _wrap_articulation(self, prim_path: str, robot_name: str) -> Optional[object]:
        """Articulation 래퍼 객체 반환 (Isaac Sim 5.x 우선)."""
        # Isaac Sim 5.x 경로
        try:
            from isaacsim.core.prims import SingleArticulation  # type: ignore
            art = SingleArticulation(prim_path=prim_path, name=robot_name)
            return art
        except Exception as e:
            logger.debug("[RobotLoader] SingleArticulation failed: %s", e)

        # Isaac Sim 4.x 경로
        try:
            from omni.isaac.core.articulations import Articulation  # type: ignore
            art = Articulation(prim_path=prim_path, name=robot_name)
            return art
        except Exception as e:
            logger.debug("[RobotLoader] Articulation (4.x) failed: %s", e)

        logger.warning("[RobotLoader] no articulation API available; "
                       "robot %s loaded as visual-only", robot_name)
        return None

    # ------------------------------------------------------------------
    def get_spec(self, prim_path: str) -> Optional[RobotSpec]:
        info = self._loaded.get(prim_path)
        return info["spec"] if info else None

    def get_articulation(self, prim_path: str) -> Optional[object]:
        info = self._loaded.get(prim_path)
        return info["articulation"] if info else None

    def list_loaded(self) -> List[str]:
        return list(self._loaded.keys())
