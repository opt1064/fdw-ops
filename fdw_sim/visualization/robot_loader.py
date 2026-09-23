"""RobotLoader — Isaac Sim에 실제 로봇팔 USD 에셋을 로드.

Isaac Sim 5.x에서 기본 제공되는 로봇 모델(Franka Panda, UR10 등)을
NVIDIA Nucleus 경로에서 가져와 USD 스테이지에 배치한다.

USD 경로 (Isaac Sim 5.1 — NVIDIA가 5.0+에서 디렉토리 구조를 재편함):
    Franka Panda           : /Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd
    Franka Panda (inst.)   : /Isaac/Robots/FrankaRobotics/FrankaEmika/panda_instanceable.usd
    Franka FR3             : /Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd
    Factory Franka         : /Isaac/Robots/FrankaRobotics/FactoryFranka/factory_franka.usd
    UR10                   : /Isaac/Robots/UniversalRobots/ur10/ur10.usd
    Carter v1              : /Isaac/Robots/NVIDIA/Carter/carter_v1.usd

⚠️ Isaac 4.x의 옛 경로(/Isaac/Robots/Franka/franka.usd 등)는 5.1 S3 버킷에
   더 이상 존재하지 않으며 NoSuchKey 오류를 반환한다.

Isaac Sim의 자산 서버는 기본적으로 `omniverse://localhost/NVIDIA/Assets/Isaac`
또는 로컬 캐시(`~/Documents/Kit/shared/exts/...`)에 마운트된다.
환경 변수 `ISAAC_NUCLEUS_DIR`로 재정의 가능.

이 모듈은 모든 import를 lazy하게 처리하여,
Isaac Sim이 시작되지 않은 환경에서도 모듈 자체는 import 가능하다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import logging
import os

logger = logging.getLogger(__name__)


# =============================================================================
# 자산 경로 후보 — 로컬 디스크 우선 탐색
# =============================================================================
# AGX Thor / JetPack 환경에서 Nucleus 서버가 안 떠있고 S3 fallback이
# 비실용적인 경우가 많아, 로컬 디스크 후보 디렉토리를 먼저 시도한다.
# 각 후보는 그 안에 "Isaac/Robots/Franka/franka.usd"가 존재해야 매치.
LOCAL_ASSET_CANDIDATES = [
    # 우리 프로젝트가 추천하는 위치
    "~/isaac_assets",
    "~/isaac_assets/Isaac/5.1",
    # Omniverse Launcher 기본 설치 경로
    "~/Documents/Omniverse/Library/Isaac-Sim Full/Assets/Isaac/5.1",
    "~/Documents/Omniverse/Library/Isaac/5.1",
    # Omniverse user data
    "~/.local/share/ov/data/assets/Isaac/5.1",
    # 시스템 전역
    "/opt/nvidia/isaac-sim-assets/5.1",
    "/opt/ov/assets/Isaac/5.1",
]


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


# Isaac Sim 5.1 기본 카탈로그 (NVIDIA 공식 경로 — 2025-10 기준)
#   확인: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html
ROBOT_CATALOG: Dict[str, RobotSpec] = {
    "franka_panda": RobotSpec(
        name="franka_panda",
        usd_subpath="Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
        end_effector_frame="panda_hand",
        home_joint_positions=[0.012, -0.57, 0.0, -2.81, 0.0, 3.04, 0.741, 0.04, 0.04],
        base_offset_z=0.0,
        description="Franka Emika Panda 7-DOF arm with parallel gripper (Isaac 5.1)",
    ),
    "franka_panda_instanceable": RobotSpec(
        name="franka_panda_instanceable",
        usd_subpath="Isaac/Robots/FrankaRobotics/FrankaEmika/panda_instanceable.usd",
        end_effector_frame="panda_hand",
        home_joint_positions=[0.012, -0.57, 0.0, -2.81, 0.0, 3.04, 0.741, 0.04, 0.04],
        base_offset_z=0.0,
        description="Franka Panda instanceable variant (efficient duplication)",
    ),
    "franka_fr3": RobotSpec(
        name="franka_fr3",
        usd_subpath="Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd",
        end_effector_frame="fr3_hand",
        home_joint_positions=[0.0, -0.57, 0.0, -2.81, 0.0, 3.04, 0.741],
        base_offset_z=0.0,
        description="Franka FR3 next-gen 7-DOF arm (Isaac 5.1)",
    ),
    "factory_franka": RobotSpec(
        name="factory_franka",
        usd_subpath="Isaac/Robots/FrankaRobotics/FactoryFranka/factory_franka.usd",
        end_effector_frame="panda_hand",
        home_joint_positions=[0.012, -0.57, 0.0, -2.81, 0.0, 3.04, 0.741, 0.04, 0.04],
        base_offset_z=0.0,
        description="Factory-tuned Franka for IndustrialReal-style simulation",
    ),
    "ur10": RobotSpec(
        name="ur10",
        usd_subpath="Isaac/Robots/UniversalRobots/ur10/ur10.usd",
        end_effector_frame="ee_link",
        home_joint_positions=[0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        base_offset_z=0.0,
        description="Universal Robots UR10 6-DOF arm (Isaac 5.1)",
    ),
    # === Legacy fallbacks (Isaac 4.x 경로 — 더 이상 작동하지 않음) ===
    # 일부 로컬 디스크 인스톨이 옛 구조를 유지하는 경우를 위해 남겨둠.
    # 원격 fetch는 NoSuchKey가 나므로 LOCAL에서만 매치되어야 함.
    "franka_panda_legacy": RobotSpec(
        name="franka_panda_legacy",
        usd_subpath="Isaac/Robots/Franka/franka.usd",
        end_effector_frame="panda_hand",
        home_joint_positions=[0.012, -0.57, 0.0, -2.81, 0.0, 3.04, 0.741, 0.04, 0.04],
        description="[LEGACY 4.x] only resolves if locally present",
    ),
}


# =============================================================================
# Nucleus 경로 해석
# =============================================================================
def _check_local_candidate(candidate_root: str, spec_subpath: str) -> Optional[str]:
    """후보 디렉토리에 spec_subpath의 USD가 실제로 존재하는지 검사.

    Returns:
        존재하면 절대 경로(file:// 형태 아님 — pxr USD가 절대 경로도 받음),
        없으면 None.
    """
    full = Path(os.path.expanduser(candidate_root)) / spec_subpath
    if full.is_file():
        return str(full)
    return None


def find_local_robot_usd(spec: "RobotSpec") -> Optional[str]:
    """로컬 디스크에서 robot USD 파일을 찾음.

    LOCAL_ASSET_CANDIDATES + ISAAC_NUCLEUS_DIR_LOCAL 환경변수 + 명시적 파일 경로
    (FDW_FRANKA_USD 등)를 차례로 시도한다.
    """
    # 1) 직접 파일 경로 환경변수 (가장 명시적)
    env_key = f"FDW_{spec.name.upper()}_USD"
    direct = os.environ.get(env_key)
    if direct and Path(direct).is_file():
        logger.info("[RobotLoader] using %s=%s", env_key, direct)
        return direct

    # 2) ISAAC_NUCLEUS_DIR_LOCAL — 로컬 디스크 전용 루트
    local_root = os.environ.get("ISAAC_NUCLEUS_DIR_LOCAL")
    if local_root:
        found = _check_local_candidate(local_root, spec.usd_subpath)
        if found:
            logger.info("[RobotLoader] resolved via ISAAC_NUCLEUS_DIR_LOCAL: %s",
                        found)
            return found

    # 3) 표준 후보 디렉토리
    for cand in LOCAL_ASSET_CANDIDATES:
        found = _check_local_candidate(cand, spec.usd_subpath)
        if found:
            logger.info("[RobotLoader] resolved via local candidate: %s", found)
            return found

    return None


def get_isaac_assets_root() -> str:
    """Isaac Sim 5.x 자산 루트 경로 — 환경에 따라 자동 선택 (원격용).

    우선순위:
      1) 환경변수 ISAAC_NUCLEUS_DIR
      2) Isaac Sim 5.x의 get_assets_root_path()
      3) 기본값 omniverse://localhost/NVIDIA/Assets/Isaac

    주의: 이 함수는 원격(Nucleus/S3) 경로만 반환. 로컬 디스크 자산은
    find_local_robot_usd()로 별도 탐색한다.
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
    """RobotSpec → 절대 USD 경로 (로컬 우선, 원격 fallback).

    탐색 순서:
      1) 로컬 디스크 (FDW_*_USD env / ISAAC_NUCLEUS_DIR_LOCAL / 표준 후보)
      2) ISAAC_NUCLEUS_DIR / Nucleus / S3 fallback
    """
    local = find_local_robot_usd(spec)
    if local:
        return local

    root = get_isaac_assets_root()
    return f"{root}/{spec.usd_subpath}"


def diagnose_robot_assets() -> Dict[str, object]:
    """현재 환경에서 robot 자산이 어떻게 해석되는지 사람이 읽을 수 있는
    진단 정보를 반환. 콘솔에 출력하거나 troubleshooting 메시지에 포함.
    """
    report: Dict[str, object] = {
        "env_ISAAC_NUCLEUS_DIR": os.environ.get("ISAAC_NUCLEUS_DIR", "<unset>"),
        "env_ISAAC_NUCLEUS_DIR_LOCAL":
            os.environ.get("ISAAC_NUCLEUS_DIR_LOCAL", "<unset>"),
        "remote_root": get_isaac_assets_root(),
        "candidates": [],
        "resolved": {},
    }
    for cand in LOCAL_ASSET_CANDIDATES:
        expanded = os.path.expanduser(cand)
        report["candidates"].append({  # type: ignore[union-attr]
            "path": expanded,
            "exists": Path(expanded).is_dir(),
        })
    for name, spec in ROBOT_CATALOG.items():
        report["resolved"][name] = resolve_robot_usd_path(spec)  # type: ignore[index]
    return report


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
        is_local = not (usd_path.startswith("omniverse://")
                        or usd_path.startswith("http://")
                        or usd_path.startswith("https://"))
        logger.info("[RobotLoader] loading %s from %s (local=%s)",
                    robot_name, usd_path, is_local)

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
        try:
            refs.AddReference(usd_path)
        except Exception as e:
            raise RuntimeError(
                f"AddReference failed for {robot_name} ({usd_path}): {e}"
            ) from e

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

        # 1.5) USD payload load 검증 — 자식 prim이 0이면 reference resolve 실패
        # 비동기 로드라 가능성 있으니 짧게 한 두 번 stage update 후 재확인.
        if not self._verify_reference_loaded(prim, usd_path):
            # 명시적 에러 — 호출자(_spawn_robot_arm)가 placeholder로 fallback 가능
            raise RuntimeError(
                f"Robot USD reference failed to resolve: {usd_path}\n"
                f"  prim {prim_path} has 0 children after AddReference.\n"
                f"  Likely cause: file not found / Nucleus not running / "
                f"S3 fetch blocked.\n"
                f"  Hint: place franka.usd locally and set ISAAC_NUCLEUS_DIR_LOCAL "
                f"or FDW_{spec.name.upper()}_USD env var, or run "
                f"`bash scripts/download_franka_usd.sh` to download."
            )

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
    def _verify_reference_loaded(self, prim, usd_path: str,
                                   max_updates: int = 3) -> bool:
        """AddReference 후 자식 prim이 채워졌는지 확인.

        Omniverse USD payload는 일부 비동기로 resolve되므로 stage update를
        몇 번 돌려본 뒤 자식이 생기는지 본다.
        """
        def _has_children() -> bool:
            try:
                return len(list(prim.GetChildren())) > 0
            except Exception:
                return False

        if _has_children():
            return True

        # stage update를 짧게 돌려 비동기 payload resolve를 트리거
        try:
            import omni.kit.app  # type: ignore
            app = omni.kit.app.get_app()
            for _ in range(max_updates):
                app.update()
                if _has_children():
                    return True
        except Exception:
            # omni.kit.app가 없는 환경(테스트 등)에서는 그냥 현재 상태로 판단
            pass

        return _has_children()

    # ------------------------------------------------------------------
    def _wrap_articulation(self, prim_path: str, robot_name: str) -> Optional[object]:
        """Articulation 래퍼 객체 반환 (Isaac Sim 5.x 우선).

        생성만으로는 부족하다 — Articulation/SingleArticulation은
        world.scene에 등록해 world.reset()이 대신 초기화해주는 경로를 쓰지
        않는 한(여기서는 안 씀) 반드시 initialize()를 직접 호출해야
        PhysX 뷰가 연결되어 get_joint_positions() 등이 동작한다. 이걸
        빼먹으면 RMPflow의 ArticulationMotionPolicy가 "Attempted to
        compute an action, but the robot Articulation has not been
        initialized" 에러를 내며 매 step마다 조용히 실패한다 (DGX Spark
        실측: 'NoneType' object has no attribute 'astype').
        """
        # Isaac Sim 5.x 경로
        try:
            from isaacsim.core.prims import SingleArticulation  # type: ignore
            art = SingleArticulation(prim_path=prim_path, name=robot_name)
            self._initialize_articulation(art, robot_name)
            return art
        except Exception as e:
            logger.debug("[RobotLoader] SingleArticulation failed: %s", e)

        # Isaac Sim 4.x 경로
        try:
            from omni.isaac.core.articulations import Articulation  # type: ignore
            art = Articulation(prim_path=prim_path, name=robot_name)
            self._initialize_articulation(art, robot_name)
            return art
        except Exception as e:
            logger.debug("[RobotLoader] Articulation (4.x) failed: %s", e)

        logger.warning("[RobotLoader] no articulation API available; "
                       "robot %s loaded as visual-only", robot_name)
        return None

    @staticmethod
    def _initialize_articulation(art: object, robot_name: str) -> None:
        try:
            art.initialize()
        except Exception as e:
            logger.warning("[RobotLoader] %s articulation.initialize() failed "
                           "(joint control may not work): %s: %s",
                           robot_name, type(e).__name__, e)

    # ------------------------------------------------------------------
    def get_spec(self, prim_path: str) -> Optional[RobotSpec]:
        info = self._loaded.get(prim_path)
        return info["spec"] if info else None

    def get_articulation(self, prim_path: str) -> Optional[object]:
        info = self._loaded.get(prim_path)
        return info["articulation"] if info else None

    def list_loaded(self) -> List[str]:
        return list(self._loaded.keys())
