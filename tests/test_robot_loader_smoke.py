"""Smoke test — robot_loader & ik_controller가 Isaac Sim 없이도 import 가능한지 확인.

이 테스트의 목적:
  * Lazy import 설계가 정상 동작하는지 확인 (모듈 로드 시 omni/pxr/isaacsim 의존하지 않음)
  * ROBOT_CATALOG가 잘 정의되어 있는지 검증
  * RobotLoader / IKController 인스턴스화 시점에 명확한 에러를 발생시키는지 확인

실제 Isaac Sim 환경(AGX Thor)에서는 별도 통합 테스트로 검증한다.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


class TestRobotLoaderImport(unittest.TestCase):
    """robot_loader 모듈이 Isaac Sim 없이 import 되는지."""

    def test_import_module(self):
        from fdw_sim.visualization import robot_loader  # noqa: F401

    def test_robot_catalog_defined(self):
        from fdw_sim.visualization.robot_loader import ROBOT_CATALOG, RobotSpec

        # Isaac 5.1 정식 카탈로그 엔트리
        self.assertIn("franka_panda", ROBOT_CATALOG)
        self.assertIn("ur10", ROBOT_CATALOG)
        # Isaac 5.1에서 추가된 변종들
        self.assertIn("franka_panda_instanceable", ROBOT_CATALOG)
        self.assertIn("franka_fr3", ROBOT_CATALOG)
        self.assertIn("factory_franka", ROBOT_CATALOG)
        # 레거시 4.x 경로도 로컬 fallback용으로 유지
        self.assertIn("franka_panda_legacy", ROBOT_CATALOG)
        # 용접 셀 실 기체 (Franka -> CRX-10iA 교체)
        self.assertIn("fanuc_crx10ia", ROBOT_CATALOG)

        crx = ROBOT_CATALOG["fanuc_crx10ia"]
        self.assertIsInstance(crx, RobotSpec)
        self.assertEqual(crx.end_effector_frame, "tool0")
        self.assertEqual(len(crx.home_joint_positions), 6)
        self.assertTrue(crx.usd_subpath.endswith(".usd"))

        franka = ROBOT_CATALOG["franka_panda"]
        self.assertIsInstance(franka, RobotSpec)
        self.assertEqual(franka.end_effector_frame, "panda_hand")
        self.assertEqual(len(franka.home_joint_positions), 9)  # 7 joints + 2 finger
        self.assertTrue(franka.usd_subpath.endswith("franka.usd"))
        # Isaac 5.1 새 경로 구조 검증
        self.assertEqual(
            franka.usd_subpath,
            "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
        )

        ur10 = ROBOT_CATALOG["ur10"]
        self.assertEqual(ur10.end_effector_frame, "ee_link")
        self.assertEqual(len(ur10.home_joint_positions), 6)
        # Isaac 5.1에서 UR10도 UniversalRobots/ 아래로 이동
        self.assertEqual(
            ur10.usd_subpath,
            "Isaac/Robots/UniversalRobots/ur10/ur10.usd",
        )

    def test_resolve_assets_root_default(self):
        """환경변수 미지정 시에도 default 경로를 반환."""
        import os
        from fdw_sim.visualization.robot_loader import get_isaac_assets_root

        # 환경변수 임시 제거
        prev = os.environ.pop("ISAAC_NUCLEUS_DIR", None)
        try:
            root = get_isaac_assets_root()
            self.assertIsInstance(root, str)
            self.assertTrue(len(root) > 0)
        finally:
            if prev is not None:
                os.environ["ISAAC_NUCLEUS_DIR"] = prev

    def test_resolve_assets_root_env(self):
        """ISAAC_NUCLEUS_DIR 환경변수가 우선."""
        import os
        from fdw_sim.visualization.robot_loader import (
            get_isaac_assets_root, resolve_robot_usd_path, ROBOT_CATALOG,
        )

        prev = os.environ.get("ISAAC_NUCLEUS_DIR")
        os.environ["ISAAC_NUCLEUS_DIR"] = "/tmp/fake_isaac_assets"
        try:
            root = get_isaac_assets_root()
            self.assertEqual(root, "/tmp/fake_isaac_assets")

            franka_path = resolve_robot_usd_path(ROBOT_CATALOG["franka_panda"])
            # Isaac 5.1 정식 경로: FrankaRobotics/FrankaPanda/
            self.assertEqual(
                franka_path,
                "/tmp/fake_isaac_assets/Isaac/Robots/"
                "FrankaRobotics/FrankaPanda/franka.usd",
            )
        finally:
            if prev is None:
                os.environ.pop("ISAAC_NUCLEUS_DIR", None)
            else:
                os.environ["ISAAC_NUCLEUS_DIR"] = prev

    def test_robot_loader_requires_isaac(self):
        """Isaac Sim 미설치 환경에서 RobotLoader() 호출 시 명확한 에러."""
        from fdw_sim.visualization.robot_loader import RobotLoader

        # Isaac Sim이 있으면 통과, 없으면 RuntimeError
        try:
            from pxr import Usd  # noqa: F401
            isaac_available = True
        except ImportError:
            isaac_available = False

        if isaac_available:
            # 환경에 따라 stage가 없어도 인스턴스화는 가능
            try:
                loader = RobotLoader()
                self.assertIsNotNone(loader)
            except RuntimeError:
                pass  # SimulationApp이 시작 안 됐을 수 있음
        else:
            with self.assertRaises(RuntimeError):
                RobotLoader()


class TestIKControllerImport(unittest.TestCase):
    """ik_controller 모듈이 Isaac Sim 없이 import 되는지."""

    def test_import_module(self):
        from fdw_sim.visualization import ik_controller  # noqa: F401

    def test_welding_path_dataclass(self):
        from fdw_sim.visualization.ik_controller import WeldingPath

        path = WeldingPath(
            start=(1.0, 0.0, 0.5),
            end=(1.5, 0.0, 0.5),
            travel_time_sec=5.0,
        )
        self.assertEqual(path.start, (1.0, 0.0, 0.5))
        self.assertEqual(path.end, (1.5, 0.0, 0.5))
        self.assertEqual(path.travel_time_sec, 5.0)
        # 기본값 검증
        self.assertGreater(path.approach_height, 0.0)
        self.assertIn(path.phase, ("approach", "weld", "retreat", "done"))

    def test_ik_config_defaults(self):
        from fdw_sim.visualization.ik_controller import IKConfig

        cfg = IKConfig()
        self.assertTrue(cfg.use_lula)
        self.assertGreater(cfg.waypoint_blend_time, 0.0)
        self.assertGreater(cfg.home_blend_time, 0.0)

    def test_get_tcp_position_exists_and_tracks_commanded_target(self):
        """IKController도 RMPflowController와 동일하게 get_tcp_position()을
        노출해야 한다 — 없으면 workshop_visualizer의
        ``hasattr(ik, "get_tcp_position")`` 체크가 항상 실패해 spark emitter가
        용접 셀에서 TCP 위치를 한 번도 못 받는다 (회귀: 787c2a4가 rmpflow_controller
        에만 적용되고 ik_controller에는 빠졌던 문제)."""
        from fdw_sim.visualization.ik_controller import (
            IKConfig, IKController, WeldingPath,
        )

        class _FakeSpec:
            end_effector_frame = "panda_hand"
            home_joint_positions = [0.0] * 7

        ctrl = IKController(articulation=None, spec=_FakeSpec(), config=IKConfig())
        self.assertTrue(hasattr(ctrl, "get_tcp_position"))
        self.assertIsNone(ctrl.get_tcp_position())

        path = WeldingPath(start=(1.0, 0.0, 1.0), end=(1.0, 0.2, 1.0),
                            travel_time_sec=2.0)
        ctrl.start_path(path)
        ctrl.update(0.1)

        tcp = ctrl.get_tcp_position()
        self.assertIsNotNone(tcp, "articulation=None이어도 commanded target으로 "
                                    "fallback 되어야 한다")
        self.assertEqual(len(tcp), 3)


class TestWorkshopVisualizerImport(unittest.TestCase):
    """Level 2.1 필드가 추가된 WorkshopVizConfig 검증."""

    def test_viz_config_level2_1_fields(self):
        from fdw_sim.visualization.workshop_visualizer import WorkshopVizConfig

        cfg = WorkshopVizConfig()
        # Level 2.1 필드들이 존재해야 함
        self.assertFalse(cfg.use_real_robot)
        self.assertEqual(cfg.robot_name, "fanuc_crx10ia")
        self.assertTrue(cfg.enable_ik)
        self.assertAlmostEqual(cfg.weld_path_offset_y, 0.25)
        self.assertAlmostEqual(cfg.weld_path_height, 0.05)

    def test_viz_config_overrides(self):
        from fdw_sim.visualization.workshop_visualizer import WorkshopVizConfig

        cfg = WorkshopVizConfig(
            use_real_robot=True,
            robot_name="ur10",
            enable_ik=False,
        )
        self.assertTrue(cfg.use_real_robot)
        self.assertEqual(cfg.robot_name, "ur10")
        self.assertFalse(cfg.enable_ik)


class TestSimulationConfigLevel21(unittest.TestCase):
    """SimulationConfig에 Level 2.1 필드가 추가되었는지 검증."""

    def test_simulation_config_fields(self):
        from fdw_sim.simulation.manager import SimulationConfig

        cfg = SimulationConfig()
        self.assertFalse(cfg.use_real_robot)
        self.assertEqual(cfg.robot_name, "fanuc_crx10ia")
        self.assertTrue(cfg.enable_ik)
        self.assertAlmostEqual(cfg.weld_path_offset_y, 0.25)
        self.assertAlmostEqual(cfg.weld_path_height, 0.05)
        # livestream>0일 때 기본적으로 AppLauncher를 건너뛰도록 하는 스위치
        # (Thor aarch64에서 AppLauncher의 experience 파일이 livestream 1/2
        # 모두 omni.kit.livestream.webrtc 미존재로 부팅 실패하는 것 확인됨).
        self.assertFalse(cfg.force_applauncher_for_livestream)

    def test_keep_viewer_alive_noop_in_discrete_mode(self):
        """keep_viewer_alive()는 discrete 모드(Isaac Sim 없음)에서 즉시
        리턴해야 한다 — --keep-alive 옵션이 discrete 모드에서 무한 루프에
        빠지지 않는지 확인."""
        from fdw_sim.simulation.manager import SimulationManager, SimulationConfig

        sim = SimulationManager(SimulationConfig(mode="discrete"))
        sim.keep_viewer_alive()  # 즉시 리턴해야 함 (hang 하면 테스트가 타임아웃)

    def test_simulation_config_overrides(self):
        from fdw_sim.simulation.manager import SimulationConfig

        cfg = SimulationConfig(
            use_real_robot=True,
            robot_name="ur10",
            enable_ik=False,
            weld_path_offset_y=0.4,
            weld_path_height=0.1,
        )
        self.assertTrue(cfg.use_real_robot)
        self.assertEqual(cfg.robot_name, "ur10")
        self.assertFalse(cfg.enable_ik)
        self.assertAlmostEqual(cfg.weld_path_offset_y, 0.4)
        self.assertAlmostEqual(cfg.weld_path_height, 0.1)


class TestRmpflowControllerNonFrankaPathLookup(unittest.TestCase):
    """RMPflowController가 franka가 아닌 로봇에 franka의 Lula 파일을
    잘못 물려주지 않는지 확인 (2026-09-28 DGX Spark 실측 회귀 버그).

    실제 버그: bundled motion_policy_configs 디렉토리에 franka 서브폴더만
    있고 대상 로봇(예: fanuc_crx10ia) 서브폴더가 없으면, 예전 코드는
    "일단 franka 걸로라도" 하고 franka의 rmpflow_common.yaml/urdf를
    반환했다. 그 결과 RmpFlow가 franka 전용 kinematics 모델로 완전히 다른
    articulation(CRX-10iA)을 제어하려다 매 update()마다 예외를 던졌다
    (컨트롤러 update가 무한 반복 실패 — 실측 로그로 확인).

    이 테스트는 Isaac Sim 없이 순수 경로탐색 로직만 검증한다.
    """

    def _make_bundle_with_only_franka(self, tmp_dir: str) -> str:
        import os
        bundled = os.path.join(tmp_dir, "motion_policy_configs")
        franka_dir = os.path.join(bundled, "franka")
        os.makedirs(os.path.join(franka_dir, "rmpflow"), exist_ok=True)
        open(os.path.join(franka_dir, "lula_franka_gen.urdf"), "w").close()
        open(os.path.join(franka_dir, "rmpflow",
                          "franka_rmpflow_common.yaml"), "w").close()
        open(os.path.join(franka_dir, "rmpflow",
                          "robot_descriptor.yaml"), "w").close()
        return bundled

    def test_non_franka_robot_does_not_fall_back_to_franka_files(self):
        import tempfile
        from fdw_sim.visualization.rmpflow_controller import (
            RMPflowController, RMPflowConfig,
        )
        from fdw_sim.visualization.robot_loader import ROBOT_CATALOG

        with tempfile.TemporaryDirectory() as tmp:
            bundled = self._make_bundle_with_only_franka(tmp)
            crx_spec = ROBOT_CATALOG["fanuc_crx10ia"]
            ctrl = RMPflowController(articulation=None, spec=crx_spec,
                                     config=RMPflowConfig())
            # 번들 디렉토리를 직접 가짜로 대체 (Isaac Sim/env var 탐색 우회)
            ctrl._locate_bundled_motion_policy_dir = lambda: bundled

            self.assertIsNone(
                ctrl._locate_rmp_config(),
                "fanuc_crx10ia는 franka의 rmpflow yaml을 받으면 안 된다")
            self.assertIsNone(
                ctrl._locate_urdf(),
                "fanuc_crx10ia는 franka의 urdf를 받으면 안 된다")
            self.assertIsNone(
                ctrl._locate_robot_description(),
                "fanuc_crx10ia는 franka의 robot_description을 받으면 안 된다")

    def test_franka_robot_still_finds_franka_files(self):
        """franka 계열 로봇은 여전히 franka 번들 파일을 정상적으로 찾아야 한다
        (하위호환 확인 — 이번 수정으로 franka 자체가 깨지면 안 됨)."""
        import tempfile
        from fdw_sim.visualization.rmpflow_controller import (
            RMPflowController, RMPflowConfig,
        )
        from fdw_sim.visualization.robot_loader import ROBOT_CATALOG

        with tempfile.TemporaryDirectory() as tmp:
            bundled = self._make_bundle_with_only_franka(tmp)
            franka_spec = ROBOT_CATALOG["franka_panda"]
            ctrl = RMPflowController(articulation=None, spec=franka_spec,
                                     config=RMPflowConfig())
            ctrl._locate_bundled_motion_policy_dir = lambda: bundled

            self.assertIsNotNone(ctrl._locate_rmp_config())
            self.assertIsNotNone(ctrl._locate_urdf())
            self.assertIsNotNone(ctrl._locate_robot_description())


if __name__ == "__main__":
    unittest.main(verbosity=2)
