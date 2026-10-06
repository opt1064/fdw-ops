"""Cartesian controller with explicit command/measurement separation.

Lula may provide kinematic targets when an appropriate robot model is supplied.
The fallback is only an unverified posture demonstration. Neither path verifies
collision safety or physical grasp/attachment. Missing feedback fails closed.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import logging
import math

from fdw_sim.visualization.robot_base_pose import RobotBasePose, read_robot_base_pose

logger = logging.getLogger(__name__)


# =============================================================================
# 데이터 클래스
# =============================================================================
@dataclass
class WeldingPath:
    """용접 경로 — 시작점 → 끝점 + 직선 보간."""
    start: Tuple[float, float, float]            # 월드 좌표
    end: Tuple[float, float, float]
    travel_time_sec: float = 8.0                 # 전체 경로 통과 시간
    approach_height: float = 0.1                 # 시작 전 토치가 위에 떠 있는 높이
    elapsed: float = 0.0
    phase: str = "approach"                      # approach | weld | retreat | done

    def reset(self) -> None:
        self.elapsed = 0.0
        self.phase = "approach"

    def update(self, dt: float) -> Tuple[float, float, float]:
        """매 step 호출 — 현재 토치 목표 위치 반환."""
        self.elapsed += dt
        t_approach = 0.5
        t_weld = self.travel_time_sec
        t_retreat = 0.5
        t_total = t_approach + t_weld + t_retreat

        if self.elapsed <= t_approach:
            self.phase = "approach"
            s = self.elapsed / t_approach
            sm = s * s * (3.0 - 2.0 * s)
            return (
                self.start[0],
                self.start[1],
                self.start[2] + self.approach_height * (1.0 - sm),
            )
        elif self.elapsed <= t_approach + t_weld:
            self.phase = "weld"
            s = (self.elapsed - t_approach) / t_weld
            return (
                self.start[0] + (self.end[0] - self.start[0]) * s,
                self.start[1] + (self.end[1] - self.start[1]) * s,
                self.start[2] + (self.end[2] - self.start[2]) * s,
            )
        elif self.elapsed <= t_total:
            self.phase = "retreat"
            s = (self.elapsed - t_approach - t_weld) / t_retreat
            sm = s * s * (3.0 - 2.0 * s)
            return (
                self.end[0],
                self.end[1],
                self.end[2] + self.approach_height * sm,
            )
        else:
            self.phase = "done"
            return (self.end[0], self.end[1],
                    self.end[2] + self.approach_height)

    def is_active(self) -> bool:
        return self.phase != "done"


# =============================================================================
# 키네매틱스 백엔드 (Lula)
# =============================================================================
class _LulaIKBackend:
    """Lula Kinematics 기반 IK 솔버 래퍼."""

    def __init__(self, robot_description_path: str,
                 urdf_path: str,
                 end_effector_frame: str) -> None:
        from isaacsim.robot_motion.motion_generation.lula import (  # type: ignore
            LulaKinematicsSolver,
        )
        self._solver = LulaKinematicsSolver(
            robot_description_path=robot_description_path,
            urdf_path=urdf_path,
        )
        self.ee_frame = end_effector_frame

    def set_robot_base_pose(self, pose: RobotBasePose) -> None:
        import numpy as np  # type: ignore
        self._solver.set_robot_base_pose(
            np.array(pose.position, dtype=float),
            np.array(pose.orientation, dtype=float),
        )

    def compute(self, target_position: Tuple[float, float, float],
                target_orientation_quat_wxyz: Optional[Tuple[float, float, float, float]] = None,
                warm_start_q: Optional[List[float]] = None,
                ) -> Optional[List[float]]:
        import numpy as np  # type: ignore
        pos = np.array(target_position, dtype=float)
        if target_orientation_quat_wxyz is None:
            # 기본: 토치가 아래를 향함 (Z- 방향)
            target_orientation_quat_wxyz = (0.0, 1.0, 0.0, 0.0)
        rot = np.array(target_orientation_quat_wxyz, dtype=float)
        joint_positions, success = self._solver.compute_inverse_kinematics(
            frame_name=self.ee_frame,
            target_position=pos,
            target_orientation=rot,
            warm_start=(np.array(warm_start_q, dtype=float)
                        if warm_start_q is not None else None),
        )
        if not success:
            return None
        return list(joint_positions)


class _MotionTruthMixin:
    """Shared command/measurement boundary; completion is never grasp evidence.

    A solver result is only a command. Both controllers require measured initial
    joints before sending drive targets and measured TCP reach before reporting
    Cartesian completion. No current backend has validated grasp integration.
    """

    def _init_motion_truth(self) -> None:
        self._commanded_tcp = None
        self._measured_tcp = None
        self._motion_status = "idle"
        self._motion_failure = None
        self._completion_pending = False
        self._settle_elapsed = 0.0
        self._blend_start_q = None
        self._current_q = self._read_measured_joints()

    @staticmethod
    def _finite_vector(value, length=None):
        try:
            result = [float(v) for v in value]
            if not result or (length is not None and len(result) != length):
                return None
            return result if all(math.isfinite(v) for v in result) else None
        except (TypeError, ValueError, OverflowError):
            return None

    def _read_measured_joints(self):
        try:
            getter = getattr(self.articulation, "get_joint_positions", None)
            return self._finite_vector(getter()) if callable(getter) else None
        except Exception:
            return None

    @property
    def supports_physical_manipulation(self) -> bool:
        """No validated gripper/contact/attachment implementation exists here."""
        return False

    def get_capabilities(self) -> dict:
        self._ensure_backend()
        backend = self.backend_name
        return {
            "backend": backend,
            "cartesian_target_solver": backend in ("ik", "rmpflow"),
            "measured_tcp": self.get_measured_tcp_position() is not None,
            "supports_physical_manipulation": False,
            "verified_collision_safety": False,
            "unverified_demo": backend not in ("ik", "rmpflow"),
        }

    def get_commanded_tcp_position(self):
        """Planned world-space point, never evidence of actual robot reach."""
        return self._commanded_tcp

    def get_measured_tcp_position(self):
        """Fresh observed end-effector world position, or None; no fallback."""
        try:
            actual = self._finite_vector(self._read_actual_tcp_from_articulation(), 3)
        except Exception:
            actual = None
        self._measured_tcp = tuple(actual) if actual is not None else None
        return self._measured_tcp

    def has_reached_target(self, target, tolerance_m=0.03) -> bool:
        point = self._finite_vector(target, 3)
        actual = self.get_measured_tcp_position()
        if point is None or actual is None or not math.isfinite(tolerance_m) or tolerance_m < 0:
            return False
        return math.dist(actual, point) <= tolerance_m

    def get_motion_status(self) -> str:
        """idle/running/succeeded/failed/unverified; idle alone is not success."""
        return self._motion_status

    def motion_succeeded(self) -> bool:
        """Measured endpoint completion only; never asserts a successful grasp."""
        return self._motion_status == "succeeded"

    def get_motion_failure(self):
        return self._motion_failure

    def _begin_motion(self) -> None:
        self._motion_status = "running"
        self._motion_failure = None
        self._completion_pending = False
        self._settle_elapsed = 0.0
        self._blend_remaining = 0.0
        self._target_q = None
        self._commanded_tcp = None
        self._current_q = self._read_measured_joints()

    def _fail_motion(self, reason: str) -> None:
        self._motion_status = "failed"
        self._motion_failure = reason
        self._active_path = None
        self._completion_pending = False
        self._blend_remaining = 0.0
        # Replace any outstanding drive target with an observed hold pose. A
        # failed command must not leave a previous trajectory running silently.
        measured = self._read_measured_joints()
        if measured is not None:
            self._apply_joints(measured)
        logger.warning("Motion stopped: %s", reason)

    def _set_target_joints(self, q: List[float], blend_time: float) -> bool:
        target = self._finite_vector(q)
        if target is None:
            self._fail_motion("invalid_joint_target")
            return False
        self._target_q = target  # Retain the attempted command for diagnostics.
        if self._current_q is None:
            self._current_q = self._read_measured_joints()
        if self._current_q is None:
            self._fail_motion("measured_initial_joints_unavailable")
            return False
        if len(self._current_q) != len(target):
            self._fail_motion("joint_count_mismatch_requires_verified_mapping")
            return False
        if not math.isfinite(blend_time) or blend_time < 0:
            self._fail_motion("invalid_blend_time")
            return False
        self._blend_start_q = list(self._current_q)
        self._blend_total = max(1e-3, blend_time)
        self._blend_remaining = self._blend_total
        return True

    def _apply_joints(self, q: List[float]) -> bool:
        """Send drive targets only. Never teleport a physical articulation."""
        if self.articulation is None:
            return False
        try:
            import numpy as np
            arr = np.array(q, dtype=float)
            setter = getattr(self.articulation, "set_joint_position_targets", None)
            if callable(setter):
                setter(arr)
                return True
            apply_action = getattr(self.articulation, "apply_action", None)
            if callable(apply_action):
                from isaacsim.core.utils.types import ArticulationAction
                apply_action(ArticulationAction(joint_positions=arr))
                return True
        except Exception as exc:
            logger.warning("Joint drive command failed: %s", exc)
        return False

    def _update_motion(self, dt: float):
        if not math.isfinite(dt) or dt < 0:
            self._fail_motion("invalid_time_step")
            return None
        try:
            self._ensure_backend()
        except Exception as exc:
            self._fail_motion(f"backend_initialization_failed: {exc}")
            return None
        was_settling = self._completion_pending
        target = None
        if self._active_path is not None:
            try:
                target = self._finite_vector(self._active_path.update(dt), 3)
                if target is None:
                    raise ValueError("trajectory returned an invalid target")
                target = tuple(target)
                self._commanded_tcp = target
                self._last_tcp = target  # Legacy display API, not feedback.
                finished = not self._active_path.is_active()
                if not self._track_target(target):
                    if self._motion_status != "failed":
                        self._fail_motion("target_solver_failed")
                elif finished:
                    self._active_path = None
                    self._completion_pending = True
                    self._settle_elapsed = 0.0
            except Exception as exc:
                self._fail_motion(f"trajectory_or_solver_error: {type(exc).__name__}: {exc}")

        if self._blend_remaining > 0.0 and self._target_q is not None:
            self._blend_remaining = max(0.0, self._blend_remaining - dt)
            k = 1.0 - self._blend_remaining / self._blend_total
            k = k * k * (3.0 - 2.0 * k)
            self._current_q = [a + (b - a) * k
                               for a, b in zip(self._blend_start_q, self._target_q)]
            if not self._apply_joints(self._current_q):
                self._fail_motion("joint_drive_command_unavailable_or_failed")

        actual = self.get_measured_tcp_position()
        if actual is not None:
            self._last_tcp = actual
        if self._completion_pending:
            # Time before the final command was issued is not settling time.
            if was_settling:
                self._settle_elapsed += dt
            if self.backend_name not in ("ik", "rmpflow"):
                if self._blend_remaining <= 0:
                    self._motion_status = "unverified"
                    self._motion_failure = "heuristic_demo_has_no_cartesian_reach_guarantee"
                    self._completion_pending = False
            elif actual is not None and math.dist(actual, self._commanded_tcp) <= 0.03:
                self._motion_status = "succeeded"
                self._completion_pending = False
                self._blend_remaining = 0.0
            elif self._blend_remaining <= 0 and actual is None:
                self._motion_status = "unverified"
                self._motion_failure = "measured_tcp_unavailable"
                self._completion_pending = False
            elif self._settle_elapsed >= 2.0:
                self._fail_motion("measured_tcp_did_not_reach_endpoint")
        elif self._motion_status == "running" and self._active_path is None and self._blend_remaining <= 0:
            # Joint-only go_home completion also requires actual readback.
            measured = self._read_measured_joints()
            if measured is not None and self._target_q is not None and len(measured) == len(self._target_q) and all(
                    abs(a - b) <= 0.03 for a, b in zip(measured, self._target_q)):
                self._motion_status = "succeeded"
            else:
                self._motion_status = "unverified"
                self._motion_failure = "joint_target_not_verified"
        return target


# =============================================================================
# IKController
# =============================================================================
@dataclass
class IKConfig:
    """IK 컨트롤러 옵션."""
    use_lula: bool = True                        # Lula IK 우선 시도
    fallback_joint_waypoints: List[List[float]] = field(default_factory=list)
    waypoint_blend_time: float = 0.5             # waypoint 간 보간 시간
    home_blend_time: float = 1.0


class IKController(_MotionTruthMixin):
    """로봇팔의 엔드 이펙터를 목표 위치로 부드럽게 이동.

    사용 예:

        ctrl = IKController(articulation, spec, config=IKConfig())
        ctrl.go_home()

        # 용접 경로 시작
        path = WeldingPath(start=(5.0, 0.3, 1.0), end=(5.0, -0.3, 1.0),
                            travel_time_sec=10.0)
        ctrl.start_path(path)

        # 매 step:
        ctrl.update(dt)
        if ctrl.is_idle():
            ctrl.go_home()
    """

    def __init__(self, articulation, spec, config: Optional[IKConfig] = None,
                 *,
                 base_position: Tuple[float, float, float] = (5.0, 0.0, 0.0),
                 base_orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
                 ) -> None:
        self.articulation = articulation
        self.spec = spec
        self.config = config or IKConfig()
        self.base_pose = read_robot_base_pose(
            articulation, RobotBasePose(base_position, base_orientation))

        # IK 백엔드 (lazy 로드)
        self._ik_backend: Optional[_LulaIKBackend] = None
        self._ik_disabled: bool = False

        # 현재 경로
        self._active_path: Optional[WeldingPath] = None
        # 현재 적용 중인 joint positions (보간용)
        self._current_q: Optional[List[float]] = None
        # 보간 타깃
        self._target_q: Optional[List[float]] = None
        self._blend_remaining: float = 0.0
        self._blend_total: float = 1.0

        # 현재 추적 중인 TCP(엔드이펙터) 월드 위치 — spark emitter용.
        self._last_tcp: Optional[Tuple[float, float, float]] = None
        self._init_motion_truth()

    # ------------------------------------------------------------------
    def get_tcp_position(self) -> Optional[Tuple[float, float, float]]:
        """Legacy display position, possibly a planned target when FK is absent.

        Never use this value as evidence of reach, attachment, or grasp. Use
        get_measured_tcp_position() and motion status for feedback checks.
        """
        return self._last_tcp

    def _read_actual_tcp_from_articulation(self) -> Optional[Tuple[float, float, float]]:
        """articulation의 end-effector 링크에서 실제 월드 좌표를 FK로 읽어 반환.

        RMPflowController._read_actual_tcp_from_articulation과 동일한 전략
        (best-effort, 예외를 위로 던지지 않음, 실패 시 None).
        """
        art = getattr(self, "articulation", None)
        if art is None:
            return None
        ee_frame = getattr(self.spec, "end_effector_frame", None) if self.spec else None
        if not ee_frame:
            return None

        try:
            for method_name in ("get_link_world_pose",
                                "get_world_pose_of_body",
                                "get_body_world_pose"):
                fn = getattr(art, method_name, None)
                if callable(fn):
                    try:
                        result = fn(ee_frame)
                    except TypeError:
                        continue
                    pos = self._extract_position_from_pose(result)
                    if pos is not None:
                        return pos
        except Exception as e:
            logger.debug("[IK] articulation link FK helper failed: %s", e)

        try:
            prim_path = self._resolve_end_effector_prim_path(art, ee_frame)
            if prim_path:
                pos = self._read_world_translation_from_prim_path(prim_path)
                if pos is not None:
                    return pos
        except Exception as e:
            logger.debug("[IK] USD xform FK read failed: %s", e)

        return None

    def _extract_position_from_pose(self, pose_result) -> Optional[Tuple[float, float, float]]:
        """Isaac Sim FK helper 반환값에서 (x,y,z)만 안전하게 뽑아낸다."""
        if pose_result is None:
            return None
        try:
            candidate = pose_result
            if isinstance(pose_result, (tuple, list)) and len(pose_result) >= 1:
                direct = self._finite_vector(pose_result, 3)
                candidate = direct if direct is not None else pose_result[0]
            position = self._finite_vector(candidate, 3)
            return tuple(position) if position is not None else None
        except Exception:
            return None

    def _resolve_end_effector_prim_path(self, art, ee_frame: str) -> Optional[str]:
        """articulation의 root prim_path + ee_frame 으로 ee prim path 추정."""
        root = None
        for attr in ("prim_path", "_prim_path"):
            v = getattr(art, attr, None)
            if isinstance(v, str) and v:
                root = v
                break
        if root is None:
            return None

        simple = f"{root.rstrip('/')}/{ee_frame}"

        stage = self._get_usd_stage(art)
        if stage is None:
            return simple

        try:
            from pxr import Sdf  # type: ignore
            if stage.GetPrimAtPath(Sdf.Path(simple)).IsValid():
                return simple
        except Exception:
            pass

        try:
            root_prim = stage.GetPrimAtPath(root)
            if not root_prim.IsValid():
                return simple
            stack = [root_prim]
            while stack:
                p = stack.pop()
                if p.GetName() == ee_frame:
                    return str(p.GetPath())
                stack.extend(p.GetChildren())
        except Exception:
            pass
        return simple

    def _get_usd_stage(self, art):
        """articulation으로부터 USD stage 핸들 best-effort 추출."""
        for attr in ("_stage", "stage"):
            s = getattr(art, attr, None)
            if s is not None:
                return s
        try:
            import omni.usd  # type: ignore
            ctx = omni.usd.get_context()
            if ctx is not None:
                return ctx.get_stage()
        except Exception:
            return None
        return None

    def _read_world_translation_from_prim_path(self, prim_path: str) -> Optional[Tuple[float, float, float]]:
        """USD prim의 world translation을 XformCache로 읽음."""
        try:
            from pxr import UsdGeom  # type: ignore
        except Exception:
            return None
        stage = self._get_usd_stage(self.articulation)
        if stage is None:
            return None
        try:
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return None
            xf_cache = UsdGeom.XformCache()
            world = xf_cache.GetLocalToWorldTransform(prim)
            t = world.ExtractTranslation()
            return (float(t[0]), float(t[1]), float(t[2]))
        except Exception as e:
            logger.debug("[IK] XformCache read failed for %s: %s", prim_path, e)
            return None

    # ------------------------------------------------------------------
    def _ensure_backend(self) -> None:
        if self._ik_backend is not None or self._ik_disabled:
            return
        if not self.config.use_lula:
            self._ik_disabled = True
            return
        # Lula는 robot_description.yaml + urdf가 있어야 동작.
        # 대부분 사용자가 별도 구성해야 하므로 본 컨트롤러는 fallback 우선.
        # (Level 2.2에서 Lula 통합)
        self._ik_disabled = True

    # ------------------------------------------------------------------
    @property
    def backend_name(self) -> str:
        return "ik" if self._ik_backend is not None else "heuristic"

    def go_home(self) -> None:
        self._active_path = None
        self._begin_motion()
        home = getattr(self.spec, "home_joint_positions", None)
        if not home:
            self._fail_motion("validated_home_configuration_unavailable")
            return
        self._set_target_joints(home, blend_time=self.config.home_blend_time)

    def start_path(self, path: WeldingPath) -> None:
        self._begin_motion()
        try:
            path.reset()
            self._active_path = path
        except Exception as exc:
            self._fail_motion(f"trajectory_reset_failed: {exc}")

    def is_idle(self) -> bool:
        return self._active_path is None and not self._completion_pending and self._blend_remaining <= 0.0

    def get_phase(self) -> str:
        if self._motion_status in ("failed", "unverified"):
            return self._motion_status
        if self._active_path is not None:
            return self._active_path.phase
        return "idle" if self.is_idle() else "moving_home"

    def update(self, dt: float) -> Optional[Tuple[float, float, float]]:
        return self._update_motion(dt)

    # ------------------------------------------------------------------
    def _track_target(self, target_pos: Tuple[float, float, float]) -> bool:
        """Solve a target; an installed solver failure never degrades to demo IK."""
        self.base_pose = read_robot_base_pose(self.articulation, self.base_pose)
        if self._ik_backend is not None:
            try:
                self._ik_backend.set_robot_base_pose(self.base_pose)
                q = self._ik_backend.compute(target_pos, warm_start_q=(self._current_q if self._current_q is not None else self._target_q))
            except Exception as exc:
                self._fail_motion(f"ik_solver_error: {exc}")
                return False
            if q is None:
                self._fail_motion("ik_no_solution")
                return False
            return self._set_target_joints(q, self.config.waypoint_blend_time)

        # This is an unverified posture demo, not analytic IK or collision avoidance.
        home = getattr(self.spec, "home_joint_positions", None)
        if not home or len(home) < 7:
            self._fail_motion("heuristic_reference_configuration_unavailable")
            return False
        base = list(home)
        dx, dy, _ = self.base_pose.to_local(target_pos)
        yaw_offset = math.atan2(dy, dx) * 0.5
        base[0] += max(-0.8, min(0.8, yaw_offset))
        base[1] += 0.4
        base[3] -= 0.5
        return self._set_target_joints(base, self.config.waypoint_blend_time)
