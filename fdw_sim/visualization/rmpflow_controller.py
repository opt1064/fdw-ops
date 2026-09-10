"""RMPflowController — Reactive Motion Policy(RMP)flow 기반 충돌회피 모션 컨트롤러.

Level 2.2의 핵심: Level 2.1의 IKController가 단순 joint 보간만 했다면,
이 컨트롤러는 다음 3가지 motion backend를 지원한다.

    1) "rmpflow"  — NVIDIA Lula RMPflow (정식 구현, robot description 필요)
    2) "ik"       — 기존 Lula IK + joint 보간 (Level 2.1 호환)
    3) "heuristic"— collision-sphere 회피 + ±법선 보정 (의존성 0, 항상 동작)

용접 셀에서 토치가 부품 위 경로를 따라 움직이되, 부품/벤치/이웃 셀과
충돌하지 않도록 동적으로 궤적을 조정한다.

설계 원칙:
- IKController와 동일한 외부 인터페이스 유지 (start_path/update/go_home/is_idle)
- backend 선택은 lazy — Isaac Sim 런타임에서 RMP config가 발견되지 않으면
  자동으로 heuristic으로 fallback
- 충돌 obstacle은 add_obstacle(center, radius)로 외부 등록 가능
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import logging
import math

logger = logging.getLogger(__name__)


# =============================================================================
# 데이터 모델
# =============================================================================
@dataclass
class CollisionSphere:
    """충돌 회피용 단일 구체 obstacle.

    가장 가벼운 형태의 obstacle 표현 — 부품/벤치/주변 셀 모두 sphere로
    근사한다. Lula RMPflow는 sphere 기반 SDF를 그대로 사용하므로
    backend 전환 시에도 동일 데이터 구조를 쓴다.
    """
    name: str
    center: Tuple[float, float, float]
    radius: float
    # 작업대 등 정적 obstacle은 static=True (RMPflow 캐시 가능)
    static: bool = True


@dataclass
class RMPflowConfig:
    """RMPflow 컨트롤러 옵션."""
    # backend 우선순위 — 위에서 아래로 시도
    preferred_backend: str = "auto"          # "auto" | "rmpflow" | "ik" | "heuristic"

    # 안전 한계
    max_step_distance_m: float = 0.05        # 한 step에서 TCP 최대 이동 거리
    obstacle_clearance_m: float = 0.08       # 토치-obstacle 최소 간격 목표
    obstacle_repulsion_gain: float = 0.4     # 회피 가속도 게인

    # joint 보간 (heuristic/ik 백엔드 공통)
    waypoint_blend_time: float = 0.4
    home_blend_time: float = 1.0

    # RMPflow YAML 자동 탐색 후보
    rmp_config_candidates: List[str] = field(default_factory=lambda: [
        # Isaac Sim 5.1 표준 위치
        "isaacsim.robot_motion.motion_generation/motion_policy_configs/"
        "franka/rmpflow/franka_rmpflow_common.yaml",
        # FrankaRobotics 신구조
        "FrankaRobotics/FrankaPanda/motion_policy_configs/franka_rmpflow_common.yaml",
    ])


# =============================================================================
# Backend 인터페이스 — 모든 backend는 동일 API를 구현
# =============================================================================
class _MotionBackendBase:
    """모션 backend 베이스 — start/step/stop 인터페이스."""
    name: str = "base"

    def reset(self, current_q: Optional[List[float]] = None) -> None:
        pass

    def set_target(self,
                   target_pos: Tuple[float, float, float],
                   warm_start_q: Optional[List[float]] = None,
                   obstacles: Optional[List[CollisionSphere]] = None,
                   ) -> Optional[List[float]]:
        """단일 step 호출 → joint positions 반환 (또는 None)."""
        raise NotImplementedError


class _HeuristicBackend(_MotionBackendBase):
    """의존성 0 휴리스틱 backend — IK 없이 그럴듯한 자세 + obstacle 회피.

    - base joint(yaw)를 TCP 목표의 x-y atan2로 회전
    - shoulder/elbow를 살짝 굽혀 토치가 아래를 향하도록
    - obstacle에 너무 가까우면 TCP 목표 자체를 법선 방향으로 push-out
    """
    name = "heuristic"

    def __init__(self, spec, config: RMPflowConfig,
                 base_xy: Tuple[float, float] = (5.0, 0.0)) -> None:
        self.spec = spec
        self.config = config
        self.base_xy = base_xy
        self._home_q: List[float] = list(spec.home_joint_positions) \
            if spec and spec.home_joint_positions else []

    def reset(self, current_q: Optional[List[float]] = None) -> None:
        pass

    # ----------------------------------------------------------------------
    def _apply_obstacle_repulsion(
        self,
        target_pos: Tuple[float, float, float],
        obstacles: List[CollisionSphere],
    ) -> Tuple[float, float, float]:
        """TCP 목표를 obstacle로부터 밀어내는 보정."""
        tx, ty, tz = target_pos
        clearance = self.config.obstacle_clearance_m
        gain = self.config.obstacle_repulsion_gain

        for obs in obstacles:
            cx, cy, cz = obs.center
            dx, dy, dz = tx - cx, ty - cy, tz - cz
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            threshold = obs.radius + clearance
            if dist >= threshold or dist < 1e-6:
                continue
            # 너무 가까움 — 법선 방향으로 push
            push = (threshold - dist) * gain
            nx, ny, nz = dx / dist, dy / dist, dz / dist
            tx += nx * push
            ty += ny * push
            tz += nz * push
        return (tx, ty, tz)

    def set_target(self,
                   target_pos: Tuple[float, float, float],
                   warm_start_q: Optional[List[float]] = None,
                   obstacles: Optional[List[CollisionSphere]] = None,
                   ) -> Optional[List[float]]:
        if not self._home_q or len(self._home_q) < 4:
            return None

        # 1) Obstacle repulsion으로 TCP 목표 보정
        if obstacles:
            target_pos = self._apply_obstacle_repulsion(target_pos, obstacles)

        tx, ty, tz = target_pos
        bx, by = self.base_xy

        # 2) base yaw — atan2(dx, dy) 형태로 Franka base 회전
        dx = tx - bx
        dy = ty - by
        yaw = math.atan2(dy, dx if abs(dx) > 1e-3 else 1e-3) * 0.6
        yaw = max(-1.2, min(1.2, yaw))

        # 3) 거리에 비례한 shoulder/elbow 굽힘
        horiz = math.sqrt(dx * dx + dy * dy)
        # 0.4 ~ 1.2 m 범위에서 정규화
        reach = max(0.4, min(1.2, horiz))
        reach_t = (reach - 0.4) / 0.8
        shoulder_bend = 0.3 + 0.4 * reach_t      # 더 멀수록 더 펴짐
        elbow_bend = -0.4 - 0.5 * (1.0 - reach_t)  # 가까울수록 더 굽힘

        # 4) 토치를 아래로 향하게 wrist 보정
        q = list(self._home_q)
        q[0] = self._home_q[0] + yaw
        if len(q) >= 2:
            q[1] = self._home_q[1] + shoulder_bend
        if len(q) >= 4:
            q[3] = self._home_q[3] + elbow_bend
        # wrist (joint 5) 살짝 회전해서 동적 효과
        if len(q) >= 6:
            q[5] = self._home_q[5] + 0.2 * math.sin(yaw * 2.0)

        return q


class _LulaIKBackend(_MotionBackendBase):
    """Lula 기반 정확한 IK — robot_description / urdf 필요."""
    name = "ik"

    def __init__(self,
                 robot_description_path: str,
                 urdf_path: str,
                 end_effector_frame: str,
                 config: RMPflowConfig) -> None:
        from isaacsim.robot_motion.motion_generation.lula import (  # type: ignore
            LulaKinematicsSolver,
        )
        self._solver = LulaKinematicsSolver(
            robot_description_path=robot_description_path,
            urdf_path=urdf_path,
        )
        self.ee_frame = end_effector_frame
        self.config = config

    def set_target(self,
                   target_pos: Tuple[float, float, float],
                   warm_start_q: Optional[List[float]] = None,
                   obstacles: Optional[List[CollisionSphere]] = None,
                   ) -> Optional[List[float]]:
        import numpy as np  # type: ignore
        pos = np.array(target_pos, dtype=float)
        # 토치가 아래를 향함
        rot = np.array((0.0, 1.0, 0.0, 0.0), dtype=float)
        if warm_start_q is not None:
            self._solver.set_warm_start(np.array(warm_start_q, dtype=float))
        joint_positions, success = self._solver.compute_inverse_kinematics(
            frame_name=self.ee_frame,
            target_position=pos,
            target_orientation=rot,
        )
        if not success:
            return None
        return list(joint_positions)


class _RmpFlowBackend(_MotionBackendBase):
    """Lula RMPflow — 충돌회피 reactive motion policy.

    Isaac Sim 5.x의 isaacsim.robot_motion.motion_generation.RmpFlow를 wrap.
    동적으로 obstacle을 add/update 할 수 있어 부품/벤치 회피에 적합.
    """
    name = "rmpflow"

    def __init__(self,
                 rmp_config_path: str,
                 urdf_path: str,
                 robot_description_path: str,
                 end_effector_frame: str,
                 articulation,
                 config: RMPflowConfig) -> None:
        from isaacsim.robot_motion.motion_generation import (  # type: ignore
            ArticulationMotionPolicy,
            RmpFlow,
        )
        self._RmpFlow = RmpFlow
        self._policy = RmpFlow(
            robot_description_path=robot_description_path,
            rmpflow_config_path=rmp_config_path,
            urdf_path=urdf_path,
            end_effector_frame_name=end_effector_frame,
            maximum_substep_size=0.0034,
        )
        # Articulation에 연결
        self._articulation_policy = ArticulationMotionPolicy(
            robot_articulation=articulation,
            motion_policy=self._policy,
        )
        self.config = config
        self._obstacle_keys: dict = {}    # name -> obstacle handle

    def _sync_obstacles(self, obstacles: Optional[List[CollisionSphere]]) -> None:
        if not obstacles:
            return
        import numpy as np  # type: ignore
        for obs in obstacles:
            key = obs.name
            if key in self._obstacle_keys:
                # 이미 추가됨 — RMPflow는 dynamic obstacle update를 지원
                handle = self._obstacle_keys[key]
                try:
                    handle.set_world_pose(
                        np.array(obs.center, dtype=float),
                        np.array([1, 0, 0, 0], dtype=float),
                    )
                except Exception:
                    pass
                continue
            try:
                # Sphere obstacle 등록
                from isaacsim.core.api.objects import VisualSphere  # type: ignore
                handle = VisualSphere(
                    prim_path=f"/World/FDW/Obstacles/{key}",
                    name=key,
                    position=np.array(obs.center, dtype=float),
                    radius=float(obs.radius),
                    visible=False,
                )
                self._policy.add_obstacle(handle)
                self._obstacle_keys[key] = handle
            except Exception as e:
                logger.debug("[RMP] add_obstacle %s failed: %s", key, e)

    def set_target(self,
                   target_pos: Tuple[float, float, float],
                   warm_start_q: Optional[List[float]] = None,
                   obstacles: Optional[List[CollisionSphere]] = None,
                   ) -> Optional[List[float]]:
        import numpy as np  # type: ignore
        self._sync_obstacles(obstacles)
        self._policy.set_end_effector_target(
            target_position=np.array(target_pos, dtype=float),
            target_orientation=np.array([0.0, 1.0, 0.0, 0.0], dtype=float),
        )
        # 한 step 적용 — RMPflow는 articulation에 직접 명령을 쓰므로
        # joint position 리턴이 아니라 articulation 상태가 바뀜
        try:
            action = self._articulation_policy.get_next_articulation_action(
                step_size=1.0 / 60.0,
            )
            joint_positions = getattr(action, "joint_positions", None)
            if joint_positions is None:
                return None
            return list(np.asarray(joint_positions).flatten())
        except Exception as e:
            logger.debug("[RMP] step failed: %s", e)
            return None


# =============================================================================
# Path (WeldingPath는 ik_controller.WeldingPath와 동일 의미 — 재사용)
# =============================================================================
@dataclass
class _PathState:
    """RMP 컨트롤러 내부의 경로 상태."""
    start: Tuple[float, float, float]
    end: Tuple[float, float, float]
    travel_time_sec: float = 8.0
    approach_height: float = 0.1
    elapsed: float = 0.0
    phase: str = "approach"

    def reset(self) -> None:
        self.elapsed = 0.0
        self.phase = "approach"

    def update(self, dt: float) -> Tuple[float, float, float]:
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
        if self.elapsed <= t_approach + t_weld:
            self.phase = "weld"
            s = (self.elapsed - t_approach) / t_weld
            return (
                self.start[0] + (self.end[0] - self.start[0]) * s,
                self.start[1] + (self.end[1] - self.start[1]) * s,
                self.start[2] + (self.end[2] - self.start[2]) * s,
            )
        if self.elapsed <= t_total:
            self.phase = "retreat"
            s = (self.elapsed - t_approach - t_weld) / t_retreat
            sm = s * s * (3.0 - 2.0 * s)
            return (
                self.end[0],
                self.end[1],
                self.end[2] + self.approach_height * sm,
            )
        self.phase = "done"
        return (self.end[0], self.end[1],
                self.end[2] + self.approach_height)

    def is_active(self) -> bool:
        return self.phase != "done"


# =============================================================================
# RMPflowController — 외부 API (IKController와 호환)
# =============================================================================
class RMPflowController:
    """충돌회피 reactive motion controller.

    사용 예 (Level 2.2):

        ctrl = RMPflowController(articulation, spec,
                                  base_xy=(5.0, -0.3),
                                  config=RMPflowConfig(preferred_backend="auto"))
        ctrl.add_obstacle(CollisionSphere("part", (5.0, 0.0, 0.95), 0.15))
        ctrl.add_obstacle(CollisionSphere("bench", (5.0, 0.0, 0.4), 0.9))

        ctrl.start_path(start=(4.7, -0.25, 1.0), end=(4.7, 0.25, 1.0),
                        travel_time_sec=8.0)
        # 매 step
        while not ctrl.is_idle():
            tcp = ctrl.update(dt)

        ctrl.go_home()
    """

    def __init__(self,
                 articulation,
                 spec,
                 base_xy: Tuple[float, float] = (5.0, 0.0),
                 config: Optional[RMPflowConfig] = None) -> None:
        self.articulation = articulation
        self.spec = spec
        self.base_xy = base_xy
        self.config = config or RMPflowConfig()

        # backend (lazy 초기화 — Isaac Sim 미설치 환경에서도 import는 가능해야 함)
        self._backend: Optional[_MotionBackendBase] = None
        self._backend_ready: bool = False

        # 경로/상태
        self._active_path: Optional[_PathState] = None
        self._obstacles: List[CollisionSphere] = []

        # joint 보간 상태
        self._current_q: Optional[List[float]] = None
        self._target_q: Optional[List[float]] = None
        self._blend_remaining: float = 0.0
        self._blend_total: float = 1.0

        # 마지막 TCP 위치 (외부에서 spark emitter 등이 추적)
        self._last_tcp: Optional[Tuple[float, float, float]] = None
        self._last_phase: str = "idle"

    # ------------------------------------------------------------------
    # backend 선택 — 처음 update 호출 시 결정
    # ------------------------------------------------------------------
    def _ensure_backend(self) -> None:
        if self._backend_ready:
            return
        self._backend_ready = True

        pref = self.config.preferred_backend.lower()

        # 1) RMPflow 시도
        if pref in ("auto", "rmpflow"):
            rmp = self._try_init_rmpflow()
            if rmp is not None:
                self._backend = rmp
                logger.info("[RMP] backend = rmpflow (Lula reactive motion policy)")
                return
            if pref == "rmpflow":
                logger.warning("[RMP] preferred_backend=rmpflow but config "
                               "not found — falling back to heuristic")

        # 2) Lula IK 시도
        if pref in ("auto", "ik"):
            ik = self._try_init_ik()
            if ik is not None:
                self._backend = ik
                logger.info("[RMP] backend = ik (Lula kinematics solver)")
                return
            if pref == "ik":
                logger.warning("[RMP] preferred_backend=ik but solver unavailable "
                               "— falling back to heuristic")

        # 3) Heuristic 항상 동작
        self._backend = _HeuristicBackend(self.spec, self.config,
                                           base_xy=self.base_xy)
        logger.info("[RMP] backend = heuristic "
                    "(no Isaac motion-policy deps; uses collision-sphere repulsion)")

    def _try_init_rmpflow(self) -> Optional[_MotionBackendBase]:
        """isaacsim.robot_motion.motion_generation.RmpFlow 시도."""
        try:
            from isaacsim.robot_motion.motion_generation import (  # type: ignore # noqa: F401
                RmpFlow,
            )
        except Exception:
            return None
        # config 경로 자동 탐색
        import os
        rmp_yaml = self._locate_rmp_config()
        urdf = self._locate_urdf()
        robot_description = self._locate_robot_description(
            hint_dir=os.path.dirname(rmp_yaml) if rmp_yaml else None)
        if not rmp_yaml or not urdf or not robot_description:
            logger.info("[RMP] rmpflow config/urdf/robot_description not found "
                        "(rmp=%s, urdf=%s, robot_description=%s)",
                        rmp_yaml, urdf, robot_description)
            return None
        try:
            return _RmpFlowBackend(
                rmp_config_path=rmp_yaml,
                urdf_path=urdf,
                robot_description_path=robot_description,
                end_effector_frame=self.spec.end_effector_frame,
                articulation=self.articulation,
                config=self.config,
            )
        except Exception as e:
            logger.warning("[RMP] rmpflow init failed: %s", e)
            return None

    def _try_init_ik(self) -> Optional[_MotionBackendBase]:
        try:
            from isaacsim.robot_motion.motion_generation.lula import (  # type: ignore # noqa: F401
                LulaKinematicsSolver,
            )
        except Exception:
            return None
        urdf = self._locate_urdf()
        rd = self._locate_robot_description()
        if not urdf or not rd:
            return None
        try:
            return _LulaIKBackend(
                robot_description_path=rd,
                urdf_path=urdf,
                end_effector_frame=self.spec.end_effector_frame,
                config=self.config,
            )
        except Exception as e:
            logger.debug("[RMP] ik init failed: %s", e)
            return None

    def _locate_bundled_motion_policy_dir(self) -> Optional[str]:
        """isaacsim.robot_motion.motion_generation 확장이 함께 배포하는
        motion_policy_configs 디렉토리를 찾는다 (Isaac Sim 5.1 pip wheel 설치 기준
        — Thor 실측: .../site-packages/isaacsim/exts/isaacsim.robot_motion.
        motion_generation/motion_policy_configs/franka/{lula_franka_gen.urdf,
        rmpflow/franka_rmpflow_common.yaml, rmpflow/robot_descriptor.yaml}).

        conda 환경/사용자 홈 경로를 하드코딩하지 않도록 `isaacsim` 패키지 자체의
        설치 위치를 기준으로 상대 경로를 계산한다.
        """
        try:
            import importlib.util
            spec = importlib.util.find_spec("isaacsim")
        except Exception:
            return None
        if spec is None or not spec.submodule_search_locations:
            return None
        import os
        isaacsim_root = list(spec.submodule_search_locations)[0]
        candidate = os.path.join(
            isaacsim_root, "exts", "isaacsim.robot_motion.motion_generation",
            "motion_policy_configs",
        )
        return candidate if os.path.isdir(candidate) else None

    def _locate_rmp_config(self) -> Optional[str]:
        import os
        # 환경변수 우선
        env = os.environ.get("FDW_RMPFLOW_YAML")
        if env and os.path.isfile(env):
            return env
        # 표준 후보들
        for cand in self.config.rmp_config_candidates:
            for root in [
                "/home/isweon/isaac_assets",
                os.path.expanduser("~/isaac_assets"),
                os.path.expanduser("~/.local/share/ov/data/assets"),
                "/opt/nvidia/isaac-sim-assets",
            ]:
                p = os.path.join(root, cand)
                if os.path.isfile(p):
                    return p
        # Isaac Sim이 자체 번들하는 예제 RMPflow 설정 (Thor 실측으로 확인된 경로)
        bundled = self._locate_bundled_motion_policy_dir()
        if bundled:
            p = os.path.join(bundled, "franka", "rmpflow",
                              "franka_rmpflow_common.yaml")
            if os.path.isfile(p):
                return p
        return None

    def _locate_urdf(self) -> Optional[str]:
        import os
        env = os.environ.get("FDW_FRANKA_URDF")
        if env and os.path.isfile(env):
            return env
        # franka urdf는 일반적으로 isaacsim_assets에 포함
        for root in [
            os.path.expanduser("~/isaac_assets/Isaac/Robots/FrankaRobotics/FrankaPanda"),
        ]:
            for fname in ("franka.urdf", "panda.urdf"):
                p = os.path.join(root, fname)
                if os.path.isfile(p):
                    return p
        # Isaac Sim이 자체 번들하는 Lula URDF (Thor 실측으로 확인된 경로)
        bundled = self._locate_bundled_motion_policy_dir()
        if bundled:
            p = os.path.join(bundled, "franka", "lula_franka_gen.urdf")
            if os.path.isfile(p):
                return p
        return None

    def _locate_robot_description(self, hint_dir: Optional[str] = None) -> Optional[str]:
        """Lula robot description(kinematics 제약 정의) yaml을 찾는다.

        NVIDIA 배포본마다 파일명이 다를 수 있다 (Thor 실측: `robot_descriptor.yaml`
        — 예전 코드는 이걸 `franka_rmpflow_common.yaml`에서 문자열 치환으로
        `franka_robot_description.yaml`을 추측했는데 실제 이름과 달라 항상
        실패했었다). 여러 흔한 이름 후보를 hint_dir(보통 rmpflow yaml이 있는
        디렉토리) 및 번들 경로에서 직접 찾는다.
        """
        import os
        env = os.environ.get("FDW_FRANKA_ROBOT_DESCRIPTION")
        if env and os.path.isfile(env):
            return env

        candidate_names = (
            "robot_descriptor.yaml",
            "robot_description.yaml",
            "franka_robot_description.yaml",
        )
        search_dirs = []
        if hint_dir:
            search_dirs.append(hint_dir)
        bundled = self._locate_bundled_motion_policy_dir()
        if bundled:
            search_dirs.append(os.path.join(bundled, "franka", "rmpflow"))

        for d in search_dirs:
            for name in candidate_names:
                p = os.path.join(d, name)
                if os.path.isfile(p):
                    return p
        return None

    # ------------------------------------------------------------------
    # obstacle API
    # ------------------------------------------------------------------
    def add_obstacle(self, obs: CollisionSphere) -> None:
        self._obstacles.append(obs)
        logger.debug("[RMP] obstacle '%s' registered @ %s r=%.2f",
                     obs.name, obs.center, obs.radius)

    def clear_obstacles(self) -> None:
        self._obstacles.clear()

    # ------------------------------------------------------------------
    # 경로 API (IKController 호환)
    # ------------------------------------------------------------------
    def start_path(self,
                   start: Tuple[float, float, float],
                   end: Tuple[float, float, float],
                   travel_time_sec: float = 8.0,
                   approach_height: float = 0.1) -> None:
        self._active_path = _PathState(
            start=start, end=end,
            travel_time_sec=travel_time_sec,
            approach_height=approach_height,
        )
        self._active_path.reset()
        self._last_phase = "approach"

    def go_home(self) -> None:
        if not self.spec or not self.spec.home_joint_positions:
            return
        self._set_target_joints(self.spec.home_joint_positions,
                                blend_time=self.config.home_blend_time)
        self._last_phase = "home"

    def is_idle(self) -> bool:
        return self._active_path is None and self._blend_remaining <= 0.0

    def get_phase(self) -> str:
        if self._active_path is not None:
            return self._active_path.phase
        return self._last_phase

    def get_tcp_position(self) -> Optional[Tuple[float, float, float]]:
        """현재 추적 중인 TCP(엔드이펙터) 월드 위치 — spark emitter용.

        가능하면 articulation의 실제 FK 결과(`_read_actual_tcp_from_articulation`)를
        반영한 위치이며, FK를 읽을 수 없는 환경(테스트, USD 로드 실패)에서는
        커맨드된 target 위치로 fallback 한다.
        """
        return self._last_tcp

    def _read_actual_tcp_from_articulation(self) -> Optional[Tuple[float, float, float]]:
        """articulation의 end-effector 링크에서 실제 월드 좌표를 FK로 읽어 반환.

        반환 우선순위:
            1) SingleArticulation에 end-effector 링크 접근자가 있으면 그쪽 사용
               (`get_link_world_pose` / `get_world_pose` on `end_effector` 핸들)
            2) USD stage에서 `<robot_prim>/<end_effector_frame>` 의 xform을
               XformCache로 읽기
            3) 위 두 가지가 모두 실패하면 None (호출자가 target_pos로 fallback)

        주의:
            - 동작은 모두 best-effort. 예외가 나도 절대 위로 던지지 않는다.
            - heuristic 백엔드는 실제 IK가 아니므로, 이 함수가 None을 반환하면
              spark emitter는 "이상적인 target" 좌표에 붙게 된다 (기존 동작).
        """
        art = getattr(self, "articulation", None)
        if art is None:
            return None
        ee_frame = getattr(self.spec, "end_effector_frame", None) if self.spec else None
        if not ee_frame:
            return None

        # --- 시도 1: Isaac Sim core API ----------------------------------
        # 일부 SingleArticulation 구현은 link 단위 FK helper를 제공한다.
        # API 가용 여부가 버전마다 다르므로 모두 best-effort.
        try:
            # 1a) get_link_world_pose / get_world_pose(link_name=...) 패턴
            for method_name in ("get_link_world_pose",
                                "get_world_pose_of_body",
                                "get_body_world_pose"):
                fn = getattr(art, method_name, None)
                if callable(fn):
                    try:
                        result = fn(ee_frame)
                    except TypeError:
                        # 인자 시그니처가 다른 경우 — 다음 후보로
                        continue
                    pos = self._extract_position_from_pose(result)
                    if pos is not None:
                        return pos
        except Exception as e:
            logger.debug("[RMP] articulation link FK helper failed: %s", e)

        # --- 시도 2: USD XformCache 로 ee 링크 prim의 world transform 직접 읽기
        try:
            prim_path = self._resolve_end_effector_prim_path(art, ee_frame)
            if prim_path:
                pos = self._read_world_translation_from_prim_path(prim_path)
                if pos is not None:
                    return pos
        except Exception as e:
            logger.debug("[RMP] USD xform FK read failed: %s", e)

        return None

    def _extract_position_from_pose(self, pose_result) -> Optional[Tuple[float, float, float]]:
        """Isaac Sim FK helper 반환값에서 (x,y,z)만 안전하게 뽑아낸다.

        반환 형태가 (pos, orient), [pos, orient], np.ndarray 등 다양해서
        모두 처리한다.
        """
        if pose_result is None:
            return None
        try:
            # 보통 (pos, orient) tuple
            candidate = pose_result
            if isinstance(pose_result, (tuple, list)) and len(pose_result) >= 1:
                candidate = pose_result[0]
            # numpy array / list / Gf.Vec3*
            x = float(candidate[0])
            y = float(candidate[1])
            z = float(candidate[2])
            return (x, y, z)
        except Exception:
            return None

    def _resolve_end_effector_prim_path(self, art, ee_frame: str) -> Optional[str]:
        """articulation의 root prim_path + ee_frame 으로 ee prim path 추정.

        USD 계층 구조상 ee_frame은 robot root 아래 어딘가에 존재한다.
        가장 흔한 패턴: `<robot_root>/<ee_frame>`.
        실제 계층이 다른 경우(`<root>/panda_link7/panda_hand`)는
        stage 탐색으로 보강한다.
        """
        # 1) articulation에서 root prim_path 추출
        root = None
        for attr in ("prim_path", "_prim_path"):
            v = getattr(art, attr, None)
            if isinstance(v, str) and v:
                root = v
                break
        if root is None:
            return None

        # 2) 가장 단순한 후보 — root/<ee_frame>
        simple = f"{root.rstrip('/')}/{ee_frame}"

        # 3) stage가 있으면 simple 후보가 valid 한지 확인, 아니면 트리 탐색
        stage = self._get_usd_stage(art)
        if stage is None:
            # stage 접근 불가 — 단순 후보를 그대로 반환 (XformCache가 None 처리)
            return simple

        try:
            from pxr import Sdf  # type: ignore
            if stage.GetPrimAtPath(Sdf.Path(simple)).IsValid():
                return simple
        except Exception:
            pass

        # 4) DFS 로 ee_frame 이름과 일치하는 prim 찾기
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
        # SingleArticulation에 stage 멤버가 있을 수 있음
        for attr in ("_stage", "stage"):
            s = getattr(art, attr, None)
            if s is not None:
                return s
        # omni.usd가 있으면 컨텍스트에서 가져오기
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
            logger.debug("[RMP] XformCache read failed for %s: %s", prim_path, e)
            return None

    # ------------------------------------------------------------------
    # 매 step 호출
    # ------------------------------------------------------------------
    def update(self, dt: float) -> Optional[Tuple[float, float, float]]:
        self._ensure_backend()

        target_pos: Optional[Tuple[float, float, float]] = None

        if self._active_path is not None:
            target_pos = self._active_path.update(dt)
            # 우선 commanded target으로 fallback 값을 채워둔다 — FK가 실패해도
            # spark emitter가 None TCP로 끊기지 않도록.
            self._last_tcp = target_pos
            if not self._active_path.is_active():
                self._active_path = None
                self.go_home()
            else:
                self._track_target(target_pos)
                # joint를 갱신한 뒤(=실제 articulation pose가 업데이트된 뒤)
                # end-effector의 실제 월드 좌표를 FK로 읽어 _last_tcp를 덮어쓴다.
                actual = self._read_actual_tcp_from_articulation()
                if actual is not None:
                    self._last_tcp = actual

        # joint 보간 진행
        if self._blend_remaining > 0.0 and self._target_q is not None:
            self._blend_remaining = max(0.0, self._blend_remaining - dt)
            if self._current_q is None:
                self._current_q = list(self._target_q)
            else:
                k = 1.0 - (self._blend_remaining / self._blend_total)
                k = max(0.0, min(1.0, k))
                # smoothstep
                k = k * k * (3.0 - 2.0 * k)
                n = min(len(self._current_q), len(self._target_q))
                for i in range(n):
                    self._current_q[i] = (
                        self._current_q[i] * (1.0 - k)
                        + self._target_q[i] * k
                    )
            self._apply_joints(self._current_q)

        return target_pos

    def _track_target(self, target_pos: Tuple[float, float, float]) -> None:
        """현재 TCP 목표 → joint positions 변환 (backend별)."""
        if self._backend is None:
            return
        q = self._backend.set_target(
            target_pos,
            warm_start_q=self._current_q,
            obstacles=self._obstacles,
        )
        if q is not None:
            self._set_target_joints(q,
                                     blend_time=self.config.waypoint_blend_time)

    def _set_target_joints(self, q: List[float], blend_time: float) -> None:
        self._target_q = list(q)
        self._blend_total = max(1e-3, blend_time)
        self._blend_remaining = self._blend_total
        if self._current_q is None:
            self._current_q = list(q)

    def _apply_joints(self, q: List[float]) -> None:
        """Articulation에 joint 적용 — IKController와 동일 로직."""
        if self.articulation is None:
            return
        try:
            import numpy as np  # type: ignore
            arr = np.array(q, dtype=float)
            if hasattr(self.articulation, "set_joint_positions"):
                self.articulation.set_joint_positions(arr)
            elif hasattr(self.articulation, "apply_action"):
                from omni.isaac.core.utils.types import ArticulationAction  # type: ignore
                self.articulation.apply_action(
                    ArticulationAction(joint_positions=arr))
        except Exception as e:
            logger.debug("[RMP] joint apply failed: %s", e)

    # ------------------------------------------------------------------
    # 디버깅
    # ------------------------------------------------------------------
    @property
    def backend_name(self) -> str:
        return self._backend.name if self._backend else "(uninitialized)"

    def diagnose(self) -> dict:
        return {
            "backend": self.backend_name,
            "preferred_backend": self.config.preferred_backend,
            "obstacles": [
                {"name": o.name, "center": o.center, "radius": o.radius}
                for o in self._obstacles
            ],
            "active_path": self._active_path is not None,
            "phase": self.get_phase(),
            "last_tcp": self._last_tcp,
        }
