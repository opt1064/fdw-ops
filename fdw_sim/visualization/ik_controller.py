"""IKController — 로봇팔 엔드 이펙터를 목표 위치로 이동시키는 간이 IK.

Isaac Sim 5.x에는 다음 IK 솔버가 내장되어 있다:
    1) Lula Kinematics Solver       (정확, URDF/Robot Description 필요)
    2) Motion Generation - RMPflow  (반응형, 충돌회피)
    3) Articulation 직접 joint 제어 (수동 trajectory)

본 모듈은 **간이 추적 모드**를 제공한다:
    * 옵션 A — Lula IK가 가능하면 그것을 사용
    * 옵션 B — 불가능하면 사전 정의된 joint waypoint trajectory를 보간

용접 셀의 토치가 부품 표면의 점 → 점을 따라 움직이는
용도로 충분한 수준이며, 정확한 충돌·동역학은 다음 단계의 RMPflow에서 다룬다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import logging
import math

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


class IKController:
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

    def __init__(self, articulation, spec, config: Optional[IKConfig] = None) -> None:
        self.articulation = articulation
        self.spec = spec
        self.config = config or IKConfig()

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
    def go_home(self) -> None:
        if not self.spec.home_joint_positions:
            return
        self._set_target_joints(self.spec.home_joint_positions,
                                 blend_time=self.config.home_blend_time)

    def start_path(self, path: WeldingPath) -> None:
        path.reset()
        self._active_path = path
        # fallback: 시작 시 home에서 살짝 내려간 자세로 prepare
        if self.spec.home_joint_positions:
            prepare_q = list(self.spec.home_joint_positions)
            # joint 1을 약간 굽혀 토치를 부품 쪽으로 향하게 (Franka 기준)
            if len(prepare_q) >= 4:
                prepare_q[1] += 0.3       # shoulder lift
                prepare_q[3] += -0.3      # elbow
            self._set_target_joints(prepare_q,
                                    blend_time=self.config.waypoint_blend_time)

    def is_idle(self) -> bool:
        return self._active_path is None and self._blend_remaining <= 0.0

    def get_phase(self) -> str:
        if self._active_path is not None:
            return self._active_path.phase
        return "idle" if self._blend_remaining <= 0.0 else "moving_home"

    # ------------------------------------------------------------------
    def update(self, dt: float) -> Optional[Tuple[float, float, float]]:
        """매 step 호출. 토치 목표 위치(있다면) 반환."""
        self._ensure_backend()

        target_pos: Optional[Tuple[float, float, float]] = None

        # 1) 경로 추적
        if self._active_path is not None:
            target_pos = self._active_path.update(dt)
            if not self._active_path.is_active():
                self._active_path = None
                # 경로 끝났으면 home으로
                self.go_home()
            else:
                # path 갱신: IK 시도 (fallback이면 보간만 진행)
                self._track_target(target_pos)

        # 2) joint 보간 (path가 없을 때도 home blend 진행)
        if self._blend_remaining > 0.0 and self._target_q is not None:
            self._blend_remaining = max(0.0, self._blend_remaining - dt)
            if self._current_q is None:
                self._current_q = list(self._target_q)
            else:
                k = 1.0 - (self._blend_remaining / self._blend_total)
                k = max(0.0, min(1.0, k))
                # smoothstep
                k = k * k * (3.0 - 2.0 * k)
                for i in range(len(self._current_q)):
                    if i < len(self._target_q):
                        self._current_q[i] = (
                            self._current_q[i] * (1.0 - k)
                            + self._target_q[i] * k
                        )
            self._apply_joints(self._current_q)

        return target_pos

    # ------------------------------------------------------------------
    def _track_target(self, target_pos: Tuple[float, float, float]) -> None:
        """현재 토치 목표 → joint positions 변환."""
        if self._ik_backend is not None:
            q = self._ik_backend.compute(target_pos,
                                          warm_start_q=self._current_q)
            if q is not None:
                self._set_target_joints(q,
                                        blend_time=self.config.waypoint_blend_time)
                return

        # ----- Fallback (analytic-ish for Franka) -----
        # 시각적으로 그럴듯한 자세만 만들어주는 휴리스틱.
        # 실제 IK가 아닌 demo 용도.
        if not self.spec.home_joint_positions or len(self.spec.home_joint_positions) < 7:
            return
        # 부품의 y 위치에 따라 base joint를 살짝 회전
        # (Franka의 joint0가 base yaw)
        base = list(self.spec.home_joint_positions)
        # 단순 매핑: y가 +면 base를 음수로 회전 (Z up, X forward 가정)
        yaw_offset = math.atan2(target_pos[1] - 0.0, target_pos[0] - 5.0) * 0.5
        base[0] = base[0] + max(-0.8, min(0.8, yaw_offset))
        # shoulder/elbow를 약간 굽혀 토치를 부품 위로
        base[1] = base[1] + 0.4
        base[3] = base[3] + -0.5
        self._set_target_joints(base, blend_time=self.config.waypoint_blend_time)

    # ------------------------------------------------------------------
    def _set_target_joints(self, q: List[float], blend_time: float) -> None:
        self._target_q = list(q)
        self._blend_total = max(1e-3, blend_time)
        self._blend_remaining = self._blend_total
        if self._current_q is None:
            self._current_q = list(q)

    def _apply_joints(self, q: List[float]) -> None:
        """Articulation에 joint 적용."""
        if self.articulation is None:
            return
        try:
            import numpy as np  # type: ignore
            arr = np.array(q, dtype=float)
            # 5.x SingleArticulation
            if hasattr(self.articulation, "set_joint_positions"):
                self.articulation.set_joint_positions(arr)
            elif hasattr(self.articulation, "apply_action"):
                # 일부 버전 fallback
                from omni.isaac.core.utils.types import ArticulationAction  # type: ignore
                self.articulation.apply_action(ArticulationAction(joint_positions=arr))
        except Exception as e:
            logger.debug("[IK] joint apply failed: %s", e)
