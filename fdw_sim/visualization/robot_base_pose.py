"""World-space robot base poses, without an import-time Isaac/USD dependency.

RobotLoader authors a parent-local transform. Motion targets, obstacle positions,
and Lula's base-pose API are world-space, so callers must never use that local
offset as the controller's base position.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass(frozen=True)
class RobotBasePose:
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        position = tuple(float(v) for v in self.position)
        orientation = tuple(float(v) for v in self.orientation)
        if (len(position) != 3 or len(orientation) != 4
                or not all(math.isfinite(v) for v in position + orientation)):
            raise ValueError("Robot base pose requires finite XYZ and WXYZ values")
        norm = math.sqrt(sum(v * v for v in orientation))
        if norm < 1e-12:
            raise ValueError("Robot base orientation must be a nonzero quaternion")
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "orientation", tuple(v / norm for v in orientation))

    def to_local(self, world_position: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Inverse rigid transform for the heuristic (quaternions use WXYZ)."""
        dx, dy, dz = (world_position[i] - self.position[i] for i in range(3))
        w, x, y, z = self.orientation
        # R(q).T @ (target_world - base_world)
        return (
            (1 - 2 * (y*y + z*z)) * dx + 2 * (x*y + w*z) * dy + 2 * (x*z - w*y) * dz,
            2 * (x*y - w*z) * dx + (1 - 2 * (x*x + z*z)) * dy + 2 * (y*z + w*x) * dz,
            2 * (x*z + w*y) * dx + 2 * (y*z - w*x) * dy + (1 - 2 * (x*x + y*y)) * dz,
        )


def read_robot_base_pose(articulation, fallback: RobotBasePose,
                         *, stage=None, prim_path=None) -> RobotBasePose:
    """Read the live world pose, falling back to USD then the last known pose.

    USD composes the entire parent hierarchy, including rotation and the robot
    catalog's height offset. A not-yet-initialized PhysX articulation may fail to
    answer get_world_pose(); that does not make its local pose a valid substitute.
    """
    try:
        position, orientation = articulation.get_world_pose()
        return RobotBasePose(position, orientation)
    except Exception:
        pass

    try:
        from pxr import UsdGeom  # type: ignore

        if prim_path is None:
            prim_path = (getattr(articulation, "prim_path", None)
                         or getattr(articulation, "_prim_path", None))
        if not prim_path:
            return fallback
        if stage is None:
            stage = getattr(articulation, "stage", None)
        if stage is None:
            stage = getattr(articulation, "_stage", None)
        if stage is None:
            import omni.usd  # type: ignore
            stage = omni.usd.get_context().get_stage()
        if stage is None:
            return fallback
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return fallback
        world = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
        rotation = world.ExtractRotationQuat()
        orientation = (rotation.GetReal(), *rotation.GetImaginary())
        return RobotBasePose(world.ExtractTranslation(), orientation)
    except Exception:
        return fallback
