"""Detached, read-only gripper USD inventory, without attachment or motor commands."""
import math


def inspect_gripper_stage(stage):
    from pxr import Usd, UsdGeom, UsdPhysics
    if stage is None:
        raise ValueError("gripper asset could not be opened")
    prims = list(Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()))
    errors = [str(e) for e in stage.GetCompositionErrors()]

    def value(v):
        if v is None or isinstance(v, (str, bool, int)):
            return v
        if isinstance(v, float):
            return v if math.isfinite(v) else str(v)
        try:
            return [value(x) for x in v]
        except TypeError:
            return str(v)

    joints, bodies, colliders, frames = [], [], [], []
    for prim in prims:
        path = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies.append({"path": path, "authored_mass_properties": {
                a.GetName(): value(a.Get()) for a in prim.GetAttributes()
                if a.GetName() in ("physics:mass", "physics:density", "physics:centerOfMass",
                                   "physics:diagonalInertia", "physics:principalAxes") and a.HasAuthoredValueOpinion()}})
        if prim.IsA(UsdPhysics.Joint):
            j = UsdPhysics.Joint(prim)
            joints.append({"name": prim.GetName(), "path": path, "type": prim.GetTypeName(),
                           "body0": list(map(str, j.GetBody0Rel().GetTargets())),
                           "body1": list(map(str, j.GetBody1Rel().GetTargets())),
                           "attributes": {a.GetName(): {"value": value(a.Get()), "authored": a.HasAuthoredValueOpinion()}
                                          for a in prim.GetAttributes() if a.GetName().startswith(
                                              ("physics:", "drive:", "physxJoint:", "physxMimicJoint:"))}})
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            colliders.append({"path": path, "enabled": UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()})
        if prim.IsA(UsdGeom.Xformable) and not prim.IsA(UsdGeom.Gprim):
            frames.append(path)
    movable = [j for j in joints if j["type"] in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint")]
    if not movable:
        errors.append("no movable joints in candidate gripper asset")
    if not bodies:
        errors.append("no rigid bodies in candidate gripper asset")
    if not any(c["enabled"] for c in colliders):
        errors.append("no enabled collision shapes in candidate gripper asset")
    return {"status": "FAILED" if errors else "INVENTORIED_NOT_PHYSICS_VALIDATED",
            "errors": errors, "scope": "detached read-only gripper model inventory",
            "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
            "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
            "joints": joints, "movable_joint_count": len(movable), "rigid_bodies": bodies,
            "colliders": colliders, "frame_paths": frames,
            "motion_commands_issued": False, "mounted_on_fanuc": False,
            "physical_grasp_verified": False, "tcp_calibration_verified": False,
            "joint_names_are_usd_inventory_not_runtime_dof_mapping": True}
