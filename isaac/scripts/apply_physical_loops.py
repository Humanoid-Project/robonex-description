#!/usr/bin/env python3
"""Add RoboNex's physical ankle and knee closed-loop constraints to a USD.

Convert ``isaac/closed_loop/robonex_closed_loop.urdf`` with fixed joints left
unmerged, then
run this script once.  The imported open articulation tree is completed with:

* four D6 ankle crank-to-coupler rod ends (translations locked, rotX/Y/Z +/-limit);
* four excluded spherical ankle coupler-to-foot closures; and
* two excluded revolute knee coupler-to-shin pin closures.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser()
parser.add_argument("usd_path", type=str)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app


from pxr import Gf, Sdf, Usd, UsdPhysics


HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC_DIR = os.path.dirname(HERE)
ROOT = os.path.dirname(ISAAC_DIR)
sys.path.insert(0, os.path.join(ROOT, "scripts"))

from model_io import load_loops


IDENTITY = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
BALL_AXIS = UsdPhysics.Tokens.x
X_TO_Z = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), -90.0).GetQuat())
ROD_END_BOLT_AXIS = "rotY"
ROD_END_TILT_AXES = ("rotX", "rotZ")

ANKLE_PASSIVE_JOINTS = {
    "l_ankle_roll_joint", "l_ankle_pitch_joint",
    "r_ankle_roll_joint", "r_ankle_pitch_joint",
}
KNEE_PASSIVE_JOINTS = {
    "l_knee_joint", "r_knee_joint",
    "l_knee_coupler_joint_a", "r_knee_coupler_joint_a",
}


def find_unique(stage: Usd.Stage, name: str) -> Usd.Prim:
    matches = [prim for prim in stage.Traverse() if prim.GetName() == name]
    if len(matches) != 1:
        raise RuntimeError("expected one prim named %s, found %d" % (name, len(matches)))
    return matches[0]


def vec3(values, label: str) -> Gf.Vec3f:
    if len(values) != 3 or not all(math.isfinite(float(value)) for value in values):
        raise ValueError("%s must contain three finite values" % label)
    return Gf.Vec3f(*(float(value) for value in values))


def make_passive(prim: Usd.Prim, expected_type: str) -> None:
    if prim.GetTypeName() != expected_type:
        raise RuntimeError("%s must be an imported %s" % (prim.GetName(), expected_type))
    drive = UsdPhysics.DriveAPI.Get(prim, UsdPhysics.Tokens.angular)
    if drive:
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.0)
        drive.CreateMaxForceAttr(0.0)


def lock_limit(prim: Usd.Prim, axis: str) -> None:
    limit = UsdPhysics.LimitAPI.Apply(prim, axis)
    limit.CreateLowAttr(1.0)
    limit.CreateHighAttr(-1.0)


def range_limit(prim: Usd.Prim, axis: str, limit_deg: float) -> None:
    limit = UsdPhysics.LimitAPI.Apply(prim, axis)
    limit.CreateLowAttr(-limit_deg)
    limit.CreateHighAttr(limit_deg)


def upgrade_to_limited_ball(stage: Usd.Stage, name: str, limit_deg: float) -> None:
    prim = find_unique(stage, name)
    old_joint = UsdPhysics.Joint(prim)
    if not old_joint or prim.GetTypeName() != "PhysicsFixedJoint":
        raise RuntimeError("%s must be an imported PhysicsFixedJoint" % name)
    body0 = old_joint.GetBody0Rel().GetTargets()
    body1 = old_joint.GetBody1Rel().GetTargets()
    pos0 = old_joint.GetLocalPos0Attr().Get()
    pos1 = old_joint.GetLocalPos1Attr().Get()
    rot0 = old_joint.GetLocalRot0Attr().Get()
    rot1 = old_joint.GetLocalRot1Attr().Get()
    if len(body0) != 1 or len(body1) != 1 or any(
        value is None for value in (pos0, pos1, rot0, rot1)
    ):
        raise RuntimeError("%s has incomplete body relationships or frames" % name)

    joint = UsdPhysics.Joint.Define(stage, prim.GetPath())
    joint.CreateBody0Rel().SetTargets(body0)
    joint.CreateBody1Rel().SetTargets(body1)
    joint.CreateLocalPos0Attr().Set(pos0)
    joint.CreateLocalPos1Attr().Set(pos1)
    joint.CreateLocalRot0Attr().Set(rot0)
    joint.CreateLocalRot1Attr().Set(rot1)
    joint.CreateCollisionEnabledAttr(False)
    prim = joint.GetPrim()
    for axis in ("transX", "transY", "transZ"):
        lock_limit(prim, axis)
    for axis in ROD_END_TILT_AXES:
        range_limit(prim, axis, limit_deg)


def add_spherical_closure(stage: Usd.Stage, scope: Sdf.Path, entry: dict) -> None:
    parent = find_unique(stage, entry["parent"])
    child = find_unique(stage, entry["child"])
    path = scope.AppendChild(entry["name"])
    if stage.GetPrimAtPath(path):
        raise RuntimeError("closure joint already exists: %s" % path)
    joint = UsdPhysics.SphericalJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([parent.GetPath()])
    joint.CreateBody1Rel().SetTargets([child.GetPath()])
    joint.CreateLocalPos0Attr().Set(vec3(entry["parent_xyz"], entry["name"]))
    joint.CreateLocalPos1Attr().Set(vec3(entry["child_xyz"], entry["name"]))
    joint.CreateLocalRot0Attr().Set(X_TO_Z)
    joint.CreateLocalRot1Attr().Set(X_TO_Z)
    joint.CreateAxisAttr(BALL_AXIS)
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateExcludeFromArticulationAttr(True)


def axis_token(values, label: str):
    axis = vec3(values, label)
    absolute = [abs(float(value)) for value in axis]
    index = max(range(3), key=absolute.__getitem__)
    if absolute[index] < 1.0 - 1.0e-6 or sum(absolute) - absolute[index] > 1.0e-6:
        raise ValueError("%s axis must be an axis-aligned unit vector" % label)
    return (UsdPhysics.Tokens.x, UsdPhysics.Tokens.y, UsdPhysics.Tokens.z)[index]


def add_pin_closure(stage: Usd.Stage, scope: Sdf.Path, entry: dict) -> None:
    parent = find_unique(stage, entry["parent"])
    child = find_unique(stage, entry["child"])
    path = scope.AppendChild(entry["name"])
    if stage.GetPrimAtPath(path):
        raise RuntimeError("closure joint already exists: %s" % path)
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([parent.GetPath()])
    joint.CreateBody1Rel().SetTargets([child.GetPath()])
    joint.CreateLocalPos0Attr().Set(vec3(entry["parent_xyz"], entry["name"]))
    joint.CreateLocalPos1Attr().Set(vec3(entry["child_xyz"], entry["name"]))
    joint.CreateLocalRot0Attr().Set(IDENTITY)
    joint.CreateLocalRot1Attr().Set(IDENTITY)
    joint.CreateAxisAttr(axis_token(entry["axis"], entry["name"]))
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateExcludeFromArticulationAttr(True)


def main() -> int:
    usd_path = os.path.abspath(args.usd_path)
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError("could not open %s" % usd_path)
    stage.SetEditTarget(stage.GetRootLayer())

    loops = load_loops()
    upgrades = list(loops.get("ball_upgrades", []))
    ball_closures = list(loops.get("ball_loops", []))
    pin_closures = list(loops.get("pin_loops", []))
    limit_deg = float(loops.get("ball_limit_deg", 0.0))
    if len(upgrades) != 4 or len(ball_closures) != 4 or len(pin_closures) != 2:
        raise ValueError("expected 4 ankle upgrades, 4 ball closures, and 2 pin closures")
    if not 0.0 < limit_deg < 180.0:
        raise ValueError("ball_limit_deg must be between 0 and 180")

    for name in ANKLE_PASSIVE_JOINTS:
        find_unique(stage, name)
    for name in KNEE_PASSIVE_JOINTS:
        find_unique(stage, name)
    for name in upgrades:
        find_unique(stage, name)
    for entry in ball_closures + pin_closures:
        find_unique(stage, entry["parent"])
        find_unique(stage, entry["child"])

    scope = find_unique(stage, next(iter(ANKLE_PASSIVE_JOINTS))).GetPath().GetParentPath()
    for name in ANKLE_PASSIVE_JOINTS:
        make_passive(find_unique(stage, name), "PhysicsRevoluteJoint")
    for name in KNEE_PASSIVE_JOINTS:
        make_passive(find_unique(stage, name), "PhysicsRevoluteJoint")
    for name in upgrades:
        upgrade_to_limited_ball(stage, name, limit_deg)
    for entry in ball_closures:
        add_spherical_closure(stage, scope, entry)
    for entry in pin_closures:
        add_pin_closure(stage, scope, entry)

    stage.GetRootLayer().Save()
    print("saved %s" % usd_path, flush=True)
    print("  passive ankle outputs       : 4", flush=True)
    print("  D6 rod-end articulation ends: 4", flush=True)
    print("  excluded spherical closures : 4", flush=True)
    print("  passive knee tree joints    : 4", flush=True)
    print("  excluded pin closures       : 2", flush=True)
    print("  rod-end tilt limit %s   : +/-%.1f deg (%s free: bolt axis)"
          % ("/".join(ROD_END_TILT_AXES), limit_deg, ROD_END_BOLT_AXIS), flush=True)
    print("  passive position/velocity limits: none (loop closure determines them)", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        simulation_app.close()
