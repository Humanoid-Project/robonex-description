#!/usr/bin/env python3
"""Apply the knee mimic constraints that convert_urdf.py silently drops.

isaac/build_isaac_urdf.py writes a standard URDF <mimic> tag on the knee
crank joints, and IsaacLab's own URDF importer supports it
(PhysxSchema.PhysxMimicJointAPI) - but IsaacLab's converter wrapper has a
sign bug (urdf_converter.py's set_parse_mimic call doesn't invert
convert_mimic_joints_to_normal_joints), so it always strips the tag
regardless of the config. Confirmed empirically: after a normal convert_urdf.py
run, HasAPI(PhysxMimicJointAPI) is False on both knee actuator joints.

Run this against a USD convert_urdf.py already produced:

    ~/IsaacLab/isaaclab.sh -p isaac/apply_knee_mimic.py isaac/robonex.usd
    ~/IsaacLab/isaaclab.sh -p isaac/apply_knee_mimic.py isaac/robonex_fixed.usd
"""
import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("usd_path", type=str)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, PhysxSchema  # noqa: E402

# joint -> (reference joint it mimics, gearing). Keep in sync with
# isaac/build_isaac_urdf.py's MIMIC dict - this script re-derives what the
# URDF <mimic> tag already said, since the tag itself gets lost on import.
#
# Sign is NEGATIVE despite the measured crank/shin relationship being
# same-sign (see scripts/robonex_serial.py) - confirmed empirically
# (isaac/dynamic_mimic_test in scratch history): the URDF importer collapses
# each joint's <axis> vector onto a bare X/Y/Z token plus a compensating
# rotation of its local frame, and that can flip the sign of the *value*
# PhysX reports even though the physical axis direction is preserved. +0.705
# gave crank/knee = -0.689 once physics actually ran; -0.705 is what makes
# the crank move the same physical way the real linkage does.
MIMIC = {
    "l_knee_pitch_joint": ("l_knee_joint", -0.705),
    "r_knee_pitch_joint": ("r_knee_joint", -0.705),
}


def find_prim(stage, name):
    for p in stage.Traverse():
        if p.GetName() == name:
            return p
    return None


def main():
    stage = Usd.Stage.Open(args.usd_path)
    if stage is None:
        print("could not open %s" % args.usd_path)
        sys.exit(1)

    for target_name, (ref_name, gearing) in MIMIC.items():
        target = find_prim(stage, target_name)
        ref = find_prim(stage, ref_name)
        if target is None or ref is None:
            print("SKIP %s: target=%s ref=%s" % (target_name, target, ref))
            continue
        if not target.IsA(UsdPhysics.RevoluteJoint):
            print("SKIP %s: not a revolute joint (%s)" % (target_name, target.GetTypeName()))
            continue

        axis = UsdPhysics.RevoluteJoint(target).GetAxisAttr().Get()
        axis_token = {"X": UsdPhysics.Tokens.rotX,
                      "Y": UsdPhysics.Tokens.rotY,
                      "Z": UsdPhysics.Tokens.rotZ}[axis]

        api = PhysxSchema.PhysxMimicJointAPI.Apply(target, axis_token)
        api.GetGearingAttr().Set(gearing)
        api.GetOffsetAttr().Set(0.0)
        api.GetReferenceJointRel().SetTargets([ref.GetPath()])
        print("OK %s: mimics %s, axis=%s, gearing=%g"
              % (target_name, ref_name, axis, gearing))

    stage.Save()
    print("saved %s" % args.usd_path)
    simulation_app.close()


if __name__ == "__main__":
    main()
