#!/usr/bin/env python3
"""Load robonex.usd into a scene with an infinite ground plane, GUI by default.

    conda activate isaacsim
    ~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py
    ~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --fixed-base
    ~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --headless

Uses whichever isaac/robonex.usd or robonex_fixed.usd is already on disk - it
does not run the URDF/USD conversion itself. Build those first with
build_isaac_urdf.py and the convert_urdf.py command in README.md.
"""
import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Load robonex into an empty stage.")
parser.add_argument("--fixed-base", action="store_true",
                    help="load robonex_fixed.usd (base welded in the air) "
                    "instead of the free-floating robonex.usd")
parser.add_argument("--spawn-height", type=float, default=None,
                    help="spawn height in meters (default 1.085 free, 1.60 fixed, "
                    "matching mujoco/build_mjcf.py's SPAWN_HEIGHT/FIXED_BASE_HEIGHT). "
                    "Named --spawn-height, not --height: AppLauncher reserves 'height' "
                    "for the viewport window size.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import omni.usd  # noqa: E402
from isaacsim.core.api.materials.physics_material import PhysicsMaterial  # noqa: E402
from isaacsim.core.api.objects import GroundPlane  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import UsdGeom, UsdLux  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FREE_USD = os.path.join(HERE, "robonex.usd")
FIXED_USD = os.path.join(HERE, "robonex_fixed.usd")

SPAWN_HEIGHT = 1.085       # matches mujoco/build_mjcf.py SPAWN_HEIGHT
FIXED_BASE_HEIGHT = 1.60   # matches mujoco/build_mjcf.py FIXED_BASE_HEIGHT

# same floor friction MuJoCo/Gazebo assume (scripts/robonex_serial.py
# FOOT_FRICTION/BODY_FRICTION = 1.0) - unmeasured, see isaac README's
# "physical parameters are provisional" note.
FLOOR_FRICTION = 1.0


def main():
    usd_path = FIXED_USD if args.fixed_base else FREE_USD
    if not os.path.isfile(usd_path):
        raise FileNotFoundError(
            "%s not found. Build it first - see isaac/README.md for the "
            "convert_urdf.py command (%s)."
            % (usd_path, "add --fix-base for robonex_fixed.usd" if args.fixed_base else ""))

    height = args.spawn_height if args.spawn_height is not None else (
        FIXED_BASE_HEIGHT if args.fixed_base else SPAWN_HEIGHT)

    stage = omni.usd.get_context().get_stage()

    # PhysX ground planes are collision-infinite regardless of `size`; it only
    # sets how large a tile the visual mesh draws.
    floor_material = PhysicsMaterial(
        prim_path="/World/physicsMaterials/floor",
        static_friction=FLOOR_FRICTION,
        dynamic_friction=FLOOR_FRICTION,
    )
    GroundPlane(prim_path="/World/ground", size=50.0, physics_material=floor_material)

    light = UsdLux.DistantLight.Define(stage, "/World/light")
    light.CreateIntensityAttr(3000.0)
    light.AddRotateXYZOp().Set((-45.0, 30.0, 0.0))

    # The referenced USD's root prim already carries a translate/orient/scale
    # xformOpOrder from the converter, and adding a second translate op to it
    # directly raises (USD rejects a duplicate op in the order). Simplest fix:
    # never touch it - park the reference under a fresh Xform we own instead.
    container = UsdGeom.Xform.Define(stage, "/World/robonex")
    container.AddTranslateOp().Set((0.0, 0.0, height))
    add_reference_to_stage(usd_path, "/World/robonex/asset")

    # flush explicitly: Kit's shutdown on simulation_app.close() tears the
    # process down hard enough that ordinary buffered stdout can be lost.
    print("[load_robonex] loaded %s at z=%.3f m (%s base)"
          % (os.path.basename(usd_path), height, "fixed" if args.fixed_base else "free"),
          flush=True)
    print("[load_robonex] press Play to drop it - with no controller yet it will fall, "
          "same as the MuJoCo/Gazebo builds with ctrl=0", flush=True)

    while simulation_app.is_running():
        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    main()
