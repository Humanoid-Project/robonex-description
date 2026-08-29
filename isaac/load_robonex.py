#!/usr/bin/env python3
"""Load a RoboNex Isaac USD variant into a scene, GUI by default.

    conda activate isaacsim
    ~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py
    ~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --fixed-base
    ~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --headless

Uses the generated USD under isaac/closed_loop/ and does not run conversion.
Build it first with the command in README.md.
"""
import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Load robonex into an empty stage.")
parser.add_argument("--fixed-base", action="store_true",
                    help="load the fixed-base variant (base welded in the air) "
                    "instead of the free-floating variant")
parser.add_argument("--collision", choices=("mesh", "box"), default="mesh",
                    help="collision geometry variant to load")
parser.add_argument("--spawn-height", type=float, default=None,
                    help="spawn height in meters (default 1.085 free, 1.60 fixed, "
                    "matching mujoco/build_mjcf.py's SPAWN_HEIGHT/FIXED_BASE_HEIGHT). "
                    "Named --spawn-height, not --height: AppLauncher reserves 'height' "
                    "for the viewport window size.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import omni.usd
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import PhysxSchema, UsdGeom, UsdLux, UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__))


def usd_for(collision, fixed_base):
    name = "closed_loop_%s" % collision
    suffix = "_fixed" if fixed_base else ""
    return os.path.join(HERE, name, "robonex_%s%s.usd" % (name, suffix))

SPAWN_HEIGHT = 1.085
FIXED_BASE_HEIGHT = 1.60

FLOOR_FRICTION = 0.6
LOOP_PHYSICS_HZ = 250
LOOP_POSITION_ITERATIONS = 64
LOOP_VELOCITY_ITERATIONS = 4


def main():
    usd_path = usd_for(args.collision, args.fixed_base)
    if not os.path.isfile(usd_path):
        raise FileNotFoundError(
            "%s not found. Build it first - see isaac/README.md for the "
            "convert_urdf.py command (%s)."
            % (usd_path, "add --fix-base for the fixed-base asset" if args.fixed_base else ""))

    height = args.spawn_height if args.spawn_height is not None else (
        FIXED_BASE_HEIGHT if args.fixed_base else SPAWN_HEIGHT)

    stage = omni.usd.get_context().get_stage()

    physics_scene_prim = stage.GetPrimAtPath("/physicsScene")
    if physics_scene_prim and physics_scene_prim.IsA(UsdPhysics.Scene):
        physics_scene = UsdPhysics.Scene(physics_scene_prim)
    else:
        physics_scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
    physx_scene.CreateSolverTypeAttr("TGS")
    physx_scene.CreateTimeStepsPerSecondAttr(LOOP_PHYSICS_HZ)

    floor_material = PhysicsMaterial(
        prim_path="/World/physicsMaterials/floor",
        static_friction=FLOOR_FRICTION,
        dynamic_friction=FLOOR_FRICTION,
    )
    GroundPlane(prim_path="/World/ground", size=50.0, physics_material=floor_material)

    light = UsdLux.DistantLight.Define(stage, "/World/light")
    light.CreateIntensityAttr(3000.0)
    light.AddRotateXYZOp().Set((-45.0, 30.0, 0.0))

    container = UsdGeom.Xform.Define(stage, "/World/robonex")
    container.AddTranslateOp().Set((0.0, 0.0, height))
    add_reference_to_stage(usd_path, "/World/robonex/asset")

    articulation_roots = [
        prim for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(articulation_roots) != 1:
        raise RuntimeError(
            "expected one closed-loop articulation root, found %d" % len(articulation_roots)
        )
    articulation = PhysxSchema.PhysxArticulationAPI.Apply(articulation_roots[0])
    articulation.CreateSolverPositionIterationCountAttr(LOOP_POSITION_ITERATIONS)
    articulation.CreateSolverVelocityIterationCountAttr(LOOP_VELOCITY_ITERATIONS)

    print("[load_robonex] loaded %s at z=%.3f m (closed loop, %s collision, %s base)"
          % (os.path.basename(usd_path), height, args.collision,
             "fixed" if args.fixed_base else "free"),
          flush=True)
    print("[load_robonex] closed-loop physics: TGS, %d Hz, %d/%d solver iterations"
          % (LOOP_PHYSICS_HZ, LOOP_POSITION_ITERATIONS,
             LOOP_VELOCITY_ITERATIONS), flush=True)
    if args.fixed_base:
        print("[load_robonex] press Play to start physics; the base remains welded",
              flush=True)
    else:
        print("[load_robonex] press Play to drop it - with no controller yet it will fall, "
              "same as the MuJoCo/Gazebo builds with ctrl=0", flush=True)

    while simulation_app.is_running():
        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    main()
