#!/usr/bin/env python3
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from robonex_common import load_urdf, fmt, ROOT
from robonex_serial import (
    DEG, SERIAL_JOINTS, DAMPING, SERIAL_FRICTION, motor_physics_for, COLLISION_BOX,
)

MECHANISMS = ("closed_loop", "serial")
COLLISIONS = ("mesh", "box")


def variant_name(mechanism, collision):
    return "%s_%s" % (mechanism, collision)


def output_path(mechanism, collision):
    name = variant_name(mechanism, collision)
    return os.path.join(ROOT, "isaac", name, "robonex_%s.urdf" % name)


MESH_URI = "../meshes/%s"

MATERIALS = (("black", "0.05 0.05 0.05 1.0"), ("gray", "0.647 0.647 0.647 1.0"))


def emit_link(o, name, lk, collision):
    o.append('  <link name="%s">' % name)

    if lk.mass > 0.0:
        ixx, ixy, ixz, iyy, iyz, izz = lk.inertia
        o.append("    <inertial>")
        o.append('      <origin xyz="%s" rpy="0 0 0"/>' % fmt(lk.com))
        o.append('      <mass value="%.6f"/>' % lk.mass)
        o.append('      <inertia ixx="%.8f" ixy="%.8f" ixz="%.8f"'
                 ' iyy="%.8f" iyz="%.8f" izz="%.8f"/>'
                 % (ixx, ixy, ixz, iyy, iyz, izz))
        o.append("    </inertial>")

    for g in lk.visuals:
        colour = "gray" if g.mesh.startswith("rs0") else "black"
        o.append("    <visual>")
        o.append('      <origin xyz="%s" rpy="%s"/>' % (fmt(g.xyz), fmt(g.rpy)))
        o.append("      <geometry>")
        o.append('        <mesh filename="%s" scale="%s"/>'
                 % (MESH_URI % g.mesh, fmt(g.scale)))
        o.append("      </geometry>")
        o.append('      <material name="%s"/>' % colour)
        o.append("    </visual>")

    if collision == "box":
        box = COLLISION_BOX.get(name)
        if box is not None:
            size, centre = box
            o.append('    <collision name="%s_collision">' % name)
            o.append('      <origin xyz="%s" rpy="0 0 0"/>' % fmt(centre))
            o.append('      <geometry><box size="%s"/></geometry>' % fmt(size))
            o.append("    </collision>")
    else:
        for g in lk.visuals:
            o.append("    <collision>")
            o.append('      <origin xyz="%s" rpy="%s"/>' % (fmt(g.xyz), fmt(g.rpy)))
            o.append("      <geometry>")
            o.append('        <mesh filename="%s" scale="%s"/>'
                     % (MESH_URI % g.mesh, fmt(g.scale)))
            o.append("      </geometry>")
            o.append("    </collision>")

    o.append("  </link>")
    o.append("")


ANKLE_MOTOR_JOINTS = {
    "l_ankle_upper_joint", "l_ankle_lower_joint",
    "r_ankle_upper_joint", "r_ankle_lower_joint",
}
ANKLE_OUTPUT_JOINTS = {
    "l_ankle_roll_joint", "l_ankle_pitch_joint",
    "r_ankle_roll_joint", "r_ankle_pitch_joint",
}
KNEE_MOTOR_JOINTS = {"l_knee_pitch_joint", "r_knee_pitch_joint"}
KNEE_PASSIVE_JOINTS = {
    "l_knee_joint", "r_knee_joint",
    "l_knee_coupler_joint_a", "r_knee_coupler_joint_a",
}


def emit_joint(o, j, mechanism):
    if mechanism == "serial":
        spec = SERIAL_JOINTS.get(j.name)
        passive_output = False
    else:
        if j.name in ANKLE_MOTOR_JOINTS:
            spec = (j.effort, j.velocity, j.lower / DEG, j.upper / DEG)
        elif j.name in KNEE_MOTOR_JOINTS:
            spec = (j.effort, j.velocity, j.lower / DEG, j.upper / DEG)
        elif j.name in ANKLE_OUTPUT_JOINTS or j.name in KNEE_PASSIVE_JOINTS:
            spec = None
        else:
            spec = SERIAL_JOINTS.get(j.name)
        passive_output = j.name in ANKLE_OUTPUT_JOINTS or j.name in KNEE_PASSIVE_JOINTS
    kind = "revolute" if (spec or passive_output) else "fixed"
    o.append('  <joint name="%s" type="%s">' % (j.name, kind))
    o.append('    <parent link="%s"/>' % j.parent)
    o.append('    <child link="%s"/>' % j.child)
    o.append('    <origin xyz="%s" rpy="%s"/>' % (fmt(j.xyz), fmt(j.rpy)))
    if spec:
        effort, velocity, lo_deg, hi_deg = spec
        o.append('    <axis xyz="%s"/>' % fmt(j.axis))
        o.append('    <limit lower="%.6f" upper="%.6f" effort="%g" velocity="%g"/>'
                 % (lo_deg * DEG, hi_deg * DEG, effort, velocity))
        phys = motor_physics_for(j.name)
        friction = (phys["frictionloss"] if phys is not None
                    else SERIAL_FRICTION.get(j.name, 0.0))
        o.append('    <dynamics damping="%g" friction="%g"/>' % (DAMPING, friction))
    elif passive_output:
        o.append('    <axis xyz="%s"/>' % fmt(j.axis))
        o.append('    <limit lower="%.6f" upper="%.6f" effort="0" velocity="0"/>'
                 % (j.lower, j.upper))
        o.append('    <dynamics damping="0" friction="0.0"/>')
    o.append("  </joint>")
    o.append("")


def build(mechanism, collision):
    if mechanism not in MECHANISMS:
        raise ValueError("unknown mechanism %r" % mechanism)
    if collision not in COLLISIONS:
        raise ValueError("unknown collision %r" % collision)

    out_path = output_path(mechanism, collision)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    links, joints, base = load_urdf()

    o = []
    o.append('<?xml version="1.0"?>')
    o.append("<!-- generated by isaac/build_isaac_urdf.py - do not edit by hand -->")
    o.append("<!-- variant: %s -->" % variant_name(mechanism, collision))
    if mechanism == "closed_loop":
        o.append("<!-- physical-loop model: knee and ankle motor/output trees remain")
        o.append("     movable for USD revolute and spherical loop closures. -->")
    else:
        o.append("<!-- serial tree model: linkage output joints are driven directly;")
        o.append("     crank/coupler joints are fixed and no loop constraints are used. -->")
    if collision == "box":
        o.append("<!-- collision: box primitives from robonex_serial.COLLISION_BOX. -->")
    else:
        o.append("<!-- collision: visual meshes. -->")
    o.append('<robot name="robonex">')
    o.append("")
    for name, rgba in MATERIALS:
        o.append('  <material name="%s"><color rgba="%s"/></material>' % (name, rgba))
    o.append("")

    for name, lk in links.items():
        emit_link(o, name, lk, collision)
    for j in joints.values():
        emit_joint(o, j, mechanism)

    o.append("</robot>")

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(o) + "\n")

    if mechanism == "closed_loop":
        active = set(SERIAL_JOINTS)
        passive = set()
        active.difference_update(ANKLE_OUTPUT_JOINTS)
        active.difference_update({"l_knee_joint", "r_knee_joint"})
        active.update(ANKLE_MOTOR_JOINTS)
        active.update(KNEE_MOTOR_JOINTS)
        passive.update(ANKLE_OUTPUT_JOINTS)
        passive.update(KNEE_PASSIVE_JOINTS)
    else:
        active = set(SERIAL_JOINTS)
        passive = set()
    frozen = [j.name for j in joints.values()
              if j.name not in active and j.name not in passive]
    print("wrote %s" % out_path)
    print("  variant          : %s" % variant_name(mechanism, collision))
    print("  links            : %d  (total mass %.6f kg)"
          % (len(links), sum(lk.mass for lk in links.values())))
    print("  actuated / passive / frozen : %d / %d / %d"
          % (len(active), len(passive), len(frozen)))
    if collision == "box":
        boxed = sum(1 for name in links if name in COLLISION_BOX)
        print("  collision        : %d boxes, %d links left without collision"
              % (boxed, len(links) - boxed))
    else:
        print("  collision        : meshes")
    print()
    if mechanism == "closed_loop":
        print("  open tree is completed by scripts/apply_physical_loops.py.")
    else:
        print("  linkage outputs are directly actuated; no loop post-processing is needed.")
    return out_path


def main():
    parser = argparse.ArgumentParser(description="Build a RoboNex Isaac URDF variant.")
    parser.add_argument(
        "--mechanism", choices=MECHANISMS, default="closed_loop",
        help="closed_loop keeps the physical mechanisms; serial directly drives their outputs",
    )
    parser.add_argument(
        "--collision", choices=COLLISIONS, default="mesh",
        help="mesh uses the visual meshes; box uses robonex_serial.COLLISION_BOX primitives",
    )
    args = parser.parse_args()
    build(args.mechanism, args.collision)


if __name__ == "__main__":
    main()
