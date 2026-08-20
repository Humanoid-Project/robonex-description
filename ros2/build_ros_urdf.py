#!/usr/bin/env python3
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from robonex_common import load_urdf, fmt, ROOT
from robonex_serial import (
    DEG, SERIAL_JOINTS, SERIAL_FRICTION, JOINT_ORDER, DAMPING, COLLISION_BOX, FEET,
    FOOT_FRICTION, BODY_FRICTION, PACKAGE,
)

OUT = os.path.join(ROOT, "ros2", "robonex.urdf.xacro")

MESH_URI = "file://$(find %s)/meshes/%%s" % PACKAGE
PARAMS_URI = "$(find %s)/config/ros2_control.yaml" % PACKAGE

MATERIALS = (("black", "0.05 0.05 0.05 1.0"), ("gray", "0.647 0.647 0.647 1.0"))


def emit_link(o, name, lk):
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

    for i, g in enumerate(lk.visuals):
        colour = "gray" if g.mesh.startswith("rs0") else "black"
        o.append("    <visual>")
        o.append('      <origin xyz="%s" rpy="%s"/>' % (fmt(g.xyz), fmt(g.rpy)))
        o.append("      <geometry>")
        o.append('        <mesh filename="%s" scale="%s"/>'
                 % (MESH_URI % g.mesh, fmt(g.scale)))
        o.append("      </geometry>")
        o.append('      <material name="%s"/>' % colour)
        o.append("    </visual>")

    box = COLLISION_BOX.get(name)
    if box is not None:
        size, centre = box
        o.append('    <collision name="%s_collision">' % name)
        o.append('      <origin xyz="%s" rpy="0 0 0"/>' % fmt(centre))
        o.append('      <geometry><box size="%s"/></geometry>' % fmt(size))
        o.append("    </collision>")

    o.append("  </link>")
    o.append("")


def emit_joint(o, j):
    spec = SERIAL_JOINTS.get(j.name)
    kind = "revolute" if spec else "fixed"
    o.append('  <joint name="%s" type="%s">' % (j.name, kind))
    o.append('    <parent link="%s"/>' % j.parent)
    o.append('    <child link="%s"/>' % j.child)
    o.append('    <origin xyz="%s" rpy="%s"/>' % (fmt(j.xyz), fmt(j.rpy)))
    if spec:
        effort, velocity, lo_deg, hi_deg = spec
        o.append('    <axis xyz="%s"/>' % fmt(j.axis))
        o.append('    <limit lower="%.6f" upper="%.6f" effort="%g" velocity="%g"/>'
                 % (lo_deg * DEG, hi_deg * DEG, effort, velocity))
        o.append('    <dynamics damping="%g" friction="%g"/>'
                 % (DAMPING, SERIAL_FRICTION[j.name]))
    o.append("  </joint>")
    o.append("")


def emit_ros2_control(o):
    o.append('  <ros2_control name="RoboNexSystem" type="system">')
    o.append("    <hardware>")
    o.append("      <plugin>ign_ros2_control/IgnitionSystem</plugin>")
    o.append("    </hardware>")
    for name in JOINT_ORDER:
        effort, velocity, lo_deg, hi_deg = SERIAL_JOINTS[name]
        o.append('    <joint name="%s">' % name)
        o.append('      <command_interface name="effort">')
        o.append('        <param name="min">%g</param>' % -effort)
        o.append('        <param name="max">%g</param>' % effort)
        o.append("      </command_interface>")
        o.append('      <state_interface name="position">')
        o.append('        <param name="initial_value">0.0</param>')
        o.append("      </state_interface>")
        o.append('      <state_interface name="velocity"/>')
        o.append('      <state_interface name="effort"/>')
        o.append("    </joint>")
    o.append("  </ros2_control>")
    o.append("")


def emit_gazebo(o):
    o.append("  <gazebo>")
    o.append('    <plugin filename="ign_ros2_control-system"'
             ' name="ign_ros2_control::IgnitionROS2ControlPlugin">')
    o.append("      <parameters>%s</parameters>" % PARAMS_URI)
    o.append("      <robot_param>robot_description</robot_param>")
    o.append("      <robot_param_node>robot_state_publisher</robot_param_node>")
    o.append("    </plugin>")
    o.append("  </gazebo>")
    o.append("")

    o.append('  <gazebo reference="base_link">')
    o.append('    <sensor name="imu" type="imu">')
    o.append("      <always_on>1</always_on>")
    o.append("      <update_rate>200</update_rate>")
    o.append("      <topic>imu</topic>")
    o.append("    </sensor>")
    o.append("  </gazebo>")
    o.append("")

    for foot in FEET:
        o.append('  <gazebo reference="%s">' % foot)
        o.append("    <mu1>%g</mu1><mu2>%g</mu2>" % (FOOT_FRICTION, FOOT_FRICTION))
        o.append("    <self_collide>true</self_collide>")
        o.append('    <sensor name="%s_contact" type="contact">' % foot)
        o.append("      <always_on>1</always_on>")
        o.append("      <update_rate>200</update_rate>")
        o.append("      <topic>%s_contact</topic>" % foot)
        o.append("      <contact><collision>%s_collision</collision></contact>" % foot)
        o.append("    </sensor>")
        o.append("  </gazebo>")
        o.append("")

    for name in COLLISION_BOX:
        if name in FEET:
            continue
        o.append('  <gazebo reference="%s">' % name)
        o.append("    <mu1>%g</mu1><mu2>%g</mu2>" % (BODY_FRICTION, BODY_FRICTION))
        o.append("  </gazebo>")
    o.append("")


def main():
    parser = argparse.ArgumentParser(
        description="Build the RoboNex ROS 2 Xacro model from the source URDF."
    )
    parser.parse_args()

    links, joints, base = load_urdf()

    o = []
    o.append('<?xml version="1.0"?>')
    o.append('<robot name="robonex" xmlns:xacro="http://www.ros.org/wiki/xacro">')
    o.append("")
    for name, rgba in MATERIALS:
        o.append('  <material name="%s"><color rgba="%s"/></material>' % (name, rgba))
    o.append("")

    for name, lk in links.items():
        emit_link(o, name, lk)
    for j in joints.values():
        emit_joint(o, j)

    emit_ros2_control(o)
    emit_gazebo(o)
    o.append("</robot>")

    with open(OUT, "w", encoding="utf-8") as f:
        f.write("\n".join(o) + "\n")

    frozen = [j.name for j in joints.values() if j.name not in SERIAL_JOINTS]
    print("wrote %s" % OUT)
    print("  links            : %d  (total mass %.6f kg)"
          % (len(links), sum(lk.mass for lk in links.values())))
    print("  revolute / fixed : %d / %d" % (len(SERIAL_JOINTS), len(frozen)))
    print("  collision boxes  : %d" % len(COLLISION_BOX))
    print("  ros2_control     : %d joints, effort command interface, position/velocity/effort states"
          % len(JOINT_ORDER))
    print("  meshes           : %s" % (MESH_URI % "*.stl"))
    print()
    print("  this is a xacro only so that $(find %s) resolves; there are no macros." % PACKAGE)


if __name__ == "__main__":
    main()
