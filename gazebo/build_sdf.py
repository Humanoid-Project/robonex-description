#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from robonex_common import load_urdf, load_loops, fmt, ROOT
from robonex_serial import (
    DEG, SPAWN_HEIGHT, MUJOCO_SPAWN_HEIGHT, SERIAL_GAINS, DAMPING, TIMESTEP,
    SERIAL_JOINTS, JOINT_ORDER, COLLISION_BOX, FEET,
    FOOT_FRICTION, BODY_FRICTION,
)

MODEL_OUT = os.path.join(ROOT, "gazebo", "robonex.sdf")
WORLD_OUT = os.path.join(ROOT, "gazebo", "robonex_world.sdf")
EMPTY_OUT = os.path.join(ROOT, "gazebo", "empty_world.sdf")

MESH_URI = "../meshes/%s"

PLUGIN = 'ignition-gazebo-%s-system'
KLASS = 'ignition::gazebo::systems::%s'


def pose6(xyz, rpy):
    return "%s %s" % (fmt(xyz), fmt(rpy))


def emit_link(o, name, lk, parent_joint):
    o.append('    <link name="%s">' % name)
    if parent_joint is not None:
        o.append('      <pose relative_to="%s">%s</pose>'
                 % (parent_joint.parent, pose6(parent_joint.xyz, parent_joint.rpy)))

    if lk.mass > 0.0:
        ixx, ixy, ixz, iyy, iyz, izz = lk.inertia
        o.append("      <inertial>")
        o.append("        <pose>%s 0 0 0</pose>" % fmt(lk.com))
        o.append("        <mass>%.6g</mass>" % lk.mass)
        o.append("        <inertia>")
        for tag, val in (("ixx", ixx), ("ixy", ixy), ("ixz", ixz),
                         ("iyy", iyy), ("iyz", iyz), ("izz", izz)):
            o.append("          <%s>%.8g</%s>" % (tag, val, tag))
        o.append("        </inertia>")
        o.append("      </inertial>")

    for i, g in enumerate(lk.visuals):
        o.append('      <visual name="%s_visual_%d">' % (name, i))
        o.append("        <pose>%s</pose>" % pose6(g.xyz, g.rpy))
        o.append("        <geometry><mesh>")
        o.append("          <uri>%s</uri>" % (MESH_URI % g.mesh))
        o.append("          <scale>%s</scale>" % fmt(g.scale))
        o.append("        </mesh></geometry>")
        o.append("      </visual>")

    box = COLLISION_BOX.get(name)
    if box is not None:
        size, centre = box
        mu = FOOT_FRICTION if name in FEET else BODY_FRICTION
        o.append('      <collision name="%s_collision">' % name)
        o.append("        <pose>%s 0 0 0</pose>" % fmt(centre))
        o.append("        <geometry><box><size>%s</size></box></geometry>" % fmt(size))
        o.append("        <surface><friction><ode>")
        o.append("          <mu>%g</mu><mu2>%g</mu2>" % (mu, mu))
        o.append("        </ode></friction></surface>")
        o.append("      </collision>")

    if name in FEET:
        o.append('      <self_collide>true</self_collide>')
        o.append('      <sensor name="%s_contact" type="contact">' % name)
        o.append("        <always_on>1</always_on>")
        o.append("        <update_rate>200</update_rate>")
        o.append("        <topic>/robonex/%s/contact</topic>" % name)
        o.append("        <contact><collision>%s_collision</collision></contact>" % name)
        o.append("      </sensor>")

    if name == "base_link":
        o.append('      <sensor name="imu" type="imu">')
        o.append("        <always_on>1</always_on>")
        o.append("        <update_rate>200</update_rate>")
        o.append("        <topic>/robonex/imu</topic>")
        o.append("        <pose>0 0 0 0 0 0</pose>")
        o.append("      </sensor>")

    o.append("    </link>")
    o.append("")


def emit_joint(o, j):
    spec = SERIAL_JOINTS.get(j.name)
    if spec is None:
        o.append('    <joint name="%s" type="fixed">' % j.name)
        o.append("      <parent>%s</parent>" % j.parent)
        o.append("      <child>%s</child>" % j.child)
        o.append("    </joint>")
        return

    effort, velocity, lo_deg, hi_deg = spec
    o.append('    <joint name="%s" type="revolute">' % j.name)
    o.append("      <parent>%s</parent>" % j.parent)
    o.append("      <child>%s</child>" % j.child)
    o.append("      <axis>")
    o.append("        <xyz>%s</xyz>" % fmt(j.axis))
    o.append("        <limit>")
    o.append("          <lower>%.6f</lower><upper>%.6f</upper>"
             % (lo_deg * DEG, hi_deg * DEG))
    o.append("          <effort>%g</effort><velocity>%g</velocity>" % (effort, velocity))
    o.append("        </limit>")
    o.append("        <dynamics><damping>%g</damping><friction>0</friction></dynamics>"
             % DAMPING)
    o.append("      </axis>")
    o.append("    </joint>")


def emit_controllers(o):
    o.append("")
    for name in JOINT_ORDER:
        effort = SERIAL_JOINTS[name][0]
        o.append('    <plugin filename="%s" name="%s">'
                 % (PLUGIN % "joint-position-controller",
                    KLASS % "JointPositionController"))
        o.append("      <joint_name>%s</joint_name>" % name)
        o.append("      <topic>/robonex/%s/cmd</topic>" % name)
        kp, kd = SERIAL_GAINS[name]
        o.append("      <p_gain>%g</p_gain><i_gain>0</i_gain><d_gain>%g</d_gain>" % (kp, kd))
        o.append("      <cmd_max>%g</cmd_max><cmd_min>%g</cmd_min>" % (effort, -effort))
        o.append("    </plugin>")
    o.append('    <plugin filename="%s" name="%s">'
             % (PLUGIN % "joint-state-publisher", KLASS % "JointStatePublisher"))
    o.append("      <topic>/robonex/joint_state</topic>")
    o.append("    </plugin>")


def build_model(links, joints, height):
    parent_joint = {j.child: j for j in joints.values()}

    o = []
    o.append('  <model name="robonex">')
    o.append("    <pose>0 0 %g 0 0 0</pose>" % height)
    o.append("    <self_collide>false</self_collide>")
    o.append("")

    for name, lk in links.items():
        emit_link(o, name, lk, parent_joint.get(name))

    for j in joints.values():
        emit_joint(o, j)

    emit_controllers(o)
    o.append("  </model>")
    return o


def build_world(model=None):
    w = []
    w.append('  <world name="robonex_world">')
    w.append('    <physics name="%gms" type="dart">' % (TIMESTEP * 1000))
    w.append("      <max_step_size>%g</max_step_size>" % TIMESTEP)
    w.append("      <real_time_factor>1.0</real_time_factor>")
    w.append("    </physics>")
    for system, klass in (("physics", "Physics"),
                          ("user-commands", "UserCommands"),
                          ("scene-broadcaster", "SceneBroadcaster"),
                          ("contact", "Contact"),
                          ("imu", "Imu")):
        w.append('    <plugin filename="%s" name="%s"/>' % (PLUGIN % system, KLASS % klass))
    # robonex's base has no parent joint, so robot_state_publisher only ever
    # emits the RELATIVE joint tree (base_link -> ... -> feet); nothing carries
    # base_link's own pose into ROS 2. SceneBroadcaster (above) already reports
    # every entity's world pose on .../dynamic_pose/info, robonex's own root
    # included - no extra plugin needed. (PosePublisher looked like the more
    # direct tool, but ign-gazebo6 refuses to attach it at world scope: "should
    # be attached to a model entity" - and attached to a model it only reports
    # *nested* models inside that model, which robonex has none of, so it is a
    # dead end here.) See ros2/scripts/base_pose_tf.py for how the one entry we
    # want gets pulled out of that topic and republished as world -> base_link.
    w.append("    <gravity>0 0 -9.81</gravity>")
    w.append("")
    w.append('    <light type="directional" name="sun">')
    w.append("      <pose>0 0 10 0 0 0</pose>")
    w.append("      <direction>-0.3 0.2 -0.9</direction>")
    w.append("      <diffuse>1 1 1 1</diffuse>")
    w.append("      <specular>0.2 0.2 0.2 1</specular>")
    w.append("      <cast_shadows>true</cast_shadows>")
    w.append("    </light>")
    w.append("")
    w.append('    <model name="ground_plane">')
    w.append("      <static>true</static>")
    w.append('      <link name="link">')
    w.append('        <collision name="collision">')
    w.append("          <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>")
    w.append("          <surface><friction><ode>")
    w.append("            <mu>%g</mu><mu2>%g</mu2>" % (FOOT_FRICTION, FOOT_FRICTION))
    w.append("          </ode></friction></surface>")
    w.append("        </collision>")
    w.append('        <visual name="visual">')
    w.append("          <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>")
    w.append("          <material>")
    w.append("            <ambient>0.4 0.45 0.5 1</ambient>")
    w.append("            <diffuse>0.5 0.55 0.6 1</diffuse>")
    w.append("          </material>")
    w.append("        </visual>")
    w.append("      </link>")
    w.append("    </model>")
    if model is not None:
        w.append("")
        w.extend(model)
    w.append("  </world>")
    return w


def write(path, body):
    with open(path, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0"?>\n')
        f.write("<!-- generated by gazebo/build_sdf.py - do not edit by hand -->\n")
        f.write('<sdf version="1.9">\n')
        f.write("\n".join(body) + "\n")
        f.write("</sdf>\n")


def main():
    drop = "--drop" in sys.argv
    height = MUJOCO_SPAWN_HEIGHT if drop else SPAWN_HEIGHT

    links, joints, base = load_urdf()
    loops = load_loops()

    model = build_model(links, joints, height)
    write(MODEL_OUT, model)
    write(WORLD_OUT, build_world(model))
    write(EMPTY_OUT, build_world())

    frozen = [j.name for j in joints.values() if j.name not in SERIAL_JOINTS]
    total_mass = sum(lk.mass for lk in links.values())

    print("wrote %s" % MODEL_OUT)
    print("wrote %s" % WORLD_OUT)
    print("wrote %s  (no robot, for the ROS 2 launch)" % EMPTY_OUT)
    print("  links          : %d  (total mass %.6f kg)" % (len(links), total_mass))
    print("  revolute       : %d  serial-equivalent, one per motor" % len(SERIAL_JOINTS))
    print("  fixed          : %d  crank/coupler links frozen at the home pose" % len(frozen))
    print("  collision      : %d boxes, %d links left without collision"
          % (len(COLLISION_BOX), len(links) - len(COLLISION_BOX)))
    print("  controllers    : %d position, RobStride MIT gains through the linkage"
          % len(JOINT_ORDER))
    for n in ("l_hip_yaw_joint", "l_knee_joint", "l_ankle_pitch_joint"):
        print("                   %-20s p=%-6g d=%g" % ((n,) + SERIAL_GAINS[n]))
    print("  sensors        : 1 imu, 2 foot contact")
    print("  base           : free-floating at z = %.4f m%s"
          % (height, "  (--drop: MuJoCo height)" if drop else "  (soles on the ground)"))
    print()
    print("  the %d loop closures in loop_closures.yaml are NOT emitted." % (
        len(loops.get("pin_loops", [])) + len(loops.get("ball_loops", []))))
    print("  gz-physics gives a link exactly one parent joint, so the knee four-bar")
    print("  and the ankle differential cannot be closed. The joints they drive are")
    print("  actuated directly instead - see gazebo/README.md.")


if __name__ == "__main__":
    main()
