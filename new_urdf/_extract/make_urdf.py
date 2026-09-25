"""Write new_urdf/urdf/robonex.urdf and new_urdf/loop_closures.yaml for the Ver.2 lower body.

Run make_meshes.py first: every visual/collision mesh is already in URDF axes and in its link frame,
so every visual origin is 0 0 0 / rpy 0 0 0.
"""
import os

import numpy as np

from lower_model import HERE, LINK_ORDER, LINK_PARTS, R_FU, build, group_inertia, link_origins_f, mesh_of, to_urdf

ROOT = os.path.dirname(HERE)
URDF_OUT = os.path.join(ROOT, "urdf", "robonex.urdf")
LOOPS_OUT = os.path.join(ROOT, "loop_closures.yaml")

ACTUATED = ["l_hip_yaw_joint", "l_hip_pitch_joint", "l_hip_roll_joint", "l_knee_pitch_joint",
            "l_ankle_upper_joint", "l_ankle_lower_joint",
            "r_hip_yaw_joint", "r_hip_pitch_joint", "r_hip_roll_joint", "r_knee_pitch_joint",
            "r_ankle_lower_joint", "r_ankle_upper_joint"]
JOINT_ORDER = [n % s for s in ("l", "r") for n in (
    "%s_hip_yaw_joint", "%s_hip_pitch_joint", "%s_hip_roll_joint", "%s_knee_joint", "%s_knee_pitch_joint",
    "%s_knee_coupler_joint_a")] + [
    "l_ankle_upper_joint", "l_ankle_lower_joint", "r_ankle_lower_joint", "r_ankle_upper_joint",
    "l_ankle_coupler_joint_a", "l_ankle_coupler_joint_b", "r_ankle_coupler_joint_a", "r_ankle_coupler_joint_b",
    "l_ankle_roll_joint", "r_ankle_roll_joint", "l_ankle_pitch_joint", "r_ankle_pitch_joint",
    "neck_pitch_joint"] + [n % s for s in ("l", "r") for n in (
    "%s_shoulder_pitch_joint", "%s_shoulder_roll_joint", "%s_shoulder_yaw_joint", "%s_elbow_joint")]
ARM_JOINTS = [n % s for s in ("l", "r") for n in (
    "%s_shoulder_pitch_joint", "%s_shoulder_roll_joint", "%s_shoulder_yaw_joint", "%s_elbow_joint")]
CAN_ID = {"l_hip_yaw_joint": ("RS02", 1), "l_hip_pitch_joint": ("RS03", 2), "l_hip_roll_joint": ("RS03", 3),
          "l_knee_pitch_joint": ("RS03", 4), "l_ankle_upper_joint": ("RS02", 5), "l_ankle_lower_joint": ("RS02", 6),
          "r_hip_yaw_joint": ("RS02", 7), "r_hip_pitch_joint": ("RS03", 8), "r_hip_roll_joint": ("RS03", 9),
          "r_knee_pitch_joint": ("RS03", 10), "r_ankle_upper_joint": ("RS02", 11), "r_ankle_lower_joint": ("RS02", 12)}


def f6(v):
    return " ".join("%.6f" % (0.0 if abs(x) < 5e-7 else x) for x in v)


def main():
    J, loops = build()
    origin_f = link_origins_f(J)
    origin_u = {k: to_urdf(v) / 1000.0 for k, v in origin_f.items()}  # metres

    o = []
    o.append('<?xml version="1.0"?>')
    o.append('<robot name="robonex">')
    o.append("")
    o.append("    <!-- RoboNex Ver.2, generated from Fusion 360 '2_RoboNex_test v1' by new_urdf/_extract (arms from the copy")
    o.append("         '2_RoboNex_urdf' with both elbows rotated 90 deg forward; every other part unchanged).")
    o.append("         Zero pose = that assembly pose (legs straight, elbows 90 deg forward); every link frame is parallel")
    o.append("         to base_link.")
    o.append("         Axes: X forward, Y left, Z up. Meshes are in their link frame (mm, scale 0.001).")
    o.append("         Actuated joint limits are -pi..+pi until the Ver.2 joint-limit sweep.")
    o.append("         Arm joints (RS02) and neck_pitch_joint (RS05) are not in robonex-common. -->")
    o.append("")
    o.append('    <material name="black">')
    o.append('        <color rgba="0.05 0.05 0.05 1.0"/>')
    o.append("    </material>")
    o.append("")
    o.append('    <material name="gray">')
    o.append('        <color rgba="0.647 0.647 0.647 1.0"/>')
    o.append("    </material>")
    o.append("")

    mass_total = 0.0
    link_rows = {}
    for link in LINK_ORDER:
        parts = LINK_PARTS[link]
        m, com_f, I_f = group_inertia(parts)
        mass_total += m
        com_u = to_urdf(com_f) / 1000.0 - origin_u[link]
        I_u = R_FU @ I_f @ R_FU.T
        link_rows[link] = (m, com_u, I_u)
        o.append("    <!-- [Link] %s (%s) -->" % (link, ", ".join(parts)))
        o.append('    <link name="%s">' % link)
        o.append("        <inertial>")
        o.append('            <origin xyz="%s" rpy="0 0 0"/>' % f6(com_u))
        o.append('            <mass value="%.6f"/>' % m)
        o.append('            <inertia ixx="%.8f" ixy="%.8f" ixz="%.8f" iyy="%.8f" iyz="%.8f" izz="%.8f"/>'
                 % (I_u[0, 0], I_u[0, 1], I_u[0, 2], I_u[1, 1], I_u[1, 2], I_u[2, 2]))
        o.append("        </inertial>")
        mesh_names = list(dict.fromkeys(mesh_of(p) for p in parts))
        for tag in ("visual", "collision"):
            o.append("")
            for p in mesh_names:
                o.append("        <%s>" % tag)
                o.append('            <origin xyz="0 0 0" rpy="0 0 0"/>')
                o.append("            <geometry>")
                o.append('                <mesh filename="../meshes/%s.stl" scale="0.001 0.001 0.001"/>' % p)
                o.append("            </geometry>")
                if tag == "visual":
                    o.append('            <material name="%s"/>' % ("gray" if p.startswith("rs0") else "black"))
                o.append("        </%s>" % tag)
        o.append("    </link>")
        o.append("")

    for jn in JOINT_ORDER:
        j = J[jn]
        xyz = to_urdf(j["p_f"]) / 1000.0 - origin_u[j["parent"]]
        o.append("    <!-- [Joint] %s -->" % jn)
        o.append('    <joint name="%s" type="%s">' % (jn, j["type"]))
        o.append('        <parent link="%s"/>' % j["parent"])
        o.append('        <child link="%s"/>' % j["child"])
        o.append('        <origin xyz="%s" rpy="0 0 0"/>' % f6(xyz))
        if j["type"] == "revolute":
            o.append('        <axis xyz="%s"/>' % " ".join("%d" % int(round(a)) for a in j["axis"]))
            lower, upper = j["lower"], j["upper"]
            if jn in CAN_ID:
                # Actuated joints: +/-pi until the Ver.2 joint-limit sweep.
                lower, upper = -3.141593, 3.141593
                model, cid = CAN_ID[jn]
                note = "; knee four-bar toggles 20 deg toward extension" if jn.endswith("knee_pitch_joint") else ""
                o.append("        <!-- %s ID%d : limit -180 ~ +180 deg until the Ver.2 joint-limit sweep%s -->"
                         % (model, cid, note))
            elif jn in ARM_JOINTS:
                o.append("        <!-- RS02 (no CAN ID yet, not in robonex-common) : limit -180 ~ +180 deg until measured%s -->"
                         % ("; zero = elbow bent 90 deg forward" if jn.endswith("elbow_joint") else ""))
            elif jn == "neck_pitch_joint":
                o.append("        <!-- RS05 (no CAN ID yet, not in robonex-common) : limit -180 ~ +180 deg until measured;"
                         " effort/velocity = official peak torque / no-load speed -->")
            o.append('        <limit lower="%.6f" upper="%.6f" effort="%.1f" velocity="%.1f"/>'
                     % (lower, upper, j["effort"], j["velocity"]))
            o.append('        <dynamics damping="0.0" friction="0.0"/>')
        o.append("    </joint>")
        o.append("")
    o.append("</robot>")

    os.makedirs(os.path.dirname(URDF_OUT), exist_ok=True)
    with open(URDF_OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(o) + "\n")

    # loop_closures.yaml, same structure as robonex-description
    y = ["ball_loops:"]
    for b in loops["ball"]:
        pu = to_urdf(b["p_f"]) / 1000.0
        y += ["  - name: %s" % b["name"], "    parent: %s" % b["parent"], "    child: %s" % b["child"],
              "    parent_xyz: [%s]" % ", ".join("%.6f" % v for v in pu - origin_u[b["parent"]]),
              "    child_xyz: [%s]" % ", ".join("%.6f" % v for v in pu - origin_u[b["child"]]), ""]
    y += ["", "pin_loops:"]
    for p in loops["pin"]:
        pu = to_urdf(p["p_f"]) / 1000.0
        y += ["  - name: %s" % p["name"], "    parent: %s" % p["parent"], "    child: %s" % p["child"],
              "    parent_xyz: [%s]" % ", ".join("%.6f" % v for v in pu - origin_u[p["parent"]]),
              "    child_xyz: [%s]" % ", ".join("%.6f" % v for v in pu - origin_u[p["child"]]),
              "    axis: [%s]" % ", ".join("%.1f" % v for v in p["axis"]), ""]
    y += ["", "ball_upgrades:"] + ["  - %s" % n for n in (
        "l_ankle_coupler_joint_a", "l_ankle_coupler_joint_b", "r_ankle_coupler_joint_a", "r_ankle_coupler_joint_b")]
    y += ["", "ball_limit_deg: 15.0", "", "", "actuated_joints:"] + ["  - %s" % n for n in ACTUATED]
    with open(LOOPS_OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(y) + "\n")

    print("wrote", URDF_OUT)
    print("wrote", LOOPS_OUT)
    print("links %d, joints %d, total mass %.5f kg" % (len(LINK_ORDER), len(JOINT_ORDER), mass_total))


if __name__ == "__main__":
    main()
