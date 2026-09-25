import os
import sys

import numpy as np

from repose import ASIS, HERE, ROOT, main as repose, read_stl, write_stl

sys.path.insert(0, os.path.join(ROOT, "_extract"))
from lower_model import R_FU, group_inertia

URDF_DIR = os.path.join(ROOT, "urdf")
MESH_DIR = os.path.join(ROOT, "meshes")
LOOPS_OUT = os.path.join(ROOT, "loop_closures.yaml")

ARM_LINKS = [n % s for s in ("l", "r") for n in (
    "%s_shoulder_pitch_link", "%s_shoulder_roll_link", "%s_shoulder_yaw_link", "%s_elbow_link")]
HEAD_LINKS = ["head_link"]
BASE_PARTS = {
    "edu": ["base_link", "rs02_l_hip_yaw", "rs02_r_hip_yaw"],
    "pro": ["base_link", "rs02_l_hip_yaw", "rs02_r_hip_yaw", "rs02_l_shoulder_pitch", "rs02_r_shoulder_pitch"],
    "max": ["base_link", "rs02_l_hip_yaw", "rs02_r_hip_yaw", "rs02_l_shoulder_pitch", "rs02_r_shoulder_pitch",
            "rs05_neck", "neck_BK_032"],
}
BASE_MESHES = {
    "edu": ["base_link", "rs02_l_hip_yaw", "rs02_r_hip_yaw"],
    "pro": ["base_link", "rs02_l_hip_yaw", "rs02_r_hip_yaw", "rs02_l_shoulder_pitch", "rs02_r_shoulder_pitch"],
    "max": ["base_link", "rs02_l_hip_yaw", "rs02_r_hip_yaw", "rs05_neck", "neck_mount", "rs02_l_shoulder_pitch",
            "rs02_r_shoulder_pitch"],
}
DROP = {"edu": set(ARM_LINKS) | set(HEAD_LINKS), "pro": set(HEAD_LINKS), "max": set()}
TITLE = {"edu": "lower body", "pro": "lower body + arms", "max": "lower body + arms + head"}
ACTUATED = ["l_hip_yaw_joint", "l_hip_pitch_joint", "l_hip_roll_joint", "l_knee_pitch_joint",
            "l_ankle_upper_joint", "l_ankle_lower_joint",
            "r_hip_yaw_joint", "r_hip_pitch_joint", "r_hip_roll_joint", "r_knee_pitch_joint",
            "r_ankle_lower_joint", "r_ankle_upper_joint"]


def f6(v):
    return " ".join("%.6f" % (0.0 if abs(x) < 5e-7 else x) for x in v)


def mesh_name(path):
    return os.path.splitext(os.path.basename(path))[0]


def main():
    L, J, loops, G, motion, report = repose()
    origin = {k: (motion[k][1] if k in motion else G[k]) for k in L}
    new_L = {}
    for name, link in L.items():
        R = motion[name][0] if name in motion else np.eye(3)
        new_L[name] = dict(mass=link["mass"], com=R @ link["com"], I=R @ link["I"] @ R.T,
                           meshes=[mesh_name(m[0]) for m in link["meshes"]])
    new_J = {}
    for name, j in J.items():
        new_J[name] = dict(j, xyz=origin[j["child"]] - origin[j["parent"]])

    os.makedirs(MESH_DIR, exist_ok=True)
    for name in motion:
        R = motion[name][0]
        for m in new_L[name]["meshes"]:
            tris = read_stl(os.path.join(ASIS, "meshes", m + ".stl"))
            out = tris.copy()
            out["v"] = (tris["v"].astype(np.float64).reshape(-1, 3) @ R.T).reshape(-1, 3, 3).astype(np.float32)
            out["n"] = (tris["n"].astype(np.float64) @ R.T).astype(np.float32)
            write_stl(os.path.join(MESH_DIR, m + ".stl"), out,
                      "RoboNex Ver.2 %s, mm, URDF axes, frame of %s, level-sole zero" % (m, name))

    new_loops = {"ball_loops": [], "pin_loops": loops["pin_loops"]}
    closure = {}
    for lp in loops["ball_loops"]:
        Rp = motion[lp["parent"]][0] if lp["parent"] in motion else np.eye(3)
        Rc = motion[lp["child"]][0] if lp["child"] in motion else np.eye(3)
        pxyz = Rp @ np.array(lp["parent_xyz"])
        cxyz = Rc @ np.array(lp["child_xyz"])
        closure[lp["name"]] = np.linalg.norm(origin[lp["parent"]] + pxyz - origin[lp["child"]] - cxyz)
        new_loops["ball_loops"].append(dict(lp, parent_xyz=pxyz, child_xyz=cxyz))
    for lp in loops["pin_loops"]:
        closure[lp["name"]] = np.linalg.norm(origin[lp["parent"]] + np.array(lp["parent_xyz"])
                                             - origin[lp["child"]] - np.array(lp["child_xyz"]))

    y = ["ball_loops:"]
    for b in new_loops["ball_loops"]:
        y += ["  - name: %s" % b["name"], "    parent: %s" % b["parent"], "    child: %s" % b["child"],
              "    parent_xyz: [%s]" % ", ".join("%.6f" % v for v in b["parent_xyz"]),
              "    child_xyz: [%s]" % ", ".join("%.6f" % v for v in b["child_xyz"]), ""]
    y += ["", "pin_loops:"]
    for p in new_loops["pin_loops"]:
        y += ["  - name: %s" % p["name"], "    parent: %s" % p["parent"], "    child: %s" % p["child"],
              "    parent_xyz: [%s]" % ", ".join("%.6f" % v for v in p["parent_xyz"]),
              "    child_xyz: [%s]" % ", ".join("%.6f" % v for v in p["child_xyz"]),
              "    axis: [%s]" % ", ".join("%.1f" % v for v in p["axis"]), ""]
    y += ["", "ball_upgrades:"] + ["  - %s" % n for n in loops["ball_upgrades"]]
    y += ["", "ball_limit_deg: %.1f" % loops["ball_limit_deg"], "", "", "actuated_joints:"]
    y += ["  - %s" % n for n in ACTUATED]
    with open(LOOPS_OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(y) + "\n")

    base_check = None
    summary = {}
    for variant in ("edu", "pro", "max"):
        m, c_f, I_f = group_inertia(BASE_PARTS[variant])
        base = dict(mass=m, com=R_FU @ c_f / 1000.0, I=R_FU @ I_f @ R_FU.T, meshes=BASE_MESHES[variant])
        if variant == "max":
            base_check = (abs(m - new_L["base_link"]["mass"]), np.abs(base["com"] - new_L["base_link"]["com"]).max(),
                          np.abs(base["I"] - new_L["base_link"]["I"]).max())
        links = {k: (base if k == "base_link" else v) for k, v in new_L.items() if k not in DROP[variant]}
        joints = {k: v for k, v in new_J.items() if v["child"] in links and v["parent"] in links}
        o = ['<?xml version="1.0"?>', '<robot name="robonex_%s">' % variant, "",
             "    <!-- RoboNex Ver.2 %s (%s): new_urdf/build/make_ver2.py from the Fusion 360 extraction in" % (variant, TITLE[variant]),
             "         new_urdf/asis, re-posed so that both soles are level at the zero pose (model-report.md section 5).",
             "         Every link frame is parallel to base_link at zero: X forward, Y left, Z up. Meshes in link frames (mm).",
             "         Joint limits are -pi..+pi until the Ver.2 joint-limit sweep. -->", "",
             '    <material name="black">', '        <color rgba="0.05 0.05 0.05 1.0"/>', "    </material>", "",
             '    <material name="gray">', '        <color rgba="0.647 0.647 0.647 1.0"/>', "    </material>", ""]
        for name, lk in links.items():
            I = lk["I"]
            o += ["    <!-- [Link] %s -->" % name, '    <link name="%s">' % name, "        <inertial>",
                  '            <origin xyz="%s" rpy="0 0 0"/>' % f6(lk["com"]),
                  '            <mass value="%.6f"/>' % lk["mass"],
                  '            <inertia ixx="%.8f" ixy="%.8f" ixz="%.8f" iyy="%.8f" iyz="%.8f" izz="%.8f"/>'
                  % (I[0, 0], I[0, 1], I[0, 2], I[1, 1], I[1, 2], I[2, 2]), "        </inertial>"]
            for tag in ("visual", "collision"):
                o.append("")
                for msh in lk["meshes"]:
                    o += ["        <%s>" % tag, '            <origin xyz="0 0 0" rpy="0 0 0"/>', "            <geometry>",
                          '                <mesh filename="../meshes/%s.stl" scale="0.001 0.001 0.001"/>' % msh,
                          "            </geometry>"]
                    if tag == "visual":
                        o.append('            <material name="%s"/>' % ("gray" if msh.startswith("rs0") else "black"))
                    o.append("        </%s>" % tag)
            o += ["    </link>", ""]
        for name, j in joints.items():
            o += ["    <!-- [Joint] %s -->" % name, '    <joint name="%s" type="%s">' % (name, j["type"]),
                  '        <parent link="%s"/>' % j["parent"], '        <child link="%s"/>' % j["child"],
                  '        <origin xyz="%s" rpy="0 0 0"/>' % f6(j["xyz"])]
            if j["type"] == "revolute":
                lim = j["limit"]
                o += ['        <axis xyz="%s"/>' % " ".join("%d" % int(round(a)) for a in j["axis"]),
                      '        <limit lower="%.6f" upper="%.6f" effort="%.1f" velocity="%.1f"/>'
                      % (lim["lower"], lim["upper"], lim["effort"], lim["velocity"]),
                      '        <dynamics damping="0.0" friction="0.0"/>']
            o += ["    </joint>", ""]
        o.append("</robot>")
        path = os.path.join(URDF_DIR, "robonex_%s.urdf" % variant)
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write("\n".join(o) + "\n")
        summary[variant] = (len(links), len(joints), sum(l["mass"] for l in links.values()), path)
    return report, closure, base_check, summary


if __name__ == "__main__":
    report, closure, base_check, summary = main()
    print("loop closure at the new zero (mm):", {k: round(v * 1000, 6) for k, v in closure.items()})
    print("max base_link vs as-is base_link: dmass %.2e kg, dCOM %.2e m, dI %.2e" % base_check)
    for v, (nl, nj, m, p) in summary.items():
        print("%s: %d links, %d joints, %.5f kg -> %s" % (v, nl, nj, m, p))
