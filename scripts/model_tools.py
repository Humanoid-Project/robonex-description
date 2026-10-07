"""RoboNex model tools: build the model from a Fusion 360 export package, solve the constants, check it.

Usage: python -I scripts/model_tools.py --version {ver2,ver2-2} <command> [options]
       (build and constants: ver2-2 only; ver2's urdf/, meshes/ and ver2_constants.json are frozen)

  build --package DIR     re-pose the Fusion export DIR (ver2-2_Edu.urdf or urdf/robonex.urdf, loop_closures.yaml,
                          meshes/*.stl) to level soles, apply MEASURED_LIMITS; writes meshes/, loop_closures.yaml,
                          urdf/robonex_edu.urdf
  constants               solve the home pose on mujoco/robot/edu/scene_fixed.xml; writes ver2-2_constants.json
  constants --seed-limits-only
                          write only provisional_limits (bootstrap before the first fixed-base MJCF)
  loops [--pose zero|home] [--scene PATH]
                          loop-closure residuals of a MuJoCo scene
  limits [--variant edu]  interactive joint viewer with the loops solved
  check [--urdf PATH]     link/joint counts, total mass, SPD inertias, L/R mirror residuals

Regeneration order (from robonex-description/):
  1. .venv/bin/python -I scripts/model_tools.py --version ver2-2 build --package DIR
  2. .venv/bin/python -I ver2-2/mujoco/build_mjcf.py --variant edu --fixed-base
     (first build without ver2-2_constants.json: run `model_tools.py --version ver2-2 constants --seed-limits-only`
     before it)
  3. .venv/bin/python -I scripts/model_tools.py --version ver2-2 constants
  4. .venv/bin/python -I ver2-2/mujoco/build_mjcf.py --variant edu
  5. .venv/bin/python -I ver2-2/isaac/build_isaac_urdf.py --variant edu --collision mesh   (and --collision box)
  6. .venv/bin/python -I scripts/model_tools.py --version ver2-2 loops --pose home   (and check, limits)
"""
import argparse
import json
import math
import os
import shutil
import struct
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
GENERATOR_VERSIONS = ("ver2-2",)
ROOT = URDF_DIR = MESH_DIR = LOOPS_OUT = CONSTANTS_OUT = SCENE_FIXED = None


def select_version():
    global ROOT, URDF_DIR, MESH_DIR, LOOPS_OUT, CONSTANTS_OUT, SCENE_FIXED
    sys.path.insert(0, HERE)
    import model_io
    ROOT = model_io.ROOT
    URDF_DIR = os.path.join(ROOT, "urdf")
    MESH_DIR = model_io.MESH_DIR
    LOOPS_OUT = model_io.LOOPS_PATH
    CONSTANTS_OUT = model_io.CONSTANTS_PATH
    SCENE_FIXED = os.path.join(ROOT, "mujoco", "robot", "edu", "scene_fixed.xml")
    return model_io.VERSION

VARIANT = "edu"
TITLE = "lower body, battery and e-stop split out of base_link"
D = math.pi / 180.0
MEASURED_LIMITS = {
    "l_hip_yaw_joint": (-48 * D, 48 * D), "r_hip_yaw_joint": (-48 * D, 48 * D),
    "l_hip_pitch_joint": (-95 * D, 95 * D), "r_hip_pitch_joint": (-95 * D, 95 * D),
    "l_hip_roll_joint": (-120 * D, 10 * D), "r_hip_roll_joint": (-10 * D, 120 * D),
}
ACTUATED = ["l_hip_yaw_joint", "l_hip_pitch_joint", "l_hip_roll_joint", "l_knee_pitch_joint",
            "l_ankle_upper_joint", "l_ankle_lower_joint",
            "r_hip_yaw_joint", "r_hip_pitch_joint", "r_hip_roll_joint", "r_knee_pitch_joint",
            "r_ankle_lower_joint", "r_ankle_upper_joint"]

HIP_PITCH = 0.1
KNEE_OUT = 0.29669690697178175
ANKLE_PITCH = KNEE_OUT - HIP_PITCH
PROVISIONAL_LIMITS = {
    "l_hip_yaw_joint": [-0.837758, 0.837758], "l_hip_pitch_joint": [-1.658063, 1.658063],
    "l_hip_roll_joint": [-2.094395, 0.174533], "l_knee_pitch_joint": [-1.221730, 0.174533],
    "l_ankle_upper_joint": [-0.279253, 0.872665], "l_ankle_lower_joint": [-0.872665, 0.523599],
    "r_hip_yaw_joint": [-0.837758, 0.837758], "r_hip_pitch_joint": [-1.658063, 1.658063],
    "r_hip_roll_joint": [-0.174533, 2.094395], "r_knee_pitch_joint": [-0.174533, 1.221730],
    "r_ankle_upper_joint": [-0.872665, 0.279253], "r_ankle_lower_joint": [-0.523599, 0.872665],
}
PROVISIONAL_LIMITS_NOTE = (
    "hips: Ver.2 limits set by the user in the viewer (2026-09-26): yaw +-48 (the hip-yaw link touches the other leg's "
    "hip-pitch motor at 50), pitch +-95 (user 2026-09-27: CAD hip_roll_link-base_link contact from -96 backward), roll "
    "-120..+10 (left, mirrored right); knee crank -70..+10 (CAD hip_roll_link-knee_coupler_link contact "
    "from +13, knee output peaks near -80); ankle upper -16..+50, lower -50..+30 with a coupled foot "
    "roll limit of 12 deg (fcl crank-pair scan, user 2026-09-27)")
UPPER_BODY_LIMITS_NOTE = (
    "neck pitch +-73 deg and shoulder pitch +-100 deg set by the user (2026-10-01, bench); other arm joints +-45 deg are "
    "robonex-common v1.2.0 AUXILIARY_JOINTS PLACEHOLDERS, not measured (open item H29); "
    "pro/max only, edu has no upper-body joints")
COLLISION_LINKS = ["base_link", "estop_link"] + ["%s_%s" % (s, n) for s in "lr" for n in (
    "hip_yaw_link", "hip_pitch_link", "hip_roll_link", "knee_link", "ankle_link", "foot")]
SPAWN_MARGIN = 0.006

STL_DTYPE = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])


def read_stl(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    return np.frombuffer(b[84:], dtype=STL_DTYPE, count=n).copy()


PARTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "parts")
PART_PREFIXES = ("rs02_", "rs03_")


def part_fit(V):
    ext = V.max(0) - V.min(0)
    k = int(np.argmin(ext))
    ij = [i for i in range(3) if i != k]
    zc = (V[:, k].max() + V[:, k].min()) / 2
    mid = V[np.abs(V[:, k] - zc) < ext[k] * 0.15][:, ij]
    ang = np.arctan2(mid[:, 1] - mid[:, 1].mean(), mid[:, 0] - mid[:, 0].mean())
    pts = []
    for a in np.linspace(-np.pi, np.pi, 36, endpoint=False):
        sel = mid[(ang >= a) & (ang < a + 2 * np.pi / 36)]
        if len(sel):
            pts.append(sel[np.argmax(np.linalg.norm(sel - mid.mean(0), axis=1))])
    xy = np.array(pts)
    A = np.c_[2 * xy, np.ones(len(xy))]
    cx, cy, _ = np.linalg.lstsq(A, (xy ** 2).sum(1), rcond=None)[0]
    centre = np.zeros(3)
    centre[ij] = (cx, cy)
    centre[k] = zc
    rel = V - centre
    zz = rel[:, k]
    r = np.linalg.norm(rel[:, ij], axis=1)
    lo = r[zz < zz.min() + 3.0].max()
    hi = r[zz > zz.max() - 3.0].max()
    axis = np.zeros(3)
    axis[k] = 1.0 if lo < hi else -1.0
    x = np.zeros(3)
    x[ij[0]] = 1.0
    return centre, np.c_[x, np.cross(axis, x), axis]


def replace_part_mesh(path, mesh):
    prefix = next((p for p in PART_PREFIXES if mesh.startswith(p)), None)
    if prefix is None:
        return False
    canon = read_stl(os.path.join(PARTS_DIR, prefix[:-1] + ".stl"))
    V = np.unique(read_stl(path)["v"].reshape(-1, 3), axis=0).astype(np.float64)
    centre, R = part_fit(V)
    out = canon.copy()
    T = canon["v"].astype(np.float64).reshape(-1, 3) @ R.T + centre
    out["v"] = T.reshape(-1, 3, 3).astype(np.float32)
    tri = out["v"].astype(np.float64)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    out["n"] = (n / np.maximum(np.linalg.norm(n, axis=1)[:, None], 1e-30)).astype(np.float32)
    write_stl(path, out, "RoboNex simplified %s motor (stepped cylinder, measured dims), mm, link frame of %s"
              % (prefix[:-1].upper(), mesh))
    return True


def write_stl(path, tris, header):
    with open(path, "wb") as f:
        f.write(header.encode("ascii")[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(tris.tobytes())


def load_urdf(path):
    r = ET.parse(path).getroot()
    links, joints = {}, {}
    for l in r.findall("link"):
        i = l.find("inertial")
        d = {"name": l.get("name"), "meshes": [], "col": []}
        if i is not None:
            I = i.find("inertia").attrib
            d["mass"] = float(i.find("mass").get("value"))
            d["com"] = np.array([float(v) for v in i.find("origin").get("xyz").split()])
            d["com_rpy"] = i.find("origin").get("rpy")
            d["I"] = np.array([[float(I["ixx"]), float(I["ixy"]), float(I["ixz"])],
                               [float(I["ixy"]), float(I["iyy"]), float(I["iyz"])],
                               [float(I["ixz"]), float(I["iyz"]), float(I["izz"])]])
        for tag, key in (("visual", "meshes"), ("collision", "col")):
            for v in l.findall(tag):
                m = v.find("geometry/mesh")
                o = v.find("origin")
                d[key].append((m.get("filename") if m is not None else None,
                               o.get("xyz") if o is not None else None, o.get("rpy") if o is not None else None,
                               m.get("scale") if m is not None else None))
        links[d["name"]] = d
    for j in r.findall("joint"):
        o = j.find("origin")
        a = j.find("axis")
        lim = j.find("limit")
        joints[j.get("name")] = {
            "type": j.get("type"), "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
            "xyz": np.array([float(v) for v in o.get("xyz").split()]),
            "rpy": np.array([float(v) for v in (o.get("rpy") or "0 0 0").split()]),
            "axis": np.array([float(v) for v in a.get("xyz").split()]) if a is not None else None,
            "limit": {k: float(v) for k, v in lim.attrib.items()} if lim is not None else None,
        }
    return links, joints


def global_origins(joints):
    child = {j["child"]: j for j in joints.values()}
    P = {}

    def g(link):
        if link in P:
            return P[link]
        if link not in child:
            P[link] = np.zeros(3)
            return P[link]
        j = child[link]
        P[link] = g(j["parent"]) + j["xyz"]
        return P[link]

    for j in joints.values():
        g(j["child"])
    return P


def rot(axis, angle):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def align(u0, u1):
    u0 = u0 / np.linalg.norm(u0)
    u1 = u1 / np.linalg.norm(u1)
    v = np.cross(u0, u1)
    s = np.linalg.norm(v)
    if s < 1e-15:
        return np.eye(3)
    return rot(v / s, np.arctan2(s, u0 @ u1))


def mirror_name(name):
    return ("r_" + name[2:]) if name.startswith("l_") else ("l_" + name[2:])


def sole_normal(tris):
    v = tris["v"].astype(np.float64)
    e1, e2 = v[:, 1] - v[:, 0], v[:, 2] - v[:, 0]
    n = np.cross(e1, e2)
    area = 0.5 * np.linalg.norm(n, axis=1)
    n = n / np.maximum(2 * area[:, None], 1e-30)
    cz = v[:, :, 2].mean(axis=1)
    sole = (n[:, 2] < -0.95) & (cz < v[:, :, 2].min() + 3.0)
    c = v[sole].mean(axis=1)
    A = np.c_[c[:, 0], c[:, 1], np.ones(len(c))]
    k = np.linalg.lstsq(A * area[sole, None], c[:, 2] * area[sole], rcond=None)[0]
    N = np.array([k[0], k[1], -1.0])
    return N / np.linalg.norm(N), int(sole.sum())


def package_paths(package):
    package = os.path.abspath(os.path.expanduser(package))
    urdf = None
    for rel in ("ver2-2_Edu.urdf", os.path.join("urdf", "robonex.urdf")):
        if os.path.isfile(os.path.join(package, rel)):
            urdf = os.path.join(package, rel)
            break
    if urdf is None:
        raise SystemExit("%s: no ver2-2_Edu.urdf or urdf/robonex.urdf" % package)
    loops = os.path.join(package, "loop_closures.yaml")
    meshes = os.path.join(package, "meshes")
    for p in (loops, meshes):
        if not os.path.exists(p):
            raise SystemExit("%s: missing %s" % (package, os.path.basename(p)))
    return urdf, loops, meshes


def repose(urdf_path, loops_path, mesh_src):
    from scipy.optimize import brentq, fsolve
    L, J = load_urdf(urdf_path)
    loops = yaml.safe_load(open(loops_path))
    G = global_origins(J)
    G["base_link"] = np.zeros(3)
    motion = {}
    report = {}
    for s in ("l", "r"):
        roll_j, pitch_j = J["%s_ankle_roll_joint" % s], J["%s_ankle_pitch_joint" % s]
        p_roll = G["%s_ankle_link" % s]
        p_pitch = G["%s_foot" % s]
        foot_tris = read_stl(os.path.join(mesh_src, "%s_foot.stl" % s))
        n0, nfaces = sole_normal(foot_tris)

        def foot_R(x):
            return rot(roll_j["axis"], x[0]) @ rot(pitch_j["axis"], x[1])

        phi, psi = fsolve(lambda x: (foot_R(x) @ n0)[:2], [0.0, 0.0], xtol=1e-14)
        R_ank = rot(roll_j["axis"], phi)
        R_foot = foot_R([phi, psi])
        p_pitch_new = p_roll + R_ank @ (p_pitch - p_roll)
        motion["%s_ankle_link" % s] = (R_ank, p_roll)
        motion["%s_foot" % s] = (R_foot, p_pitch_new)
        rep = {"sole_normal_asis": n0, "sole_faces": nfaces, "ankle_roll_deg": np.degrees(phi),
               "ankle_pitch_deg": np.degrees(psi)}
        for lp in loops["ball_loops"]:
            if not lp["name"].startswith(s + "_"):
                continue
            rod = lp["parent"]
            rj = next(j for j in J.values() if j["child"] == rod)
            crank = rj["parent"]
            cj = next(j for j in J.values() if j["child"] == crank)
            pivot = G[crank]
            c0 = G[rod]
            b_rod0 = c0 + np.array(lp["parent_xyz"])
            b_foot0 = p_pitch + np.array(lp["child_xyz"])
            b_foot1 = p_pitch_new + R_foot @ (b_foot0 - p_pitch)
            length = np.linalg.norm(b_rod0 - c0)

            def err(t):
                return np.linalg.norm(b_foot1 - (pivot + rot(cj["axis"], t) @ (c0 - pivot))) - length

            theta = brentq(err, -0.2, 0.2, xtol=1e-15)
            R_crank = rot(cj["axis"], theta)
            c1 = pivot + R_crank @ (c0 - pivot)
            R_rod = align(b_rod0 - c0, b_foot1 - c1)
            motion[crank] = (R_crank, pivot)
            motion[rod] = (R_rod, c1)
            tilt_change = np.degrees(np.arccos(np.clip((R_rod @ cj["axis"]) @ cj["axis"], -1, 1)))
            rep[crank] = {"crank_deg": np.degrees(theta), "rod": rod, "rod_axis_tilt_change_deg": tilt_change,
                          "closure_asis_mm": np.linalg.norm(b_rod0 - b_foot0) * 1000}
        report[s] = rep
    return L, J, loops, G, motion, report


def f6(v):
    return " ".join("%.6f" % (0.0 if abs(x) < 5e-7 else x) for x in v)


def mesh_name(path):
    return os.path.splitext(os.path.basename(path))[0]


def build(package):
    urdf_src, loops_src, mesh_src = package_paths(package)
    L, J, loops, G, motion, report = repose(urdf_src, loops_src, mesh_src)
    for name, link in L.items():
        if "mass" not in link:
            raise ValueError("package link %s has no inertial" % name)
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
    written = set()
    for name in motion:
        R = motion[name][0]
        for m in new_L[name]["meshes"]:
            tris = read_stl(os.path.join(mesh_src, m + ".stl"))
            out = tris.copy()
            out["v"] = (tris["v"].astype(np.float64).reshape(-1, 3) @ R.T).reshape(-1, 3, 3).astype(np.float32)
            out["n"] = (tris["n"].astype(np.float64) @ R.T).astype(np.float32)
            write_stl(os.path.join(MESH_DIR, m + ".stl"), out,
                      "RoboNex Ver.2-2 %s, mm, URDF axes, frame of %s, level-sole zero" % (m, name))
            written.add(m)
    copied = []
    for name, lk in new_L.items():
        for m in lk["meshes"]:
            if m not in written:
                shutil.copyfile(os.path.join(mesh_src, m + ".stl"), os.path.join(MESH_DIR, m + ".stl"))
                copied.append(m)
    replaced = [m for m in sorted(copied + list(written)) if replace_part_mesh(os.path.join(MESH_DIR, m + ".stl"), m)]

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

    links = new_L
    joints = {k: v for k, v in new_J.items() if v["child"] in links and v["parent"] in links}
    o = ['<?xml version="1.0"?>', '<robot name="robonex_%s">' % VARIANT, "",
         "    <!-- RoboNex Ver.2-2 %s (%s): ver2-2/scripts/model_tools.py build" % (VARIANT, TITLE),
         "         from the Fusion 360 export package %s (link-level inertials with motors included," % os.path.basename(
             os.path.dirname(mesh_src)),
         "         user-confirmed masses), re-posed so that both soles are",
         "         level at the zero pose (same motion as Ver.2). battery_link and estop_link are fixed to base_link;",
         "         their frames are their COMs. Every link frame is parallel to base_link at zero: X forward, Y left,",
         "         Z up. Meshes in link frames (mm). Joint limits are -pi..+pi until the Ver.2 joint-limit sweep. -->", "",
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
            lim = dict(j["limit"])
            if name in MEASURED_LIMITS:
                lim["lower"], lim["upper"] = MEASURED_LIMITS[name]
            o += ['        <axis xyz="%s"/>' % " ".join("%d" % int(round(a)) for a in j["axis"]),
                  '        <limit lower="%.6f" upper="%.6f" effort="%.1f" velocity="%.1f"/>'
                  % (lim["lower"], lim["upper"], lim["effort"], lim["velocity"]),
                  '        <dynamics damping="0.0" friction="0.0"/>']
        o += ["    </joint>", ""]
    o.append("</robot>")
    os.makedirs(URDF_DIR, exist_ok=True)
    path = os.path.join(URDF_DIR, "robonex_%s.urdf" % VARIANT)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(o) + "\n")

    for s, rep in report.items():
        print(s, {k: (np.round(v, 5) if isinstance(v, (float, np.ndarray)) else v) for k, v in rep.items()})
    print("loop closure at the new zero (mm):", {k: round(v * 1000, 6) for k, v in closure.items()})
    print("meshes re-posed: %d %s" % (len(written), sorted(written)))
    print("meshes copied unchanged: %d" % len(copied))
    print("motor meshes replaced by scripts/parts: %d %s" % (len(replaced), replaced))
    print("%s: %d links, %d joints, %.5f kg -> %s" % (VARIANT, len(links), len(joints),
                                                     sum(l["mass"] for l in links.values()), path))


class Loops:
    def __init__(self, variant="edu", path=None):
        self.model = mujoco.MjModel.from_xml_path(path or os.path.join(ROOT, "mujoco", "robot", variant, "scene_fixed.xml"))
        self.model.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt)]
        self.qadr = {n: m.jnt_qposadr[j] for j, n in enumerate(self.names)}
        self.dadr = {n: m.jnt_dofadr[j] for j, n in enumerate(self.names)}

    def q(self, name):
        return float(self.data.qpos[self.qadr[name]])

    def set(self, values):
        for n, v in values.items():
            self.data.qpos[self.qadr[n]] = v

    def residual_rows(self):
        return [r for r in range(self.data.nefc) if self.data.efc_type[r] == mujoco.mjtConstraint.mjCNSTR_EQUALITY]

    def solve(self, fixed, free, tol=1e-12, iters=100, damping=1e-9):
        self.set(fixed)
        cols = [self.dadr[n] for n in free]
        adr = [self.qadr[n] for n in free]
        res = np.inf
        for it in range(iters):
            mujoco.mj_forward(self.model, self.data)
            rows = self.residual_rows()
            r = np.array([self.data.efc_pos[i] for i in rows])
            res = np.abs(r).max() if len(r) else 0.0
            if res < tol:
                break
            Jm = self.data.efc_J.reshape(self.data.nefc, self.model.nv)[rows][:, cols]
            dq = -np.linalg.solve(Jm.T @ Jm + damping * np.eye(len(cols)), Jm.T @ r)
            step = np.abs(dq).max()
            if step > 0.2:
                dq *= 0.2 / step
            for a, d in zip(adr, dq):
                self.data.qpos[a] += d
        mujoco.mj_forward(self.model, self.data)
        return res, it

    def jac_cols(self, names):
        mujoco.mj_forward(self.model, self.data)
        rows = self.residual_rows()
        return self.data.efc_J.reshape(self.data.nefc, self.model.nv)[rows][:, [self.dadr[n] for n in names]]

    def body_R(self, name):
        return self.data.xmat[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)].reshape(3, 3).copy()

    def body_p(self, name):
        return self.data.xpos[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)].copy()


def leg_passive(s):
    return ["%s_knee_joint" % s, "%s_knee_coupler_joint_a" % s, "%s_ankle_roll_joint" % s, "%s_ankle_pitch_joint" % s] + [
        "%s_ankle_coupler_joint_%s_%s" % (s, ab, ax) for ab in "ab" for ax in "xyz"]


def provisional_limits():
    from robonex_common.joints import AUXILIARY_JOINTS
    return {**PROVISIONAL_LIMITS, **{joint.model_name: [joint.lower, joint.upper] for joint in AUXILIARY_JOINTS}}


def variant_masses():
    out = {}
    for v in ("edu",):
        root = ET.parse(os.path.join(ROOT, "urdf", "robonex_%s.urdf" % v)).getroot()
        out[v] = round(sum(float(m.get("value")) for m in root.iter("mass")), 6)
    return out


def sole_corners():
    ext = []
    for s in "lr":
        V = read_stl(os.path.join(ROOT, "meshes", "%s_foot.stl" % s))["v"].reshape(-1, 3).astype(float) / 1000.0
        z0 = V[:, 2].min()
        sole = V[V[:, 2] < z0 + 0.0005]
        ext.append((sole[:, 0].min(), sole[:, 0].max(), sole[:, 1].min(), sole[:, 1].max(), z0))
    e = np.array(ext)
    x0, x1, y0, y1 = e[:, 0].min(), e[:, 1].max(), e[:, 2].min(), e[:, 3].max()
    z = e[:, 4].min()
    return [(round(x0, 4), round(y0, 4), round(z, 4)), (round(x0, 4), round(y1, 4), round(z, 4)),
            (round(x1, 4), round(y0, 4), round(z, 4)), (round(x1, 4), round(y1, 4), round(z, 4))]


def lowest_corner_z(lp, corners):
    zs = []
    for s in "lr":
        R, p = lp.body_R("%s_foot" % s), lp.body_p("%s_foot" % s)
        zs.append(((np.array(corners) @ R.T + p)[:, 2]).min())
    return min(zs)


def seed_limits():
    const = {}
    if os.path.exists(CONSTANTS_OUT):
        const = json.load(open(CONSTANTS_OUT))
    const["provisional_limits"] = provisional_limits()
    with open(CONSTANTS_OUT, "w") as f:
        json.dump(const, f, indent=2)
    print("wrote provisional_limits (%d joints) to %s" % (len(const["provisional_limits"]), CONSTANTS_OUT))


def constants():
    from robonex_common.joints import AUXILIARY_JOINTS
    if not os.path.exists(SCENE_FIXED):
        raise SystemExit("constants needs %s: run `mujoco/build_mjcf.py --variant edu --fixed-base` first (on a first "
                         "build, `model_tools.py --version ver2-2 constants --seed-limits-only` before it)" % SCENE_FIXED)
    lp = Loops("edu")
    corners = sole_corners()
    lp.set({n: 0.0 for n in lp.names})
    mujoco.mj_forward(lp.model, lp.data)
    base_z = lp.body_p("base_link")[2]
    zero_height = base_z - lowest_corner_z(lp, corners)

    fixed, free = {}, []
    for s, sg in (("l", 1.0), ("r", -1.0)):
        fixed.update({"%s_hip_yaw_joint" % s: 0.0, "%s_hip_roll_joint" % s: 0.0,
                      "%s_hip_pitch_joint" % s: sg * HIP_PITCH, "%s_knee_joint" % s: -sg * KNEE_OUT,
                      "%s_ankle_roll_joint" % s: 0.0, "%s_ankle_pitch_joint" % s: ANKLE_PITCH})
        free += ["%s_knee_pitch_joint" % s, "%s_knee_coupler_joint_a" % s,
                 "%s_ankle_upper_joint" % s, "%s_ankle_lower_joint" % s] + [
            "%s_ankle_coupler_joint_%s_%s" % (s, ab, ax) for ab in "ab" for ax in "xyz"]
    res, it = lp.solve(fixed, free, tol=1e-13)
    actuated = [n for n in lp.names if n.endswith(("hip_yaw_joint", "hip_pitch_joint", "hip_roll_joint",
                                                   "knee_pitch_joint", "ankle_upper_joint", "ankle_lower_joint"))]
    solved = {n: lp.q(n) for n in actuated}
    default_act = {}
    for n in actuated:
        mag = 0.5 * (abs(solved[n]) + abs(solved[mirror_name(n)]))
        default_act[n] = float(np.copysign(mag, solved[n])) if abs(solved[n]) > 1e-12 else 0.0
    passive_names = [n for s in "lr" for n in leg_passive(s)]
    res2, it2 = lp.solve(default_act, passive_names, tol=1e-13)
    passive = {n: lp.q(n) for n in passive_names}
    asym = max(abs(solved[n] - default_act[n]) for n in actuated)
    home_height = base_z - lowest_corner_z(lp, corners)
    feet_y = abs(lp.body_p("l_foot")[1] - lp.body_p("r_foot")[1])
    hip_h = lp.body_p("l_hip_pitch_link")[2] - base_z + home_height
    sole_level = [np.degrees(np.arccos(np.clip(lp.body_R("%s_foot" % s)[2, 2], -1, 1))) for s in "lr"]

    boxes = {}
    for link in COLLISION_LINKS:
        V = read_stl(os.path.join(ROOT, "meshes", "%s.stl" % link))["v"].reshape(-1, 3).astype(float) / 1000.0
        lo, hi = V.min(0), V.max(0)
        boxes[link] = [[round(float(v), 4) for v in hi - lo], [round(float(v), 4) for v in (hi + lo) / 2]]

    const = {
        "zero_pose_base_height": round(float(zero_height), 5),
        "mujoco_spawn_height": round(float(zero_height + SPAWN_MARGIN), 4),
        "home_base_height": round(float(home_height), 5),
        "default_actuated_pos": {k: float(v) for k, v in default_act.items()},
        "home_passive_pos": {k: float(v) for k, v in passive.items()},
        "default_output_pose": {"hip_pitch": HIP_PITCH, "knee": KNEE_OUT, "ankle_pitch": ANKLE_PITCH},
        "foot_sole_corners": corners,
        "foot_origin_rest_height": round(-corners[0][2], 5),
        "stance_width_default": round(float(feet_y), 5),
        "hip_pitch_height_default": round(float(hip_h), 5),
        "collision_box": boxes,
        "solve_residual_m": float(max(res, res2)),
        "symmetrization_max_rad": float(asym),
        "sole_tilt_deg_default": sole_level,
        "variant_mass_kg": variant_masses(),
        "provisional_limits": provisional_limits(),
        "provisional_limits_note": PROVISIONAL_LIMITS_NOTE,
        "upper_body_limits_note": UPPER_BODY_LIMITS_NOTE,
        "upper_body_default_pos": {joint.model_name: 0.0 for joint in AUXILIARY_JOINTS},
    }
    with open(CONSTANTS_OUT, "w") as f:
        json.dump(const, f, indent=2)
    print("solve iterations", it, "residual %.2e m" % const["solve_residual_m"])
    for k in ("zero_pose_base_height", "mujoco_spawn_height", "home_base_height", "foot_origin_rest_height",
              "stance_width_default", "hip_pitch_height_default", "sole_tilt_deg_default", "foot_sole_corners"):
        print("%-26s %s" % (k, const[k]))
    print("default actuated (rad):")
    for k, v in const["default_actuated_pos"].items():
        print("   %-24s %+.6f  (%+.3f deg)" % (k, v, np.degrees(v)))
    print("wrote", CONSTANTS_OUT)


def loops_check(pose, scene, tol):
    if not os.path.exists(scene):
        raise SystemExit("%s not found: build it with mujoco/build_mjcf.py" % scene)
    lp = Loops(path=scene)
    m, d = lp.model, lp.data
    mujoco.mj_resetData(m, d)
    legs = [n for s in "lr" for n in ["%s_%s_joint" % (s, j) for j in (
        "hip_yaw", "hip_pitch", "hip_roll", "knee_pitch", "ankle_upper", "ankle_lower")] + leg_passive(s)]
    if pose == "home":
        if not os.path.exists(CONSTANTS_OUT):
            raise SystemExit("--pose home needs %s" % CONSTANTS_OUT)
        c = json.load(open(CONSTANTS_OUT))
        lp.set({**c["default_actuated_pos"], **c["home_passive_pos"]})
    else:
        lp.set({n: 0.0 for n in legs if n in lp.qadr})
    mujoco.mj_forward(m, d)
    per_eq = {}
    for r in lp.residual_rows():
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_EQUALITY, d.efc_id[r]) or "eq%d" % d.efc_id[r]
        per_eq[name] = max(per_eq.get(name, 0.0), abs(float(d.efc_pos[r])))
    print("%s, pose %s: %d equality constraints, %d rows" % (os.path.relpath(scene, ROOT), pose, len(per_eq),
                                                            len(lp.residual_rows())))
    for name, v in per_eq.items():
        print("   %-28s %.3e m" % (name, v))
    worst = max(per_eq.values()) if per_eq else 0.0
    print("max residual %.3e m (%s, tol %.0e)" % (worst, "closed" if worst < tol else "OPEN", tol))
    return 0 if worst < tol else 1


def check(urdf_path):
    links, joints = load_urdf(urdf_path)
    types = {}
    for j in joints.values():
        types[j["type"]] = types.get(j["type"], 0) + 1
    total = sum(l.get("mass", 0.0) for l in links.values())
    print("%s: %d links, %d joints %s, total mass %.6f kg" % (os.path.relpath(urdf_path), len(links), len(joints),
                                                             dict(sorted(types.items())), total))
    bad, missing = [], []
    for name, l in links.items():
        if "I" not in l:
            bad.append("%s: no inertial" % name)
            continue
        ev = np.linalg.eigvalsh(l["I"])
        if ev.min() <= 0 or ev[2] > ev[0] + ev[1] + 1e-12:
            bad.append("%s: eig %s" % (name, ev))
        for f in l["meshes"] + l["col"]:
            if f[0] and not os.path.exists(os.path.join(os.path.dirname(urdf_path), f[0])):
                missing.append(f[0])
    print("inertias SPD + triangle inequality: %s" % ("all %d ok" % len(links) if not bad else bad))
    print("missing mesh files: %s" % (sorted(set(missing)) or "none"))
    F = np.diag([1.0, -1.0, 1.0])

    def partners(name, pool):
        r = mirror_name(name)
        cands = [r] + ([r[:-1] + {"a": "b", "b": "a"}[r[-1]]] if r[-2:] in ("_a", "_b") else [])
        return [c for c in cands if c in pool]

    print("L/R mirror (y -> -y), max |l - mirror(r)|: mass kg, com m, inertia kg*m^2 (a/b partner with the smaller com residual)")
    for name in sorted(links):
        if not name.startswith("l_") or "I" not in links[name] or not partners(name, links):
            continue
        a = links[name]
        res = min((np.abs(F @ a["com"] - links[c]["com"]).max(), c) for c in partners(name, links))
        b = links[res[1]]
        print("   %-24s %-24s %.2e  %.2e  %.2e" % (name, res[1], abs(a["mass"] - b["mass"]), res[0],
                                                  np.abs(F @ a["I"] @ F - b["I"]).max()))
    worst = (0.0, "")
    for name in sorted(joints):
        if name.startswith("l_") and partners(name, joints):
            v = min(np.abs(F @ joints[name]["xyz"] - joints[c]["xyz"]).max() for c in partners(name, joints))
            worst = max(worst, (v, name))
    print("L/R mirror joint origins, max |l - mirror(r)|: %.2e m (%s)" % worst)
    return 0 if not bad and not missing else 1


def limits(variant):
    import threading
    import time
    import mujoco.viewer

    alias = {}
    for s in "lr":
        for short, name in (("yaw", "hip_yaw"), ("pitch", "hip_pitch"), ("roll", "hip_roll"), ("knee", "knee_pitch"),
                            ("up", "ankle_upper"), ("low", "ankle_lower")):
            alias[s + short] = "%s_%s_joint" % (s, name)
    act = list(alias.values())
    passive = leg_passive("l") + leg_passive("r")
    lp = Loops(variant)
    m, d = lp.model, lp.data
    for n in lp.names:
        if n in lp.qadr:
            d.qpos[lp.qadr[n]] = 0.0
    target = {n: 0.0 for n in act}
    state = {"ok": True, "res": 0.0, "leg_bodies": None}
    lock = threading.Lock()

    def apply():
        good = {n: d.qpos[lp.qadr[n]] for n in act + passive}
        start = {n: math.degrees(good[n]) for n in act}
        steps = max(1, int(math.ceil(max(abs(target[n] - start[n]) for n in act))))
        res = 0.0
        for k in range(1, steps + 1):
            f = k / steps
            res, _ = lp.solve({n: math.radians(start[n] + f * (target[n] - start[n])) for n in act}, passive)
            if res >= 1e-7:
                break
        state["res"] = res
        state["ok"] = res < 1e-7
        if not state["ok"]:
            for n, v in good.items():
                d.qpos[lp.qadr[n]] = v
            for n in act:
                target[n] = start[n]
            mujoco.mj_forward(m, d)

    def contacts():
        out = set()
        for c in d.contact[:d.ncon]:
            b1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom1])
            b2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom2])
            if b1 and b2 and c.dist < -0.0005:
                out.add("%s-%s" % tuple(sorted((b1, b2))))
        return sorted(out)

    def exact_check():
        sys.path.insert(0, "/home/polygon/humanoid_project/robonex-walking/etc/delegates/2026-09-24_physopt/ankle_verification")
        os.environ.setdefault("ANKLE_SRC", "unused")
        from common import geoms, place, depth, any_hit
        if state["leg_bodies"] is None:
            names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(m.nbody)]
            state["leg_bodies"] = geoms(m, [n for n in names if n and n != "world"])
        G = state["leg_bodies"]
        pairs = [(i, k) for i in range(len(G)) for k in range(i + 1, len(G)) if G[i][1] != G[k][1]]

        def depths():
            place(d, G)
            return {pk: (depth(G[pk[0]][3], G[pk[1]][3]) if any_hit(G[pk[0]][3], G[pk[1]][3]) else 0.0) for pk in pairs}

        if "baseline" not in state:
            saved = d.qpos.copy()
            for n in act + passive:
                d.qpos[lp.qadr[n]] = 0.0
            mujoco.mj_forward(m, d)
            state["baseline"] = depths()
            d.qpos[:] = saved
            mujoco.mj_forward(m, d)
        now = depths()
        hits = [((now[pk] - state["baseline"][pk]) * 1000, G[pk[0]][2], G[pk[1]][2]) for pk in pairs
                if now[pk] - state["baseline"][pk] > 0.0003]
        return sorted(hits, reverse=True)

    def deg(n):
        return math.degrees(d.qpos[lp.qadr[n]])

    def show():
        lines = []
        for s in "lr":
            lines.append("%s  yaw %+7.2f  pitch %+7.2f  roll %+7.2f  knee crank %+7.2f  ankle up %+7.2f  low %+7.2f" % (
                s.upper(), deg("%s_hip_yaw_joint" % s), deg("%s_hip_pitch_joint" % s), deg("%s_hip_roll_joint" % s),
                deg("%s_knee_pitch_joint" % s), deg("%s_ankle_upper_joint" % s), deg("%s_ankle_lower_joint" % s)))
            tilt = max(abs(deg("%s_ankle_coupler_joint_%s_%s" % (s, ab, ax))) for ab in "ab" for ax in "xz")
            lines.append("   knee output %+7.2f   ankle roll %+7.2f  pitch %+7.2f   rod-end tilt %5.2f%s" % (
                deg("%s_knee_joint" % s), deg("%s_ankle_roll_joint" % s), deg("%s_ankle_pitch_joint" % s), tilt,
                "  <-- over 15" if tilt > 15 else ""))
        lines.append("loop: %s (residual %.1e m)   contacts (convex, neighbours excluded): %s" % (
            "closed" if state["ok"] else "NO SOLUTION - last valid pose kept", state["res"], ", ".join(contacts()) or "none"))
        print("\n".join(lines), flush=True)

    help_text = """commands (degrees):
  <joint> <deg>        e.g. lknee -30, rup 20, lroll -45   (joints: l/r + yaw pitch roll knee up low)
  sym <joint> <deg>    set the left joint and its mirror (right gets the mirrored sign)
  both <joint> <deg>   set left and right to the same value
  zero | home          all zero | training default pose
  check                exact mesh collision check of every body pair (python-fcl, a few seconds)
  show | help | quit"""

    def home():
        c = json.load(open(CONSTANTS_OUT))
        for n in act:
            target[n] = math.degrees(c["default_actuated_pos"][n])

    def command(line):
        parts = line.split()
        if not parts:
            return
        if parts[0] in ("quit", "exit", "q"):
            os._exit(0)
        if parts[0] == "help":
            print(help_text)
            return
        if parts[0] == "zero":
            for n in act:
                target[n] = 0.0
        elif parts[0] == "home":
            home()
        elif parts[0] == "check":
            with lock:
                hits = exact_check()
            print("exact mesh penetration beyond the zero-pose contact (> 0.3 mm):" if hits else "exact mesh check: no penetration beyond the zero-pose contact", flush=True)
            for dep, a, b in hits:
                print("   %6.2f mm  %s - %s" % (dep, a, b), flush=True)
            return
        elif parts[0] in ("sym", "both") and len(parts) == 3 and (parts[1] in alias or "l" + parts[1] in alias):
            name, v = alias[parts[1] if parts[1] in alias else "l" + parts[1]], float(parts[2])
            target[name] = v
            target[mirror_name(name)] = -v if parts[0] == "sym" else v
        elif parts[0] in alias and len(parts) == 2:
            target[alias[parts[0]]] = float(parts[1])
        elif parts[0] != "show":
            print("?", help_text, flush=True)
            return
        with lock:
            apply()
        show()

    def reader():
        print(help_text, flush=True)
        while True:
            try:
                line = input("> ")
            except EOFError:
                os._exit(0)
            try:
                command(line.strip())
            except Exception as error:
                print("error:", error, flush=True)

    with lock:
        apply()
    threading.Thread(target=reader, daemon=True).start()
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            with lock:
                viewer.sync()
            time.sleep(0.03)
    os._exit(0)


def main():
    parser = argparse.ArgumentParser(description="RoboNex model tools (build, constants, loops, limits, check).")
    parser.add_argument("--version", required=True, help="model folder: ver2 or ver2-2 (build, constants: ver2-2 only)")
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("build", help="re-pose a Fusion export package into meshes/, loop_closures.yaml, urdf/")
    p.add_argument("--package", required=True, help="Fusion export folder (ver2-2_Edu.urdf or urdf/robonex.urdf, "
                                                     "loop_closures.yaml, meshes/)")
    p = sub.add_parser("constants", help="solve the home pose and write <version>_constants.json")
    p.add_argument("--seed-limits-only", action="store_true", help="write only provisional_limits (first build)")
    p = sub.add_parser("loops", help="loop-closure residuals of a MuJoCo scene")
    p.add_argument("--pose", default="zero", choices=("zero", "home"))
    p.add_argument("--scene", default=None, help="default <version>/mujoco/robot/edu/scene_fixed.xml")
    p.add_argument("--tol", type=float, default=1e-6, help="residual tolerance in m (exit 1 above it)")
    p = sub.add_parser("limits", help="interactive joint viewer with the loops solved")
    p.add_argument("--variant", default="edu", help="a variant of the selected version")
    p = sub.add_parser("check", help="structural checks of a URDF")
    p.add_argument("--urdf", default=None, help="default <version>/urdf/robonex_edu.urdf")
    args = parser.parse_args()
    version = select_version()
    if args.command in ("build", "constants") and version not in GENERATOR_VERSIONS:
        raise SystemExit("%s is Ver.2-2 only (%s: urdf/, meshes/ and the constants file are frozen)"
                         % (args.command, version))
    if args.command == "build":
        build(args.package)
    elif args.command == "constants":
        seed_limits() if args.seed_limits_only else constants()
    elif args.command == "loops":
        return loops_check(args.pose, os.path.abspath(args.scene or SCENE_FIXED), args.tol)
    elif args.command == "limits":
        limits(args.variant)
    elif args.command == "check":
        return check(os.path.abspath(args.urdf or os.path.join(URDF_DIR, "robonex_edu.urdf")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
