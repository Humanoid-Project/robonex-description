"""RoboNex Ver.2 lower body: joint points, link frames and mass groups from the Fusion extraction.

Inputs (written by read-only Fusion scripts from `2_RoboNex_test v1`):
  transforms_mm.json  occurrence local->world 4x4 (mm)
  massprops.json      mass, COM, inertia about the world origin (kg, mm, kg*cm^2)
  geom/<part>.json    cylinder lines and sphere centres of the pivot parts (mm)

Frames:
  Fusion world  : forward = -Y, left = +X, up = +Z, origin = base_link origin
  URDF          : forward = +X, left = +Y, up = +Z  ->  u = (-y_f, x_f, z_f)
The model is taken as-is: every joint is 0 in the current assembly pose.
"""
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

R_FU = np.array([[0.0, -1.0, 0.0],
                 [1.0, 0.0, 0.0],
                 [0.0, 0.0, 1.0]])

T = {k: np.array(v) for k, v in json.load(open(os.path.join(HERE, "transforms_mm.json")))["T"].items()}
MASS = json.load(open(os.path.join(HERE, "massprops.json")))["parts"]
# head: neck motor, neck mount (BK_032), head shell (BK_033), RealSense (BK_034)
T.update({k: np.array(v) for k, v in json.load(open(os.path.join(HERE, "head_transforms_mm.json")))["T"].items()})
MASS.update(json.load(open(os.path.join(HERE, "head_massprops.json")))["parts"])
# arms: read from the Fusion copy '2_RoboNex_urdf' with both elbows rotated 90 deg forward (as-built joints);
# every other part is identical to '2_RoboNex_test v1' (checked transform by transform).
T.update({k: np.array(v) for k, v in json.load(open(os.path.join(HERE, "arm_transforms_mm.json")))["T"].items()})
MASS.update(json.load(open(os.path.join(HERE, "arm_massprops.json")))["parts"])

# The Fusion RS05 body computes to 95.6 g although its material is named "RS05 191g"; the official mass is 191 g.
# Same geometry, uniform density: scale mass and inertia to 191 g.
MASS_SCALE = {"rs05_neck": 0.191 / MASS["rs05_neck"]["mass_kg"]}

# Mesh file per Fusion part (default: the part name). Parts sharing a file are merged into one STL.
MESH_OF = {"neck_BK_032": "neck_mount", "head_BK_033": "head_link", "realsense_BK_034": "head_link"}


def mesh_of(part):
    return MESH_OF.get(part, part)


def geom(part):
    return json.load(open(os.path.join(HERE, "geom", part + ".json")))


def to_urdf(p_fusion_mm):
    return R_FU @ np.asarray(p_fusion_mm, dtype=float)


# Motor bodies: rotation axis = local Z through the local axis point; output flange face at local +Z.
# Offsets measured on the RS02/RS03 components (identical in Ver.1 and Ver.2 except the RS02 origin shift).
# RS05: output face at local z 30.5 (the head shell seats there with 6 bolts and 3 dowels; 33.5 is the dowel tips).
MOTOR_LOCAL_AXIS = {"rs02": (-6.95, 0.40, 22.7), "rs03": (-0.49, 0.00, 28.3), "rs05": (0.0, 0.0, 30.5)}


def motor_flange(occ):
    kind = occ[:4]
    ax, ay, h = MOTOR_LOCAL_AXIS[kind]
    M = T[occ]
    p = M[:3, :3] @ np.array([ax, ay, h]) + M[:3, 3]
    z_local = M[:3, 2]
    return p, z_local


def cyl_line(part, axis, near, tol=0.6):
    for c in geom(part)["cylinders"]:
        if c["axis"] == axis and abs(c["line"][0] - near[0]) < tol and abs(c["line"][1] - near[1]) < tol:
            return c
    raise KeyError("%s: no %s line near %s" % (part, axis, near))


def point_on_line(axis, line, along):
    i = "XYZ".index(axis)
    rest = [k for k in range(3) if k != i]
    p = np.zeros(3)
    p[i] = along
    p[rest[0]], p[rest[1]] = line
    return p


def big_sphere(part, near, tol=1.0):
    for s in geom(part)["spheres"]:
        if s[3] > 6 and np.linalg.norm(np.array(s[:3]) - np.array(near)) < tol:
            return np.array(s[:3])
    raise KeyError("%s: no ball near %s" % (part, near))


def axis_from_motor(occ):
    """URDF axis = -(motor output direction), the rule that reproduces all 12 Ver.1 axis signs."""
    _, z = motor_flange(occ)
    a = to_urdf(-z)
    a = np.round(a, 6)
    k = int(np.argmax(np.abs(a)))
    if abs(abs(a[k]) - 1.0) > 1e-6:
        raise ValueError("%s axis is not axis-aligned: %s" % (occ, a))
    out = np.zeros(3)
    out[k] = np.sign(a[k])
    return out


def build():
    """Return joints (name -> dict) with fusion-frame points, and link groups."""
    J = {}

    def add(name, jtype, parent, child, p_fusion, axis=None, lower=None, upper=None, effort=0.0, velocity=0.0,
            comment=""):
        J[name] = dict(type=jtype, parent=parent, child=child, p_f=np.asarray(p_fusion, dtype=float),
                       axis=None if axis is None else np.asarray(axis, dtype=float),
                       lower=lower, upper=upper, effort=effort, velocity=velocity, comment=comment)

    RS02 = dict(effort=17.0, velocity=42.9)
    RS03 = dict(effort=60.0, velocity=20.9)
    PI = 3.141593

    # Limits: Ver.1 hardware sweep values as PLACEHOLDERS (robonex-common 0.2.0) until the Ver.2 sweep.
    lim = {
        "l_hip_yaw_joint": (-1.658063, 1.658063), "r_hip_yaw_joint": (-1.658063, 1.658063),
        "l_hip_pitch_joint": (-1.745329, 1.745329), "r_hip_pitch_joint": (-1.745329, 1.745329),
        "l_hip_roll_joint": (-2.146755, 0.453786), "r_hip_roll_joint": (-0.453786, 2.146755),
        # knee crank: Ver.1 upper +54 deg is unreachable on Ver.2 (four-bar toggles at +20.0 deg);
        # CAD geometric placeholder +15 deg keeps the crank limit ahead of the linkage limit.
        "l_knee_pitch_joint": (-1.500983, 0.261799), "r_knee_pitch_joint": (-0.261799, 1.500983),
        "l_ankle_upper_joint": (-0.610865, 0.575959), "l_ankle_lower_joint": (-0.610865, 0.575959),
        "r_ankle_upper_joint": (-0.575959, 0.610865), "r_ankle_lower_joint": (-0.575959, 0.610865),
    }

    for s in ("l", "r"):
        sx = 1.0 if s == "l" else -1.0  # fusion +X is left

        # --- actuated (motor flange face centre on the axis) ---
        for jn, parent, child, motor, spec in (
            ("%s_hip_yaw_joint", "base_link", "%s_hip_yaw_link", "rs02_%s_hip_yaw", RS02),
            ("%s_hip_pitch_joint", "%s_hip_yaw_link", "%s_hip_pitch_link", "rs03_%s_hip_pitch", RS03),
            ("%s_hip_roll_joint", "%s_hip_pitch_link", "%s_hip_roll_link", "rs03_%s_hip_roll", RS03),
            ("%s_knee_pitch_joint", "%s_hip_roll_link", "%s_knee_crank_link", "rs03_%s_knee", RS03),
        ):
            jname = jn % s
            p, _ = motor_flange(motor % s)
            lo, hi = lim[jname]
            add(jname, "revolute", parent if parent == "base_link" else parent % s, child % s, p,
                axis_from_motor(motor % s), lo, hi, **spec)

        # ankle motors: a = upper (both sides); left upper -> crank_link_a, right upper -> crank_link_b
        upper_crank = "l_ankle_crank_link_a" if s == "l" else "r_ankle_crank_link_b"
        lower_crank = "l_ankle_crank_link_b" if s == "l" else "r_ankle_crank_link_a"
        for jname, child, motor in (("%s_ankle_upper_joint" % s, upper_crank, "rs02_%s_ankle_a" % s),
                                    ("%s_ankle_lower_joint" % s, lower_crank, "rs02_%s_ankle_b" % s)):
            p, _ = motor_flange(motor)
            lo, hi = lim[jname]
            add(jname, "revolute", "%s_knee_link" % s, child, p, axis_from_motor(motor), lo, hi, **RS02)

        # --- passive (bore centre of the child part, on the axis) ---
        kl = cyl_line("%s_knee_link" % s, "X", (2.71, -566.74))
        add("%s_knee_joint" % s, "revolute", "%s_hip_roll_link" % s, "%s_knee_link" % s,
            point_on_line("X", kl["line"], 0.5 * sum(kl["axial"])),
            [0, -1, 0] if s == "l" else [0, 1, 0], -PI, PI)

        cl = cyl_line("%s_knee_coupler_link" % s, "X", (-63.07, -367.8))
        add("%s_knee_coupler_joint_a" % s, "revolute", "%s_knee_crank_link" % s, "%s_knee_coupler_link" % s,
            point_on_line("X", cl["line"], 0.5 * sum(cl["axial"])), [0, -1, 0], -PI, PI)

        rl = cyl_line("%s_ankle_link" % s, "Y", (134.42 * sx if s == "l" else -134.28, -833.45))
        add("%s_ankle_roll_joint" % s, "revolute", "%s_knee_link" % s, "%s_ankle_link" % s,
            point_on_line("Y", rl["line"], 0.5 * sum(rl["axial"])), [1, 0, 0], -PI, PI)

        # pitch axis: ankle_link main bore line; point laterally on the ankle_link mid-plane (= roll axis x)
        pl = cyl_line("%s_ankle_link" % s, "X", (2.6, -883.45), tol=0.06)
        add("%s_ankle_pitch_joint" % s, "revolute", "%s_ankle_link" % s, "%s_foot" % s,
            point_on_line("X", pl["line"], rl["line"][0]), [0, -1, 0], -PI, PI)

    # --- arms (Unitree G1 chain): each RS02 housing sits in the upper part, its output face drives the next ---
    for s in ("l", "r"):
        for jn, parent, child, motor in (
            ("shoulder_pitch", "base_link", "shoulder_pitch_link", "shoulder_pitch"),
            ("shoulder_roll", "shoulder_pitch_link", "shoulder_roll_link", "shoulder_roll"),
            ("shoulder_yaw", "shoulder_roll_link", "shoulder_yaw_link", "shoulder_yaw"),
            ("elbow", "shoulder_yaw_link", "elbow_link", "elbow"),
        ):
            p, _ = motor_flange("rs02_%s_%s" % (s, motor))
            add("%s_%s_joint" % (s, jn), "revolute", parent if parent == "base_link" else "%s_%s" % (s, parent),
                "%s_%s" % (s, child), p, axis_from_motor("rs02_%s_%s" % (s, motor)), -PI, PI, **RS02)

    # --- neck: RS05 housing on base_link (through the neck mount), head shell on the output face ---
    p, _ = motor_flange("rs05_neck")
    add("neck_pitch_joint", "revolute", "base_link", "head_link", p, axis_from_motor("rs05_neck"),
        -PI, PI, effort=5.5, velocity=50.3)

    # --- ankle rod ends on the cranks (fixed in the URDF, ball-upgraded downstream) ---
    add("l_ankle_coupler_joint_a", "fixed", "l_ankle_crank_link_b", "l_ankle_coupler_link_a",
        big_sphere("l_ankle_crank_link_b", (85.62, -45.322, -761.338)))
    add("l_ankle_coupler_joint_b", "fixed", "l_ankle_crank_link_a", "l_ankle_coupler_link_b",
        big_sphere("l_ankle_crank_link_a", (183.622, -44.138, -650.886)))
    add("r_ankle_coupler_joint_a", "fixed", "r_ankle_crank_link_a", "r_ankle_coupler_link_a",
        big_sphere("r_ankle_crank_link_a", (-84.977, -45.356, -762.64)))
    add("r_ankle_coupler_joint_b", "fixed", "r_ankle_crank_link_b", "r_ankle_coupler_link_b",
        big_sphere("r_ankle_crank_link_b", (-182.978, -44.253, -651.645)))

    # --- loop closures (same physical point, fusion frame) ---
    loops = {"ball": [], "pin": []}
    for s in ("l", "r"):
        cl2 = cyl_line("%s_knee_coupler_link" % s, "X", (-63.07, -501.8))
        kl2 = cyl_line("%s_knee_link" % s, "X", (-63.07, -501.8))
        assert abs(cl2["line"][0] - kl2["line"][0]) < 1e-6 and abs(cl2["line"][1] - kl2["line"][1]) < 1e-6
        loops["pin"].append(dict(name="%s_knee_coupler_joint_b" % s, parent="%s_knee_coupler_link" % s,
                                 child="%s_knee_link" % s,
                                 p_f=point_on_line("X", cl2["line"], 0.5 * sum(cl2["axial"])),
                                 axis=[0.0, 1.0, 0.0] if s == "l" else [0.0, -1.0, 0.0]))
    for name, rod, foot, near in (
        ("l_ankle_ball_c", "l_ankle_coupler_link_a", "l_foot", (84.807, -47.843, -908.314)),
        ("l_ankle_ball_d", "l_ankle_coupler_link_b", "l_foot", (183.205, -47.843, -908.859)),
        ("r_ankle_ball_c", "r_ankle_coupler_link_a", "r_foot", (-85.077, -47.311, -909.627)),
        ("r_ankle_ball_d", "r_ankle_coupler_link_b", "r_foot", (-183.477, -47.311, -909.627)),
    ):
        p_rod = big_sphere(rod, near)
        p_foot = big_sphere(foot, near)
        loops["ball"].append(dict(name=name, parent=rod, child=foot, p_f=p_rod, p_f_child=p_foot))

    return J, loops


# Link -> Fusion occurrences (Ver.1 convention: a motor belongs to the link its housing is bolted to).
LINK_PARTS = {"base_link": ["base_link", "rs02_l_hip_yaw", "rs02_r_hip_yaw", "rs05_neck", "neck_BK_032",
                            "rs02_l_shoulder_pitch", "rs02_r_shoulder_pitch"],
              "head_link": ["head_BK_033", "realsense_BK_034"]}
for _s in ("l", "r"):
    LINK_PARTS.update({
        "%s_shoulder_pitch_link" % _s: ["%s_shoulder_pitch_link" % _s, "rs02_%s_shoulder_roll" % _s],
        "%s_shoulder_roll_link" % _s: ["%s_shoulder_roll_link" % _s, "rs02_%s_shoulder_yaw" % _s],
        "%s_shoulder_yaw_link" % _s: ["%s_shoulder_yaw_link" % _s, "rs02_%s_elbow" % _s],
        "%s_elbow_link" % _s: ["%s_elbow_link" % _s],
    })
for _s in ("l", "r"):
    LINK_PARTS.update({
        "%s_hip_yaw_link" % _s: ["%s_hip_yaw_link" % _s, "rs03_%s_hip_pitch" % _s],
        "%s_hip_pitch_link" % _s: ["%s_hip_pitch_link" % _s, "rs03_%s_hip_roll" % _s],
        "%s_hip_roll_link" % _s: ["%s_hip_roll_link" % _s, "rs03_%s_knee" % _s],
        "%s_knee_crank_link" % _s: ["%s_knee_crank_link" % _s],
        "%s_knee_coupler_link" % _s: ["%s_knee_coupler_link" % _s],
        "%s_knee_link" % _s: ["%s_knee_link" % _s, "rs02_%s_ankle_a" % _s, "rs02_%s_ankle_b" % _s],
        "%s_ankle_crank_link_a" % _s: ["%s_ankle_crank_link_a" % _s],
        "%s_ankle_crank_link_b" % _s: ["%s_ankle_crank_link_b" % _s],
        "%s_ankle_coupler_link_a" % _s: ["%s_ankle_coupler_link_a" % _s],
        "%s_ankle_coupler_link_b" % _s: ["%s_ankle_coupler_link_b" % _s],
        "%s_ankle_link" % _s: ["%s_ankle_link" % _s],
        "%s_foot" % _s: ["%s_foot" % _s],
    })

LINK_ORDER = ["base_link"] + [n % s for s in ("l", "r") for n in (
    "%s_hip_yaw_link", "%s_hip_pitch_link", "%s_hip_roll_link", "%s_knee_crank_link", "%s_knee_coupler_link",
    "%s_knee_link", "%s_ankle_crank_link_a", "%s_ankle_crank_link_b", "%s_ankle_coupler_link_a",
    "%s_ankle_coupler_link_b", "%s_ankle_link", "%s_foot")] + ["head_link"] + [n % s for s in ("l", "r") for n in (
    "%s_shoulder_pitch_link", "%s_shoulder_roll_link", "%s_shoulder_yaw_link", "%s_elbow_link")]


def link_origins_f(J):
    """Link frame origin (fusion mm) = point of the joint whose child it is; base_link at the origin."""
    o = {"base_link": np.zeros(3)}
    for j in J.values():
        o[j["child"]] = j["p_f"]
    return o


def group_inertia(parts):
    """Combined mass (kg), COM (fusion mm) and inertia tensor about the COM (fusion axes, kg*m^2)."""
    ps = []
    for n in parts:
        d = MASS[n]
        k = MASS_SCALE.get(n, 1.0)
        m = d["mass_kg"] * k
        c = np.array(d["com_mm"]) / 10.0  # cm
        I = d["I_origin_kgcm2"]
        Io = k * np.array([[I["xx"], I["xy"], I["xz"]], [I["xy"], I["yy"], I["yz"]], [I["xz"], I["yz"], I["zz"]]])
        Ic = Io - m * (c @ c * np.eye(3) - np.outer(c, c))
        ps.append((m, c, Ic))
    M = sum(p[0] for p in ps)
    C = sum(p[0] * p[1] for p in ps) / M
    I = np.zeros((3, 3))
    for m, c, Ic in ps:
        d = c - C
        I += Ic + m * (d @ d * np.eye(3) - np.outer(d, d))
    return M, C * 10.0, I * 1e-4
