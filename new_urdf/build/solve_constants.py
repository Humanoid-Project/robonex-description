import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "verify"))
from loops import Loops, leg_passive
sys.path.insert(0, HERE)
from stl_io import read_stl

OUT = os.path.join(ROOT, "scripts", "ver2_constants.json")
HIP_PITCH = 0.1
KNEE_OUT = 0.29669690697178175
ANKLE_PITCH = KNEE_OUT - HIP_PITCH
FIXED_BASE_HEIGHT = 1.60
SPAWN_MARGIN = 0.006
PROVISIONAL_LIMITS = {
    "l_hip_yaw_joint": [-1.570796, 1.570796], "l_hip_pitch_joint": [-1.745329, 1.745329],
    "l_hip_roll_joint": [-2.094395, 0.174533], "l_knee_pitch_joint": [-1.361357, 0.261799],
    "l_ankle_upper_joint": [-0.610865, 0.575959], "l_ankle_lower_joint": [-0.610865, 0.575959],
    "r_hip_yaw_joint": [-1.570796, 1.570796], "r_hip_pitch_joint": [-1.745329, 1.745329],
    "r_hip_roll_joint": [-0.174533, 2.094395], "r_knee_pitch_joint": [-0.261799, 1.361357],
    "r_ankle_upper_joint": [-0.575959, 0.610865], "r_ankle_lower_joint": [-0.575959, 0.610865],
}
COLLISION_LINKS = ["base_link"] + ["%s_%s" % (s, n) for s in "lr" for n in (
    "hip_yaw_link", "hip_pitch_link", "hip_roll_link", "knee_link", "ankle_link", "foot")]


def variant_masses():
    import xml.etree.ElementTree as ET
    out = {}
    for v in ("edu", "pro", "max"):
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


def main():
    lp = Loops("edu")
    corners = sole_corners()
    lp.set({n: 0.0 for n in lp.names})
    import mujoco
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
        mirror = ("r_" + n[2:]) if n.startswith("l_") else ("l_" + n[2:])
        mag = 0.5 * (abs(solved[n]) + abs(solved[mirror]))
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
        "provisional_limits": PROVISIONAL_LIMITS,
        "provisional_limits_note": "hips: Ver.2 limits set by the user in the viewer (2026-09-26): yaw +-90, pitch +-100, roll "
                                   "-120..+10 (left, mirrored right); knee crank [-78, +15] deg from the four-bar; ankle cranks "
                                   "still the Ver.1 measured values (placeholder until measured in MuJoCo)",
    }
    with open(OUT, "w") as f:
        json.dump(const, f, indent=2)
    return const, it


if __name__ == "__main__":
    c, it = main()
    print("solve iterations", it, "residual %.2e m" % c["solve_residual_m"])
    for k in ("zero_pose_base_height", "mujoco_spawn_height", "home_base_height", "foot_origin_rest_height",
              "stance_width_default", "hip_pitch_height_default", "sole_tilt_deg_default", "foot_sole_corners"):
        print("%-26s %s" % (k, c[k]))
    print("default actuated (rad):")
    for k, v in c["default_actuated_pos"].items():
        print("   %-24s %+.6f  (%+.3f deg)" % (k, v, np.degrees(v)))
    print("wrote", OUT)
