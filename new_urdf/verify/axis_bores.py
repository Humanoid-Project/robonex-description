import json
import sys

import numpy as np

sys.path.insert(0, "../_extract")
from lower_model import R_FU, motor_flange
from urdf_io import load, global_origins

L, J = load("../asis/urdf/robonex.urdf")
G = global_origins(J)
AX = {0: "X", 1: "Y", 2: "Z"}


def fusion(p_u_m):
    return R_FU.T @ (np.asarray(p_u_m) * 1000.0)


def cylinders(part):
    try:
        return json.load(open("../_extract/geom/%s.json" % part))["cylinders"]
    except FileNotFoundError:
        return None


def nearest(part, point_f, axis_f):
    cyl = cylinders(part)
    if cyl is None:
        return None
    k = int(np.argmax(np.abs(axis_f)))
    rest = [i for i in range(3) if i != k]
    best = None
    for c in cyl:
        if c["axis"] != AX[k]:
            continue
        d = np.hypot(c["line"][0] - point_f[rest[0]], c["line"][1] - point_f[rest[1]])
        if best is None or d < best[0]:
            best = (d, c["radii"], c["area_mm2"])
    return best


CHECKS = []
for s in "lr":
    CHECKS += [
        ("%s_knee_joint" % s, ["%s_hip_roll_link" % s, "%s_knee_link" % s]),
        ("%s_knee_coupler_joint_a" % s, ["%s_knee_crank_link" % s, "%s_knee_coupler_link" % s]),
        ("%s_knee_pitch_joint" % s, ["%s_knee_crank_link" % s, "%s_hip_roll_link" % s]),
        ("%s_ankle_roll_joint" % s, ["%s_knee_link" % s, "%s_ankle_link" % s]),
        ("%s_ankle_pitch_joint" % s, ["%s_ankle_link" % s, "%s_foot" % s]),
        ("%s_ankle_upper_joint" % s, ["%s_ankle_crank_link_%s" % (s, "a" if s == "l" else "b")]),
        ("%s_ankle_lower_joint" % s, ["%s_ankle_crank_link_%s" % (s, "b" if s == "l" else "a")]),
    ]
print("%-26s %-24s %10s  %s" % ("joint", "part", "dist(mm)", "bore radii"))
worst = 0
for jn, parts in CHECKS:
    j = J[jn]
    p_f = fusion(G[j["child"]])
    a_f = R_FU.T @ j["axis"]
    for part in parts:
        r = nearest(part, p_f, a_f)
        if r is None:
            print("%-26s %-24s %10s" % (jn, part, "no geom"))
            continue
        worst = max(worst, r[0]) if r[0] < 5 else worst
        print("%-26s %-24s %10.3f  %s" % (jn, part, r[0], r[1]))
for s in "lr":
    pin = [p for p in __import__("yaml").safe_load(open("../asis/loop_closures.yaml"))["pin_loops"] if p["name"].startswith(s)][0]
    p_f = fusion(G[pin["parent"]] + np.array(pin["parent_xyz"]))
    for part in (pin["parent"], pin["child"]):
        r = nearest(part, p_f, np.array([1.0, 0, 0]))
        print("%-26s %-24s %10.3f  %s" % (pin["name"], part, r[0], r[1]))
print("\nmotor output axis vs URDF joint (flange point on the axis):")
MOT = {"hip_yaw": "rs02_%s_hip_yaw", "hip_pitch": "rs03_%s_hip_pitch", "hip_roll": "rs03_%s_hip_roll",
       "knee_pitch": "rs03_%s_knee", "ankle_upper": "rs02_%s_ankle_a", "ankle_lower": "rs02_%s_ankle_b"}
for s in "lr":
    for jn, mot in MOT.items():
        j = J["%s_%s_joint" % (s, jn)]
        p, z = motor_flange(mot % s)
        pu = R_FU @ p / 1000.0
        au = R_FU @ z
        d = np.linalg.norm(np.cross(pu - G[j["child"]], j["axis"])) * 1000
        ang = np.degrees(np.arccos(min(1.0, abs(au @ j["axis"]))))
        print("  %-22s point-to-axis %.4f mm, axis angle %.4f deg" % ("%s_%s_joint" % (s, jn), d, ang))
