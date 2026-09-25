import os
import sys

import numpy as np
import yaml
from scipy.optimize import brentq, fsolve

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "verify"))
from urdf_io import load, global_origins
from stl_io import read_stl, write_stl

ASIS = os.path.join(ROOT, "asis")
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


def main():
    L, J = load(os.path.join(ASIS, "urdf", "robonex.urdf"))
    loops = yaml.safe_load(open(os.path.join(ASIS, "loop_closures.yaml")))
    G = global_origins(J)
    G["base_link"] = np.zeros(3)
    motion = {}
    report = {}
    for s in ("l", "r"):
        roll_j, pitch_j = J["%s_ankle_roll_joint" % s], J["%s_ankle_pitch_joint" % s]
        p_roll = G["%s_ankle_link" % s]
        p_pitch = G["%s_foot" % s]
        foot_tris = read_stl(os.path.join(ASIS, "meshes", "%s_foot.stl" % s))
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


if __name__ == "__main__":
    L, J, loops, G, motion, report = main()
    for s, rep in report.items():
        print(s, {k: (np.round(v, 5) if isinstance(v, (float, np.ndarray)) else v) for k, v in rep.items()})
