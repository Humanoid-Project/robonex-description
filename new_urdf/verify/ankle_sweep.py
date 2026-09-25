import sys

import numpy as np

from loops import Loops, leg_passive

MODEL = sys.argv[1] if len(sys.argv) > 1 else "edu"
V1_PATH = "/home/polygon/humanoid_project/robonex-description/mujoco/robot/scene_fixed.xml"


def sweep(lp, s, step=2.0, span=60):
    up, lo = "%s_ankle_upper_joint" % s, "%s_ankle_lower_joint" % s
    free = leg_passive(s)
    grid = np.arange(-span, span + 1e-9, step)
    out = {}
    lp.set({n: 0.0 for n in free})
    lp.solve({"%s_knee_pitch_joint" % s: 0.0, up: 0.0, lo: 0.0}, free)
    for a in grid:
        order = grid if (np.where(grid == a)[0][0] % 2 == 0) else grid[::-1]
        for b in order:
            res, _ = lp.solve({up: np.radians(a), lo: np.radians(b)}, free)
            if res > 1e-9:
                out[(a, b)] = None
                lp.set({n: 0.0 for n in free})
                lp.solve({up: 0.0, lo: 0.0}, free)
                continue
            tilt = max(abs(np.degrees(lp.q("%s_ankle_coupler_joint_%s_%s" % (s, ab, ax)))) for ab in "ab" for ax in "xz")
            Jc = lp.jac_cols([up, lo, "%s_ankle_roll_joint" % s, "%s_ankle_pitch_joint" % s] + [
                "%s_ankle_coupler_joint_%s_%s" % (s, ab, ax) for ab in "ab" for ax in "xyz"])
            A, B = Jc[:, :2], Jc[:, 2:]
            dP = -np.linalg.lstsq(B, A, rcond=None)[0][:2]
            sv = np.linalg.svd(dP, compute_uv=False)
            out[(a, b)] = dict(roll=np.degrees(lp.q("%s_ankle_roll_joint" % s)),
                               pitch=np.degrees(lp.q("%s_ankle_pitch_joint" % s)), tilt=tilt,
                               cond=sv[0] / sv[1], smin=sv[1])
    return grid, out


if __name__ == "__main__":
    lp = Loops(path=V1_PATH) if MODEL == "v1" else Loops(MODEL)
    for s in "lr":
        grid, out = sweep(lp, s)
        ok = {k: v for k, v in out.items() if v is not None}
        inside = {k: v for k, v in ok.items() if v["tilt"] <= 15.0}
        print("== %s: grid %d, loop solvable %d, rod-end tilt <= 15 deg %d" % (s, len(out), len(ok), len(inside)))
        R = np.array([v["roll"] for v in inside.values()])
        P = np.array([v["pitch"] for v in inside.values()])
        C = np.array([v["cond"] for v in inside.values()])
        S = np.array([v["smin"] for v in inside.values()])
        print("   foot roll %.1f..%.1f deg, pitch %.1f..%.1f deg (within tilt limit)" % (R.min(), R.max(), P.min(), P.max()))
        print("   Jacobian cond median %.2f, p90 %.2f, max %.2f; min singular value %.3f" % (
            np.median(C), np.percentile(C, 90), C.max(), S.min()))
        box = {k: v for k, v in inside.items() if abs(k[0]) <= 34 and abs(k[1]) <= 34}
        Cb = np.array([v["cond"] for v in box.values()])
        Sb = np.array([v["smin"] for v in box.values()])
        nbox = sum(1 for k in out if abs(k[0]) <= 34 and abs(k[1]) <= 34)
        print("   crank box +-34 deg: %d/%d grid points feasible (tilt <= 15); cond median %.2f p90 %.2f max %.2f; min singular %.3f" % (
            len(box), nbox, np.median(Cb), np.percentile(Cb, 90), Cb.max(), Sb.min()))
        for (a, b) in [(0, 0), (-34, -34), (34, 34), (-34, 34), (34, -34)]:
            v = out.get((a, b))
            print("   upper %+d lower %+d: %s" % (a, b, "no solution" if v is None else
                  "roll %+.2f pitch %+.2f tilt %.2f cond %.2f" % (v["roll"], v["pitch"], v["tilt"], v["cond"])))
        edge = {}
        for (a, b), v in inside.items():
            edge.setdefault(a, [b, b])
            edge[a][0] = min(edge[a][0], b)
            edge[a][1] = max(edge[a][1], b)
        print("   per upper-crank angle, lower-crank range with tilt <= 15 deg:")
        print("   " + "  ".join("%+d:[%+d,%+d]" % (a, *edge[a]) for a in sorted(edge) if a % 10 == 0))
