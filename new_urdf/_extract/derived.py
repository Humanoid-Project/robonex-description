import numpy as np, json, os
from lower_model import build, to_urdf, LINK_PARTS, link_origins_f
from make_meshes import read_stl
J, loops = build()
P = {n: to_urdf(j["p_f"]) for n, j in J.items()}
pin = {p["name"][0]: to_urdf(p["p_f"]) for p in loops["pin"]}
def xz(v): return np.array([v[0], v[2]])
out = {}
for s in "lr":
    A = xz(P[s + "_knee_pitch_joint"]); B = xz(P[s + "_knee_coupler_joint_a"]); C = xz(pin[s]); D = xz(P[s + "_knee_joint"])
    crank, coup, rock, ground = [np.linalg.norm(u) for u in (B - A, C - B, C - D, D - A)]
    # four-bar ratio d(knee)/d(crank) at zero: rotate crank by +-h about A, solve C on circles (B,coup) & (D,rock)
    def solve(th):
        R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        Bn = A + R @ (B - A)
        d = np.linalg.norm(Bn - D); a = (coup**2 - rock**2 + d**2) / (2 * d); h = np.sqrt(coup**2 - a**2)
        base = Bn + a * (D - Bn) / d; perp = np.array([-(D - Bn)[1], (D - Bn)[0]]) / d
        cands = [base + h * perp, base - h * perp]
        Cn = min(cands, key=lambda c: np.linalg.norm(c - C))
        return np.arctan2(*(Cn - D)[::-1]) - np.arctan2(*(C - D)[::-1])
    hstep = 1e-4
    ratio = (solve(hstep) - solve(-hstep)) / (2 * hstep)
    out[s] = dict(crank=crank, coupler=coup, rocker=rock, ground=ground, ratio=ratio)
    print("%s knee four-bar: crank %.3f coupler %.3f rocker %.3f ground %.3f mm | d(knee)/d(crank) at zero = %.4f" % (s, crank, coup, rock, ground, ratio))
for s, up, lo in (("l", "l_ankle_coupler_joint_b", "l_ankle_coupler_joint_a"), ("r", "r_ankle_coupler_joint_b", "r_ankle_coupler_joint_a")):
    for tag, ball, mot in (("upper", up, s + "_ankle_upper_joint"), ("lower", lo, s + "_ankle_lower_joint")):
        v = P[ball] - P[mot]
        print("%s ankle %s crank: radius(XZ) %.3f mm, ball along axis %.3f mm, angle above horizontal %.2f deg" % (
            s, tag, np.hypot(v[0], v[2]), v[1], np.degrees(np.arctan2(v[2], v[0]))))
    for b in loops["ball"]:
        if b["name"].startswith(s):
            rod = next(j for j in J.values() if j["child"] == b["parent"])
            print("   rod %s length %.3f mm" % (b["parent"], np.linalg.norm(to_urdf(b["p_f"]) - to_urdf(rod["p_f"]))))
L = lambda a, b: P[a][2] - P[b][2]
print("hip pitch -> knee joint  %.3f | knee joint -> ankle roll %.3f | roll -> pitch %.3f | hip pitch -> ankle pitch %.3f mm" % (
    L("l_hip_pitch_joint", "l_knee_joint"), L("l_knee_joint", "l_ankle_roll_joint"), L("l_ankle_roll_joint", "l_ankle_pitch_joint"), L("l_hip_pitch_joint", "l_ankle_pitch_joint")))
print("hip roll -> knee joint (thigh) %.3f | ankle motors below knee joint: upper %.3f lower %.3f mm" % (
    L("l_hip_roll_joint", "l_knee_joint"), L("l_knee_joint", "l_ankle_upper_joint"), L("l_knee_joint", "l_ankle_lower_joint")))
# collision boxes from the link-part mesh only (Ver.1 convention), link frame, metres
print("COLLISION_BOX (link part mesh bbox, m): size, centre")
here = os.path.dirname(os.path.abspath(__file__))
for link in ["base_link", "l_hip_yaw_link", "r_hip_yaw_link", "l_hip_pitch_link", "r_hip_pitch_link", "l_hip_roll_link", "r_hip_roll_link",
             "l_knee_link", "r_knee_link", "l_ankle_link", "r_ankle_link", "l_foot", "r_foot"]:
    v = read_stl(os.path.join(os.path.dirname(here), "meshes", link + ".stl"))["v"].reshape(-1, 3) / 1000.0
    lo, hi = v.min(0), v.max(0)
    print('    "%s": ((%.4f, %.4f, %.4f), (%.4f, %.4f, %.4f)),' % (link, *(hi - lo), *((hi + lo) / 2)))
print("zero-pose base height with the lowest sole vertex on the ground: L %.4f R %.4f m" % (0.95011, 0.94934))
