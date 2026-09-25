import sys

import numpy as np
import trimesh

sys.path.insert(0, "../_extract")
from urdf_io import load
from lower_model import LINK_PARTS, MASS, MASS_SCALE, mesh_of

L, J = load(sys.argv[1] if len(sys.argv) > 1 else "../urdf/robonex_max.urdf")
rows = []
for link, parts in LINK_PARTS.items():
    files = list(dict.fromkeys(mesh_of(p) for p in parts))
    if len(files) != len(parts):
        continue
    acc = []
    ok = True
    info = []
    for p in parts:
        m = trimesh.load("../meshes/%s.stl" % mesh_of(p), force="mesh")
        m.apply_scale(0.001)
        info.append("%s wt=%s vol=%.1fcm3" % (mesh_of(p), m.is_watertight, m.volume * 1e6))
        if not m.is_watertight or m.volume <= 0:
            ok = False
        mass = MASS[p]["mass_kg"] * MASS_SCALE.get(p, 1.0)
        m.density = mass / m.volume
        acc.append((mass, m.center_mass, m.moment_inertia))
    M = sum(a[0] for a in acc)
    C = sum(a[0] * a[1] for a in acc) / M
    I = np.zeros((3, 3))
    for m_, c, Ic in acc:
        d = c - C
        I += Ic + m_ * (d @ d * np.eye(3) - np.outer(d, d))
    u = L[link]
    dc = np.abs(C - u["com"]).max() * 1000
    dI = np.abs(I - u["I"]).max() / np.abs(u["I"]).max()
    sign = [(i, j) for i, j in ((0, 1), (0, 2), (1, 2)) if abs(u["I"][i, j]) > 1e-3 * np.abs(u["I"]).max()
            and np.sign(I[i, j]) != np.sign(u["I"][i, j])]
    rows.append((link, ok, dc, dI, sign, info))
    print(f"{link:<24} watertight={ok!s:<5} COM diff {dc:7.3f} mm  I rel diff {dI:6.3f}  sign flips {sign}")
    if dc > 1 or dI > 0.05:
        print("    urdf I", np.round(u["I"] * 1e6, 3).tolist())
        print("    mesh I", np.round(I * 1e6, 3).tolist(), info)
