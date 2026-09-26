import sys

import mujoco
import numpy as np

sys.path.insert(0, "/home/polygon/humanoid_project/robonex-walking/etc/delegates/2026-09-24_physopt/ankle_verification")
import os
os.environ["ANKLE_SRC"] = "unused"
from common import geoms, place, depth, any_hit
from loops import Loops, leg_passive

STEP = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
lp = Loops(path=os.environ["LOOPS_PATH"]) if os.environ.get("LOOPS_PATH") else Loops("edu")
m, d = lp.model, lp.data
for s in "lr":
    bodies = ["%s_%s" % (s, n) for n in ("knee_link", "ankle_crank_link_a", "ankle_crank_link_b",
                                         "ankle_coupler_link_a", "ankle_coupler_link_b", "ankle_link", "foot")]
    G = geoms(m, bodies)
    pairs = [(i, k) for i in range(len(G)) for k in range(i + 1, len(G)) if G[i][1] != G[k][1]]
    up, lo = "%s_ankle_upper_joint" % s, "%s_ankle_lower_joint" % s
    free = leg_passive(s)
    lp.set({n: 0.0 for n in lp.names})
    lp.solve({up: 0.0, lo: 0.0}, free)
    place(d, G)
    base = {pk: (depth(G[pk[0]][3], G[pk[1]][3]) if any_hit(G[pk[0]][3], G[pk[1]][3]) else 0.0) for pk in pairs}
    iface = {pk for pk, v in base.items() if v > 0}
    grid = np.arange(-34, 34.1, STEP)
    hits = {}
    for i, a in enumerate(grid):
        for b in (grid if i % 2 == 0 else grid[::-1]):
            res, _ = lp.solve({up: np.radians(a), lo: np.radians(b)}, free)
            if res > 1e-9:
                continue
            place(d, G)
            for pk in pairs:
                o1, o2 = G[pk[0]][3], G[pk[1]][3]
                if not any_hit(o1, o2):
                    continue
                dep = depth(o1, o2)
                if dep - base[pk] > 0.0005:
                    hits.setdefault((G[pk[0]][2], G[pk[1]][2]), []).append((a, b, (dep - base[pk]) * 1000))
    print("%s: %d grid points (step %.0f deg), %d mesh pairs, %d already touching at zero:" % (
        s, len(grid) ** 2, STEP, len(pairs), len(iface)))
    for pk in iface:
        print("   at zero: %s - %s depth %.2f mm" % (G[pk[0]][2], G[pk[1]][2], base[pk] * 1000))
    if not hits:
        print("   no additional penetration > 0.5 mm anywhere in the grid")
    for k, v in sorted(hits.items(), key=lambda kv: -len(kv[1])):
        w = max(v, key=lambda r: r[2])
        print("   %-48s %4d points, deepest +%.1f mm at upper %+d lower %+d" % (" - ".join(k), len(v), w[2], w[0], w[1]))
