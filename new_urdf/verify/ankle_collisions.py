import mujoco
import numpy as np

from loops import Loops, leg_passive

lp = Loops("edu")
m = lp.model
ANK = {"%s_%s" % (s, n) for s in "lr" for n in ("knee_link", "ankle_crank_link_a", "ankle_crank_link_b",
                                                  "ankle_coupler_link_a", "ankle_coupler_link_b", "ankle_link", "foot")}
for s in "lr":
    up, lo = "%s_ankle_upper_joint" % s, "%s_ankle_lower_joint" % s
    free = leg_passive(s)
    lp.set({n: 0.0 for n in lp.names})
    hits = {}
    grid = np.arange(-34, 34.1, 2.0)
    for i, a in enumerate(grid):
        for b in (grid if i % 2 == 0 else grid[::-1]):
            res, _ = lp.solve({up: np.radians(a), lo: np.radians(b)}, free)
            if res > 1e-9:
                continue
            for c in lp.data.contact[:lp.data.ncon]:
                b1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom1])
                b2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom2])
                if b1 in ANK and b2 in ANK and c.dist < -0.0005:
                    hits.setdefault(tuple(sorted((b1, b2))), []).append((a, b, c.dist * 1000))
    print("%s: %d grid points; ankle-region contacts (convex hulls, excluded neighbour pairs skipped):" % (s, len(grid) ** 2))
    if not hits:
        print("   none")
    for k, v in hits.items():
        worst = min(v, key=lambda r: r[2])
        print("   %-50s %4d points, deepest %.1f mm at upper %+d lower %+d" % (" - ".join(k), len(v), worst[2], worst[0], worst[1]))
