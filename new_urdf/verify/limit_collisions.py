import json
import sys

import mujoco
import numpy as np

from loops import Loops, leg_passive

V1 = {"hip_yaw": (-1.658063, 1.658063), "hip_pitch": (-1.745329, 1.745329), "hip_roll": (-2.146755, 0.453786),
      "knee_pitch": (-1.361357, 0.261799), "ankle_upper": (-0.610865, 0.575959), "ankle_lower": (-0.610865, 0.575959)}
C = json.load(open("../scripts/ver2_constants.json"))


def contacts(lp):
    m, d = lp.model, lp.data
    mujoco.mj_forward(m, d)
    out = set()
    for c in d.contact[:d.ncon]:
        b1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom1])
        b2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom2])
        if b1 and b2 and c.dist < -0.001:
            out.add(tuple(sorted((b1, b2))))
    return out


for variant in sys.argv[1:] or ["edu", "pro"]:
    lp = Loops(variant)
    for s, sg in (("l", 1.0), ("r", -1.0)):
        for jn, (lo, hi) in V1.items():
            name = "%s_%s_joint" % (s, jn)
            if s == "r":
                lo, hi = -hi, -lo
            base = dict(C["default_actuated_pos"])
            for direction, end in ((-1, lo), (1, hi)):
                lp.set({n: 0.0 for n in lp.names})
                first = None
                for t in np.linspace(base[name], end, 40):
                    q = dict(base)
                    q[name] = t
                    lp.solve(q, leg_passive("l") + leg_passive("r"))
                    cs = contacts(lp)
                    if cs:
                        first = (np.degrees(t), sorted(cs))
                        break
                print("%s %-22s -> %+7.1f deg: %s" % (variant, name, np.degrees(end),
                      "clear" if first is None else "contact at %+.1f deg %s" % first))
