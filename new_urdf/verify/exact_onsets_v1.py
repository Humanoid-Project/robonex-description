import math
import os
import sys

import mujoco

sys.path.insert(0, "/home/polygon/humanoid_project/robonex-common/src")
sys.path.insert(0, "/home/polygon/humanoid_project/robonex-walking/etc/delegates/2026-09-24_physopt/ankle_verification")
os.environ["ANKLE_SRC"] = "unused"
from common import geoms, place, depth, any_hit
from robonex_common.joints import DEFAULT_JOINT_POS
from loops import Loops, leg_passive

lp = Loops(path="/home/polygon/humanoid_project/robonex-description/mujoco/robot/scene_fixed.xml")
m, d = lp.model, lp.data
ACT = list(DEFAULT_JOINT_POS)
PASSIVE = leg_passive("l") + leg_passive("r")
names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(m.nbody)]
G = geoms(m, [n for n in names if n and n != "world"])
pairs = [(i, k) for i in range(len(G)) for k in range(i + 1, len(G)) if G[i][1] != G[k][1]]


def depths():
    place(d, G)
    return {pk: (depth(G[pk[0]][3], G[pk[1]][3]) if any_hit(G[pk[0]][3], G[pk[1]][3]) else 0.0) for pk in pairs}


for n in lp.names:
    d.qpos[lp.qadr[n]] = 0.0
mujoco.mj_forward(m, d)
base = depths()
for name, lo, hi in [(a.split(":")[0], int(a.split(":")[1]), int(a.split(":")[2])) for a in sys.argv[1:]]:
    for end in (lo, hi):
        q = {n: math.degrees(v) for n, v in DEFAULT_JOINT_POS.items()}
        for n in lp.names:
            d.qpos[lp.qadr[n]] = 0.0
        lp.solve({n: math.radians(v) for n, v in q.items()}, PASSIVE)
        a, found = q[name], None
        step = 1.0 if end > a else -1.0
        while (end - a) * step > 0:
            a += step
            q[name] = a
            res, _ = lp.solve({n: math.radians(v) for n, v in q.items()}, PASSIVE)
            if res > 1e-7:
                found = (a, "loop cannot close")
                break
            now = depths()
            hits = sorted(((now[pk] - base[pk]) * 1000, G[pk[0]][2], G[pk[1]][2]) for pk in pairs if now[pk] - base[pk] > 0.0005)
            if hits:
                h = hits[-1]
                found = (a, "%s-%s %.1f mm" % (h[1], h[2], h[0]))
                break
        print("Ver.1 %-20s toward %+4d: %s" % (name, end, "clear" if found is None else "first contact at %+.0f deg: %s" % found), flush=True)
