import argparse
import math
import os
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from loops import Loops, leg_passive

parser = argparse.ArgumentParser(description="Move RoboNex Ver.2 leg joints in degrees with the loops solved.")
parser.add_argument("--variant", default="edu", choices=("edu", "pro", "max"))
args = parser.parse_args()

ALIAS = {}
for s in "lr":
    for short, name in (("yaw", "hip_yaw"), ("pitch", "hip_pitch"), ("roll", "hip_roll"), ("knee", "knee_pitch"),
                        ("up", "ankle_upper"), ("low", "ankle_lower")):
        ALIAS[s + short] = "%s_%s_joint" % (s, name)
ACT = list(ALIAS.values())
PASSIVE = leg_passive("l") + leg_passive("r")
lp = Loops(args.variant)
m, d = lp.model, lp.data
for n in lp.names:
    if n in lp.qadr:
        d.qpos[lp.qadr[n]] = 0.0
target = {n: 0.0 for n in ACT}
state = {"ok": True, "res": 0.0}
lock = threading.Lock()
LEG_BODIES = None


def mirror(name):
    return ("r_" + name[2:]) if name.startswith("l_") else ("l_" + name[2:])


def apply():
    good = {n: d.qpos[lp.qadr[n]] for n in ACT + PASSIVE}
    start = {n: math.degrees(good[n]) for n in ACT}
    steps = max(1, int(math.ceil(max(abs(target[n] - start[n]) for n in ACT))))
    res = 0.0
    for k in range(1, steps + 1):
        f = k / steps
        res, _ = lp.solve({n: math.radians(start[n] + f * (target[n] - start[n])) for n in ACT}, PASSIVE)
        if res >= 1e-7:
            break
    state["res"] = res
    state["ok"] = res < 1e-7
    if not state["ok"]:
        for n, v in good.items():
            d.qpos[lp.qadr[n]] = v
        for n in ACT:
            target[n] = start[n]
        mujoco.mj_forward(m, d)


def contacts():
    out = set()
    for c in d.contact[:d.ncon]:
        b1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom1])
        b2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom2])
        if b1 and b2 and c.dist < -0.0005:
            out.add("%s-%s" % tuple(sorted((b1, b2))))
    return sorted(out)


def exact_check():
    global LEG_BODIES
    sys.path.insert(0, "/home/polygon/humanoid_project/robonex-walking/etc/delegates/2026-09-24_physopt/ankle_verification")
    os.environ.setdefault("ANKLE_SRC", "unused")
    from common import geoms, place, depth, any_hit
    if LEG_BODIES is None:
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(m.nbody)]
        LEG_BODIES = geoms(m, [n for n in names if n and n != "world"])
    G = LEG_BODIES
    pairs = [(i, k) for i in range(len(G)) for k in range(i + 1, len(G)) if G[i][1] != G[k][1]]

    def depths():
        place(d, G)
        return {pk: (depth(G[pk[0]][3], G[pk[1]][3]) if any_hit(G[pk[0]][3], G[pk[1]][3]) else 0.0) for pk in pairs}

    if "baseline" not in state:
        saved = d.qpos.copy()
        for n in ACT + PASSIVE:
            d.qpos[lp.qadr[n]] = 0.0
        mujoco.mj_forward(m, d)
        state["baseline"] = depths()
        d.qpos[:] = saved
        mujoco.mj_forward(m, d)
    now = depths()
    hits = [((now[pk] - state["baseline"][pk]) * 1000, G[pk[0]][2], G[pk[1]][2]) for pk in pairs
            if now[pk] - state["baseline"][pk] > 0.0003]
    return sorted(hits, reverse=True)


def deg(n):
    return math.degrees(d.qpos[lp.qadr[n]])


def show():
    lines = []
    for s in "lr":
        lines.append("%s  yaw %+7.2f  pitch %+7.2f  roll %+7.2f  knee crank %+7.2f  ankle up %+7.2f  low %+7.2f" % (
            s.upper(), deg("%s_hip_yaw_joint" % s), deg("%s_hip_pitch_joint" % s), deg("%s_hip_roll_joint" % s),
            deg("%s_knee_pitch_joint" % s), deg("%s_ankle_upper_joint" % s), deg("%s_ankle_lower_joint" % s)))
        tilt = max(abs(deg("%s_ankle_coupler_joint_%s_%s" % (s, ab, ax))) for ab in "ab" for ax in "xz")
        lines.append("   knee output %+7.2f   ankle roll %+7.2f  pitch %+7.2f   rod-end tilt %5.2f%s" % (
            deg("%s_knee_joint" % s), deg("%s_ankle_roll_joint" % s), deg("%s_ankle_pitch_joint" % s), tilt,
            "  <-- over 15" if tilt > 15 else ""))
    lines.append("loop: %s (residual %.1e m)   contacts (convex, neighbours excluded): %s" % (
        "closed" if state["ok"] else "NO SOLUTION - last valid pose kept", state["res"], ", ".join(contacts()) or "none"))
    print("\n".join(lines), flush=True)


HELP = """commands (degrees):
  <joint> <deg>        e.g. lknee -30, rup 20, lroll -45   (joints: l/r + yaw pitch roll knee up low)
  sym <joint> <deg>    set the left joint and its mirror (right gets the mirrored sign)
  both <joint> <deg>   set left and right to the same value
  zero | home          all zero | training default pose
  check                exact mesh collision check of every body pair (python-fcl, a few seconds)
  show | help | quit"""


def home():
    import json
    c = json.load(open(os.path.join(os.path.dirname(HERE), "scripts", "ver2_constants.json")))
    for n in ACT:
        target[n] = math.degrees(c["default_actuated_pos"][n])


def command(line):
    parts = line.split()
    if not parts:
        return
    if parts[0] in ("quit", "exit", "q"):
        os._exit(0)
    if parts[0] == "help":
        print(HELP)
        return
    if parts[0] == "zero":
        for n in ACT:
            target[n] = 0.0
    elif parts[0] == "home":
        home()
    elif parts[0] == "check":
        with lock:
            hits = exact_check()
        print("exact mesh penetration beyond the zero-pose contact (> 0.3 mm):" if hits else "exact mesh check: no penetration beyond the zero-pose contact", flush=True)
        for dep, a, b in hits:
            print("   %6.2f mm  %s - %s" % (dep, a, b), flush=True)
        return
    elif parts[0] in ("sym", "both") and len(parts) == 3 and (parts[1] in ALIAS or "l" + parts[1] in ALIAS):
        name, v = ALIAS[parts[1] if parts[1] in ALIAS else "l" + parts[1]], float(parts[2])
        target[name] = v
        target[mirror(name)] = -v if parts[0] == "sym" else v
    elif parts[0] in ALIAS and len(parts) == 2:
        target[ALIAS[parts[0]]] = float(parts[1])
    elif parts[0] != "show":
        print("?", HELP, flush=True)
        return
    with lock:
        apply()
    show()


def reader():
    print(HELP, flush=True)
    while True:
        try:
            line = input("> ")
        except EOFError:
            os._exit(0)
        try:
            command(line.strip())
        except Exception as error:
            print("error:", error, flush=True)


with lock:
    apply()
threading.Thread(target=reader, daemon=True).start()
with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        with lock:
            viewer.sync()
        time.sleep(0.03)
os._exit(0)
