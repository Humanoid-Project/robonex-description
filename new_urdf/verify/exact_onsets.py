import math
import sys

src = open("limit_tool.py").read().split("with lock:\n    apply()\nthreading.Thread")[0]
sys.argv = ["limit_tool.py"]
g = {"__name__": "onsets", "__file__": __file__.replace("exact_onsets.py", "limit_tool.py")}
exec(compile(src, "limit_tool.py", "exec"), g)
target, apply, exact, home, state = g["target"], g["apply"], g["exact_check"], g["home"], g["state"]
SWEEPS = [("l_hip_yaw_joint", -90, 90), ("l_hip_pitch_joint", -100, 100), ("l_hip_roll_joint", -120, 10),
          ("l_knee_pitch_joint", -83, 19), ("l_ankle_upper_joint", -60, 60), ("l_ankle_lower_joint", -60, 60)]
for name, lo, hi in SWEEPS:
    for direction, end in ((-1, lo), (1, hi)):
        home()
        apply()
        start = target[name]
        found = None
        a = start
        while (a - end) * direction < 0:
            a = a + direction * 1.0
            if (a - end) * direction > 0:
                a = end
            target[name] = a
            apply()
            if not state["ok"]:
                found = (a, "loop cannot close")
                break
            hits = [h for h in exact() if h[0] > 0.5]
            if hits:
                found = (a, "%s-%s %.1f mm" % (hits[0][1], hits[0][2], hits[0][0]))
                break
        print("%-20s from %+6.1f toward %+6.1f: %s" % (name, start, end,
              "clear to the end" if found is None else "first contact at %+.0f deg: %s" % found), flush=True)
