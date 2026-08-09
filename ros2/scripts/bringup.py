#!/usr/bin/env python3
import re
import subprocess
import sys
import threading
import time

WORLD = "robonex_world"
NEEDED = ("joint_state_broadcaster", "joint_trajectory_controller")
DEADLINE = 150.0
STEPS_PER_CALL = 5
ANSI = re.compile(r"\x1b\[[0-9;]*m")

CONTROL = "/world/%s/control" % WORLD
REQ = ["ign", "service", "-s", CONTROL,
       "--reqtype", "ignition.msgs.WorldControl",
       "--reptype", "ignition.msgs.Boolean"]

JOINTS = [
    "l_hip_yaw_joint", "l_hip_pitch_joint", "l_hip_roll_joint",
    "l_knee_joint", "l_ankle_roll_joint", "l_ankle_pitch_joint",
    "r_hip_yaw_joint", "r_hip_pitch_joint", "r_hip_roll_joint",
    "r_knee_joint", "r_ankle_roll_joint", "r_ankle_pitch_joint",
]

stepping = threading.Event()


def hold_stance():
    msg = ("{joint_names: [%s], points: [{positions: [%s],"
           " time_from_start: {sec: 1}}]}"
           % (",".join(JOINTS), ",".join("0.0" for _ in JOINTS)))
    try:
        subprocess.run(["ros2", "topic", "pub", "-t", "10", "-w", "1", "-r", "5",
                        "/joint_trajectory_controller/joint_trajectory",
                        "trajectory_msgs/msg/JointTrajectory", msg],
                       capture_output=True, text=True, timeout=20)
    except subprocess.TimeoutExpired:
        pass


def world_control(req, timeout="400"):
    try:
        return subprocess.run(REQ + ["--timeout", timeout, "--req", req],
                              capture_output=True, text=True, timeout=6).stdout
    except subprocess.TimeoutExpired:
        return ""


def step_forever():
    while stepping.is_set():
        world_control("pause: true, multi_step: %d" % STEPS_PER_CALL)


def controllers_up():
    try:
        out = subprocess.run(["ros2", "control", "list_controllers"],
                             capture_output=True, text=True, timeout=15).stdout
    except subprocess.TimeoutExpired:
        return False
    out = ANSI.sub("", out)
    return all(re.search(r"^%s\s+\S+\s+active" % name, out, re.M) for name in NEEDED)


def main():
    start = time.time()
    while time.time() - start < 60.0:
        if world_control("pause: true"):
            break
        time.sleep(0.5)
    else:
        print("[bringup] world control service never answered", flush=True)
        return 1

    print("[bringup] world paused, stepping while the controllers come up", flush=True)
    stepping.set()
    worker = threading.Thread(target=step_forever, daemon=True)
    worker.start()

    ok = False
    while time.time() - start < DEADLINE:
        if controllers_up():
            ok = True
            break
        time.sleep(1.0)

    if not ok:
        stepping.clear()
        worker.join(timeout=8)
        print("[bringup] controllers did not activate; world left paused", flush=True)
        return 1

    print("[bringup] controllers active after %.1f s, commanding the nominal stance"
          % (time.time() - start), flush=True)
    hold_stance()
    time.sleep(2.0)

    stepping.clear()
    worker.join(timeout=8)

    print("[bringup] running", flush=True)
    world_control("pause: false", timeout="5000")
    return 0


if __name__ == "__main__":
    sys.exit(main())
