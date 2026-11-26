#!/usr/bin/env python3
"""RoboNex Ver.3 model tools.

Usage (from robonex-description/): .venv/bin/python -I ver3/tools/ver3_model.py <command>

  install --package DIR   copy the Windows package (ver3.urdf, loop_closures.yaml, meshes/) into ver3/
  constants --seed-limits-only
                          write only provisional_limits (before the first fixed-base MJCF)
  constants               solve the home pose on mujoco/robot/full/scene_fixed.xml; writes ver3_constants.json
  loops [--pose zero|home] [--scene PATH]
  check                   link/joint counts, total mass, SPD inertias, L/R mirror residuals

Regeneration order:
  1. ver3/tools/ver3_model.py install --package DIR
  2. ver3/tools/ver3_model.py constants --seed-limits-only
  3. ver3/mujoco/build_mjcf.py --variant full --fixed-base
  4. ver3/tools/ver3_model.py constants
  5. ver3/mujoco/build_mjcf.py --variant {full,legs} [--fixed-base]
  6. ver3/isaac/build_isaac_urdf.py --variant {full,legs} --collision {mesh,box}
  7. ver3/tools/ver3_model.py loops --pose home; check
"""
import argparse
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(ROOT)
os.environ["ROBONEX_VERSION"] = os.path.basename(ROOT)
sys.path.insert(0, os.path.join(REPO, "scripts"))

import mujoco
import numpy as np

import model_tools as mt
from model_io import CONSTANTS_PATH, LOOPS_PATH, MESH_DIR, URDF_PATH
from robonex_data_ver3 import LEG_JOINTS, MOTOR_IDS, UPPER_BODY_JOINTS, common_limits, common_name

from robonex_common.joints import DEFAULT_JOINT_POS as COMMON_DEFAULT_POS, POLICY_JOINT_ORDER

SCENE_FIXED = os.path.join(ROOT, "mujoco", "robot", "full", "scene_fixed.xml")
HIP_PITCH = 0.13
KNEE_OUT = 0.35
ANKLE_PITCH = KNEE_OUT - HIP_PITCH
SPAWN_MARGIN = 0.006
SIDES = ("left", "right")
LEG_BOX_LINKS = ["%s_%s" % (s, n) for s in SIDES for n in (
    "hip_yaw_link", "hip_pitch_link", "hip_roll_link", "knee_link", "ankle_roll_link", "ankle_pitch_link")]
OWN_MESH_BOX_LINKS = ["base_link"] + LEG_BOX_LINKS
UNION_BOX_LINKS = ["neck_yaw_link", "neck_pitch_link", "head_link"] + ["%s_%s" % (s, n) for s in SIDES for n in (
    "shoulder_pitch_link", "shoulder_roll_link", "shoulder_yaw_link", "elbow_link", "wrist_roll_link")]
PROVISIONAL_LIMITS_NOTE = (
    "robonex-common 2.5.0 limits by motor ID: legs = the Ver.2-4 command limits (common 2.4.0, 2026-11-18; the legs "
    "of ver3 are identical to ver2), arms and neck = AUXILIARY_JOINTS placeholders (shoulder pitch +-100 deg, neck "
    "pitch +-73 deg hand-checked on Ver.2; the rest +-45 / +-30 deg). The ver3 elbow zero is new (forearm forward, "
    "elbow 90 deg): its placeholder range is centred on the new zero and was never measured")
UPPER_BODY_NOTE = (
    "variant full: arms and neck are actuated position joints (common placeholder gains), the walking policy may hold "
    "them at upper_body_default_pos; variant legs: arms and neck fixed at 0 rad. Axes follow the retired Ver.2 max "
    "convention and are not checked on the robot")


def leg_passive(s):
    return ["%s_knee_joint" % s, "%s_knee_coupler_joint" % s, "%s_ankle_roll_joint" % s, "%s_ankle_pitch_joint" % s] + [
        "%s_ankle_%s_coupler_joint_%s" % (s, ul, ax) for ul in ("upper", "lower") for ax in "xyz"]


def mirror(name):
    if name.startswith("left_"):
        return "right_" + name[5:]
    if name.startswith("right_"):
        return "left_" + name[6:]
    return name


def install(package):
    package = os.path.abspath(os.path.expanduser(package))
    src_urdf = os.path.join(package, "ver3.urdf")
    src_loops = os.path.join(package, "loop_closures.yaml")
    src_meshes = os.path.join(package, "meshes")
    for p in (src_urdf, src_loops, src_meshes):
        if not os.path.exists(p):
            raise SystemExit("%s: missing %s" % (package, os.path.basename(p)))
    text = open(src_urdf, encoding="utf-8").read()
    n = text.count('filename="meshes/')
    text = text.replace('filename="meshes/', 'filename="../meshes/')
    os.makedirs(os.path.dirname(URDF_PATH), exist_ok=True)
    with open(URDF_PATH, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    shutil.copyfile(src_loops, LOOPS_PATH)
    os.makedirs(MESH_DIR, exist_ok=True)
    meshes = sorted(m for m in os.listdir(src_meshes) if m.endswith(".stl"))
    for m in meshes:
        shutil.copyfile(os.path.join(src_meshes, m), os.path.join(MESH_DIR, m))
    print("wrote %s (%d mesh paths -> ../meshes/)" % (URDF_PATH, n))
    print("wrote %s" % LOOPS_PATH)
    print("copied %d meshes to %s" % (len(meshes), MESH_DIR))


def box_of(paths):
    V = np.concatenate([mt.read_stl(p)["v"].reshape(-1, 3).astype(float) / 1000.0 for p in paths])
    lo, hi = V.min(0), V.max(0)
    return [[round(float(v), 4) for v in hi - lo], [round(float(v), 4) for v in (hi + lo) / 2]]


def collision_boxes():
    L, _ = mt.load_urdf(URDF_PATH)
    boxes = {}
    for link in OWN_MESH_BOX_LINKS:
        boxes[link] = box_of([os.path.join(MESH_DIR, link + ".stl")])
    for link in UNION_BOX_LINKS:
        boxes[link] = box_of([os.path.join(MESH_DIR, mt.mesh_name(m[0]) + ".stl") for m in L[link]["meshes"]])
    return boxes


def sole_corners(lp=None):
    ext = []
    for s in SIDES:
        V = mt.read_stl(os.path.join(MESH_DIR, "%s_ankle_pitch_link.stl" % s))["v"].reshape(-1, 3).astype(float) / 1000.0
        z0 = V[:, 2].min()
        sole = V[V[:, 2] < z0 + 0.0005]
        ext.append((sole[:, 0].min(), sole[:, 0].max(), sole[:, 1].min(), sole[:, 1].max(), z0))
    e = np.array(ext)
    x0, x1, y0, y1 = e[:, 0].min(), e[:, 1].max(), e[:, 2].min(), e[:, 3].max()
    z = e[:, 4].min()
    return [(round(x0, 4), round(y0, 4), round(z, 4)), (round(x0, 4), round(y1, 4), round(z, 4)),
            (round(x1, 4), round(y0, 4), round(z, 4)), (round(x1, 4), round(y1, 4), round(z, 4))]


def lowest_corner_z(lp, corners):
    zs = []
    for s in SIDES:
        R, p = lp.body_R("%s_ankle_pitch_link" % s), lp.body_p("%s_ankle_pitch_link" % s)
        zs.append(((np.array(corners) @ R.T + p)[:, 2]).min())
    return min(zs)


def total_mass():
    L, _ = mt.load_urdf(URDF_PATH)
    return round(sum(l.get("mass", 0.0) for l in L.values()), 6)


def seed_limits():
    const = json.load(open(CONSTANTS_PATH)) if os.path.exists(CONSTANTS_PATH) else {}
    const["provisional_limits"] = common_limits()
    with open(CONSTANTS_PATH, "w") as f:
        json.dump(const, f, indent=2)
    print("wrote provisional_limits (%d joints) to %s" % (len(const["provisional_limits"]), CONSTANTS_PATH))


def constants():
    if not os.path.exists(SCENE_FIXED):
        raise SystemExit("constants needs %s: run ver3/mujoco/build_mjcf.py --variant full --fixed-base first"
                         % SCENE_FIXED)
    lp = mt.Loops(path=SCENE_FIXED)
    corners = sole_corners()
    lp.set({n: 0.0 for n in lp.names})
    mujoco.mj_forward(lp.model, lp.data)
    base_z = lp.body_p("base_link")[2]
    zero_height = base_z - lowest_corner_z(lp, corners)

    fixed, free = {}, []
    for s, sg in (("left", 1.0), ("right", -1.0)):
        fixed.update({"%s_hip_yaw_joint" % s: 0.0, "%s_hip_roll_joint" % s: 0.0,
                      "%s_hip_pitch_joint" % s: sg * HIP_PITCH, "%s_knee_joint" % s: -sg * KNEE_OUT,
                      "%s_ankle_roll_joint" % s: 0.0, "%s_ankle_pitch_joint" % s: ANKLE_PITCH})
        free += ["%s_knee_pitch_joint" % s, "%s_knee_coupler_joint" % s,
                 "%s_ankle_upper_joint" % s, "%s_ankle_lower_joint" % s] + [
            "%s_ankle_%s_coupler_joint_%s" % (s, ul, ax) for ul in ("upper", "lower") for ax in "xyz"]
    res, it = lp.solve(fixed, free, tol=1e-13)
    solved = {n: lp.q(n) for n in LEG_JOINTS}
    contract = {n: float(COMMON_DEFAULT_POS[common_name(n)]) for n in LEG_JOINTS}
    pose_shift = max(abs(solved[n] - contract[n]) for n in LEG_JOINTS)
    if pose_shift > 1e-4:
        raise SystemExit("solved default pose differs from robonex-common DEFAULT_JOINT_POS by %.3e rad" % pose_shift)
    default_act = contract
    passive_names = [n for s in SIDES for n in leg_passive(s)]
    res2, it2 = lp.solve({**default_act, **{n: 0.0 for n in UPPER_BODY_JOINTS}}, passive_names, tol=1e-13)
    passive = {n: lp.q(n) for n in passive_names}
    home_height = base_z - lowest_corner_z(lp, corners)
    feet_y = abs(lp.body_p("left_ankle_pitch_link")[1] - lp.body_p("right_ankle_pitch_link")[1])
    hip_h = lp.body_p("left_hip_pitch_link")[2] - base_z + home_height
    sole_level = [float(np.degrees(np.arccos(np.clip(lp.body_R("%s_ankle_pitch_link" % s)[2, 2], -1, 1))))
                  for s in SIDES]
    const = {
        "zero_pose_base_height": round(float(zero_height), 5),
        "mujoco_spawn_height": round(float(zero_height + SPAWN_MARGIN), 4),
        "home_base_height": round(float(home_height), 5),
        "default_actuated_pos": {k: float(v) for k, v in default_act.items()},
        "home_passive_pos": {k: float(v) for k, v in passive.items()},
        "default_output_pose": {"hip_pitch": HIP_PITCH, "knee": KNEE_OUT, "ankle_pitch": ANKLE_PITCH},
        "foot_sole_corners": corners,
        "foot_origin_rest_height": round(-corners[0][2], 5),
        "stance_width_default": round(float(feet_y), 5),
        "hip_pitch_height_default": round(float(hip_h), 5),
        "collision_box": collision_boxes(),
        "solve_residual_m": float(max(res, res2)),
        "default_pose_contract_shift_rad": float(pose_shift),
        "sole_tilt_deg_default": sole_level,
        "variant_mass_kg": {"full": total_mass(), "legs": total_mass()},
        "provisional_limits": common_limits(),
        "provisional_limits_note": PROVISIONAL_LIMITS_NOTE,
        "upper_body_limits_note": UPPER_BODY_NOTE,
        "upper_body_default_pos": {n: 0.0 for n in UPPER_BODY_JOINTS},
        "motor_id_by_joint": dict(sorted(MOTOR_IDS.items(), key=lambda kv: kv[1])),
        "robonex_common_name_by_joint": {n: common_name(n) for n in sorted(MOTOR_IDS, key=MOTOR_IDS.get)},
        "policy_joint_order": [next(n for n in LEG_JOINTS if common_name(n) == c) for c in POLICY_JOINT_ORDER],
        "leg_joints": list(LEG_JOINTS),
        "upper_body_joints": list(UPPER_BODY_JOINTS),
    }
    with open(CONSTANTS_PATH, "w") as f:
        json.dump(const, f, indent=2)
    print("solve iterations %d / %d, residual %.2e m, contract shift %.2e rad" % (it, it2, const["solve_residual_m"],
                                                                             pose_shift))
    for k in ("zero_pose_base_height", "mujoco_spawn_height", "home_base_height", "foot_origin_rest_height",
              "stance_width_default", "hip_pitch_height_default", "sole_tilt_deg_default", "foot_sole_corners",
              "variant_mass_kg"):
        print("%-26s %s" % (k, const[k]))
    print("wrote", CONSTANTS_PATH)


def loops_check(pose, scene, tol):
    lp = mt.Loops(path=scene)
    m, d = lp.model, lp.data
    mujoco.mj_resetData(m, d)
    if pose == "home":
        c = json.load(open(CONSTANTS_PATH))
        lp.set({k: v for k, v in {**c["default_actuated_pos"], **c["home_passive_pos"],
                                  **c["upper_body_default_pos"]}.items() if k in lp.qadr})
    mujoco.mj_forward(m, d)
    per_eq = {}
    for r in lp.residual_rows():
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_EQUALITY, d.efc_id[r]) or "eq%d" % d.efc_id[r]
        per_eq[name] = max(per_eq.get(name, 0.0), abs(float(d.efc_pos[r])))
    print("%s, pose %s: %d equality constraints" % (os.path.relpath(scene, ROOT), pose, len(per_eq)))
    for name, v in per_eq.items():
        print("   %-34s %.3e m" % (name, v))
    worst = max(per_eq.values()) if per_eq else 0.0
    print("max residual %.3e m (%s, tol %.0e)" % (worst, "closed" if worst < tol else "OPEN", tol))
    return 0 if worst < tol else 1


def check():
    links, joints = mt.load_urdf(URDF_PATH)
    types = {}
    for j in joints.values():
        types[j["type"]] = types.get(j["type"], 0) + 1
    total = sum(l.get("mass", 0.0) for l in links.values())
    print("%s: %d links, %d joints %s, total mass %.6f kg" % (os.path.relpath(URDF_PATH, REPO), len(links),
                                                             len(joints), dict(sorted(types.items())), total))
    bad, missing = [], []
    frames = [n for n, l in links.items() if mt.is_frame_link(l, joints)]
    for name, l in links.items():
        if name in frames:
            continue
        if "I" not in l:
            bad.append("%s: no inertial" % name)
            continue
        ev = np.linalg.eigvalsh(l["I"])
        if ev.min() <= 0 or ev[2] > ev[0] + ev[1] + 1e-12:
            bad.append("%s: eig %s" % (name, ev))
        for f in l["meshes"] + l["col"]:
            if f[0] and not os.path.exists(os.path.join(os.path.dirname(URDF_PATH), f[0])):
                missing.append(f[0])
    print("inertias SPD + triangle inequality: %s" % ("all %d ok" % (len(links) - len(frames)) if not bad else bad))
    print("frame-only links: %s" % (frames or "none"))
    print("missing mesh files: %s" % (sorted(set(missing)) or "none"))
    F = np.diag([1.0, -1.0, 1.0])
    worst_l = (0.0, "")
    for name in sorted(links):
        if not name.startswith("left_") or "I" not in links[name]:
            continue
        a, b = links[name], links[mirror(name)]
        v = max(abs(a["mass"] - b["mass"]), np.abs(F @ a["com"] - b["com"]).max(), np.abs(F @ a["I"] @ F - b["I"]).max())
        worst_l = max(worst_l, (float(v), name))
    worst_j = (0.0, "")
    for name in sorted(joints):
        if name.startswith("left_"):
            v = float(np.abs(F @ joints[name]["xyz"] - joints[mirror(name)]["xyz"]).max())
            worst_j = max(worst_j, (v, name))
    print("L/R mirror links (mass kg / com m / inertia), max residual: %.2e (%s)" % worst_l)
    print("L/R mirror joint origins, max residual: %.2e m (%s)" % worst_j)
    unknown = [n for n in MOTOR_IDS if n not in joints]
    print("motor joints missing from the URDF: %s" % (unknown or "none"))
    return 0 if not bad and not missing and not unknown else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("install")
    p.add_argument("--package", required=True)
    p = sub.add_parser("constants")
    p.add_argument("--seed-limits-only", action="store_true")
    p = sub.add_parser("loops")
    p.add_argument("--pose", choices=("zero", "home"), default="zero")
    p.add_argument("--scene", default=os.path.join(ROOT, "mujoco", "robot", "full", "scene.xml"))
    p.add_argument("--tol", type=float, default=1e-6)
    sub.add_parser("check")
    a = parser.parse_args()
    if a.command == "install":
        install(a.package)
    elif a.command == "constants":
        seed_limits() if a.seed_limits_only else constants()
    elif a.command == "loops":
        sys.exit(loops_check(a.pose, a.scene, a.tol))
    else:
        sys.exit(check())


if __name__ == "__main__":
    main()
