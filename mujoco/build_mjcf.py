#!/usr/bin/env python3
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from robonex_common import (
    load_urdf, load_loops, children_of, rpy_to_quat, fmt, ROOT,
)

OUT = os.path.join(ROOT, "mujoco", "robonex.xml")
SCENE_OUT = os.path.join(ROOT, "mujoco", "scene.xml")

NEIGHBOUR_DEPTH = 2

TIMESTEP = 0.001
SOLREF = "%g 1" % (4 * TIMESTEP)
SOLIMP = "0.99 0.9999 0.0001"

SPAWN_HEIGHT = 1.085
FIXED_BASE_HEIGHT = 1.60
LEG_DROP = 1.0789


def emit_body(links, joints, kids, name, ball_set, actuated, depth, out,
              fixed_base=False, height=SPAWN_HEIGHT, ball_limit=None):
    pad = "  " * depth
    lk = links[name]
    parent_joint = None
    for j in joints.values():
        if j.child == name:
            parent_joint = j
            break

    if parent_joint is None:
        out.append('%s<body name="%s" pos="0 0 %g">' % (pad, name, height))
        if not fixed_base:
            out.append('%s  <freejoint name="root"/>' % pad)
    else:
        q = rpy_to_quat(parent_joint.rpy)
        attrs = 'pos="%s"' % fmt(parent_joint.xyz)
        if abs(q[0] - 1.0) > 1e-9:
            attrs += ' quat="%s"' % fmt(q)
        out.append('%s<body name="%s" %s>' % (pad, name, attrs))

        if parent_joint.name in ball_set:
            rng = ""
            if ball_limit is not None:
                rng = (' range="%s" solreflimit="%s" solimplimit="%s"'
                       % (fmt((-ball_limit, ball_limit)), SOLREF, SOLIMP))
            for suffix, axis in (("rx", "1 0 0"), ("ry", "0 1 0")):
                out.append('%s  <joint name="%s_%s" type="hinge" axis="%s"%s class="passive"/>'
                           % (pad, parent_joint.name, suffix, axis, rng))
        elif parent_joint.jtype in ("revolute", "continuous"):
            is_act = parent_joint.name in actuated
            cls = "act" if is_act else "passive"
            extra = ""
            if is_act:
                arm = 0.017 if parent_joint.effort > 30.0 else 0.003
                extra = ' armature="%g"' % arm
            out.append(
                '%s  <joint name="%s" type="hinge" axis="%s" range="%s"%s class="%s"/>'
                % (pad, parent_joint.name, fmt(parent_joint.axis),
                   fmt((parent_joint.lower, parent_joint.upper)), extra, cls)
            )

    ixx, ixy, ixz, iyy, iyz, izz = lk.inertia
    if lk.mass > 0.0:
        out.append(
            '%s  <inertial pos="%s" mass="%.6g" fullinertia="%s"/>'
            % (pad, fmt(lk.com), lk.mass,
               fmt((ixx, iyy, izz, ixy, ixz, iyz), 8))
        )

    for g in lk.visuals:
        mesh = os.path.splitext(g.mesh)[0]
        q = rpy_to_quat(g.rpy)
        attrs = 'type="mesh" mesh="%s" pos="%s"' % (mesh, fmt(g.xyz))
        if abs(q[0] - 1.0) > 1e-9:
            attrs += ' quat="%s"' % fmt(q)
        out.append('%s  <geom %s class="visual"/>' % (pad, attrs))
        out.append('%s  <geom %s class="collision"/>' % (pad, attrs))

    for j in kids.get(name, []):
        emit_body(links, joints, kids, j.child, ball_set, actuated, depth + 1, out,
                  fixed_base, height, ball_limit)

    out.append("%s</body>" % pad)


def build_excludes(links, joints, loops, depth=NEIGHBOUR_DEPTH):
    adj = {n: set() for n in links}
    for j in joints.values():
        adj[j.parent].add(j.child)
        adj[j.child].add(j.parent)
    for key in ("pin_loops", "ball_loops"):
        for e in loops.get(key, []):
            adj[e["parent"]].add(e["child"])
            adj[e["child"]].add(e["parent"])

    pairs = set()
    for start in links:
        frontier, seen = {start}, {start}
        for _ in range(depth):
            nxt = set()
            for n in frontier:
                nxt |= adj[n] - seen
            seen |= nxt
            frontier = nxt
        for other in seen:
            if other != start:
                pairs.add(tuple(sorted((start, other))))
    return sorted(pairs)


def write_scene():
    s = [
        '<?xml version="1.0"?>',
        '<mujoco model="robonex scene">',
        '  <include file="robonex.xml"/>',
        '',
        '  <statistic center="0 0 0.6" extent="1.2"/>',
        '',
        '  <visual>',
        '    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>',
        '    <rgba haze="0.15 0.25 0.35 1"/>',
        '    <global azimuth="140" elevation="-20"/>',
        '  </visual>',
        '',
        '  <asset>',
        '    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"'
        ' width="512" height="3072"/>',
        '    <texture type="2d" name="groundplane" builtin="checker" mark="edge"'
        ' rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"',
        '      markrgb="0.8 0.8 0.8" width="300" height="300"/>',
        '    <material name="groundplane" texture="groundplane" texuniform="true"'
        ' texrepeat="5 5" reflectance="0.2"/>',
        '  </asset>',
        '',
        '  <worldbody>',
        '    <light pos="0 0 3" dir="0 0 -1" directional="true"/>',
        '    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"'
        ' condim="3" contype="1" conaffinity="1"/>',
        '  </worldbody>',
        '</mujoco>',
    ]
    with open(SCENE_OUT, "w", encoding="utf-8") as f:
        f.write("\n".join(s) + "\n")


def append_home_keyframe():
    # Used to settle N seconds of physics and save wherever that landed. That
    # assumed a position-hold equilibrium worth capturing precisely - true
    # while the servo gains were tuning parameters (kp=400), false now that
    # they are the real RobStride bench values (kp=40): nothing holds this
    # robot standing under its own control, so "settle and see where it ends
    # up" reliably ends up on the floor, at some arbitrary point mid-fall.
    #
    # So "home" is qpos0 verbatim: the spawn pose, feet 6 mm above the floor,
    # zero velocity - exactly what the viewer's own Reset/Reload already give
    # you. No physics dependency, nothing to fall over during the build.
    try:
        import mujoco
    except ImportError:
        return "skipped (mujoco not installed)"

    model = mujoco.MjModel.from_xml_path(SCENE_OUT)

    qpos = " ".join("%.6g" % v for v in model.qpos0)
    block = [
        "",
        "  <keyframe>",
        '    <key name="home" qpos="%s" ctrl="%s"/>'
        % (qpos, " ".join("0" for _ in range(model.nu))),
        "  </keyframe>",
    ]

    with open(OUT, encoding="utf-8") as f:
        text = f.read()
    text = text.replace("</mujoco>", "\n".join(block) + "\n</mujoco>")
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(text)
    return "home, %d qpos values = qpos0 (spawn pose, unsettled)" % model.nq


def main():
    fixed_base = "--fixed-base" in sys.argv
    height = FIXED_BASE_HEIGHT if fixed_base else SPAWN_HEIGHT

    links, joints, base = load_urdf()
    loops = load_loops()
    kids = children_of(joints)
    ball_set = set(loops.get("ball_upgrades", []))
    actuated = list(loops.get("actuated_joints", []))
    ball_limit_deg = loops.get("ball_limit_deg")
    ball_limit = math.radians(ball_limit_deg) if ball_limit_deg else None

    meshes = sorted({g.mesh for lk in links.values() for g in lk.visuals})

    out = []
    out.append('<?xml version="1.0"?>')
    out.append('<mujoco model="robonex">')
    out.append('  <compiler angle="radian" meshdir="../meshes" autolimits="true"/>')
    out.append('  <option timestep="%g" integrator="implicitfast" cone="elliptic"/>'
               % TIMESTEP)
    out.append("")
    out.append("  <default>")
    out.append('    <joint damping="0.01" armature="0"/>')
    out.append('    <default class="passive">')
    out.append('      <joint damping="0.01" armature="0"/>')
    out.append("    </default>")
    out.append('    <default class="act">')
    out.append('      <joint damping="0.2"/>')
    out.append("    </default>")
    out.append('    <default class="visual">')
    out.append('      <geom group="2" contype="0" conaffinity="0" density="0"/>')
    out.append("    </default>")
    out.append('    <default class="collision">')
    out.append('      <geom group="3" contype="1" conaffinity="1" condim="3" density="0" rgba="0.6 0.6 0.6 0.4"/>')
    out.append("    </default>")
    out.append('    <default class="motor">')
    out.append('      <position kp="40" kv="2"/>')
    out.append("    </default>")
    out.append("  </default>")
    out.append("")
    out.append("  <asset>")
    for m in meshes:
        out.append('    <mesh name="%s" file="%s" scale="0.001 0.001 0.001"/>'
                   % (os.path.splitext(m)[0], m))
    out.append("  </asset>")
    out.append("")
    out.append("  <worldbody>")
    emit_body(links, joints, kids, base, ball_set, set(actuated), 2, out,
              fixed_base, height, ball_limit)
    out.append("  </worldbody>")
    out.append("")

    out.append("  <equality>")
    for p in loops.get("pin_loops", []):
        out.append(
            '    <connect name="%s" body1="%s" body2="%s" anchor="%s"'
            ' solref="%s" solimp="%s"/>'
            % (p["name"], p["parent"], p["child"], fmt(p["parent_xyz"]),
               SOLREF, SOLIMP)
        )
    for b in loops.get("ball_loops", []):
        out.append(
            '    <connect name="%s" body1="%s" body2="%s" anchor="%s"'
            ' solref="%s" solimp="%s"/>'
            % (b["name"], b["parent"], b["child"], fmt(b["parent_xyz"]),
               SOLREF, SOLIMP)
        )
    out.append("  </equality>")
    out.append("")

    excludes = build_excludes(links, joints, loops)
    out.append("  <contact>")
    for a, b in excludes:
        out.append('    <exclude body1="%s" body2="%s"/>' % (a, b))
    out.append("  </contact>")
    out.append("")

    out.append("  <actuator>")
    for name in actuated:
        j = joints[name]
        out.append('    <position name="%s" joint="%s" ctrlrange="%s"'
                   ' forcerange="%s" class="motor"/>'
                   % (name.replace("_joint", ""), name, fmt((j.lower, j.upper)),
                      fmt((-j.effort, j.effort))))
    out.append("  </actuator>")
    out.append("")

    out.append("  <sensor>")
    for name in actuated:
        out.append('    <actuatorfrc name="%s_trq" actuator="%s"/>'
                   % (name.replace("_joint", ""), name.replace("_joint", "")))
    out.append("  </sensor>")
    out.append("</mujoco>")

    with open(OUT, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")
    write_scene()

    kf = ("skipped (welded base has nothing to settle onto)" if fixed_base
          else append_home_keyframe())

    print("wrote %s" % OUT)
    print("wrote %s" % SCENE_OUT)
    print("  bodies      : %d" % len(links))
    print("  hinges      : %d" % sum(
        1 for j in joints.values()
        if j.jtype in ("revolute", "continuous") and j.name not in ball_set))
    print("  ball joints : %d" % len(ball_set))
    print("  equalities  : %d" % (len(loops.get("pin_loops", [])) + len(loops.get("ball_loops", []))))
    print("  actuators   : %d" % len(actuated))
    print("  sensors     : %d torque" % len(actuated))
    print("  keyframe    : %s" % kf)
    for lim in sorted({(joints[n].effort, joints[n].velocity) for n in actuated}):
        n_of = sum(1 for n in actuated
                   if (joints[n].effort, joints[n].velocity) == lim)
        print("     %2d joints at %.0f Nm / %.1f rad/s" % (n_of, lim[0], lim[1]))
    print("  self-collision: ON, %d neighbour pairs excluded" % len(excludes))
    if ball_limit_deg:
        print("  rod-end swing limit: +/-%.1f deg on %d universal joints"
              % (ball_limit_deg, len(ball_set)))
    print("  base        : %s at z = %.3f m" %
          ("WELDED" if fixed_base else "free-floating", height))
    print("  foot clearance: %.3f m" % (height - LEG_DROP))
    print()
    print("  NOTE: the viewer's Joint panel writes qpos directly and bypasses the")
    print("  equality constraints, so dragging it WILL pull the linkages apart.")
    print("  Use the Control panel with the simulation running instead.")


if __name__ == "__main__":
    main()
