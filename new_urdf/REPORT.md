# RoboNex Ver.2 lower-body URDF: extraction report

Source: Fusion 360 `PolyGon / Ver.2 / 2_RoboNex_test v1`, read-only scripts through the Fusion MCP (2026-09-26).
Scope: lower body (25 links, 24 joints) + head (1 link, 1 joint) + arms (8 links, 8 joints): 34 links, 33 joints,
26.241 kg.

## Structure

```text
new_urdf/
├── REPORT.md
├── HANDOFF.md                  handoff notes for the integrating session
├── loop_closures.yaml          same structure as robonex-description
├── urdf/robonex.urdf
├── meshes/                     56 STL, mm, URDF axes, each in its link frame (visual origin 0)
└── _extract/
    ├── transforms_mm.json      occurrence local->world 4x4 (Fusion frame)
    ├── massprops.json          mass, COM, inertia about the world origin (Fusion frame)
    ├── geom/<part>.json        pivot bores and ball centres
    ├── stl_raw/                STL as exported by Fusion (component frame)
    ├── lower_model.py          joint points, axes, link groups, inertia
    ├── make_meshes.py          stl_raw -> meshes (axis unification)
    ├── make_urdf.py            -> urdf/robonex.urdf, loop_closures.yaml
    ├── derived.py              four-bar, ankle cranks, lengths, collision boxes
    ├── check_mujoco.py, render.py, render_zero_pose.png
```

Regenerate: `cd _extract && python make_meshes.py && python make_urdf.py`

## Conventions (same as Ver.1)

- Fusion frame of Ver.2 is forward = -Y, left = +X, up = +Z. Converted with `x_u = -Y`, `y_u = +X`, `z_u = Z`.
- `base_link` origin = Fusion origin. Zero pose = the assembly pose as-is. All joint `rpy` are 0, axes are ±X/±Y/±Z.
- Names are the Ver.1 URDF names (25 links, 24 joints, 37 mesh files). Ankle chain as Ver.1:
  left upper -> `crank_link_a` -> `coupler_joint_b`, right upper -> `crank_link_b` -> `coupler_joint_b`;
  `coupler_joint_a` is on the lower motor on both legs.
- Actuated joint point = motor output-flange centre on the axis. Passive joint point = bore centre of the child part.
  Rod-end joints = crank-side ball centre.
- Actuated axis sign = opposite of the motor output direction. This rule reproduces all 12 Ver.1 signs, and the
  Ver.2 motors give exactly the Ver.1 signs (no motor re-mounted). Passive axes copy Ver.1.
- Mass: Ver.1 grouping (motor inside the link its housing is bolted to). Inertia about the COM, URDF axes.
  The Fusion inertia pipeline reproduces the Ver.1 URDF values to 1e-8 on `00_RoboNex v9`.
- Meshes: STL in the link frame with visual origin 0 (Ver.1 used centred meshes with visual rpy). Collision = visual meshes.

## Checks

| check | result |
|---|---|
| motor axis vs crank bore | < 0.005 mm |
| loop closure residual at zero (4 balls, 2 knee pins) | <= 1e-6 m |
| rod lengths | 147.000 / 258.000 mm (Ver.1 147 / 258) |
| ankle crank radius | 50.0 mm (Ver.1 50) |
| MuJoCo URDF load | 34 bodies, 56 meshes, 26.24098 kg (lower body 21.375 + head 0.609 + arms 4.257, incl. 2 shoulder-pitch RS02 on base_link) |
| sole z at zero pose (lowest vertex) | L -950.11, R -949.34 mm below base origin |

## Ver.1 vs Ver.2 (left leg, mm)

| quantity | Ver.1 | Ver.2 |
|---|---:|---:|
| hip pitch -> knee joint | 374.0 | 346.7 |
| hip roll -> knee joint (thigh) | 340.0 | 312.7 |
| knee joint -> ankle roll | 326.5 | 266.7 |
| ankle roll -> ankle pitch | 50.0 | 50.0 |
| hip pitch -> ankle pitch | 750.5 | 663.5 |
| base origin -> sole | 1078.9 | 950.1 |
| ankle motors below knee joint (upper / lower) | 144.5 / 255.5 | 95.2 / 196.2 |
| knee four-bar crank / coupler / rocker / ground | 100 / 134 / 103 / 85 | 70.0 / 134.0 / 92.4 / 175.0 |
| knee ratio d(knee)/d(crank) at zero | 0.7037 | 1.000 |
| total mass (lower body) | 20.514 kg | 21.375 kg |

## Head (added after the lower body)

| item | value |
|---|---|
| joint | `neck_pitch_joint`, revolute, `base_link` -> `head_link`, axis `0 -1 0` (+ = head tilts back / looks up), ±pi |
| joint point | RS05 output face centre on the axis, (0.009861, 0.020159, 0.285242) m |
| effort / velocity | 5.5 N·m / 50.3 rad/s (official RS05 peak torque, 480 rpm no-load) |
| `head_link` | head shell `BK_033` + RealSense `BK_034`, one mesh `head_link.stl`, 0.332 kg |
| `base_link` additions | `rs05_neck.stl` (motor, 0.191 kg) and `neck_mount.stl` (`BK_032`, 0.086 kg) |

Mechanism, from the Fusion geometry: `BK_032` sleeves the RS05 housing (bore r 23.01) and is screwed to its back face
(holes 19.2 mm off-axis = housing screws); the head shell seats on the RS05 output face (motor local z 30.5) with
6 bolts on a 12 mm radius (r 3.5 counterbores) and 3 dowels on a 9.5 mm radius. So the motor housing and `BK_032`
belong to `base_link`, and the head shell and RealSense rotate about the lateral axis (pitch).

- RS05 mass: the Fusion body computes to 95.6 g although its material is named "RS05 191g". Mass and inertia are
  scaled to the official 191 g (uniform density).
- Axis sign uses the Ver.1 motor rule (axis = -output direction), verified only on RS02/RS03. Check the RS05
  positive direction on hardware.
- `neck_pitch_joint` is not in `loop_closures.yaml` `actuated_joints`: `robonex-common` has no gains or limits for it.

## Arms (Unitree G1 names and chain, RoboNex `l_`/`r_` prefix)

Source: Fusion copy `Ver.2/2_RoboNex_urdf` (made from `2_RoboNex_test v1` with `DataFile.copy`). Two as-built
revolute joints `l_elbow_joint` / `r_elbow_joint` (forearm <-> elbow RS02 output cylinder) were added and set to
+90 / -90 deg, position captured and saved. Check against the copy's baseline: only `BK_006` and `BK_007` moved,
each an exact 90 deg rotation about its elbow axis (rotation error < 4e-15, translation 0.0007 mm = axis rounding);
the other 55 parts are unchanged (< 1e-6 mm).

| joint (RS02) | parent -> child | axis L / R | housing in | output drives |
|---|---|---|---|---|
| `*_shoulder_pitch_joint` | `base_link` -> `*_shoulder_pitch_link` | `0 -1 0` / `0 1 0` | torso | `BK_000/001` shoulder |
| `*_shoulder_roll_joint` | `*_shoulder_pitch_link` -> `*_shoulder_roll_link` | `-1 0 0` / `-1 0 0` | `BK_000/001` | `BK_002/003` upper arm 1 (yoke, rear support) |
| `*_shoulder_yaw_joint` | `*_shoulder_roll_link` -> `*_shoulder_yaw_link` | `0 0 1` / `0 0 1` | `BK_002/003` | `BK_004/005` upper arm 2 |
| `*_elbow_joint` | `*_shoulder_yaw_link` -> `*_elbow_link` | `0 -1 0` / `0 1 0` | `BK_004/005` | `BK_006/007` forearm |

- Housing/output found from the Fusion geometry: housing sleeve (r 39.25-45) on the upper part, 12 mm bolt circle
  on the output face (motor local z >= 22.7) of the lower part.
- Meshes: `l/r_shoulder_pitch_link`, `..._roll_link`, `..._yaw_link`, `l/r_elbow_link` (G1 style, one per link) and
  the motors `rs02_l/r_shoulder_pitch`, `..._shoulder_roll`, `..._shoulder_yaw`, `rs02_l/r_elbow` (RoboNex style).
- Zero pose: arms hanging, elbows bent 90 deg forward (as G1). Limits ±pi, effort 17, velocity 42.9 (RS02).
- As-is asymmetry: the right arm sits about 0.5 mm further inboard than the mirror of the left.
- Arm joints are not in `robonex-common` and not in `loop_closures.yaml` `actuated_joints`.

## Open items (not resolved here)

1. As-is asymmetry of the assembly, kept in the URDF:
   - left foot tilted about 1.0 deg pitch (toe up) and 0.3 deg roll; right foot about level;
   - ankle crank ball angles above horizontal: L upper 12.78, L lower 1.85, R upper 11.90, R lower 0.35 deg
     (rod length difference 111 mm vs motor spacing 101 mm);
   - hip yaw axes at y +43.073 / -42.927 mm (0.146 mm offset).
   Decide the Ver.2 zero pose (= RobStride Set Zero reference) before the hardware sweep.
2. Joint limits: every revolute joint is -pi..+pi (actuated keep effort/velocity, passive 0/0) until the Ver.2
   hardware sweep. The knee four-bar toggles at crank 20 deg toward extension (Ver.1 +54 deg is unreachable), so
   closed-loop MuJoCo/Isaac runs can drive the linkage through its toggle or the rod-end ±15 deg limit: view only,
   no training or random-action tests until the sweep limits are in.
3. Not yet run (needs Ubuntu + robonex-common): `build_mjcf.py`, `home_pose.py`, ankle workspace and rod-end
   ±15 deg sweep over the crank range, Isaac closed-loop conversion.
4. Length-coupled constants to update later (`length-coupled-constants.md`): spawn height ~0.9501, home height and
   passive angles from `home_pose.py`, `COLLISION_BOX` (values printed by `derived.py`), walking contract
   (base height, foot rest height, sole corners, stance width), gait period.
5. Arm and neck joints need CAN IDs, motor directions, gains and limits before they can be driven; the policy
   contract (12 joints) is unchanged.
