# RoboNex Ver.2 model report

Verification, calculations and decisions for the Ver.2 model, written as the work proceeds (autonomous session
robonex-walking-2f, from 2026-09-26 03:23 KST). Input: `new_urdf/` from the Windows Fusion 360 extraction
(`HANDOFF.md`, `REPORT.md`, `urdf/robonex.urdf`, `meshes/`, `loop_closures.yaml`, `_extract/`). The Ver.1 files in
`robonex-description` stay untouched until the user deletes them.

Scripts for every check below are in `new_urdf/verify/`.

## 1. Structure (`verify/urdf_io.py`, inline check)

| check | result |
|---|---|
| Ver.1 link and joint names (25 / 24) | all present, same parent/child, same type |
| Ver.1 axis signs (24 joints) | all identical |
| joint `rpy` | all `0 0 0` |
| axes | all unit ±X/±Y/±Z |
| new | 9 links (`head_link`, 4 per arm), 9 joints (`neck_pitch_joint`, `l/r_shoulder_pitch/roll/yaw_joint`, `l/r_elbow_joint`) |
| root | `base_link` only |
| visual = collision mesh list, visual origin 0, scale 0.001 | yes for all links |
| joint limits | every revolute ±π (user decision: re-measure on hardware) |

## 2. Mass (`RoboNex_부품별_질량표.docx` vs extraction)

All 57 parts match the user's mass table within 0.05 g (arm parts: 어깨 부품2 101.40 = `shoulder_pitch_link`,
이두 부품1 119.28 = `shoulder_roll_link`, 이두 부품2 146.37 = `shoulder_yaw_link`, 전환근 141.57 = `elbow_link`).
RS05 is 191 g (official; the table's 95.55 g is the Fusion body-volume error the table itself flags). RS02 is 405 g
in the table (official spec 380 ± 3 g) and is kept as the user's measured value. Every URDF link mass equals the sum
of its parts. Total 26.241 kg = lower body 21.375 + arms 4.257 + head group 0.609.

## 3. Inertia and COM, independent check (`verify/mesh_inertia.py`)

Each link's COM and inertia tensor recomputed by volume integration of its STL meshes (uniform density per part,
part masses from the table) and compared with the URDF: **COM within 0.03 mm, tensor within 0.1 %, no
product-of-inertia sign flip on any link** (34 links). This confirms the tensor sign convention, the link-frame
placement of the meshes and the mass grouping independently of the Fusion export. Every link tensor is positive
definite and satisfies the triangle inequality.

## 4. Left/right symmetry (`verify`, inline)

Mirror pairs compared (ankle cranks paired by function: left `crank_link_a` ↔ right `crank_link_b`).

| item | left vs mirrored right | cause |
|---|---|---|
| hip yaw joint y | 0.146 mm | CAD placement |
| shoulder pitch joint y | 0.518 mm (right arm inboard) | CAD placement |
| ankle motor axes | 0.10 mm | CAD placement |
| ankle rod-end points on the cranks | 1.30 / 0.76 mm | **left cranks rotated +1.50° / +0.88° more than the mirror of the right** |
| foot COM | 0.82 mm, tensor 1.5 % | left foot tilted |
| every other link and joint | < 0.05 mm, < 1 % | — |

All Fusion occurrence rotations are identity: the parts are modelled in place, so the pose asymmetry lives in the
geometry itself.

Sole planes measured on the foot meshes (plane fit over the lowest down-facing faces):

| foot | tilt about y (+ = toe up) | tilt about x |
|---|---|---|
| left | +0.91° | −0.31° |
| right | −0.18° | 0.00° |

## 5. Decision: zero pose = both soles exactly level

The RobStride Set Zero reference must be a pose a person can reproduce on the real robot: legs straight, both feet
flat on the floor. The Fusion as-is pose is not that (left foot +0.9° toe-up with its cranks 0.9–1.5° off the
mirror of the right). The Ver.2 URDF zero is therefore re-posed: for each leg the ankle roll/pitch passive angles
that make the sole plane exactly horizontal are applied, the two ankle cranks are rotated to close their rod loops
at that foot pose, the rods are re-oriented between their new ball centres, and the resulting geometry (meshes, COM,
inertia, child joint points, loop points) is baked in with every frame still parallel to `base_link`. The as-is
extraction is kept unchanged in `asis/` for reference.

Result of the re-pose (`build/repose.py`, `build/make_ver2.py`):

| leg | ankle roll | ankle pitch | lower crank | upper crank |
|---|---|---|---|---|
| left | +0.317° | −0.982° | +1.291° | −0.685° |
| right | +0.000° | +0.203° | +0.202° | −0.204° |

Afterwards both soles are level within 0.0005°, and the crank-side ball angles above horizontal are left/right
0.56°/0.555° (lower) and 12.13°/12.11° (upper) — the mirror asymmetry of 0.9–1.5° disappears, which confirms that it
was assembly state, not design. Loop closure residual at the new zero is 0 for all six loops. The left ankle link
itself was rolled ≈0.37° in the as-is assembly (its bore z differs by 0.19 mm over 30 mm), so correcting at the roll
joint is right. Remaining left/right differences, all CAD placement and all ≤ 0.6 mm: hip yaw axis 0.146 mm,
shoulder pitch 0.518 mm, ankle motor axes 0.10 mm, rod-end position along the bolt 0.598 mm, and the left foot frame
origin 0.276 mm along the pitch axis (a choice of origin on the axis, kinematically irrelevant). Mesh inertia check
repeated on the re-posed model: unchanged (COM ≤ 0.03 mm, tensor ≤ 0.1 %, no sign flip).

## 6. Files and variants

| file | content |
|---|---|
| `asis/` | the Windows extraction untouched: `urdf/robonex.urdf` (34 links), `meshes/`, `loop_closures.yaml` |
| `urdf/robonex_edu.urdf` | lower body: 25 links, 24 joints, 21.375 kg |
| `urdf/robonex_pro.urdf` | + arms: 33 links, 32 joints, 25.632 kg |
| `urdf/robonex_max.urdf` | + neck motor, neck mount, head: 34 links, 33 joints, 26.241 kg |
| `meshes/` | re-posed set (12 ankle/foot meshes rewritten, the rest identical to `asis/meshes`) |
| `loop_closures.yaml` | re-posed loop points, shared by all variants (12 actuated leg joints) |
| `build/` | `repose.py`, `make_ver2.py` (URDFs, meshes, loops), `solve_constants.py` (→ `scripts/ver2_constants.json`), `stl_io.py` |
| `scripts/`, `mujoco/`, `isaac/` | copies of the `robonex-description` builders adapted for Ver.2 (`--variant edu|pro|max`) |

`base_link` differs per variant: edu = torso + 2 hip-yaw RS02; pro adds the 2 shoulder-pitch RS02; max adds RS05
and the neck mount. The max `base_link` recomputed from parts equals the extraction's to 6e-8 kg / 3e-7 m. Arm and
neck joints are **held rigid** at their zero pose in every simulator model (no CAN ID, gains or limits exist; the
arm zero pose is elbows bent 90° forward).

## 7. Knee four-bar (`verify/loops.py`, MuJoCo loop solve; cross-checked with a planar solution)

The same solver reproduces the documented Ver.1 value (crank −50° → knee −42.21°).

| crank (left sign) | Ver.1 knee | Ver.2 knee | Ver.2 d(knee)/d(crank) |
|---|---|---|---|
| +19° | +12.2° | +25.9° | 2.58 (linkage limit at +19.x°) |
| 0° | 0 | 0 | ≈1.00 |
| −20° | — | −17.9° | 0.81 |
| −30° | −23.7° | −25.6° | 0.73 |
| −50° | −42.2° | −38.6° | 0.55 |
| −70° | — | −47.1° | 0.27 |
| −83° | — | **−48.97° (maximum flexion)** | 0 |
| −86° | −81.5° | −48.9° (reversing) | < 0 |

**Finding: the Ver.2 knee flexes at most 48.97°** (crank −83°), where the transmission ratio reaches zero; beyond
that the knee extends again. Ver.1 reached −81.5° at its −86° crank limit and was monotonic. The Ver.1 walking policy
in simulation used knee flexion down to −46.2° (1st percentile −43.0°, from `R_1000_255_4` traces of W65), so Ver.2
covers normal walking with almost no margin, and near full flexion the knee can barely move (ratio < 0.3 below
≈ −45°). The crank must be limited to about [−80°, +15°] (monotonic region, clear of the +19° linkage limit) once
limits are set; the hardware sweep decides the final values.

## 8. Ankle differential (`verify/ankle_sweep.py`, crank grid 2°, ±60°)

| within crank box ±34° | Ver.1 | Ver.2 |
|---|---|---|
| grid points with rod-end tilt ≤ 15° | 1143 / 1225 | 1142 / 1225 |
| Jacobian condition number, median / p90 / max | 1.25 / 1.49 / 1.89 | 1.30 / 1.59 / 1.96 |
| minimum singular value (foot deg per crank deg) | 0.499 | 0.44 |
| same-sign corners (−34,−34) / (+34,+34): rod-end tilt | 18.2° / 23.4° | 18.6° / 22.8° |

The ankle behaves like Ver.1 (same rods 147/258 mm and 50 mm cranks; motor spacing 101 vs 111 mm). The same-sign
corner still exceeds the ±15° rod-end limit (open item M3 carries over). Caveat: a rod between two balls can spin
about its own axis; the reported tilt is for the spin the continuation solve lands on, not the minimum over spin,
so it can overstate the tilt (same method for both versions).

## 9. Default pose and constants (`build/solve_constants.py` → `scripts/ver2_constants.json`)

Default pose = Ver.1 output angles: hip pitch 0.1 rad, knee −0.29670 rad, ankle pitch +0.19670 rad (sole exactly
level), roll 0; the cranks are solved through the loops (residual 2e-16 m).

| constant | Ver.1 | Ver.2 |
|---|---|---|
| default knee crank (left) | −0.38578 (−22.1°) | −0.32987 (−18.90°) |
| default ankle cranks (left upper / lower) | +0.20566 / −0.20566 | +0.21250 / −0.20387 |
| default ankle cranks (right lower / upper) | +0.20566 / −0.20566 | +0.20374 / −0.21262 |
| base height, default pose (sole on ground) | 1.0710 | 0.94226 |
| base height, zero pose | 1.0789 | 0.94885 |
| MuJoCo free spawn | 1.085 | 0.9549 |
| foot origin rest height | 0.06545 | 0.0654 |
| sole corners (foot frame) | x −0.0623..0.1505, y −0.0445..0.0845 | x −0.0591..0.1441, y −0.0591..0.0588, z −0.0654 |
| foot distance at default (stance width) | 0.321 | **0.269** |
| hip-pitch height at default | 0.808 | 0.722 |
| total mass | 20.514 | 21.375 / 25.632 / 26.241 |

Sole corners are never optimistic against the full foot mesh over pitch −16..20° and roll ±10° (at most 1.62 mm
conservative). The left/right default cranks differ by ≤ 0.001 rad (the CAD placement asymmetries above).

## 10. MuJoCo sanity (free base, home keyframe, Ver.1 gains, 5 s)

Self-contacts at zero and home pose: none (all variants). Standing on PD alone:

| variant | ankle kp 40 (Ver.1 gain) | ankle kp 80 | ankle kp 160 |
|---|---|---|---|
| edu | stands, leans 4.6°, drifts 66 mm | 0.9°, 16 mm | 0.1°, 2 mm |
| pro | falls after ≈1.5 s | stands, 31 mm | stands, 4 mm |
| max | falls after ≈1.5 s | stands, 30 mm | stands, 4 mm |

The COM is mid-support in every variant (≈ 100 mm to toe and heel). Falling at kp 40 is the passive
inverted-pendulum limit, not a model error: ankle stiffness ≈ 160 N·m/rad (two cranks per foot) against
m·g·h ≈ 130 (edu), 173 (pro), 180 (max) N·m/rad. The trained policy balances actively; the ankle gain is a
candidate to revisit if the heavier variants train badly.
