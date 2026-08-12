# RoboNex Isaac USD models

## Build

This variant restores both planar knee four-bars as well as the ankle loops.
Each knee motor crank is an actuated revolute joint.  The knee output and
coupler joints are passive, and a revolute pin constraint closes each coupler
back onto its shin link.  The generated assets are isolated under
`isaac/closed_loop/`.

```bash
cd ~/humanoid_project/robonex_description
python3 isaac/build_isaac_urdf.py
mkdir -p isaac/closed_loop
conda activate isaacsim

# Free-floating
~/IsaacLab/isaaclab.sh -p ~/IsaacLab/scripts/tools/convert_urdf.py \
  $PWD/isaac/closed_loop/robonex_closed_loop.urdf \
  $PWD/isaac/closed_loop/robonex_closed_loop.usd \
  --joint-stiffness 40.0 --joint-damping 2.0 --headless
~/IsaacLab/isaaclab.sh -p isaac/scripts/apply_physical_loops.py \
  isaac/closed_loop/robonex_closed_loop.usd --headless

# Fixed-base mechanism test
~/IsaacLab/isaaclab.sh -p ~/IsaacLab/scripts/tools/convert_urdf.py \
  $PWD/isaac/closed_loop/robonex_closed_loop.urdf \
  $PWD/isaac/closed_loop/robonex_closed_loop_fixed.usd \
  --joint-stiffness 40.0 --joint-damping 2.0 --fix-base --headless
~/IsaacLab/isaaclab.sh -p isaac/scripts/apply_physical_loops.py \
  isaac/closed_loop/robonex_closed_loop_fixed.usd --headless
```

Only `l_knee_pitch_joint` and `r_knee_pitch_joint` are knee drive inputs.
`l/r_knee_joint` and `l/r_knee_coupler_joint_a` are passive mechanism joints;
do not give them drive targets or edit their Joint State positions.  For a
manual check, use the fixed-base command below, press **Play**, and edit the
motor joint's **Angular > Target Position**.  Begin around 0.25 degrees and
change the target gradually.  TGS/240 Hz/64/4 solver settings are applied
automatically.  The ankle drive inputs are `l/r_ankle_upper_joint` and
`l/r_ankle_lower_joint`; ankle roll/pitch and all spherical joints are passive.
Do not edit Joint State positions on any passive or closure joint because that
teleports a constraint-connected body and can destabilize the mechanism.

### Simplified tree model

This variant deliberately removes every knee and ankle loop.  It directly
actuates the joints that the mechanisms drive (`knee`, `ankle_roll`, and
`ankle_pitch`) while fixing the crank and coupler joints.  Therefore it is a
stable 12-DoF tree model for comparison or fast training; it is not a
motor-space replica of the hardware mechanism.

```bash
cd ~/humanoid_project/robonex_description
python3 isaac/build_isaac_urdf.py --model simplified
conda activate isaacsim

# Free-floating
~/IsaacLab/isaaclab.sh -p ~/IsaacLab/scripts/tools/convert_urdf.py \
  $PWD/isaac/simplified/robonex_simplified.urdf \
  $PWD/isaac/simplified/robonex_simplified.usd \
  --joint-stiffness 40.0 --joint-damping 2.0 --headless

# Fixed-base mechanism/pose test
~/IsaacLab/isaaclab.sh -p ~/IsaacLab/scripts/tools/convert_urdf.py \
  $PWD/isaac/simplified/robonex_simplified.urdf \
  $PWD/isaac/simplified/robonex_simplified_fixed.usd \
  --joint-stiffness 40.0 --joint-damping 2.0 --fix-base --headless
```

## Run

```bash
cd ~/humanoid_project/robonex_description

~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py
~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --fixed-base
~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --model simplified
~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --model simplified --fixed-base
~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --headless
```

Press **▶ Play** in the viewport toolbar to start physics - nothing moves until then.

## Notes

- Both physical planar knee four-bars use excluded-from-articulation revolute
  closure constraints.  The ankle 2-motor→roll/pitch mechanism uses physical
  spherical joints and excluded-from-articulation closure constraints.
- The simplified model preserves link visuals, collisions, mass, and the
  12 output-space control joints, but it does not preserve the closed-loop
  mechanism's configuration-dependent transmission or its internal dynamics.
- The spherical joint frames use their X axis as the twist axis.  This is
  geometrically equivalent to the original Z-axis frames and keeps the asset
  compatible with Isaac Lab's articulation tensor interface.
