# RoboNex USD

## Build

```bash
# 1. URDF -> serial-equivalent URDF
python3 isaac/build_isaac_urdf.py

# 2. URDF -> USD (free-floating base)
conda activate isaacsim
~/IsaacLab/isaaclab.sh -p ~/IsaacLab/scripts/tools/convert_urdf.py \
  $PWD/isaac/robonex_serial.urdf \
  $PWD/isaac/robonex.usd \
  --joint-stiffness 40.0 \
  --joint-damping 2.0 \
  --headless

# 2b. URDF -> USD (base welded in the air)
~/IsaacLab/isaaclab.sh -p ~/IsaacLab/scripts/tools/convert_urdf.py \
  $PWD/isaac/robonex_serial.urdf \
  $PWD/isaac/robonex_fixed.usd \
  --joint-stiffness 40.0 \
  --joint-damping 2.0 \
  --fix-base \
  --headless

# 3. Fix up the knee mimic constraints IsaacLab's converter silently drops
#    (see apply_knee_mimic.py's docstring for why) - run on whichever USD(s)
#    you just built
~/IsaacLab/isaaclab.sh -p isaac/apply_knee_mimic.py isaac/robonex.usd --headless
~/IsaacLab/isaaclab.sh -p isaac/apply_knee_mimic.py isaac/robonex_fixed.usd --headless
```

## Run

```bash
cd ~/humanoid_project/robonex_description

~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py
~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --fixed-base
~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --headless
```

Press **▶ Play** in the viewport toolbar to start physics - nothing moves until then.

## Notes

- **Knee crank** tracks the knee joint via a PhysX Mimic Joint (gearing −0.705,
  the measured ratio at the home pose). Cosmetic + adds the crank's real
  inertia to the knee's dynamics; not a substitute for the true nonlinear
  ratio (0.21..1.23 across travel).
- **Ankle crank** (2 motors → roll+pitch) has no equivalent fix - a mimic
  joint only takes one reference joint, and the ankle coupling is 2-in/2-out.
  Left frozen. Only matters if the control interface ever becomes motor-space
  end to end; with a task-space policy + conversion layer (the plan), it's
  purely cosmetic and not worth the effort.
