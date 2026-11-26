# ver3

## Models
| Folder | Description | README |
| --- | --- | :---: |
| `meshes` | 67 link and actuator STL meshes from the Fusion package (zero pose: legs straight with level soles, upper arms down, elbows 90° with the forearms forward, head forward) | - |
| `urdf` | Source URDF `robonex_ver3` (one full-body model, 29.711 kg; G1-format names; 24 motor joints: legs 12, arms 10, neck 2; `d435_link` camera frame; all limits ±π, training limits come from `ver3_constants.json`) | - |
| `mujoco` | Closed-loop MJCF models and fixed/free-base scenes, variants `full` (24 actuators) and `legs` (arms and neck fixed at 0 rad) | [📖](mujoco/) |
| `isaac` | Closed-loop URDF/USD models for Isaac Sim, variants `full` and `legs` | [📖](isaac/) |

<br>

## Build

### `tools/ver3_model.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| `install` | `--package` | `Required` | Windows package folder (`ver3.urdf`, `loop_closures.yaml`, `meshes/`); writes `urdf/robonex_ver3.urdf`, `loop_closures.yaml`, `meshes/` |
| `constants` | `--seed-limits-only` | Off | Write only `provisional_limits` (first build, before `scene_fixed.xml` exists) |
| `loops` | `--pose` | `zero` | Pose to check (`zero`, `home`) |
| - | `--scene` | `mujoco/robot/full/scene.xml` | MuJoCo scene to check |
| - | `--tol` | `1e-06` | Residual tolerance in m (exit 1 above it) |
| `check` | - | - | Counts, total mass, SPD inertias, L/R mirror, motor joints present |

Joint values from robonex-common are looked up by motor ID (`scripts/robonex_data_ver3.py`): legs 1–12, arms 16–25,
neck yaw 26, neck pitch 27.

```bash
# Example
cd ~/humanoid_project/robonex-description

.venv/bin/python -I ver3/tools/ver3_model.py install --package "$HOME/Downloads/new5_urdf"
.venv/bin/python -I ver3/tools/ver3_model.py constants --seed-limits-only
.venv/bin/python -I ver3/mujoco/build_mjcf.py --variant full --fixed-base
.venv/bin/python -I ver3/tools/ver3_model.py constants
.venv/bin/python -I ver3/mujoco/build_mjcf.py --variant full
.venv/bin/python -I ver3/mujoco/build_mjcf.py --variant legs
.venv/bin/python -I ver3/mujoco/build_mjcf.py --variant legs --fixed-base
.venv/bin/python -I ver3/isaac/build_isaac_urdf.py --variant full --collision mesh
.venv/bin/python -I ver3/isaac/build_isaac_urdf.py --variant full --collision box
.venv/bin/python -I ver3/isaac/build_isaac_urdf.py --variant legs --collision mesh
.venv/bin/python -I ver3/isaac/build_isaac_urdf.py --variant legs --collision box

# checks
.venv/bin/python -I ver3/tools/ver3_model.py loops --pose home
.venv/bin/python -I ver3/tools/ver3_model.py check
```

| Output | Description |
| --- | --- |
| `meshes/`, `loop_closures.yaml`, `urdf/robonex_ver3.urdf` | `install` |
| `ver3_constants.json` | `constants` |
