# ver2-2

## Models
| Folder | Description | README |
| --- | --- | :---: |
| `meshes` | Link STL meshes (level-sole zero pose) with `battery_link` and `estop_link`; actuator meshes are the simplified `scripts/parts/` cylinders | - |
| `urdf` | Source URDF: `robonex_edu` (lower body, battery and E-stop as fixed links) | - |
| `mujoco` | Closed-loop MJCF models and fixed/free-base scenes | [📖](mujoco/) |
| `isaac` | Closed-loop URDF/USD models for Isaac Sim | [📖](isaac/) |

<br>

## Build

### `scripts/model_tools.py`

Repo-root [`scripts/`](../#scripts), run with `--version ver2-2`. Regeneration order: `build` → `mujoco/build_mjcf.py
--fixed-base` → `constants` → `mujoco/build_mjcf.py` → `isaac/build_isaac_urdf.py` → `loops`, `check`, `limits`.

```bash
# Example
cd ~/humanoid_project/robonex-description

.venv/bin/python -I scripts/model_tools.py --version ver2-2 build --package "$HOME/Downloads/new2_urdf (1)/new2_urdf"
.venv/bin/python -I ver2-2/mujoco/build_mjcf.py --variant edu --fixed-base
.venv/bin/python -I scripts/model_tools.py --version ver2-2 constants
.venv/bin/python -I ver2-2/mujoco/build_mjcf.py --variant edu
.venv/bin/python -I ver2-2/isaac/build_isaac_urdf.py --variant edu --collision mesh
.venv/bin/python -I ver2-2/isaac/build_isaac_urdf.py --variant edu --collision box

# first build without ver2-2_constants.json
.venv/bin/python -I scripts/model_tools.py --version ver2-2 constants --seed-limits-only

# checks
.venv/bin/python -I scripts/model_tools.py --version ver2-2 loops --pose home
.venv/bin/python -I scripts/model_tools.py --version ver2-2 check
.venv/bin/python -I scripts/model_tools.py --version ver2-2 limits --variant edu
```

| Output | Description |
| --- | --- |
| `meshes/`, `loop_closures.yaml`, `urdf/robonex_edu.urdf` | `build` |
| `ver2-2_constants.json` | `constants` |
