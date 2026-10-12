# ver2-3

## Models
| Folder | Description | README |
| --- | --- | :---: |
| `meshes` | Link STL meshes (level-sole zero pose); battery and E-stop are in `base_link`, `head_link` is included; actuator meshes are the simplified `scripts/parts/` cylinders | - |
| `urdf` | Source URDF: `robonex_edu` (lower body, `neck_pitch_joint` revolute) | - |
| `mujoco` | Closed-loop MJCF models and fixed/free-base scenes; neck fixed at 0 rad | [📖](mujoco/) |
| `isaac` | Closed-loop URDF/USD models for Isaac Sim; neck fixed at 0 rad | [📖](isaac/) |

<br>

## Build

### `scripts/model_tools.py`

Repo-root [`scripts/`](../#scripts), run with `--version ver2-3`. Regeneration order: `build` → `mujoco/build_mjcf.py
--fixed-base` → `constants` → `mujoco/build_mjcf.py` → `isaac/build_isaac_urdf.py` → `loops`, `check`, `limits`.

`neck_pitch_joint` stays revolute in `urdf/robonex_edu.urdf`. The MuJoCo and Isaac models fix it at 0 rad.

```bash
# Example
cd ~/humanoid_project/robonex-description

.venv/bin/python -I scripts/model_tools.py --version ver2-3 build --package "$HOME/humanoid_project/robonex-description/new3_urdf"
.venv/bin/python -I ver2-3/mujoco/build_mjcf.py --variant edu --fixed-base
.venv/bin/python -I scripts/model_tools.py --version ver2-3 constants
.venv/bin/python -I ver2-3/mujoco/build_mjcf.py --variant edu
.venv/bin/python -I ver2-3/isaac/build_isaac_urdf.py --variant edu --collision mesh
.venv/bin/python -I ver2-3/isaac/build_isaac_urdf.py --variant edu --collision box

# first build without ver2-3_constants.json
.venv/bin/python -I scripts/model_tools.py --version ver2-3 constants --seed-limits-only

# checks
.venv/bin/python -I scripts/model_tools.py --version ver2-3 loops --pose home
.venv/bin/python -I scripts/model_tools.py --version ver2-3 check
.venv/bin/python -I scripts/model_tools.py --version ver2-3 limits --variant edu
```

| Output | Description |
| --- | --- |
| `meshes/`, `loop_closures.yaml`, `urdf/robonex_edu.urdf` | `build` |
| `ver2-3_constants.json` | `constants` |
