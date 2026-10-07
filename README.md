# RoboNex Description

## Setup
```bash
# Example
cd ~/humanoid_project
git clone https://github.com/Humanoid-Project/robonex-description.git
cd robonex-description
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`robonex-common` is pinned in `requirements.txt` — see [`robonex-common/setup/SETUP.md`](https://github.com/Humanoid-Project/robonex-common/blob/main/setup/SETUP.md).

<br>

## Models
| Folder | Description | README |
| --- | --- | :---: |
| `ver1` | Ver.1 lower body: meshes, URDF, closed-loop MJCF and Isaac USD | [📖](ver1/) |
| `ver2` | Ver.2 `edu` (legs), `pro` (legs + arms), `max` (legs + arms + head) | [📖](ver2/) |
| `ver2-2` | Ver.2-2 `edu`: battery and E-stop as their own links, measured masses (24.49 kg) | [📖](ver2-2/) |

<br>

## Scripts

Shared by `ver2` and `ver2-2` (`ver1` keeps its own `ver1/scripts/`). The builders under `<version>/mujoco/` and
`<version>/isaac/` import `model_io.py` and `robonex_data.py` from here and pass their own version.

```text
scripts/
├── model_io.py              VERSIONS table (variants, constants file), URDF/YAML parser
├── robonex_data.py          build constants read from <version>/<version>_constants.json
├── model_tools.py           build, constants, loops, limits, check
├── parts/                   rs02.stl, rs03.stl, d435i.stl — simplified actuator / camera meshes placed by `build`
└── REPORT_fusion_v14.md     Ver.2-2 Fusion export report
```

### `scripts/model_tools.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--version` | `Required` | Model folder (`ver2`, `ver2-2`); `build` and `constants` are `ver2-2` only |
| `build` | `--package` | `Required` | Fusion 360 export folder (`ver2-2_Edu.urdf` or `urdf/robonex.urdf`, `loop_closures.yaml`, `meshes/`); writes `meshes/`, `loop_closures.yaml`, `urdf/robonex_edu.urdf` |
| `constants` | `--seed-limits-only` | Off | Write only `provisional_limits` (first build, before `scene_fixed.xml` exists) |
| `loops` | `--pose` | `zero` | Pose to check (`zero`, `home`) |
| - | `--scene` | `<version>/mujoco/robot/edu/scene_fixed.xml` | MuJoCo scene to check |
| - | `--tol` | `1e-06` | Residual tolerance in m (exit 1 above it) |
| `limits` | `--variant` | `edu` | Model variant of the selected version |
| `check` | `--urdf` | `<version>/urdf/robonex_edu.urdf` | URDF to check |

```bash
# Example
cd ~/humanoid_project/robonex-description

.venv/bin/python -I scripts/model_tools.py --version ver2-2 build --package "$HOME/Downloads/new2_urdf (1)/new2_urdf"
.venv/bin/python -I scripts/model_tools.py --version ver2-2 constants

# checks
.venv/bin/python -I scripts/model_tools.py --version ver2-2 loops --pose home
.venv/bin/python -I scripts/model_tools.py --version ver2 loops --pose home --scene ver2/mujoco/robot/pro/scene_fixed.xml
.venv/bin/python -I scripts/model_tools.py --version ver2 check
.venv/bin/python -I scripts/model_tools.py --version ver2-2 limits --variant edu
```

| Output | Description |
| --- | --- |
| `<version>/meshes/`, `<version>/loop_closures.yaml`, `<version>/urdf/robonex_edu.urdf` | `build` |
| `<version>/<version>_constants.json` | `constants` |
