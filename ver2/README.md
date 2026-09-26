# ver2

## Structure

```text
ver2/
├── README.md
├── INFO.md
├── ver2_constants.json
├── asis/                   Fusion 360 export as-is (input to build/)
├── _extract/               Fusion 360 extraction scripts that produced asis/
├── build/
│   ├── make_ver2.py
│   ├── solve_constants.py
│   ├── repose.py
│   └── stl_io.py
├── verify/                 Loop, limit and collision checks
├── meshes/
├── urdf/
│   ├── robonex_edu.urdf
│   ├── robonex_pro.urdf
│   └── robonex_max.urdf
├── loop_closures.yaml
├── scripts/
│   ├── model_io.py
│   └── robonex_data.py
├── mujoco/
│   ├── build_mjcf.py
│   └── robot/{edu,pro,max}/
└── isaac/
    ├── build_isaac_urdf.py
    ├── scripts/
    └── {edu,pro,max}/closed_loop_{mesh,box}/
```

| Variant | Body |
| --- | --- |
| `edu` | Lower body |
| `pro` | Lower body + arms |
| `max` | Lower body + arms + head |

<br>

## Generate

`asis/`, `_extract/`, `build/`, `verify/` are local only (gitignored); they exist on the machine that built Ver.2.

### `build/make_ver2.py`

Writes `meshes/`, `loop_closures.yaml` and the three `urdf/robonex_<variant>.urdf` from `asis/`.

```bash
# Example
cd ~/humanoid_project/robonex-description
python3 ver2/build/make_ver2.py
```

<br>

### `build/solve_constants.py`

Needs `mujoco/robot/edu/scene_fixed.xml` built first.

```bash
# Example
python3 ver2/build/solve_constants.py
```

| Output | Description |
| --- | --- |
| `ver2_constants.json` | Home pose, spawn heights, provisional joint limits, variant masses |

<br>

## Build

### `mujoco/build_mjcf.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--variant` | `edu` | Model variant (`edu`, `pro`, `max`) |
| - | `--fixed-base` | Off | Weld the base in mid-air so the legs swing free |
| - | `--training-limits` | Off | Use the provisional training joint limits |
| - | `--movable-arms` | Off | Actuate the arm joints instead of holding them (`pro`, `max`) |

```bash
# Example
cd ~/humanoid_project/robonex-description
python3 ver2/mujoco/build_mjcf.py --variant edu --fixed-base
python3 ver2/mujoco/build_mjcf.py --variant edu
python3 ver2/mujoco/build_mjcf.py --variant edu --training-limits
python3 ver2/mujoco/build_mjcf.py --variant max --movable-arms
```

| Output | Description |
| --- | --- |
| `robot/<variant>/robonex.xml`, `robot/<variant>/scene.xml` | No option |
| `robot/<variant>/robonex_fixed.xml`, `robot/<variant>/scene_fixed.xml` | `--fixed-base` |
| `robot/<variant>/*_limits.xml` | `--training-limits` |
| `robot/<variant>/*_arms.xml` | `--movable-arms` |

<br>

### `isaac/build_isaac_urdf.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--variant` | `edu` | Model variant (`edu`, `pro`, `max`) |
| - | `--collision` | `mesh` | Collision geometry source (`mesh`, `box`) |

```bash
# Example
python3 ver2/isaac/build_isaac_urdf.py --variant edu --collision mesh
python3 ver2/isaac/build_isaac_urdf.py --variant pro --collision mesh
python3 ver2/isaac/build_isaac_urdf.py --variant max --collision mesh
```

| Output | Description |
| --- | --- |
| `isaac/<variant>/closed_loop_<collision>/robonex_<variant>_closed_loop_<collision>.urdf` | URDF for the selected variant |

<br>

## Convert

### `convert_urdf.py`

```bash
# Example
cd ~/humanoid_project/robonex-description
conda activate isaacsim

# Free-floating
~/IsaacLab/isaaclab.sh -p ~/IsaacLab/scripts/tools/convert_urdf.py \
  $PWD/ver2/isaac/edu/closed_loop_mesh/robonex_edu_closed_loop_mesh.urdf \
  $PWD/ver2/isaac/edu/closed_loop_mesh/robonex_edu_closed_loop_mesh.usd \
  --joint-stiffness 40.0 --joint-damping 2.0 --headless

# Fixed-base
~/IsaacLab/isaaclab.sh -p ~/IsaacLab/scripts/tools/convert_urdf.py \
  $PWD/ver2/isaac/edu/closed_loop_mesh/robonex_edu_closed_loop_mesh.urdf \
  $PWD/ver2/isaac/edu/closed_loop_mesh/robonex_edu_closed_loop_mesh_fixed.usd \
  --joint-stiffness 40.0 --joint-damping 2.0 --fix-base --headless
```

<br>

### `isaac/scripts/apply_physical_loops.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `usd_path` | `Required` | USD to close the loops in |

```bash
# Example
~/IsaacLab/isaaclab.sh -p ver2/isaac/scripts/apply_physical_loops.py \
  ver2/isaac/edu/closed_loop_mesh/robonex_edu_closed_loop_mesh.usd --headless

~/IsaacLab/isaaclab.sh -p ver2/isaac/scripts/apply_physical_loops.py \
  ver2/isaac/edu/closed_loop_mesh/robonex_edu_closed_loop_mesh_fixed.usd --headless
```

<br>

## Run

### `verify/limit_tool.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--variant` | `edu` | Model variant (`edu`, `pro`, `max`) |

```bash
# Example
cd ~/humanoid_project/robonex-description
python3 ver2/verify/limit_tool.py --variant edu
```

<br>

### `mujoco.viewer`

```bash
# Example
python3 -m mujoco.viewer --mjcf=ver2/mujoco/robot/edu/scene.xml
python3 -m mujoco.viewer --mjcf=ver2/mujoco/robot/edu/scene_fixed.xml
python3 -m mujoco.viewer --mjcf=ver2/mujoco/robot/max/scene.xml
```
