# isaac

`conda activate isaacsim` below assumes the env exists — see [`robonex-common/setup/SETUP.md`](https://github.com/Humanoid-Project/robonex-common/blob/main/setup/SETUP.md) (`setup_isaacsim.sh`) to create it.

## Structure

```text
isaac/
├── README.md
├── build_isaac_urdf.py
├── load_robonex.py
├── scripts/
│   ├── apply_physical_loops.py
│   ├── build_closed_loop_mesh.py
│   └── build_closed_loop_box.py
├── closed_loop_mesh/
└── closed_loop_box/
```

| Variant | Mechanism | Collision |
| --- | --- | --- |
| `closed_loop_mesh` | Four-bar and differential closures kept; 12 cranks actuated | Visual meshes |
| `closed_loop_box` | Four-bar and differential closures kept; 12 cranks actuated | Box primitives |

<br>

## Build

### `scripts/build_<variant>.py`

```bash
# Example
cd ~/humanoid_project/robonex-description

python3 isaac/scripts/build_closed_loop_mesh.py
python3 isaac/scripts/build_closed_loop_box.py
```

### `build_isaac_urdf.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--collision` | No | `mesh` | Collision geometry source (`mesh`, `box`) |

```bash
# Example
python3 isaac/build_isaac_urdf.py --collision mesh
python3 isaac/build_isaac_urdf.py --collision box
```

| Output | Description |
| --- | --- |
| `<variant>/robonex_<variant>.urdf` | URDF for the selected variant |

<br>

## Convert

### `convert_urdf.py`

```bash
# Example
cd ~/humanoid_project/robonex-description
conda activate isaacsim

# Free-floating
~/IsaacLab/isaaclab.sh -p ~/IsaacLab/scripts/tools/convert_urdf.py \
  $PWD/isaac/closed_loop_mesh/robonex_closed_loop_mesh.urdf \
  $PWD/isaac/closed_loop_mesh/robonex_closed_loop_mesh.usd \
  --joint-stiffness 40.0 --joint-damping 2.0 --headless

# Fixed-base
~/IsaacLab/isaaclab.sh -p ~/IsaacLab/scripts/tools/convert_urdf.py \
  $PWD/isaac/closed_loop_mesh/robonex_closed_loop_mesh.urdf \
  $PWD/isaac/closed_loop_mesh/robonex_closed_loop_mesh_fixed.usd \
  --joint-stiffness 40.0 --joint-damping 2.0 --fix-base --headless
```

### `scripts/apply_physical_loops.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `usd_path` | Yes | - | USD to close the loops in |

```bash
# Example
~/IsaacLab/isaaclab.sh -p isaac/scripts/apply_physical_loops.py \
  isaac/closed_loop_mesh/robonex_closed_loop_mesh.usd --headless

~/IsaacLab/isaaclab.sh -p isaac/scripts/apply_physical_loops.py \
  isaac/closed_loop_mesh/robonex_closed_loop_mesh_fixed.usd --headless
```

<br>

## Run

### `load_robonex.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--collision` | No | `mesh` | Variant collision (`mesh`, `box`) |
| `--fixed-base` | No | Off | Load the fixed-base USD (base welded in the air) |
| `--spawn-height` | No | `1.085` free, `1.60` fixed | Spawn height (m) |
| `--headless` | No | Off | Run without the viewport |

```bash
# Example
cd ~/humanoid_project/robonex-description
conda activate isaacsim

~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --collision mesh
~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --collision box
~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --collision mesh --fixed-base
```

Press **Play** in the viewport toolbar to start physics.
