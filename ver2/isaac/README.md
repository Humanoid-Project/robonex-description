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
├── edu/
│   ├── closed_loop_mesh/
│   └── closed_loop_box/
├── pro/
└── max/
```

| Variant | Mechanism | Collision |
| --- | --- | --- |
| `closed_loop_mesh` | Four-bar and differential closures kept; 12 cranks actuated (pro/max + 8 arms, max + neck pitch) | Visual meshes |
| `closed_loop_box` | Four-bar and differential closures kept; 12 cranks actuated (pro/max + 8 arms, max + neck pitch) | Box primitives |

<br>

## Build

### `scripts/build_<variant>.py`

```bash
# Example
cd ~/humanoid_project/robonex-description

python3 ver2/isaac/scripts/build_closed_loop_mesh.py --variant edu
python3 ver2/isaac/scripts/build_closed_loop_box.py --variant edu
```

<br>

### `build_isaac_urdf.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--variant` | `edu` | Model variant (`edu`, `pro`, `max`) |
| - | `--collision` | `mesh` | Collision geometry source (`mesh`, `box`) |

```bash
# Example
python3 ver2/isaac/build_isaac_urdf.py --variant edu --collision mesh
python3 ver2/isaac/build_isaac_urdf.py --variant edu --collision box
```

| Output | Description |
| --- | --- |
| `<variant>/closed_loop_<collision>/robonex_<variant>_closed_loop_<collision>.urdf` | URDF for the selected variant and collision |

<br>

## Convert

### `convert_urdf.py`

```bash
# Example
cd ~/humanoid_project/robonex-description
conda activate isaacsim

~/IsaacLab/isaaclab.sh -p ~/IsaacLab/scripts/tools/convert_urdf.py \
  $PWD/ver2/isaac/edu/closed_loop_mesh/robonex_edu_closed_loop_mesh.urdf \
  $PWD/ver2/isaac/edu/closed_loop_mesh/robonex_edu_closed_loop_mesh.usd \
  --joint-stiffness 40.0 --joint-damping 2.0 --headless
```

<br>

### `scripts/apply_physical_loops.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `usd_path` | `Required` | USD to close the loops in |

```bash
# Example
~/IsaacLab/isaaclab.sh -p ver2/isaac/scripts/apply_physical_loops.py \
  ver2/isaac/edu/closed_loop_mesh/robonex_edu_closed_loop_mesh.usd --headless
```
