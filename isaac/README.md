# isaac

## Structure

```text
isaac/
├── README.md
├── build_isaac_urdf.py
├── load_robonex.py
├── scripts/
│   ├── apply_physical_loops.py
│   ├── build_closed_loop_mesh.py
│   ├── build_closed_loop_box.py
│   ├── build_serial_mesh.py
│   └── build_serial_box.py
├── closed_loop_mesh/
├── closed_loop_box/
├── serial_mesh/
└── serial_box/
```

| Variant | Mechanism | Collision |
| --- | --- | --- |
| `closed_loop_mesh` | Four-bar and differential closures kept; 12 cranks actuated | Visual meshes |
| `closed_loop_box` | Four-bar and differential closures kept; 12 cranks actuated | Box primitives |
| `serial_mesh` | Loops removed; linkage output joints actuated directly | Visual meshes |
| `serial_box` | Loops removed; linkage output joints actuated directly | Box primitives |

<br>

## Build

### `scripts/build_<variant>.py`

```bash
# Example
cd ~/humanoid_project/robonex_description

python3 isaac/scripts/build_closed_loop_mesh.py
python3 isaac/scripts/build_closed_loop_box.py
python3 isaac/scripts/build_serial_mesh.py
python3 isaac/scripts/build_serial_box.py
```

### `build_isaac_urdf.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--mechanism` | No | `closed_loop` | Keep the physical mechanisms, or drive their outputs directly (`closed_loop`, `serial`) |
| `--collision` | No | `mesh` | Collision geometry source (`mesh`, `box`) |

```bash
# Example
python3 isaac/build_isaac_urdf.py --mechanism closed_loop --collision mesh
python3 isaac/build_isaac_urdf.py --mechanism serial --collision box
```

| Output | Description |
| --- | --- |
| `<variant>/robonex_<variant>.urdf` | URDF for the selected variant |

<br>

## Convert

### `convert_urdf.py`

```bash
# Example
cd ~/humanoid_project/robonex_description
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

Required for `closed_loop_*` variants only.

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
| `--mechanism` | No | `closed_loop` | Variant mechanism (`closed_loop`, `serial`) |
| `--collision` | No | `mesh` | Variant collision (`mesh`, `box`) |
| `--fixed-base` | No | Off | Load the fixed-base USD (base welded in the air) |
| `--spawn-height` | No | `1.085` free, `1.60` fixed | Spawn height (m) |
| `--headless` | No | Off | Run without the viewport |

```bash
# Example
cd ~/humanoid_project/robonex_description
conda activate isaacsim

~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --mechanism closed_loop --collision mesh
~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --mechanism serial --collision box
~/IsaacLab/isaaclab.sh -p isaac/load_robonex.py --mechanism closed_loop --collision mesh --fixed-base
```

Press **Play** in the viewport toolbar to start physics.
