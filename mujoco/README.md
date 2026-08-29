# RoboNex Mujoco

## Structure

```text
mujoco/
├── README.md
├── build_mjcf.py
├── robonex.xml              free base, mesh collision
├── scene.xml
├── robonex_fixed.xml        base welded in mid-air
├── scene_fixed.xml
├── robonex_fixed_box.xml    fixed base, box collision
├── scene_fixed_box.xml
└── full_limit/              kinematic-maximum joint ranges
```

<br>

## Build

### `build_mjcf.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--fixed-base` | No | Off | Weld the base in mid-air so the legs swing free |
| `--collision-box` | No | Off | Box collision primitives instead of visual meshes; requires `--fixed-base` |

```bash
# Example
python3 mujoco/build_mjcf.py
python3 mujoco/build_mjcf.py --fixed-base
python3 mujoco/build_mjcf.py --fixed-base --collision-box
```

| Output | Description |
| --- | --- |
| `robonex.xml`, `scene.xml` | No option |
| `robonex_fixed.xml`, `scene_fixed.xml` | `--fixed-base` |
| `robonex_fixed_box.xml`, `scene_fixed_box.xml` | `--fixed-base --collision-box` |

<br>

## Run

```bash
# Example
python3 -m mujoco.viewer --mjcf=mujoco/scene.xml
python3 -m mujoco.viewer --mjcf=mujoco/scene_fixed.xml
python3 -m mujoco.viewer --mjcf=mujoco/scene_fixed_box.xml
```
