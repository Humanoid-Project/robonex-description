# RoboNex Mujoco

## Structure

```text
mujoco/
├── README.md
├── build_mjcf.py
├── basic/
│   ├── robonex.xml
│   ├── scene.xml
│   ├── robonex_fixed.xml
│   ├── scene_fixed.xml
│   └── box/
│       ├── robonex_fixed.xml
│       └── scene_fixed.xml
└── full_limit/
    ├── robonex_fixed.xml
    └── scene_fixed.xml
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
| `basic/robonex.xml`, `basic/scene.xml` | No option |
| `basic/robonex_fixed.xml`, `basic/scene_fixed.xml` | `--fixed-base` |
| `basic/box/robonex_fixed.xml`, `basic/box/scene_fixed.xml` | `--fixed-base --collision-box` |

<br>

## Run

```bash
# Example
python3 -m mujoco.viewer --mjcf=mujoco/basic/scene.xml
python3 -m mujoco.viewer --mjcf=mujoco/basic/scene_fixed.xml
python3 -m mujoco.viewer --mjcf=mujoco/basic/box/scene_fixed.xml
```
