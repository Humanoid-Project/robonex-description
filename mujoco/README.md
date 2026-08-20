# RoboNex Mujoco

## Build

```bash
python3 mujoco/build_mjcf.py
python3 mujoco/build_mjcf.py --fixed-base
```

## Options

| Option | Description |
|---|---|
| `--fixed-base` | Base welded in mid-air so the legs swing free; writes separate `robonex_fixed.xml` and `scene_fixed.xml` files |


## Run

```bash
python3 -m mujoco.viewer --mjcf=mujoco/scene.xml
python3 -m mujoco.viewer --mjcf=mujoco/scene_fixed.xml
```