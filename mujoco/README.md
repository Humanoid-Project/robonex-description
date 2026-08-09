# RoboNex Mujoco

## Build

```bash
python3 mujoco/build_mjcf.py
```

## Options

| Option | Description |
|---|---|
| `--fixed-base` | Base welded in mid-air so the legs swing free |


## Run

```bash
python3 -m mujoco.viewer --mjcf=mujoco/scene.xml
```