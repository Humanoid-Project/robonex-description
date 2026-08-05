# RoboNex MJCF

## Build

```bash
python3 mujoco/build_mjcf.py
```

| File | Contents |
|---|---|
| `robonex.xml` | The robot only |
| `scene.xml` | Includes `robonex.xml`, adds floor, light and sky. |


## Run

```bash
python3 -m mujoco.viewer --mjcf=mujoco/scene.xml
```

## Options

| Option | Description |
|---|---|
| `--fixed-base` | Base welded in mid-air so the legs swing free |