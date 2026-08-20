# gazebo

## Structure

```text
gazebo/
├── README.md
├── build_sdf.py
├── empty_world.sdf
├── robonex.sdf
└── robonex_world.sdf
```

<br>

## Build

### `build_sdf.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--drop` | No | Off | Spawn at the MuJoCo height (`1.085 m`) instead of the ground pose (`1.0789 m`) |

```bash
# Example
python3 gazebo/build_sdf.py
python3 gazebo/build_sdf.py --drop
```

| Output | Description |
| --- | --- |
| `robonex.sdf` | Robot model only |
| `robonex_world.sdf` | World with the robot |
| `empty_world.sdf` | Empty world used by the ROS 2 launch |

<br>

## Run

```bash
# Example
ign gazebo gazebo/robonex_world.sdf
ign gazebo -s gazebo/robonex_world.sdf
```