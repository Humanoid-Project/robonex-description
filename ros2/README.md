# ros2

## Structure

```text
ros2/
├── README.md
├── build_ros_urdf.py
├── robonex.urdf.xacro
├── config/
│   ├── robonex.rviz
│   └── ros2_control.yaml
├── launch/
│   ├── display.launch.py
│   └── gazebo.launch.py
└── scripts/
    ├── base_pose_tf.py
    └── bringup.py
```

<br>

## Build

### `build_ros_urdf.py`

```bash
# Example
cd ~/humanoid_project/robonex_description
source /opt/ros/humble/setup.bash

python3 ros2/build_ros_urdf.py
colcon build --symlink-install --packages-select robonex_description
source install/setup.bash
```

<br>

## RViz

### `display.launch.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `gui` | No | `true` | Run `joint_state_publisher_gui` |
| `rviz` | No | `true` | Run RViz |

```bash
# Example
ros2 launch robonex_description display.launch.py
ros2 launch robonex_description display.launch.py gui:=false
```

<br>

## Gazebo

### `gazebo.launch.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `rviz` | No | `false` | Run RViz with Gazebo |
| `headless` | No | Empty | Pass `-s` to run the server without the GUI |
| `world` | No | `gazebo/empty_world.sdf` | World file to load |

```bash
# Example
ros2 launch robonex_description gazebo.launch.py
ros2 launch robonex_description gazebo.launch.py rviz:=true
ros2 launch robonex_description gazebo.launch.py headless:=-s
```