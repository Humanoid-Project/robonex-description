# RoboNex Description

## Setup
```bash
cd ~/humanoid_project
git clone https://github.com/Humanoid-Project/robonex-common.git
git clone https://github.com/Humanoid-Project/robonex-description.git
cd robonex-description
source ../robonex-common/setup/setup.sh
```

Shared across repos — see [`robonex-common/setup/SETUP.md`](https://github.com/Humanoid-Project/robonex-common/blob/main/setup/SETUP.md).

<br>

## Models
| Folder | Description | README |
| --- | --- | :---: |
| `meshes` | Link and actuator STL meshes | - |
| `urdf` | Source URDF kinematic and inertial model | - |
| `ros2` | Xacro, RViz, Gazebo launch, and controller configuration | [📖](ros2/) |
| `gazebo` | SDF generator and Gazebo Fortress scenes | [📖](gazebo/) |
| `mujoco` | Closed-loop MJCF models and fixed/free-base scenes | [📖](mujoco/) |
| `isaac` | Closed-loop URDF/USD models for Isaac Sim | [📖](isaac/) |
