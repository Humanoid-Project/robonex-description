# RoboNex Description

## Setup
```bash
cd ~/humanoid_project
git clone https://github.com/Humanoid-Project/robonex-description.git
cd robonex-description
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`robonex-common` is pinned in `requirements.txt` — see [`robonex-common/setup/SETUP.md`](https://github.com/Humanoid-Project/robonex-common/blob/main/setup/SETUP.md).

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
