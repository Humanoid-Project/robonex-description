# RoboNex Description

## Setup
```bash
# Example
cd ~/humanoid_project
git clone https://github.com/Humanoid-Project/robonex-common.git
git clone https://github.com/Humanoid-Project/robonex_description.git
cd robonex_description
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e ../robonex-common
pip install -r requirements.txt
```

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

<br>

## License
[BSD 3-Clause](LICENSE), including the mesh files in this repository.
