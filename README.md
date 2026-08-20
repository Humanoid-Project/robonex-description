# RoboNex Description

## Setup
```bash
git clone https://github.com/Humanoid-Project/robonex_description.git
cd robonex_description
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Models
| Folder | Description | README |
| --- | --- | :---: |
| `meshes` | Link and actuator STL meshes | - |
| `urdf` | Source URDF kinematic and inertial model | - |
| `ros2` | Xacro, RViz, Gazebo launch, and controller configuration | [📖](ros2/) |
| `gazebo` | SDF generator and Gazebo Fortress scenes | [📖](gazebo/) |
| `mujoco` | Closed-loop MJCF models and fixed/free-base scenes | [📖](mujoco/) |
| `isaac` | Simplified and closed-loop URDF/USD models for Isaac Sim | [📖](isaac/) |

## License
[BSD 3-Clause](LICENSE), including the mesh files in this repository.
