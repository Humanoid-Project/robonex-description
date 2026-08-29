# RoboNex Description

## Setup
```bash
cd ~/humanoid_project
git clone https://github.com/Humanoid-Project/robonex-common.git
git clone https://github.com/Humanoid-Project/robonex_description.git
cd robonex_description
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e ../robonex-common
pip install -r requirements.txt
```

`robonex-common`과 `robonex_description`은 같은 상위 폴더에 두는 구성을 권장합니다. 생성 스크립트는 모터 종류와 물리 파라미터를 `robonex-common`에서 읽고, 형상·폐루프·충돌 정보는 이 저장소가 소유합니다.

## Models
| Folder | Description | README |
| --- | --- | :---: |
| `meshes` | Link and actuator STL meshes | - |
| `urdf` | Source URDF kinematic and inertial model | - |
| `ros2` | Xacro, RViz, Gazebo launch, and controller configuration | [📖](ros2/) |
| `gazebo` | SDF generator and Gazebo Fortress scenes | [📖](gazebo/) |
| `mujoco` | Closed-loop MJCF models and fixed/free-base scenes | [📖](mujoco/) |
| `isaac` | Closed-loop URDF/USD models for Isaac Sim (mesh and box collision) | [📖](isaac/) |

## License
[BSD 3-Clause](LICENSE), including the mesh files in this repository.
