# RoboNex Description

## Setup
```bash
git clone https://github.com/Humanoid-Project/robonex_description.git
cd ~/robonex_description

sudo apt install python3-venv
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install --upgrade pip
```

## Overview
```
robonex_description
├── urdf
│   ├── robonex.urdf
│   └── README.md
├── meshes
│   ├── base_link.stl
│   └── ...
├── loop_closures.yaml
├── mujoco
│   ├── build_mjcf.py
│   ├── robonex.xml
│   ├── scene.xml
│   └── README.md
├── gazebo
│   ├── build_sdf.py
│   ├── robonex.sdf
│   ├── robonex_world.sdf
│   └── README.md
├── rviz
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── launch
│   ├── config
│   └── README.md
├── isaac
│   ├── load_robonex.py
│   └── README.md
├── scripts
│   └── robonex_common.py
├── requirements.txt
├── LICENSE
└── README.md
```

- `meshes`: 37 STL files in millimetre units, one per link, visual and
  collision alike
- [`urdf`](urdf/README.md): the kinematic tree, and the single source every
  builder reads
- [`mujoco`](mujoco/README.md): builds the MJCF, with the loops restored as
  equality constraints
- [`gazebo`](gazebo/README.md): builds SDF for Gazebo Fortress; DART cannot close
  the loops, so the linkages are replaced by their serial equivalent
- [`rviz`](rviz/README.md): ROS 2 package `robonex_description` for viewing the
  URDF; no physics, so the loops hang free
- [`isaac`](isaac/README.md): imports into Isaac Sim and rebuilds the loops as
  PhysX joints
- `loop_closures.yaml`: the knee four-bar, the ankle ball joints and the list of
  actuated joints, none of which URDF can express
- `scripts`: the URDF and YAML parser shared by all three builders

## License

[BSD 3-Clause](LICENSE). This covers everything in the repository, the mesh
files included.