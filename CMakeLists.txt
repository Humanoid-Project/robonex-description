cmake_minimum_required(VERSION 3.8)
project(robonex_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes gazebo
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ros2/launch ros2/config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES urdf/robonex.urdf ros2/robonex.urdf.xacro
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(PROGRAMS ros2/scripts/bringup.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

# base_pose_tf.py is launched via launch_ros's Node(), which resolves
# package+executable through `ros2 run` - that only searches lib/<pkg>/, not
# share/<pkg>/scripts/ (where bringup.py lives; it's run by full path instead).
install(PROGRAMS ros2/scripts/base_pose_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES loop_closures.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
