from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE = "robonex_description"


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")

    description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([FindPackageShare(PACKAGE), "urdf", "robonex.urdf.xacro"]),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="run joint_state_publisher_gui"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="run rviz2"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": description, "publish_frequency": 100.0}],
        ),
        # robot_state_publisher only ever publishes the RELATIVE joint tree
        # (base_link -> ... -> feet); nothing else here has an opinion on where
        # base_link sits in the world. gazebo.launch.py gets that from the
        # simulator (see base_pose_tf.py); here there's no physics, so identity
        # is exactly correct - and it's what lets robonex.rviz use the same
        # Fixed Frame ("world") in both launch files.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            condition=IfCondition(rviz),
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare(PACKAGE), "config", "robonex.rviz"])],
        ),
    ])
