from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE = "robonex_description"
SPAWN_HEIGHT = "1.0789"


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")

    description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([FindPackageShare(PACKAGE), "urdf", "robonex.urdf.xacro"]),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": description,
                     "publish_frequency": 100.0,
                     "use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={
            "gz_args": [LaunchConfiguration("headless"), " ", world],
        }.items(),
    )

    bringup = ExecuteProcess(
        cmd=[PathJoinSubstitution([FindPackageShare(PACKAGE), "scripts", "bringup.py"])],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "robonex",
                   "-z", SPAWN_HEIGHT,
                   "-allow_renaming", "false"],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/l_foot_contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
            "/r_foot_contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
            "/world/robonex_world/dynamic_pose/info@tf2_msgs/msg/TFMessage"
            "[ignition.msgs.Pose_V",
        ],
        remappings=[
            ("/world/robonex_world/dynamic_pose/info", "/gz_dynamic_pose"),
        ],
    )

    # world -> base_link isn't in /gz_dynamic_pose's frames as such - see
    # ros2/scripts/base_pose_tf.py for why a filter node is doing this instead
    # of a straight bridge onto /tf.
    base_pose_tf = Node(
        package=PACKAGE,
        executable="base_pose_tf.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name, "--controller-manager", "/controller_manager"],
        )

    joint_state_broadcaster = spawner("joint_state_broadcaster")
    joint_trajectory_controller = spawner("joint_trajectory_controller")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false",
                              description="run rviz2 alongside Gazebo"),
        DeclareLaunchArgument("headless", default_value="",
                              description="pass -s to run the server without the GUI"),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare(PACKAGE), "gazebo", "empty_world.sdf"]),
            description="world file to load"),

        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        base_pose_tf,

        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=[joint_state_broadcaster, bringup])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster, on_exit=[joint_trajectory_controller])),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            condition=IfCondition(rviz),
            parameters=[{"use_sim_time": True}],
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare(PACKAGE), "config", "robonex.rviz"])],
        ),
    ])
