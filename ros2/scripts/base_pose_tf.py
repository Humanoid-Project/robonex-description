#!/usr/bin/env python3
"""Bridge the one missing piece of robonex's TF tree.

robot_state_publisher only ever sees /joint_states, so it can only publish the
RELATIVE tree base_link -> ... -> feet. It has no idea where base_link itself
is in the world - that's Gazebo's business, since the base is a free-floating
body with no parent joint.

Gazebo's own answer to "where is base_link" comes bundled with the pose of
every other entity in the sim, on .../dynamic_pose/info (bridged in
gazebo.launch.py as /gz_dynamic_pose, tf2_msgs/msg/TFMessage). Broadcasting
that whole message as-is onto /tf would give every link a SECOND, competing
parent (world, instead of the joint it actually hangs off), and TF only
tolerates one parent per frame. So: pull out the single transform named for
the model itself - which is exactly base_link's pose, since base_link is what
the URDF's root link becomes - rename it to world -> base_link, and publish
only that.
"""
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


class BasePoseTF(Node):
    def __init__(self):
        super().__init__("base_pose_tf")
        self.declare_parameter("model_name", "robonex")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.model_name = self.get_parameter("model_name").value
        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.broadcaster = TransformBroadcaster(self)
        self.last_stamp = None
        self.sub = self.create_subscription(
            TFMessage, "/gz_dynamic_pose", self.on_pose_v, 10)

    def on_pose_v(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id != self.model_name:
                continue

            # ros_gz_bridge leaves the converted TFMessage's stamp at zero, so
            # we have to supply one. use_sim_time is set, so this is the same
            # sim clock RViz looks TF up against.
            now = self.get_clock().now().nanoseconds

            # While bringup holds the world paused and steps it a few ms at a
            # time, poses keep arriving far faster than sim time advances, so
            # most of them would repeat a stamp we already published. tf2
            # rejects those ("TF_OLD_DATA ... ignoring data from the past") and
            # complains once per message, which buries the console at startup.
            if self.last_stamp is not None and now <= self.last_stamp:
                return
            self.last_stamp = now

            t.header.frame_id = self.world_frame
            t.child_frame_id = self.base_frame
            t.header.stamp = self.get_clock().now().to_msg()
            self.broadcaster.sendTransform(t)
            return


def main():
    rclpy.init()
    node = BasePoseTF()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
