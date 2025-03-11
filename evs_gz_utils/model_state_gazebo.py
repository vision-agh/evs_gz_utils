"""Node to publish model state based on the direct Gazebo messages."""

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

from nav_msgs.msg import Odometry

import numpy as np

import quaternionic

import rclpy
from rclpy.node import Node as ROSNode


class UAVStateGazebo(ROSNode):
    """Node to publish UAV state based on the direct Gazebo messages."""

    def __init__(self):
        """Initialize the node."""
        super().__init__("uav_state_gazebo")

        # Parameters
        self.declare_parameter("model_name", "x500_0")
        self.model_name = (
            self.get_parameter("model_name").get_parameter_value().string_value
        )
        self.get_logger().info(f"Model name: {self.model_name}")

        self.declare_parameter("world_name", "default")
        self.world_name = (
            self.get_parameter("world_name").get_parameter_value().string_value
        )
        self.get_logger().info(f"World name: {self.world_name}")

        # Some variables for the previous state
        self.prev_time_s = None
        self.prev_position = None
        self.prev_q = None

        # Create a publisher for the UAV state
        self.state_publisher = self.create_publisher(
            Odometry, f"/evs/{self.model_name}/state", 10
        )

        # Create a Gazebo node
        self.gz_node = GzNode()
        topic_poses = f"/world/{self.world_name}/pose/info"

        # Try to subscribe to the topic
        self.gz_node.subscribe(Pose_V, topic_poses, self._gz_pose_clb)

        self.get_logger().info("UAV State Gazebo node initialized")

    def _gz_pose_clb(self, gz_msg) -> None:
        msg_time_s = gz_msg.header.stamp.sec + gz_msg.header.stamp.nsec * 1e-9
        if self.prev_time_s is not None and msg_time_s <= self.prev_time_s:
            return

        # Search for the UAV model in the message
        for gz_pose in gz_msg.pose:
            if gz_pose.name == self.model_name:
                state_msg = Odometry()
                state_msg.header.stamp = self.get_clock().now().to_msg()
                state_msg.header.frame_id = "world"
                state_msg.child_frame_id = self.model_name

                # Position (x, y, z) in Earth frame
                state_msg.pose.pose.position.x = gz_pose.position.x
                state_msg.pose.pose.position.y = gz_pose.position.y
                state_msg.pose.pose.position.z = gz_pose.position.z

                # Orientation (quaternion) in Earth frame
                state_msg.pose.pose.orientation.w = gz_pose.orientation.w
                state_msg.pose.pose.orientation.x = gz_pose.orientation.x
                state_msg.pose.pose.orientation.y = gz_pose.orientation.y
                state_msg.pose.pose.orientation.z = gz_pose.orientation.z

                # Calculate linear velocity if possible
                np_position = np.array(
                    [gz_pose.position.x, gz_pose.position.y, gz_pose.position.z]
                )
                if self.prev_position is not None:
                    velocity = (np_position - self.prev_position) / (
                        msg_time_s - self.prev_time_s
                    )
                    state_msg.twist.twist.linear.x = velocity[0]
                    state_msg.twist.twist.linear.y = velocity[1]
                    state_msg.twist.twist.linear.z = velocity[2]

                # Calculate angular velocity if possible
                current_q = quaternionic.array(
                    [
                        gz_pose.orientation.w,
                        gz_pose.orientation.x,
                        gz_pose.orientation.y,
                        gz_pose.orientation.z,
                    ]
                )
                if self.prev_q is not None:
                    deriv = (
                        self.prev_q.conj() * current_q / (msg_time_s - self.prev_time_s)
                    )
                    angular_velocity = 2 * np.array([deriv.x, deriv.y, deriv.z])
                    state_msg.twist.twist.angular.x = angular_velocity[0]
                    state_msg.twist.twist.angular.y = angular_velocity[1]
                    state_msg.twist.twist.angular.z = angular_velocity[2]

                # Publish the state if it is complete
                if self.prev_time_s is not None:
                    self.state_publisher.publish(state_msg)

                # Update the previous state
                self.prev_time_s = msg_time_s
                self.prev_position = np_position
                self.prev_q = current_q
                return


def main(args=None):
    """Initialize the node."""
    rclpy.init(args=args)

    node = UAVStateGazebo()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
