"""Example script to demonstrate how to step Gazebo simulation using ROS2."""

import time

import rclpy
from rclpy.node import Node

from ros_gz_interfaces.srv import ControlWorld


class GzStepExample(Node):
    """Node to step the Gazebo simulation."""

    def __init__(self):
        """Initialize the node."""
        super().__init__("gz_step_example")

        # Parameters
        self.declare_parameter("world_name", "default")
        self.world_name = (
            self.get_parameter("world_name").get_parameter_value().string_value
        )
        self.get_logger().info(f"World name: {self.world_name}")

        # Create a client to call the service
        srv_name = f"/world/{self.world_name}/control"
        self.control_world_client = self.create_client(ControlWorld, srv_name)
        while not self.control_world_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("ControlWorld not available, waiting again...")
        self.get_logger().info("ControlWorld service available")

        # Ensure the simulation is paused
        self.step_simulation(steps=0)
        self.get_logger().info("Simulation paused")

    def step_simulation(self, steps: int = 1) -> None:
        """Step the simulation."""
        step_req = ControlWorld.Request()
        step_req.world_control.pause = True
        if steps == 1:
            step_req.world_control.step = True
        else:
            step_req.world_control.multi_step = steps
        future = self.control_world_client.call_async(step_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Simulation stepped {steps} times")
        else:
            self.get_logger().error("Service call failed")


def main(args=None):
    """Periodically step the simulation."""
    rclpy.init(args=args)
    node = GzStepExample()
    while rclpy.ok():
        node.step_simulation(steps=1)
        rclpy.spin_once(node)
        time.sleep(1.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
