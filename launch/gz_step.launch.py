"""Script to launch stepping example."""

from launch import LaunchDescription

from launch_ros.actions import Node

WORLD_NAME = "default"


def generate_launch_description():
    """Launch the stepping example."""
    launch_list = []

    # Bridge WorldControl service
    ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[f"/world/{WORLD_NAME}/control@ros_gz_interfaces/srv/ControlWorld"],
        output="screen",
    )
    launch_list.append(ros_bridge)

    # Node for stepping the simulation
    gz_step_example = Node(
        package="evs_gz_utils",
        executable="gz_step_example",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"world_name": WORLD_NAME},
        ],
    )
    launch_list.append(gz_step_example)

    return LaunchDescription(launch_list)
