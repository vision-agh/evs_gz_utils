"""Script to launch stepping example."""

import sys

from launch import LaunchDescription

from launch_ros.actions import Node

DEFAULT_WORLD_NAME = "default"


def generate_launch_description():
    """Launch the stepping example."""
    launch_list = []

    # World name argument for the bridge
    # NOTE: It is just a one big workaround to get the world name
    if len(sys.argv) < 5:
        world_name_str = DEFAULT_WORLD_NAME
    else:
        launch_args = sys.argv[4].split("=")
        if len(launch_args) == 2 and launch_args[0][:-1] == "world_name":
            world_name_str = launch_args[1]
        else:
            world_name_str = DEFAULT_WORLD_NAME
    print(f"[gz_step.launch.py] world_name: {world_name_str}")

    # Bridge WorldControl service
    ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"/world/{world_name_str}/control@ros_gz_interfaces/srv/ControlWorld"
        ],
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
            {"world_name": world_name_str},
        ],
    )
    launch_list.append(gz_step_example)

    return LaunchDescription(launch_list)
