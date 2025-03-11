"""Script to launch all EVS gz utils."""

from launch import LaunchDescription

from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """Launch all EVS Gazebo utils."""
    launch_list = []

    # Bridge clock from Gazebo to ROS
    ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    launch_list.append(ros_bridge)
    launch_list.append(SetParameter(name="use_sim_time", value=True))

    # UAV state directly from Gazebo
    uav_state_gazebo = Node(
        package="evs_gz_utils",
        executable="uav_state_gazebo",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"model_name": "x500_0"},
        ],
    )
    launch_list.append(uav_state_gazebo)

    return LaunchDescription(launch_list)
