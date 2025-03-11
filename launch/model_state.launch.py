"""Script to launch all EVS gz utils."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """Launch all EVS Gazebo utils."""
    launch_list = []
    world_name = LaunchConfiguration("world_name")
    model_name = LaunchConfiguration("model_name")

    # Bridge clock from Gazebo to ROS
    ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    launch_list.append(ros_bridge)
    launch_list.append(SetParameter(name="use_sim_time", value=True))

    # World name argument
    launch_list.append(
        DeclareLaunchArgument(
            "world_name",
            default_value="default",
            description="Name of the world in Gazebo",
        )
    )

    # Model name argument
    launch_list.append(
        DeclareLaunchArgument(
            "model_name",
            default_value="x500_0",
            description="Name of the model in Gazebo",
        )
    )

    # UAV state directly from Gazebo
    model_state_gazebo = Node(
        package="evs_gz_utils",
        executable="model_state_gazebo",
        output="screen",
        emulate_tty=True,
        parameters=[{"world_name": world_name, "model_name": model_name}],
    )
    launch_list.append(model_state_gazebo)

    return LaunchDescription(launch_list)
