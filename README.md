# evs_gz_utils

Utilites for working with Gazebo and ROS2.
It uses the [ros_gz](https://github.com/gazebosim/ros_gz) package provided by Gazebo developers, which facilitates integration between Gazebo and ROS.

## Prerequisites
This package uses [quaternionic](https://github.com/moble/quaternionic) - pure Python package for quaternions.
Due to some integrity reasons it is advised to install it with following dependencies:
``` bash
python3 -m pip install numpy==1.22 quaternionic
```

## Examples

This repository containes some examples of using `ros_gz_bridge` to exchange data between Gazebo and ROS:
1. [evs_gz_utils.launch.py](./launch/evs_gz_utils.launch.py):
It generates the 'ground truth' data of the model state from the internal Gazebo topic.
Note that it comes with a special bridge to the `/clock` topic, which allows synchronisation between Gazebo and ROS.
After setting the `use_sim_time` parameter, all following ROS nodes are using the clock source from Gazebo.

Usage example:
``` bash
ros2 launch evs_gz_utils model_state.launch.py world_name:=world_1 model_name:=uav
```

2. [gz_step.launch.py](./launch/gz_step.launch.py):
It runs the Gazebo simulation in 'stepping' mode, periodically calling the bridged world control service.
Note that this is only a dummy example - normally you should perform some task between subsequent steps.

Usage example:
``` bash
ros2 launch evs_gz_utils model_state.launch.py world_name:=world_1
```