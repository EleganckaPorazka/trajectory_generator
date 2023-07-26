# Trajectory generator
<img src="https://img.shields.io/badge/ros--version-humble-green"/>  <img src="https://img.shields.io/badge/platform%20-Ubuntu%2022.04-orange"/>

## Description

The `trajectory_generator` package contains functions for computing joint and Cartesian space trajectories.

A work in progress in early stages.

## Installation

It is recommended to use Ubuntu 22.04 with [**ROS 2 Humble**](https://docs.ros.org/en/humble/index.html).

### Required packages

[**rrlib_interfaces**](https://github.com/EleganckaPorazka/rrlib_interfaces.git)

### Building from source

```
mkdir -p ~/ros2_ws/src/trajectory_generator
cd ~/ros2_ws/src/trajectory_generator
git clone https://github.com/EleganckaPorazka/trajectory_generator.git .
source /opt/ros/humble/setup.bash
colcon build
. install/setup.bash
```

## Running

Right now this section is not functional anymore. The node is being changed to an action server.

To run the 'joint_sinusoidal_trajectory' node, use the following command:
```
ros2 run trajectory_generator joint_sinusoidal_trajectory 
```

To send the time from <0, end_time>:
```
ros2 topic pub --once /jnt_sin_local_time std_msgs/msg/Float64 "{data: x}"
```
where 'x' is the value in seconds.

Listen to what is computed:
```
ros2 topic echo /jnt_sin_traj
```


