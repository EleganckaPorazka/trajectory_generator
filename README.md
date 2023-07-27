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

To run the 'joint_sinusoidal_trajectory' node, use the following command:
```
ros2 run trajectory_generator joint_sinusoidal_trajectory 
```

To send the joint trajectory generation request:
```
ros2 action send_goal /ptp_motion rrlib_interfaces/action/PTP "{start_position: [x1,x2,...xn], end_position: [y1,y2,...,yn], vel_max: v, acc_max: a, dt: d}"

```
where the 'start_position' and 'end_position' are vectors in radians, 'v' is the max velocity in rad/s, 'a' is the max acceleration in rad/s^2, and 'd' is the time step in seconds.

Listen to what is computed:
```
ros2 topic echo /jnt_sin_traj
```

Get the action status:
```
ros2 topic echo /ptp_motion/_action/status
```

Get the feedback:
```
ros2 topic echo /ptp_motion/_action/feedback
```
