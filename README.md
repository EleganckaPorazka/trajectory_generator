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

### Joint sinusoidal trajectory

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

### Cartesian sinusoidal trajectory

To run the 'cartesian_sinusoidal_trajectory' node, use the following command:
```
ros2 run trajectory_generator cartesian_sinusoidal_trajectory 
```

To send the Cartesian trajectory generation request:
```
ros2 action send_goal /cartesian_motion rrlib_interfaces/action/CART "{motion_type: tp, start_pose: [x1,x2,...xn], end_pose: [y1,y2,...,yn], vel_max: v, acc_max: a, dt: d}"

```
where 'motion_type' can be either 'LIN' or 'CIRC', the 'start_pose' and 'end_pose' are the 7-element vectors of position (m) and orientation (quaternions), 'v' is the max velocity in m/s, 'a' is the max acceleration in m/s^2, and 'd' is the time step in seconds.

Listen to what is computed:
```
ros2 topic echo /cart_sin_traj
```

Get the action status:
```
ros2 topic echo /cartesian_motion/_action/status
```

Get the feedback:
```
ros2 topic echo /cartesian_motion/_action/feedback
```

## Notes

TODO: reading motion_type in a goal request. For now, it seems like the input text is not recognized.

TODO: changing orientation

TODO: CIRC motion

This code is also uploaded to [**my other repository**](https://gitlab.com/lwolinski/trajectory_generator.git).
