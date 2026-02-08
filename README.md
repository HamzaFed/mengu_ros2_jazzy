# Mengu ROS 2 Jazzy

Mengu is a ROS 2 (Jazzy) robot project.
This repository covers both the **robot description** and the **control layer**.

The project started with a simple pan-tilt mechanism and is being gradually
extended toward a more advanced robotic system (e.g. a 6-DoF manipulator).

## Packages

### mengu_description
- Robot URDF/Xacro description
- Mesh files
- RViz visualization
- Gazebo simulation

### mengu_controller
- ros2_control integration
- Controller configurations (arm & gripper)
- Joint trajectory control support
- Gazebo-based controller testing


## Features
- RViz visualization using robot_state_publisher
- Gazebo simulation with ros2_control
- Joint trajectory control via `JointTrajectoryController`
- Modular structure (description and control layers separated)
- Controller testing via ROS 2 topics

## Build

```bash
colcon build
source install/setup.bash
```

## Usage

### RViz

```bash
ros2 launch mengu_description display_launch.py
```
### Gazebo + Control

#### Gazebo Launch 
```bash
ros2 launch mengu_description gazebo_launch.py
```

#### Controller Launch
```bash
ros2 launch mengu_controller controller.launch.py
```
verify controllers:
```bash
ros2 control list_controllers
```
expected controllers:

- arm_controller
- gripper_controller
- joint_state_broadcaster

#### Sending Joint Trajectories (Example)
Arm controller example:

```bash
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names:
- joint_1
- joint_2
- joint_3
points:
- positions: [0.3, -0.4, 0.2]
  time_from_start: {sec: 2}
"
```


Gripper controller example:

```bash
ros2 topic pub /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names:
- joint_4
points:
- positions: [-0.5]
  time_from_start: {sec: 2}
"
```

## Roadmap

- Extend arm from 3-DoF to 6-DoF

- Improve controller tuning

- Integrate motion planning (MoveIt)

- Higher-level autonomy and task execution
