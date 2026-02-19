# Mengu ROS 2 Jazzy

Mengu is a ROS 2 (Jazzy) robotic arm project.

The project evolved from a simple pan-tilt mechanism into a fully modeled
**6-DoF robotic manipulator** with ros2_control integration and motion planning support.

## Packages

### mengu_description
- Robot URDF/Xacro description
- Mesh files
- RViz visualization
- Gazebo simulation

### mengu_controller
- ros2_control integration
- Controller configurations (arm)
- Joint trajectory control support
- Gazebo-based controller testing

### mengu_msgs
- Custom ROS 2 interfaces

### mengu_utils
- - Euler ↔ Quaternion conversion service


## Features

- 6DoF robotic arm model
- ros2_control position-based control
- JointTrajectoryController (action-based control)
- MoveIt motion planning support
- Gazebo simulation
- Euler ↔ Quaternion conversion service
- Modular multi-package architecture

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
- joint_state_broadcaster

### Angle Conversion

- Start The Service
```bash
ros2 run mengu_utils angle_conversion 
```
- You should not see the "Açı Dönüşüm Servisleri Hazır" message in the terminal. You can apply the following codes to send the values ​​you want to convert

#### Euler to Quaternion

```bash
ros2 service call /euler_to_quaternion mengu_msgs/srv/EulerToQuaternion "roll: 0.0
pitch: 0.0
yaw: 0.0" 
```

#### Quaternion to Euler

```bash
ros2 service call /quaternion_to_euler mengu_msgs/srv/QuaternionToEuler "x: 0.0
y: 0.0
z: 0.0
w: 0.0 
```

## Roadmap
- Implement custom inverse kinematics solver

- Improve trajectory smoothing and tuning

- Hardware integration (STM / real motors)

- Task-level control and autonomy