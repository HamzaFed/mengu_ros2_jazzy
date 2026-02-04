# Mengu ROS 2 Jazzy

Mengu is a ROS 2 (Jazzy) robot project.
This repository currently focuses on the robot description layer.

The goal of the project is to evolve from a simple pan-tilt mechanism
into a more advanced robotic system (e.g. a 6-DoF manipulator) over time.

## Packages

### mengu_description
- Robot URDF/Xacro description
- Mesh files
- RViz visualization
- Gazebo simulation (gravity enabled, no controllers yet)

## Features
- RViz visualization using robot_state_publisher
- Gazebo simulation without ros2_control (description-only stage)
- Clean separation between description and future control packages

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
### Gazebo

```bash
ros2 launch mengu_description gazebo_launch.py
```
## Roadmap

- Add ros2_control integration
- Introduce controller configuration
- Upgrade robot model to 6-DoF
- Motion planning and higher-level autonomy