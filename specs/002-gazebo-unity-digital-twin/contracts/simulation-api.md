# API Contract: Digital Twin Simulation System

## Overview
This document defines the ROS 2 interfaces for the Digital Twin simulation system, including topics, services, and actions used in the Gazebo-Unity integration.

## Message Definitions

### Sensor Data Topics

#### `/lidar_scan` - LiDAR sensor data
- **Type**: `sensor_msgs/msg/LaserScan`
- **Description**: Range data from simulated LiDAR sensor
- **Publisher**: Gazebo simulation
- **Subscribers**: Processing nodes, visualization tools
- **Frequency**: 10 Hz

#### `/depth_camera/image_raw` - Raw depth image data
- **Type**: `sensor_msgs/msg/Image`
- **Description**: Raw image data from simulated depth camera
- **Publisher**: Gazebo simulation
- **Subscribers**: Processing nodes, visualization tools
- **Frequency**: 30 Hz

#### `/imu_data` - IMU sensor data
- **Type**: `sensor_msgs/msg/Imu`
- **Description**: Acceleration, angular velocity, and orientation data from simulated IMU
- **Publisher**: Gazebo simulation
- **Subscribers**: Processing nodes, state estimation
- **Frequency**: 100 Hz

### Robot State Topics

#### `/robot_pose` - Robot's position and orientation
- **Type**: `geometry_msgs/msg/PoseStamped`
- **Description**: Current pose of the robot in the simulation world
- **Publisher**: Simulation state publisher
- **Subscribers**: Visualization tools, path planners
- **Frequency**: 50 Hz

#### `/robot_joint_states` - Robot joint positions
- **Type**: `sensor_msgs/msg/JointState`
- **Description**: Current positions, velocities, and efforts of robot joints
- **Publisher**: Simulation state publisher
- **Subscribers**: Controllers, visualization tools
- **Frequency**: 100 Hz

## Service Definitions

#### `/spawn_humanoid_robot` - Spawn a humanoid robot in simulation
- **Type**: `gazebo_msgs/srv/SpawnEntity`
- **Request**: Robot model description, pose, name
- **Response**: Spawn status, error message if failed
- **Description**: Service to dynamically spawn humanoid robot models in the simulation

#### `/reset_simulation` - Reset simulation to initial state
- **Type**: `std_srvs/srv/Empty`
- **Request**: Empty
- **Response**: Empty
- **Description**: Service to reset the simulation environment to its initial state

## Action Definitions

#### `/move_to_goal` - Send robot to a specific goal location
- **Type**: `nav2_msgs/action/NavigateToPose` (or custom action)
- **Goal**: Target pose for the robot to navigate to
- **Feedback**: Current progress toward goal
- **Result**: Success/failure of navigation attempt
- **Description**: Action client/server for sending navigation goals to the robot

## Quality of Service (QoS) Settings

For sensor data topics, the following QoS settings are recommended:
- **Reliability**: Best effort (for camera/point cloud data) or Reliable (for critical sensors)
- **Durability**: Volatile
- **History**: Keep last N samples (N varies by sensor type)

## Validation Requirements

All implementations must:
1. Publish messages at the specified frequencies
2. Use the correct message types as defined in ROS 2 standard message packages
3. Follow ROS 2 naming conventions for topics and services
4. Properly handle message serialization and deserialization
5. Include appropriate timestamps and frame IDs in all messages