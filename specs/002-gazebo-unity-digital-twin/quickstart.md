# Quickstart: Gazebo Unity Digital Twin Module

## Overview
This guide will help you set up the environment to work with the Digital Twin module covering Gazebo and Unity integration with ROS 2.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Gazebo Classic (or Garden) installed
- Unity Hub and Unity 2022.3 LTS (or compatible version)
- Git for version control

## Environment Setup

### 1. Install ROS 2 Humble
```bash
# Setup locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep2

# Install colcon and rosdep
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

### 2. Install Gazebo
```bash
# Install Gazebo Classic
sudo apt install ros-humble-gazebo-*

# Or install Ignition Gazebo (alternative)
sudo apt install ignition-harmonic
```

### 3. Set up workspace
```bash
# Create workspace
mkdir -p ~/digital-twin-workspace/src
cd ~/digital-twin-workspace

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build
source install/setup.bash
```

## Running the Examples

### Example 1: Basic Gazebo Simulation
```bash
# Launch the basic Gazebo simulation
cd ~/digital-twin-workspace
source install/setup.bash
ros2 launch digital_twin_examples basic_simulation.launch.py
```

### Example 2: LiDAR Sensor Simulation
```bash
# Launch the LiDAR simulation
cd ~/digital-twin-workspace
source install/setup.bash
ros2 launch digital_twin_examples lidar_simulation.launch.py
```

## Visualizing Data

### With RViz
```bash
# Launch RViz to visualize sensor data
source install/setup.bash
ros2 run rviz2 rviz2
```

### With Unity
1. Open the Unity project in the `src/unity-examples/` directory
2. Configure the ROS# bridge settings to connect to your ROS 2 network
3. Run the Unity scene to visualize the robot in a high-fidelity environment

## Testing Your Setup

After completing the setup, test with these commands:

```bash
# Check available ROS 2 topics
source install/setup.bash
ros2 topic list

# Run example tests
cd ~/digital-twin-workspace
source install/setup.bash
colcon test
colcon test-result --all
```

## Gazebo Fundamentals Examples

To get started with the Gazebo fundamentals examples from Chapter 1:

1. Make sure Gazebo is properly installed and integrated with ROS 2:
```bash
gzserver --version
gzclient --version
```

2. Launch the basic humanoid spawn example:
```bash
cd ~/digital-twin-workspace
source install/setup.bash
roslaunch digital_twin_examples basic-humanoid-spawn.launch
```

3. Try the detailed physics example to experiment with different parameters:
```bash
cd ~/digital-twin-workspace
source install/setup.bash
roslaunch digital_twin_examples detailed-physics-example.launch
```

## Troubleshooting

### Gazebo won't start
- Make sure you have a graphical environment
- If using a remote connection, ensure X11 forwarding is enabled

### ROS 2 nodes can't communicate
- Check that both terminals have sourced the ROS 2 environment
- Verify that `ROS_DOMAIN_ID` is the same in all terminals

### Unity can't connect to ROS 2
- Ensure the ROS# bridge is properly configured
- Check that the IP addresses and ports match between ROS 2 and Unity