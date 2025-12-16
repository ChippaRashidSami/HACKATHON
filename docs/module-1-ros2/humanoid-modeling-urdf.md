---
sidebar_position: 3
title: "Humanoid Modeling with URDF"
---

# Humanoid Modeling with URDF

## Learning Objectives

After completing this chapter, you will be able to:
- Explain the structure and purpose of URDF for humanoid kinematics
- Define robot links, joints, and sensors in URDF
- Create a minimal humanoid URDF file with annotations
- Visualize URDF in RViz
- Modify URDF files to add limbs, adjust joint limits, and change physical properties

## Introduction

URDF (Unified Robot Description Format) is XML-based format used in ROS to describe robots. For humanoid robots, URDF is essential for defining the robot's physical structure, kinematic properties, and visualization characteristics. Understanding URDF is crucial for simulation, visualization, and control of humanoid robots in ROS 2.

In this chapter, we'll explore how URDF represents the kinematic and physical properties of humanoid robots, allowing ROS tools to understand the robot's structure and simulate its behavior.

## What is URDF?

URDF stands for Unified Robot Description Format. It's an XML format that describes robot models in terms of:
- **Links**: Rigid parts of the robot (e.g., limbs, body, head)
- **Joints**: Connections between links (e.g., hinges, prismatic joints, fixed joints)
- **Visual**: Visual representation for simulation and visualization
- **Collision**: Collision properties for physics simulation
- **Inertial**: Mass, center of mass, and inertia properties for dynamics

### URDF Structure

A URDF file describes a robot as a tree of links connected by joints. Each robot must have one "base" or "root" link with no parent, and all other links connect to it through joints.

```
Root Link (base_link)
├── Joint: base_to_torso
│   └── Link: torso
│       ├── Joint: torso_to_head
│       │   └── Link: head
│       ├── Joint: torso_to_left_arm
│       │   └── Link: left_upper_arm
│       │       └── Joint: left_elbow
│       │           └── Link: left_lower_arm
│       └── Joint: torso_to_right_arm
│           └── Link: right_upper_arm
│               └── Joint: right_elbow
│                   └── Link: right_lower_arm
```

## Basic URDF Elements

### Links

A link represents a rigid part of the robot. It contains:
- Inertial properties
- Visual representation
- Collision properties

```xml
<link name="link_name">
  <inertial>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <mass value="1"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder length="1" radius="0.1"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder length="1" radius="0.1"/>
    </geometry>
  </collision>
</link>
```

### Joints

A joint connects two links. It defines:
- Parent and child links
- Joint type
- Joint limits and properties
- Transform from parent to child

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-2.0" upper="1.0" effort="30" velocity="1.0"/>
</joint>
```

Joint types:
- `revolute`: Rotational joint with limits
- `continuous`: Rotational joint without limits
- `prismatic`: Linear sliding joint
- `fixed`: No movement
- `floating`: 6 DOF
- `planar`: Motion on a plane

## Minimal Humanoid URDF Example

Here's a minimal humanoid URDF with annotations:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- 
    Base link - the root of our robot tree.
    All other links will connect to this one.
  -->
  <link name="base_link">
    <inertial>
      <!-- Mass of 10kg -->
      <mass value="10"/>
      <!-- Center of mass at the origin -->
      <origin xyz="0 0 0"/>
      <!-- Moment of inertia (simplified) -->
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <!-- Visual representation of the body -->
      <origin xyz="0 0 0"/>
      <geometry>
        <!-- Torso shape as a cylinder -->
        <cylinder radius="0.15" length="0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <!-- Collision geometry for physics simulation -->
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder radius="0.15" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Head connected to torso -->
  <link name="head">
    <inertial>
      <mass value="2"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Neck joint connecting torso to head -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.35"/> <!-- Connect head 0.35m above torso -->
    <axis xyz="0 1 0"/> <!-- Rotation about Y axis (pitch) -->
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/> <!-- Joint limits -->
  </joint>

  <!-- Left upper arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Shoulder joint connecting torso to left arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.05 0.15 0.2"/> <!-- Position of shoulder -->
    <axis xyz="1 0 0"/> <!-- Rotation about X axis (shoulder raising) -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Left lower arm -->
  <link name="left_lower_arm">
    <inertial>
      <mass value="1"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Elbow joint connecting upper and lower arm -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3"/> <!-- Connect to end of upper arm -->
    <axis xyz="1 0 0"/> <!-- Rotation about X axis (elbow bending) -->
    <limit lower="0" upper="2.5" effort="50" velocity="1"/>
  </joint>

  <!-- Right arm (similar to left, mirrored) -->
  <link name="right_upper_arm">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.05 -0.15 0.2"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <inertial>
      <mass value="1"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.5" effort="50" velocity="1"/>
  </joint>

  <!-- Left leg -->
  <link name="left_upper_leg">
    <inertial>
      <mass value="3"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="-0.05 0.1 -0.25"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="200" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.5"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.5" effort="150" velocity="1"/>
  </joint>
</robot>
```

## URDF Xacro

Xacro (XML Macros) is an XML macro language that allows URDF files to be more concise and easier to maintain. It enables:
- Variable definitions
- Mathematical expressions
- Macros
- File inclusion

Here's the same robot using Xacro:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_humanoid_xacro">

  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_height" value="0.5" />
  <xacro:property name="torso_radius" value="0.15" />
  <xacro:property name="upper_arm_length" value="0.3" />
  <xacro:property name="lower_arm_length" value="0.3" />
  <xacro:property name="upper_leg_length" value="0.5" />
  <xacro:property name="lower_leg_length" value="0.5" />

  <!-- Macro for creating an arm -->
  <xacro:macro name="arm" params="side prefix parent xyz origin_rpy axis *geometry">
    <link name="${side}_${prefix}_upper_arm">
      <inertial>
        <mass value="1.5"/>
        <origin xyz="0 0 -${upper_arm_length/2}"/>
        <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -${upper_arm_length/2}" rpy="${origin_rpy}"/>
        <geometry>
          ${geometry}
        </geometry>
        <material name="red">
          <color rgba="1 0 0 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 -${upper_arm_length/2}" rpy="${origin_rpy}"/>
        <geometry>
          ${geometry}
        </geometry>
      </collision>
    </link>

    <joint name="${side}_${prefix}_shoulder_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${side}_${prefix}_upper_arm"/>
      <origin xyz="${xyz}"/>
      <axis xyz="${axis}"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    </joint>

    <link name="${side}_${prefix}_lower_arm">
      <inertial>
        <mass value="1"/>
        <origin xyz="0 0 -${lower_arm_length/2}"/>
        <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.01"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -${lower_arm_length/2}"/>
        <geometry>
          ${geometry}
        </geometry>
        <material name="red">
          <color rgba="1 0 0 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 -${lower_arm_length/2}"/>
        <geometry>
          ${geometry}
        </geometry>
      </collision>
    </link>

    <joint name="${side}_${prefix}_elbow_joint" type="revolute">
      <parent link="${side}_${prefix}_upper_arm"/>
      <child link="${side}_${prefix}_lower_arm"/>
      <origin xyz="0 0 -${upper_arm_length}"/>
      <axis xyz="${axis}"/>
      <limit lower="0" upper="2.5" effort="50" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder radius="${torso_radius}" length="${torso_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder radius="${torso_radius}" length="${torso_height}"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 ${torso_height/2 + 0.1}"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Create arms using the macro -->
  <xacro:arm 
    side="left" 
    prefix="upper" 
    parent="base_link" 
    xyz="0.05 0.15 ${torso_height/2 - 0.05}" 
    origin_rpy="0 0 0" 
    axis="1 0 0">
    <cylinder radius="0.05" length="${upper_arm_length}"/>
  </xacro:arm>

  <xacro:arm 
    side="right" 
    prefix="upper" 
    parent="base_link" 
    xyz="0.05 -0.15 ${torso_height/2 - 0.05}" 
    origin_rpy="0 0 0" 
    axis="1 0 0">
    <cylinder radius="0.05" length="${upper_arm_length}"/>
  </xacro:arm>

</robot>
```

## Visualizing URDF in RViz

To visualize your URDF in RViz:

1. **Launch the robot_state_publisher**:
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(xacro my_robot.urdf.xacro)'
   ```

2. **Launch RViz**:
   ```bash
   ros2 run rviz2 rviz2
   ```

3. **Add the RobotModel display**:
   - In RViz, go to "Panels" → "Displays"
   - Click "Add" and select "RobotModel"
   - Set the "Robot Description" to "/robot_description"

4. **To see the joint transformations**, also add a "TF" display:
   - Click "Add" and select "TF"
   - This will show the coordinate frames of each link

## Working with URDF in Python

You can load and manipulate URDF files in Python using the `urdf_parser_py` library:

```python
from urdf_parser_py.urdf import URDF
import os

# Load URDF file
robot = URDF.from_xml_file('path/to/your/robot.urdf')

# Access robot properties
print(f"Robot name: {robot.name}")
print(f"Number of links: {len(robot.links)}")
print(f"Number of joints: {len(robot.joints)}")

# Iterate through links
for link in robot.links:
    print(f"Link: {link.name}")
    if link.inertial:
        print(f"  Mass: {link.inertial.mass}")
    if link.visual:
        print(f"  Geometry type: {link.visual.geometry.type}")

# Find specific joint
for joint in robot.joints:
    if joint.name == 'left_elbow_joint':
        print(f"Found joint: {joint.name}")
        print(f"  Type: {joint.type}")
        print(f"  Limits: [{joint.limit.lower}, {joint.limit.upper}]")
```

## Diagram: Link/Joint Tree Structure

```
            Link/Joint Tree Structure of Humanoid Robot
                            
                           [base_link (torso)]
                                  |
                    +-------------+-------------+
                    |                           |
              [neck_joint]                [left_shoulder_joint]
                    |                           |
              [head]                      [left_upper_arm]
                                               |
                                      [left_elbow_joint]
                                               |
                                        [left_lower_arm]
                    |
            [right_shoulder_joint]
                    |
              [right_upper_arm]
                    |
              [right_elbow_joint]
                    |
                [right_lower_arm]
                    |
              [left_hip_joint]
                    |
              [left_upper_leg]
                    |
              [left_knee_joint]
                    |
              [left_lower_leg]

```

## Common URDF Issues and Solutions

1. **Missing parent links**: Every child link must have a parent that exists in the URDF
2. **Unnormalized quaternions**: In origin specifications, quaternions must be normalized
3. **Zero mass or inertia**: All links should have valid mass and inertia values
4. **Self-collisions**: Links connected by joints shouldn't collide with each other
5. **Inconsistent units**: Use meters for distances and radians for angles

## Advanced URDF Features

### Transmission Elements

For controlling joints with actuators:

```xml
<transmission name="left_elbow_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_elbow_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_elbow_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Gazebo-specific elements

For simulation in Gazebo:

```xml
<gazebo reference="left_upper_arm">
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <material>Gazebo/Red</material>
</gazebo>
```

## Summary

URDF (Unified Robot Description Format) is fundamental for representing robotic hardware in ROS. For humanoid robots, it defines the kinematic structure, physical properties, and visualization characteristics. Understanding URDF is essential for:

- Simulation in tools like Gazebo
- Visualization in RViz
- Kinematic computation and motion planning
- Robot control algorithms

A well-structured URDF file contains links connected by joints, each with proper inertial, visual, and collision properties. Using Xacro can make complex URDF files more maintainable and easier to work with.

## Exercises

### Exercise 1: Add a Limb
Modify the minimal humanoid URDF to add a right leg, following the same pattern as the left leg.

### Exercise 2: Joint Limits
Adjust the joint limits in the URDF to reflect realistic human joint ranges of motion.

### Exercise 3: Visual Properties
Create a URDF file with multiple visual materials and geometric shapes to represent a more detailed humanoid robot.

## Next Steps

In the next module, we'll explore how these robot models are used in simulation environments with Gazebo and Unity. Understanding URDF is essential for creating realistic simulations of humanoid robots, where the physical properties defined in URDF determine how the robot interacts with the virtual world.