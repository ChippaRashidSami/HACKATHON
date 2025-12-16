---
sidebar_position: 1
title: "Physics Simulation with Gazebo"
---

# Physics Simulation with Gazebo

## Learning Objectives

After completing this chapter, you will be able to:
- Understand Gazebo's physics engine and how it models real-world physics
- Create world files that define simulation environments
- Configure gravity and collision properties for humanoid simulations
- Spawn a humanoid robot model in Gazebo simulation
- Understand the relationship between URDF and Gazebo simulation parameters

## Introduction

Gazebo is a powerful physics simulation environment that is widely used in robotics research and development. It provides accurate physics simulation, high-quality rendering, and realistic sensor simulation, making it an ideal environment for testing and validating humanoid robot behaviors before deploying to real hardware.

In this chapter, we'll explore Gazebo fundamentals, focusing on how physics is modeled in simulation and how to create simulation environments for humanoid robots.

## What is Gazebo?

Gazebo is a 3D simulation environment that provides:
- **Physics Simulation**: Accurate modeling of real-world physics with multiple physics engines (ODE, Bullet, Simbody)
- **Sensor Simulation**: Realistic simulation of cameras, LiDAR, IMUs, and other sensors
- **Rendering**: High-quality 3D rendering of environments and robots
- **Plugins**: Extensible architecture for custom simulation elements
- **ROS Integration**: Seamless integration with ROS/ROS 2 for robot simulation

### Why Use Simulation?

Before deploying to real robots, simulation provides crucial advantages:
- **Safety**: Test behaviors without risk of robot damage
- **Cost**: No wear and tear on physical hardware
- **Speed**: Run simulations faster than real-time
- **Scenarios**: Test in dangerous or hard-to-reach environments
- **Repeatability**: Conduct controlled experiments with identical conditions

## Gazebo World Files

A Gazebo world file is an SDF (Simulation Description Format) file that defines the environment for simulation. It includes elements like:
- World properties (gravity, magnetic field)
- Models (robots, objects, obstacles)
- Lighting
- Physics engine parameters

### Basic World Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- World properties -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>
    
    <!-- Models in the world -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Static objects -->
    <model name="table">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.5 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.5 0.05</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
    
    <!-- Humanoid robot model -->
    <include>
      <uri>model://my_humanoid_robot</uri>
    </include>
    
  </world>
</sdf>
```

### Physics Engine Configuration

Gazebo supports multiple physics engines, each with its own strengths:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.0</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Gravity Configuration

Gravity can be modified to simulate different environments:

```xml
<!-- Earth's gravity -->
<gravity>0 0 -9.8</gravity>

<!-- Moon's gravity (sixth of Earth's) -->
<gravity>0 0 -1.63</gravity>

<!-- Zero gravity (space simulation) -->
<gravity>0 0 0</gravity>

<!-- Custom gravity vector -->
<gravity>-0.1 0 -9.8</gravity>  <!-- With a slight horizontal component -->
```

## Collision Models and Physics Properties

Accurate collision models are crucial for realistic simulation. Each link in your robot should have appropriate collision properties:

```xml
<link name="link_name">
  <!-- More detailed collision for physics simulation -->
  <collision name="collision">
    <geometry>
      <mesh>
        <uri>package://my_robot/meshes/link.dae</uri>
      </mesh>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
        </ode>
      </friction>
      <contact>
        <ode>
          <kp>1e+6</kp>  <!-- Contact stiffness -->
          <kd>1e+3</kd>  <!-- Contact damping -->
        </ode>
      </contact>
    </surface>
  </collision>
  
  <!-- Visual model (may be simpler than collision model) -->
  <visual name="visual">
    <geometry>
      <mesh>
        <uri>package://my_robot/meshes/link_visual.dae</uri>
      </mesh>
    </geometry>
  </visual>
</link>
```

### Surface Properties

Surface properties affect how objects interact physically:

```xml
<surface>
  <friction>
    <ode>
      <mu>0.5</mu>    <!-- Primary friction coefficient -->
      <mu2>0.5</mu2>  <!-- Secondary friction coefficient -->
      <fdir1>0 0 1</fdir1>  <!-- Friction direction (for anisotropic friction) -->
    </ode>
  </friction>
  <contact>
    <ode>
      <max_vel>100.0</max_vel>  <!-- Maximum contact correction velocity -->
      <min_depth>0.001</min_depth>  <!-- Penetration depth allowed -->
    </ode>
  </contact>
  <bounce>
    <restitution_coefficient>0.1</restitution_coefficient>  <!-- How bouncy -->
    <threshold>100000.0</threshold>  <!-- Velocity threshold for bouncing -->
  </bounce>
</surface>
```

## Spawning Humanoid Robots in Gazebo

To spawn a humanoid robot in Gazebo, you can use several approaches:

### Method 1: URDF Integration

The most common approach is to use your URDF file with Gazebo-specific extensions:

```xml
<!-- In your robot's URDF/XACRO file -->
<link name="left_foot">
  <visual>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
  
  <!-- Gazebo-specific extensions -->
  <gazebo reference="left_foot">
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/Blue</material>
  </gazebo>
</link>
```

### Method 2: Using ROS Services

You can spawn robots programmatically using ROS services:

```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SpawnEntity.Request()

    def send_request(self, robot_description, robot_name, robot_namespace, initial_pose):
        self.req.name = robot_name
        self.req.xml = robot_description
        self.req.robot_namespace = robot_namespace
        self.req.initial_pose = initial_pose
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    spawner = RobotSpawner()
    
    # Example: spawn a simple box robot
    robot_description = """
    <robot name="simple_box_robot">
      <link name="chassis">
        <visual>
          <geometry>
            <box size="1 0.5 0.2"/>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <box size="1 0.5 0.2"/>
          </geometry>
        </collision>
        <inertial>
          <mass value="10"/>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
      </link>
    </robot>
    """
    
    # Set initial pose
    from geometry_msgs.msg import Pose
    initial_pose = Pose()
    initial_pose.position.x = 0.0
    initial_pose.position.y = 0.0
    initial_pose.position.z = 0.5  # Half a meter above ground
    
    result = spawner.send_request(robot_description, "box_robot", "", initial_pose)
    spawner.get_logger().info(f'Spawn result: {result}')
    
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Physics Tuning for Humanoid Robots

Humanoid robots have special requirements for stable simulation:

### Balance and Stability

Humanoid robots are inherently unstable, so careful attention to physics parameters is needed:

```xml
<!-- In world file or model SDF -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Small step size for stability -->
  <real_time_factor>1.0</real_time_factor>  <!-- Real-time simulation -->
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>  <!-- More iterations for stability -->
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.00000001</cfm>  <!-- Constraint Force Mixing -->
      <erp>0.2</erp>  <!-- Error Reduction Parameter -->
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Joint Damping and Stiffness

Joints need appropriate damping to prevent oscillations:

```xml
<joint name="left_knee" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <axis xyz="1 0 0"/>
  <limit lower="-0.5" upper="2.0" effort="100" velocity="1"/>
  <dynamics damping="1.0" friction="0.1"/>
</joint>
```

## Gazebo Plugins for Humanoid Robots

Gazebo supports plugins that extend simulation capabilities. For humanoid robots, common plugins include:

### Joint Control Plugin

```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <ros>
      <namespace>/my_robot</namespace>
      <publish_rate>30</publish_rate>
    </ros>
    <joint_name>left_hip_joint</joint_name>
    <joint_name>left_knee_joint</joint_name>
    <!-- Add more joint names as needed -->
  </plugin>
</gazebo>
```

### Joint Position Plugin

```xml
<gazebo>
  <plugin name="left_leg_controller" filename="libgazebo_ros_joint_position.so">
    <command_topic>left_leg/command</command_topic>
    <state_topic>left_leg/state</state_topic>
    <joint_name>left_hip_joint</joint_name>
    <update_rate>100</update_rate>
    <robot_namespace>/my_robot</robot_namespace>
  </plugin>
</gazebo>
```

## Advanced Physics Concepts

### Multi-Body Dynamics

Humanoid robots involve complex interactions between multiple rigid bodies. Gazebo uses multi-body dynamics to handle these interactions:

- **Forward Dynamics**: Computing accelerations from forces and torques
- **Inverse Dynamics**: Computing required forces and torques for desired motion
- **Constraint Solving**: Handling joint constraints and contacts

### Contacts and Collisions

When humanoid robots interact with the environment, contact handling is critical:

```xml
<collision name="left_foot_collision">
  <geometry>
    <box size="0.2 0.1 0.02"/>
  </geometry>
  <surface>
    <contact>
      <ode>
        <kp>10000000</kp>  <!-- High stiffness for realistic foot contact -->
        <kd>1000</kd>      <!-- Damping to absorb impact -->
      </ode>
    </contact>
    <friction>
      <ode>
        <mu>1.0</mu>      <!-- High friction for good grip -->
        <mu2>1.0</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

## Diagram: Gazebo Physics Simulation Architecture

```
                   Gazebo Physics Simulation Architecture
                            
        [ROS 2 Nodes] ←→ [ROS 2/Gazebo Bridge] ←→ [Gazebo Simulation]
              |                    |                        |
         [Robot Control]      [Topic/Service]         [Physics Engine]
         [AI Agents]          [Communication]         [Sensor Simulation]
              |                    |                        |
        [Perception Data] ←→ [Messages] ←→ [Visual Rendering]
```

## Common Issues and Solutions

### Robot Falling Through Ground
- Check if ground plane exists in world file
- Verify collision properties of both robot and ground
- Increase physics solver iterations

### Robot Jittering or Vibrating
- Increase joint damping values
- Use smaller physics step size
- Increase constraint solver iterations

### Unstable Balance
- Check center of mass in URDF
- Adjust controller gains
- Fine-tune contact parameters

## Summary

Gazebo provides a robust physics simulation environment essential for humanoid robot development. Understanding physics simulation concepts like gravity, collision models, and surface properties is crucial for creating realistic robot simulations.

The key components are:
- **World Files**: Define the simulation environment
- **Physics Configuration**: Tune parameters for realistic behavior
- **Collision Models**: Ensure accurate physical interactions
- **ROS Integration**: Connect simulation with your robot code

## Exercises

### Exercise 1: Create a Simple World
Create a Gazebo world file with a ground plane, a box obstacle, and appropriate physics settings.

### Exercise 2: Spawn Humanoid Model
Create a launch file that loads your humanoid URDF into Gazebo with appropriate collision properties and surface materials.

### Exercise 3: Physics Tuning
Experiment with different physics parameters (step size, solver iterations, contact properties) and observe their effects on robot stability.

## Next Steps

In the next chapter, we'll explore how to simulate sensors like LiDAR and IMUs in Gazebo and how to visualize this sensor data in RViz. Understanding sensor simulation is crucial for developing perception and navigation algorithms for humanoid robots.