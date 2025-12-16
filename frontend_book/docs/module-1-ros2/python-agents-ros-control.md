---
sidebar_position: 2
title: "Python Agents and ROS Control with rclpy"
---

# Python Agents and ROS Control with rclpy

## Learning Objectives

After completing this chapter, you will be able to:
- Interface Python agents with ROS 2 controllers
- Create action clients and servers using rclpy
- Implement a working example of a Python agent sending movement commands to a humanoid ROS controller
- Design an agent that reads sensor topics and triggers appropriate actions
- Understand the agent-to-ROS 2 control pipeline

## Introduction

Python agents play a crucial role in modern robotics, especially in AI-powered humanoid systems. Python's rich ecosystem of machine learning libraries, combined with ROS 2's communication infrastructure, enables the development of intelligent robotic behaviors.

In this chapter, we'll explore how Python agents can interface with ROS 2 controllers, focusing on the rclpy library - the Python client library for ROS 2. We'll build upon the communication fundamentals from Chapter 1 to create agents that can perceive their environment, make decisions, and execute actions on a humanoid robot.

## Python in Robotics

Python has become the language of choice for robotics research and development for several reasons:
- Rich ecosystem of scientific computing and ML libraries (NumPy, SciPy, TensorFlow, PyTorch)
- Ease of prototyping and experimentation
- Integration with ROS 2 through rclpy
- Strong community support in robotics and AI

## Agent Architecture

A typical Python agent for humanoid robot control consists of:

1. **Perception Component**: Processes sensor data from ROS 2 topics
2. **Decision/Planning Component**: Interprets sensor data and decides on actions
3. **Action Component**: Sends commands to the robot's actuators through ROS 2
4. **Communication Component**: Interfaces with ROS 2 for message passing

```
[External Sensors] -> [ROS 2 Topics] -> [Perception] -> [Decision Making] -> [Action Execution] -> [Robot Actuators]
```

## Working with rclpy

rclpy is the Python client library for ROS 2. It provides the interface between Python programs and the ROS 2 middleware.

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class RobotAgent(Node):
    def __init__(self):
        super().__init__('robot_agent')
        
        # Create subscribers for sensor data
        self.sensor_subscription = self.create_subscription(
            SensorMsgType,
            'sensor_topic',
            self.sensor_callback,
            qos_profile
        )
        
        # Create publishers for commands
        self.command_publisher = self.create_publisher(
            CommandMsgType,
            'command_topic',
            qos_profile
        )
        
        # Create action clients/servers as needed
        self.action_client = ActionClient(
            self,
            ActionType,
            'action_name'
        )

    def sensor_callback(self, msg):
        # Process sensor data and make decisions
        pass

def main(args=None):
    rclpy.init(args=args)
    agent = RobotAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Actions in ROS 2

Actions are an important part of the ROS 2 communication system, particularly for long-running tasks. They provide feedback and status updates, making them ideal for complex robotic behaviors.

### Action Client Example

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_interfaces.action import MoveToPose  # Custom action definition

class RobotActionClient(Node):

    def __init__(self):
        super().__init__('robot_action_client')
        self._action_client = ActionClient(
            self,
            MoveToPose,
            'move_to_pose'
        )

    def send_goal(self, target_pose):
        goal_msg = MoveToPose.Goal()
        goal_msg.target_pose = target_pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_pose}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = RobotActionClient()
    
    # Send a goal
    target_pose = create_pose(x=1.0, y=2.0, z=0.0)
    action_client.send_goal(target_pose)
    
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

### Action Server Example

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_interfaces.action import MoveToPose

class RobotActionServer(Node):

    def __init__(self):
        super().__init__('robot_action_server')
        self._action_server = ActionServer(
            self,
            MoveToPose,
            'move_to_pose',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Start executing the action
        feedback_msg = MoveToPose.Feedback()
        result = MoveToPose.Result()

        # Example: Move robot to target pose
        for i in range(0, 11):
            # Check if there was a cancel request
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return result

            # Update feedback
            feedback_msg.current_pose.x = goal_handle.request.target_pose.x * i / 10.0
            feedback_msg.current_pose.y = goal_handle.request.target_pose.y * i / 10.0
            goal_handle.publish_feedback(feedback_msg)
            
            # Sleep to simulate work
            time.sleep(0.5)

        # Upon successful completion
        result.success = True
        goal_handle.succeed()
        self.get_logger().info('Goal succeeded!')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    robot_action_server = RobotActionServer()
    rclpy.spin(robot_action_server)

if __name__ == '__main__':
    main()
```

## Complete Agent Example: Humanoid Movement Controller

Let's create a complete Python agent that demonstrates how to control a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from custom_interfaces.action import MoveToPose

import numpy as np
import time

class HumanoidAgent(Node):
    """
    A Python agent that controls a humanoid robot by:
    1. Reading sensor data (joint states)
    2. Making decisions based on target poses
    3. Sending movement commands via actions
    """
    
    def __init__(self):
        super().__init__('humanoid_agent')
        
        # Store current joint states
        self.current_joint_states = JointState()
        
        # Create subscribers for robot state
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Create publisher for direct velocity commands (emergency control)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create action client for complex movements
        self.move_to_pose_client = ActionClient(
            self,
            MoveToPose,
            'move_to_pose'
        )
        
        # Timer for decision making
        self.timer = self.create_timer(0.1, self.decision_loop)
        
        self.get_logger().info('Humanoid Agent initialized')

    def joint_state_callback(self, msg):
        """Update current joint state from robot"""
        self.current_joint_states = msg

    def decision_loop(self):
        """Main decision making loop"""
        # Example decision logic
        target_pose = self.calculate_target_pose()
        if target_pose is not None:
            self.execute_movement(target_pose)

    def calculate_target_pose(self):
        """
        Calculate target pose based on:
        - Current robot state
        - Environment (would come from sensors)
        - Task goals (would be input to agent)
        """
        # Simple example: move to a specific pose
        # In a real agent, this would be more sophisticated
        
        # Return a target pose (x, y, theta) or None if no action needed
        return (1.0, 2.0, 0.0)  # Example target

    def execute_movement(self, target_pose):
        """Execute a movement to the target pose"""
        goal_msg = MoveToPose.Goal()
        goal_msg.target_pose.x = target_pose[0]
        goal_msg.target_pose.y = target_pose[1]
        goal_msg.target_pose.theta = target_pose[2]

        if self.move_to_pose_client.wait_for_server(timeout_sec=1.0):
            send_goal_future = self.move_to_pose_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback)
            
            send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().error('Action server not available')

    def feedback_callback(self, feedback_msg):
        """Handle movement feedback"""
        self.get_logger().info(f'Moving to pose: {feedback_msg.feedback.current_pose}')

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle movement result"""
        result = future.result().result
        if result.success:
            self.get_logger().info('Movement completed successfully')
        else:
            self.get_logger().error('Movement failed')

def main(args=None):
    rclpy.init(args=args)
    agent = HumanoidAgent()
    
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Agent Patterns

### State Machine Agent

For more complex behaviors, agents often implement state machines:

```python
from enum import Enum

class RobotState(Enum):
    IDLE = 1
    NAVIGATING = 2
    MANIPULATING = 3
    EMERGENCY_STOP = 4

class StateMachineAgent(HumanoidAgent):
    def __init__(self):
        super().__init__()
        self.current_state = RobotState.IDLE
        self.state_timer = self.create_timer(0.05, self.state_machine_loop)
        
    def state_machine_loop(self):
        if self.current_state == RobotState.IDLE:
            self.handle_idle_state()
        elif self.current_state == RobotState.NAVIGATING:
            self.handle_navigation_state()
        elif self.current_state == RobotState.MANIPULATING:
            self.handle_manipulation_state()
        elif self.current_state == RobotState.EMERGENCY_STOP:
            self.handle_emergency_state()
            
    def handle_idle_state(self):
        # Check if a new task is available
        if self.has_task():
            self.current_state = RobotState.NAVIGATING
            
    def handle_navigation_state(self):
        # Execute navigation task
        if self.reached_destination():
            self.current_state = RobotState.MANIPULATING
```

## Diagram: Agent-to-ROS 2 Control Pipeline

```
                    Agent-to-ROS 2 Control Pipeline

    Environmental     +--------------+    Robot Commands
    Sensors           |              |    (Actions, Topics)
        |             |  Python      |        |
        v             |  Agent       |        v
    [Sensor Data] --> |  Perception  | --> [Robot Actuators]
                      |  + Decision  |    (Motors, etc.)
                      |  + Action    |
                      +--------------+
                            |
                      [ROS 2 Middleware]
                            |
                 +--------------------------+
                 | Topics, Services, Actions|
                 +--------------------------+

```

## AI Integration

Python agents can easily integrate with AI models:

```python
import tensorflow as tf  # or PyTorch
import numpy as np

class AIControlledAgent(HumanoidAgent):
    def __init__(self):
        super().__init__()
        
        # Load AI model
        self.model = tf.keras.models.load_model('robot_control_model.h5')
        
    def decision_loop(self):
        # Get sensor data
        sensor_data = self.get_sensor_input()
        
        # Process with AI model
        action = self.model.predict(sensor_data)
        
        # Execute action
        self.execute_action(action)
        
    def get_sensor_input(self):
        # Prepare sensor data for the model
        # This would typically combine multiple sensor streams
        return np.array([self.current_joint_states.position])
```

## Summary

Python agents with rclpy provide a powerful way to implement intelligent behaviors for humanoid robots. By combining ROS 2's communication infrastructure with Python's rich ecosystem of libraries, we can create sophisticated agents that perceive, decide, and act in complex environments.

The key components are:
- **Perception**: Subscribing to sensor topics
- **Decision Making**: AI or rule-based processing
- **Action**: Publishing to command topics or calling actions
- **Communication**: Integration with ROS 2 middleware

## Exercises

### Exercise 1: Extend the Agent
Modify the HumanoidAgent to read sensor topics (e.g., laser scan data) and use this information to make navigation decisions.

### Exercise 2: Implement a Behavior Tree
Create a more sophisticated agent that uses a behavior tree to decide between different behaviors (e.g., navigate, avoid obstacles, manipulate objects).

### Exercise 3: AI Integration
Create a simple neural network using TensorFlow that takes joint states as input and outputs desired joint velocities.

## Next Steps

In the next chapter, we'll explore URDF (Unified Robot Description Format), which is essential for representing robot geometry and kinematics in ROS 2. This allows agents to understand the physical structure of the humanoid robot they're controlling.