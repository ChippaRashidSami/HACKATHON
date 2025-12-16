---
sidebar_position: 1
title: "ROS 2 Foundations: Nodes, Topics, and Services"
---

# ROS 2 Foundations: Nodes, Topics, and Services

## Learning Objectives

After completing this chapter, you will be able to:
- Explain the role of ROS 2 as the robotic nervous system
- Understand node-based communication patterns
- Describe how Topics and Services enable communication between robot components
- Create a simple publisher and subscriber pair
- Implement a basic service client and server

## Introduction

Robot Operating System 2 (ROS 2) serves as the middleware enabling communication, control, and embodiment of humanoid robots. Think of ROS 2 as the "robotic nervous system" - it allows different software components of a robot to communicate with each other in a structured and reliable way.

Just as our nervous system enables different parts of our body to communicate and coordinate actions, ROS 2 provides the infrastructure for robot systems to share information, coordinate complex behaviors, and respond to environmental stimuli.

## The ROS 2 Architecture

ROS 2 is built around a distributed architecture where different software components (called "nodes") communicate with each other. The fundamental concepts include:

1. **Nodes**: Independent processes that perform computation
2. **Topics**: Named buses over which nodes exchange messages
3. **Services**: Synchronous request/response communication between nodes
4. **Actions**: Asynchronous goal-oriented communication with feedback

## Nodes: The Building Blocks

Nodes are processes that perform computation. In a humanoid robot, you might have nodes for:
- Sensor processing (camera, IMU, LiDAR)
- Motor control
- Path planning
- Behavior management

Nodes can be written in different programming languages (C++, Python, etc.) and communicate seamlessly through ROS 2's middleware.

### Creating Your First Node

Here's a simple ROS 2 node in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):

    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from minimal node!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: Asynchronous Communication

Topics enable asynchronous, many-to-many communication using a publish-subscribe model. Publishers send messages to a topic, and any number of subscribers can receive those messages.

This pattern is ideal for:
- Sensor data distribution (e.g., camera images to multiple processing nodes)
- Robot state broadcasting
- Event notifications

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services: Synchronous Communication

Services provide synchronous request/response communication. This is useful when you need a guaranteed response or when performing an action with a clear outcome.

A service has:
- A client that sends a request
- A server that processes the request and returns a response

### Service Server Example

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
from add_two_ints_srv.srv import AddTwoInts
import sys
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' % 
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Communication Patterns and Best Practices

### Node Design
- Keep nodes focused and modular
- Each node should have a single, well-defined purpose
- Use parameters to make nodes configurable

### Topic Design
- Use descriptive names that follow ROS naming conventions (/namespace/name)
- Consider message frequency and bandwidth requirements
- Use appropriate Quality of Service (QoS) settings

### Service Design
- Use services for operations that have a clear, synchronous result
- Consider response time requirements
- Design error responses appropriately

## Diagram: Node Communication

```
            ROS 2 Communication Architecture

                   [Node A] Publisher
                       |
                       | Publishes to "sensor_data"
                       |
            [Message Bus - Topic: sensor_data]
                       |
                       | Subscribes to "sensor_data"
                       |
            [Node B] Subscriber    [Node C] Subscriber
                   |                       |
                   | Requests service       | Requests service
                   |                       |
            [Service Server Node]    [Action Server Node]

```

## Summary

ROS 2 provides the fundamental communication infrastructure for humanoid robots. Understanding nodes, topics, and services is essential for building complex robotic systems that can coordinate multiple components effectively.

The publish-subscribe model (topics) enables efficient data distribution across the robot's systems, while the request-response model (services) provides synchronous interactions for operations requiring guaranteed responses.

## Exercises

### Exercise 1: Publisher/Subscriber Pair
Create a publisher that sends temperature readings (as a Float64 message) at 1 Hz, and a subscriber that logs these values to the console.

### Exercise 2: Custom Service
Create a service that takes two robot joint angles and returns the resulting end-effector position (simplified forward kinematics).

### Exercise 3: Message Customization
Modify the publisher example to send custom messages containing both a timestamp and a counter value.

## Next Steps

In the next chapter, we'll explore how Python agents interface with ROS 2 controllers, demonstrating how AI agents can control robot behavior through the ROS 2 communication infrastructure.