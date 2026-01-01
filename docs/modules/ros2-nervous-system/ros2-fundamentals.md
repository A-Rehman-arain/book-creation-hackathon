---
sidebar_position: 2
---

# ROS 2 Fundamentals: Nodes, Topics, Services, and Actions

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the core concepts of ROS 2: nodes, topics, services, and actions
- Understand how these concepts enable communication in robotic systems
- Identify nodes, topics, services, and actions in a ROS 2 system diagram
- Create a simple ROS 2 node that publishes messages to a topic

## Introduction

ROS 2 (Robot Operating System 2) is not an operating system but rather a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

At the heart of ROS 2 are several key concepts that enable distributed computation and communication between different parts of a robotic system. Understanding these concepts is crucial for connecting AI agents to robot hardware effectively.

## Nodes

A **node** is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. Each node is designed to perform a specific task, such as sensor data processing, motion planning, or hardware control.

### Key Characteristics of Nodes:
- Each node runs in its own process
- Nodes communicate with each other through topics, services, and actions
- Multiple nodes can run on the same device or across multiple devices
- Nodes can be written in different programming languages (C++, Python, etc.)

### Example Node Structure (Python):
```python
import rclpy
from rclpy.node import Node

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

## Topics

**Topics** enable asynchronous, many-to-many communication between nodes through a publish-subscribe model. Nodes publish messages to topics, and other nodes subscribe to those topics to receive the messages.

### Key Characteristics of Topics:
- Asynchronous communication
- Many-to-many: multiple nodes can publish to the same topic, and multiple nodes can subscribe to the same topic
- Data flows from publishers to subscribers
- Uses Quality of Service (QoS) settings for reliability and performance

### Example Publisher-Subscriber Pattern:
```python
# Publisher
publisher = node.create_publisher(String, 'topic_name', 10)

# Subscriber
subscriber = node.create_subscription(
    String,
    'topic_name',
    callback_function,
    10
)
```

## Services

**Services** provide synchronous, request-response communication between nodes. A service client sends a request to a service server, which processes the request and sends back a response.

### Key Characteristics of Services:
- Synchronous communication
- One-to-one: one client requests, one server responds
- Request-response pattern
- Blocking: the client waits for the response

### Example Service Client-Server:
```python
# Service Server
from example_interfaces.srv import AddTwoInts

def add_two_ints_callback(request, response):
    response.sum = request.a + request.b
    return response

service = node.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)

# Service Client
client = node.create_client(AddTwoInts, 'add_two_ints')
```

## Actions

**Actions** provide asynchronous communication for long-running tasks with feedback. They combine the features of services and topics, allowing for goals to be sent, feedback to be received during execution, and results to be returned upon completion.

### Key Characteristics of Actions:
- Asynchronous communication
- Designed for long-running tasks
- Provides feedback during execution
- Supports goal preemption (canceling ongoing tasks)
- Goal-request-result-feedback pattern

### Example Action Client-Server:
```python
# Action Server
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Connecting AI Agents to Robot Hardware

The combination of nodes, topics, services, and actions enables AI agents to interact with robot hardware in various ways:

- **Nodes** can encapsulate AI algorithms, sensor processing, or hardware control
- **Topics** allow AI agents to receive sensor data and publish commands
- **Services** enable AI agents to request specific operations from robot systems
- **Actions** allow AI agents to initiate long-running tasks like navigation or manipulation

## Summary

Understanding these fundamental ROS 2 concepts is essential for connecting AI agents to humanoid robot bodies. Each concept serves a specific purpose in the communication architecture:

- Use **nodes** to organize your system into logical components
- Use **topics** for streaming data like sensor readings or continuous commands
- Use **services** for discrete operations that require a response
- Use **actions** for complex tasks that take time and may provide feedback

These concepts form the "nervous system" of your robot, enabling the flow of information between AI decision-making and physical robot control.