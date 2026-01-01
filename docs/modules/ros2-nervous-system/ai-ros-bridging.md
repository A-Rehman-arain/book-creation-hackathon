---
sidebar_position: 3
---

# Bridging Python AI Agents to Robot Controllers with rclpy

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand how to integrate Python-based AI agents with ROS 2 using rclpy
- Create Python scripts that bridge AI decision-making to robot controllers
- Implement communication between high-level AI systems and low-level robot control
- Send commands from AI components to robot controllers using ROS 2 patterns

## Introduction

The bridge between AI agents and robot controllers is a critical component in Physical AI and humanoid robotics. AI agents make high-level decisions based on perception data, learning algorithms, and planning, while robot controllers handle low-level actuation and precise motor control. This chapter explores how to create this bridge using Python and rclpy.

## Understanding the AI-ROS Bridge Architecture

The AI-ROS bridge operates at the intersection of high-level decision making and low-level robot control. The architecture typically involves:

1. **AI Agent Layer**: High-level decision making, planning, learning algorithms
2. **Bridge Layer**: Translation and communication between AI and ROS
3. **ROS Layer**: Standard ROS 2 communication patterns
4. **Robot Controller Layer**: Low-level hardware control and actuation

## Setting Up the Bridge with rclpy

rclpy is the Python client library for ROS 2, providing the interface between Python code and the ROS 2 ecosystem. Here's how to establish the basic bridge:

### Basic Bridge Node Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')

        # Publishers for sending commands to robot
        self.command_publisher = self.create_publisher(
            JointTrajectoryControllerState,
            'robot_joint_commands',
            10
        )

        # Subscribers for receiving sensor data
        self.sensor_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.sensor_callback,
            10
        )

        # Timer for AI decision making loop
        self.ai_timer = self.create_timer(0.1, self.ai_decision_loop)

        self.get_logger().info('AI Bridge Node initialized')

    def sensor_callback(self, msg):
        """Process sensor data from the robot"""
        # Store sensor data for AI processing
        self.current_sensor_data = msg

    def ai_decision_loop(self):
        """Main AI decision making loop"""
        if hasattr(self, 'current_sensor_data'):
            # Process sensor data through AI agent
            ai_decision = self.run_ai_agent(self.current_sensor_data)

            # Convert AI decision to ROS command
            ros_command = self.convert_decision_to_command(ai_decision)

            # Publish command to robot
            self.command_publisher.publish(ros_command)

    def run_ai_agent(self, sensor_data):
        """Placeholder for AI agent logic"""
        # This would contain your actual AI algorithm
        # For example: neural network inference, planning algorithm, etc.
        return {"action": "move_joint", "joint_name": "arm_joint", "position": 0.5}

    def convert_decision_to_command(self, ai_decision):
        """Convert AI decision to ROS message format"""
        command = JointTrajectoryControllerState()
        command.joint_names = [ai_decision["joint_name"]]
        command.desired.positions = [ai_decision["position"]]
        return command

def main(args=None):
    rclpy.init(args=args)
    ai_bridge = AIBridgeNode()

    try:
        rclpy.spin(ai_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        ai_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration Patterns

### 1. Perception-Action Loop

The most common pattern in AI-ROS integration is the perception-action loop, where the AI agent continuously receives sensor data, processes it, and sends commands:

```python
class PerceptionActionBridge(Node):
    def __init__(self):
        super().__init__('perception_action_bridge')

        # Subscribe to various sensor topics
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10
        )

        # Publish commands to robot
        self.cmd_pub = self.create_publisher(
            JointTrajectory, 'joint_trajectory_controller/command', 10
        )

        # AI processing timer
        self.ai_timer = self.create_timer(0.05, self.process_ai_cycle)

        # Data storage for AI processing
        self.latest_image = None
        self.latest_joints = None

    def image_callback(self, msg):
        self.latest_image = msg

    def joint_callback(self, msg):
        self.latest_joints = msg

    def process_ai_cycle(self):
        """Process one cycle of AI perception and action"""
        if self.latest_image and self.latest_joints:
            # Run AI perception
            perception_result = self.run_perception_ai(
                self.latest_image,
                self.latest_joints
            )

            # Run AI planning/decision making
            action = self.run_decision_ai(perception_result)

            # Convert to ROS command and publish
            ros_cmd = self.ai_action_to_ros_command(action)
            self.cmd_pub.publish(ros_cmd)
```

### 2. Behavior-Based Architecture

For complex humanoid robots, a behavior-based approach often works well:

```python
class BehaviorBasedBridge(Node):
    def __init__(self):
        super().__init__('behavior_based_bridge')

        # Define different behaviors
        self.behaviors = {
            'walking': self.walk_behavior,
            'grasping': self.grasp_behavior,
            'balance': self.balance_behavior,
            'idle': self.idle_behavior
        }

        # AI decision maker
        self.ai_decision_publisher = self.create_publisher(
            String, 'ai_decision', 10
        )

        # Behavior selector
        self.current_behavior = 'idle'

    def select_behavior(self, sensor_data):
        """AI decision to select which behavior to execute"""
        # This could be a neural network, rule-based system, etc.
        # For example, if detecting an object in front:
        if self.detect_object_in_front(sensor_data):
            return 'grasping'
        elif self.is_falling(sensor_data):
            return 'balance'
        else:
            return 'idle'

    def execute_current_behavior(self):
        """Execute the current behavior"""
        if self.current_behavior in self.behaviors:
            self.behaviors[self.current_behavior]()
```

## Practical Example: Simple AI-ROS Bridge

Here's a complete example that demonstrates a simple AI agent controlling a robot arm:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class SimpleAIBridge(Node):
    def __init__(self):
        super().__init__('simple_ai_bridge')

        # Publishers and subscribers
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )

        # AI timer
        self.ai_timer = self.create_timer(0.2, self.ai_control_loop)

        self.current_joints = None
        self.target_positions = [0.0, 0.0, 0.0]  # Example 3-DOF arm
        self.step = 0

    def joint_callback(self, msg):
        self.current_joints = msg

    def ai_control_loop(self):
        """Simple AI that moves joints in a pattern"""
        if self.current_joints is None:
            return

        # Simple AI algorithm: move in a sine wave pattern
        t = self.step * 0.1
        self.target_positions[0] = 0.5 * np.sin(t)
        self.target_positions[1] = 0.3 * np.sin(t * 1.5)
        self.target_positions[2] = 0.4 * np.sin(t * 0.7)

        # Publish command
        cmd_msg = Float64MultiArray()
        cmd_msg.data = self.target_positions
        self.command_pub.publish(cmd_msg)

        self.step += 1
        self.get_logger().info(f'AI Command: {self.target_positions}')

def main(args=None):
    rclpy.init(args=args)
    ai_bridge = SimpleAIBridge()

    try:
        rclpy.spin(ai_bridge)
    except KeyboardInterrupt:
        print("Shutting down AI bridge...")
    finally:
        ai_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for AI-ROS Integration

1. **Separation of Concerns**: Keep AI logic separate from ROS communication logic
2. **Error Handling**: Implement robust error handling for communication failures
3. **Performance**: Consider the computational overhead of AI algorithms
4. **Safety**: Implement safety checks before sending commands to hardware
5. **Logging**: Maintain detailed logs for debugging AI-ROS interactions

## Summary

The bridge between AI agents and ROS 2 enables the integration of high-level decision making with low-level robot control. Key takeaways:

- Use rclpy to interface Python AI agents with ROS 2
- Implement perception-action loops for continuous AI-robot interaction
- Consider behavior-based architectures for complex humanoid robots
- Follow best practices for safety, performance, and maintainability

This bridge layer is essential for creating intelligent robotic systems that can adapt and learn while maintaining safe and reliable operation.