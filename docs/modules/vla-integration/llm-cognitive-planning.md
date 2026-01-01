---
sidebar_position: 7
title: Cognitive planning with LLMs to convert language into ROS 2 action sequences
---

# Cognitive planning with LLMs to convert language into ROS 2 action sequences

Large Language Models (LLMs) enable cognitive planning by translating natural language commands into structured action sequences for robotic systems. This approach allows robots to understand complex instructions and execute them through ROS 2 action servers and services.

## Learning Objectives

After completing this chapter, you will understand:
- The core concepts of cognitive planning with LLMs for robotics
- How to implement language-to-ROS 2 action sequence conversion
- How to map natural language to robot actions using LLMs
- The components of an LLM-based planning pipeline

## Introduction to LLM-Based Cognitive Planning

LLM-based cognitive planning uses the reasoning capabilities of large language models to generate action sequences from natural language instructions. The process involves:

- **Language Understanding**: Interpreting the natural language command
- **World Modeling**: Understanding the current state and context
- **Plan Generation**: Creating a sequence of actions to achieve the goal
- **Action Execution**: Converting the plan into ROS 2 commands

### Key Components

An LLM-based cognitive planning system includes:

1. **Language Interface**: Communicates with the LLM API
2. **Prompt Engineering**: Structures requests for optimal LLM responses
3. **Action Mapping**: Converts LLM output to ROS 2 action sequences
4. **Safety Validation**: Ensures plans are safe before execution
5. **State Tracking**: Maintains context for multi-step planning

## LLM Integration for Robotics

### OpenAI API Integration

The OpenAI API provides access to powerful language models that can be used for planning:

```python
import openai
import json
from typing import List, Dict, Any

class LLMCognitivePlanner:
    def __init__(self, api_key: str, model: str = "gpt-4"):
        openai.api_key = api_key
        self.model = model
        self.system_prompt = self.create_system_prompt()

    def create_system_prompt(self) -> str:
        """
        Create the system prompt that guides the LLM's behavior
        """
        return """
        You are a cognitive planning assistant for a humanoid robot. Your role is to:
        1. Understand natural language commands
        2. Generate step-by-step action plans
        3. Consider safety and feasibility
        4. Output plans in JSON format

        Available robot capabilities:
        - move_base: Move to a location (parameters: x, y, theta)
        - rotate: Rotate to a heading (parameters: angle_degrees)
        - grasp: Pick up an object (parameters: object_name, position)
        - place: Place an object (parameters: location)
        - detect: Detect objects in environment (parameters: target)
        - speak: Verbal feedback (parameters: message)

        Output format: {
            "plan": [
                {
                    "action": "action_name",
                    "parameters": {"param1": "value1", ...},
                    "description": "Human-readable description"
                }
            ],
            "confidence": 0.0-1.0,
            "reasoning": "Explanation of the plan"
        }

        Always ensure the plan is executable and safe.
        """

    def generate_plan(self, command: str, robot_state: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate an action plan from a natural language command
        """
        user_prompt = f"""
        Command: {command}

        Current robot state: {json.dumps(robot_state)}

        Please generate a step-by-step action plan to execute this command.
        """

        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.3,
            max_tokens=1000
        )

        plan_text = response.choices[0].message['content'].strip()

        # Extract JSON from response
        try:
            # Find JSON in the response
            json_start = plan_text.find('{')
            json_end = plan_text.rfind('}') + 1
            json_str = plan_text[json_start:json_end]

            plan = json.loads(json_str)
            return plan
        except json.JSONDecodeError:
            # If JSON parsing fails, return a basic error structure
            return {
                "plan": [],
                "confidence": 0.0,
                "reasoning": "Failed to parse LLM response",
                "error": plan_text
            }

# Example usage
planner = LLMCognitivePlanner(api_key="your-api-key")

# Example command
command = "Move to the kitchen, pick up the red cup from the table, and place it in the sink"
robot_state = {
    "location": {"x": 0.0, "y": 0.0, "theta": 0.0},
    "carrying": None,
    "known_objects": ["red cup", "table", "sink", "kitchen"],
    "robot_capabilities": ["move_base", "grasp", "place"]
}

plan = planner.generate_plan(command, robot_state)
print(json.dumps(plan, indent=2))
```

### Prompt Engineering for Robotics

Effective prompt engineering is crucial for reliable LLM-based planning:

#### Role Definition
Define the LLM's role clearly in the system prompt:
- Robot cognitive planner
- Safety-aware decision maker
- Context-aware action generator

#### Capability Description
Provide clear descriptions of robot capabilities:
- Available actions and parameters
- Physical constraints
- Environmental context

#### Output Format
Specify a structured output format (typically JSON) that can be easily parsed and executed.

## Action Sequence Generation

### ROS 2 Action Types

Common ROS 2 actions for robotic planning:

#### Navigation Actions
- `nav2_msgs/MoveToPose`: Navigate to a specific pose
- `nav2_msgs/ComputePathToPose`: Plan a path to a goal
- `nav2_msgs/FollowPath`: Follow a pre-computed path

#### Manipulation Actions
- `control_msgs/FollowJointTrajectory`: Control joint positions
- `moveit_msgs/MoveGroup`: Manipulation planning
- `gripper_action_msgs/GripperCommand`: Gripper control

#### Perception Actions
- `object_recognition_msgs/GetObjectInformation`: Object recognition
- `sensor_msgs/PointCloud2`: 3D perception

### Plan Validation

Before executing an LLM-generated plan, validation is essential:

```python
class PlanValidator:
    def __init__(self, robot_capabilities: Dict[str, Any]):
        self.robot_capabilities = robot_capabilities

    def validate_plan(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate an LLM-generated plan for safety and feasibility
        """
        validation_result = {
            "is_valid": True,
            "errors": [],
            "warnings": [],
            "suggested_fixes": []
        }

        # Check if all actions are supported
        for step in plan.get("plan", []):
            action = step.get("action")
            parameters = step.get("parameters", {})

            if action not in self.robot_capabilities:
                validation_result["is_valid"] = False
                validation_result["errors"].append(
                    f"Action '{action}' not supported by robot"
                )
                continue

            # Validate parameters for each action
            capability = self.robot_capabilities[action]
            for param, value in parameters.items():
                if param not in capability.get("required_params", []) and \
                   param not in capability.get("optional_params", []):
                    validation_result["warnings"].append(
                        f"Unknown parameter '{param}' for action '{action}'"
                    )

            # Validate parameter values
            for param, value in parameters.items():
                if param in capability.get("parameter_ranges", {}):
                    range_info = capability["parameter_ranges"][param]
                    if not (range_info["min"] <= value <= range_info["max"]):
                        validation_result["errors"].append(
                            f"Parameter '{param}' value {value} out of range "
                            f"[{range_info['min']}, {range_info['max']}]"
                        )

        return validation_result
```

## Safety and Validation Mechanisms

### Safety Checks

Implement multiple layers of safety checks:

#### Action-Level Safety
- Verify individual actions are safe to execute
- Check for environmental hazards
- Validate preconditions

#### Plan-Level Safety
- Verify the overall plan is coherent
- Check for logical inconsistencies
- Ensure plan doesn't violate constraints

#### Execution Monitoring
- Monitor execution in real-time
- Detect and handle failures
- Implement emergency stops

### Validation Pipeline

```python
class LLMPlannerWithSafety:
    def __init__(self, llm_planner: LLMCognitivePlanner, robot_interface):
        self.llm_planner = llm_planner
        self.robot_interface = robot_interface
        self.validator = PlanValidator(robot_interface.get_capabilities())

    def execute_command_with_safety(self, command: str) -> Dict[str, Any]:
        """
        Execute a command with full safety validation
        """
        # Step 1: Generate plan with LLM
        robot_state = self.robot_interface.get_state()
        plan = self.llm_planner.generate_plan(command, robot_state)

        # Step 2: Validate the plan
        validation = self.validator.validate_plan(plan)
        if not validation["is_valid"]:
            return {
                "success": False,
                "message": f"Plan validation failed: {validation['errors']}",
                "plan": plan
            }

        # Step 3: Get user confirmation if confidence is low
        if plan.get("confidence", 1.0) < 0.7:
            user_confirmed = self.request_user_confirmation(plan)
            if not user_confirmed:
                return {
                    "success": False,
                    "message": "User did not confirm plan execution",
                    "plan": plan
                }

        # Step 4: Execute the plan step by step
        execution_result = self.execute_plan_safely(plan)
        return execution_result

    def request_user_confirmation(self, plan: Dict[str, Any]) -> bool:
        """
        Request user confirmation for low-confidence plans
        """
        print("Low-confidence plan generated. Review before execution:")
        print(json.dumps(plan, indent=2))
        response = input("Execute plan? (y/n): ")
        return response.lower() in ['y', 'yes']

    def execute_plan_safely(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute plan with safety monitoring
        """
        results = []
        for i, step in enumerate(plan.get("plan", [])):
            print(f"Executing step {i+1}: {step['description']}")

            # Execute the action
            success = self.robot_interface.execute_action(
                step["action"],
                step.get("parameters", {})
            )

            results.append({
                "step": i+1,
                "action": step["action"],
                "success": success,
                "parameters": step.get("parameters", {})
            })

            if not success:
                return {
                    "success": False,
                    "message": f"Action failed at step {i+1}",
                    "results": results
                }

        return {
            "success": True,
            "message": "Plan executed successfully",
            "results": results
        }
```

## Practical Example: LLM-Based Planning System

Here's a complete example of an LLM-based cognitive planning system:

```python
import openai
import json
import time
from typing import Dict, Any, List
from dataclasses import dataclass

@dataclass
class ActionStep:
    action: str
    parameters: Dict[str, Any]
    description: str

class RobotInterface:
    """
    Simulated robot interface for demonstration
    """
    def __init__(self):
        self.location = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.carrying = None
        self.objects = {
            "red cup": {"location": {"x": 1.0, "y": 1.0}, "status": "on_table"},
            "table": {"location": {"x": 1.0, "y": 1.0}, "status": "available"},
            "sink": {"location": {"x": 2.0, "y": 2.0}, "status": "available"}
        }

    def get_state(self) -> Dict[str, Any]:
        """Get current robot state"""
        return {
            "location": self.location,
            "carrying": self.carrying,
            "objects": self.objects
        }

    def get_capabilities(self) -> Dict[str, Any]:
        """Get robot capabilities for validation"""
        return {
            "move_base": {
                "required_params": ["x", "y"],
                "optional_params": ["theta"],
                "parameter_ranges": {
                    "x": {"min": -10, "max": 10},
                    "y": {"min": -10, "max": 10},
                    "theta": {"min": -3.14, "max": 3.14}
                }
            },
            "grasp": {
                "required_params": ["object_name"],
                "optional_params": ["position"],
                "parameter_ranges": {}
            },
            "place": {
                "required_params": ["location"],
                "optional_params": [],
                "parameter_ranges": {}
            }
        }

    def execute_action(self, action: str, parameters: Dict[str, Any]) -> bool:
        """Execute an action on the robot"""
        print(f"Executing {action} with {parameters}")

        if action == "move_base":
            self.location["x"] = parameters.get("x", self.location["x"])
            self.location["y"] = parameters.get("y", self.location["y"])
            self.location["theta"] = parameters.get("theta", self.location["theta"])
            time.sleep(0.5)  # Simulate movement time
            return True
        elif action == "grasp":
            obj_name = parameters.get("object_name")
            if obj_name in self.objects:
                self.carrying = obj_name
                self.objects[obj_name]["status"] = "grasped"
                time.sleep(0.5)  # Simulate grasping time
                return True
            return False
        elif action == "place":
            if self.carrying:
                self.carrying = None
                # Update object location to where it was placed
                time.sleep(0.5)  # Simulate placing time
                return True
            return False
        else:
            print(f"Unknown action: {action}")
            return False

class LLMPromptEngineering:
    """
    Advanced prompt engineering for robotic planning
    """
    @staticmethod
    def create_advanced_system_prompt() -> str:
        return """
        You are an expert cognitive planning system for a humanoid robot.
        Your role is to generate safe, executable action plans from natural language commands.

        CRITICAL RULES:
        1. Always consider safety - never generate plans that could harm the robot or environment
        2. Verify that requested objects are physically possible to manipulate
        3. Consider robot's current state and capabilities
        4. Generate detailed, step-by-step plans with precise parameters

        ROBOT CAPABILITIES:
        - move_base: Navigate to x, y coordinates (parameters: x, y, theta_optional)
        - grasp: Pick up an object (parameters: object_name)
        - place: Place currently held object (parameters: location)
        - detect: Look for objects (parameters: target_object)

        ENVIRONMENTAL CONSTRAINTS:
        - Robot operates in indoor environment
        - Objects have known locations from robot's map
        - Robot can only carry one object at a time
        - All movements must be collision-free

        OUTPUT FORMAT (JSON only):
        {
            "plan": [
                {
                    "action": "action_name",
                    "parameters": {"param1": "value1", ...},
                    "description": "Clear explanation of this step",
                    "safety_check": "Brief safety validation"
                }
            ],
            "confidence": 0.0-1.0,
            "reasoning": "Detailed explanation of why this plan works",
            "estimated_duration": "Estimated time in seconds"
        }

        Ensure each action builds logically toward the final goal.
        """

class LLMRoboticPlanner:
    def __init__(self, api_key: str, model: str = "gpt-4"):
        openai.api_key = api_key
        self.model = model
        self.prompt_engineer = LLMPromptEngineering()

    def plan_for_command(self, command: str, robot_state: Dict[str, Any]) -> Dict[str, Any]:
        """
        Plan for a command using LLM
        """
        user_prompt = f"""
        COMMAND: {command}

        CURRENT ROBOT STATE:
        Location: {robot_state.get('location', 'unknown')}
        Carrying: {robot_state.get('carrying', 'nothing')}
        Known Objects: {list(robot_state.get('objects', {}).keys())}

        Generate a detailed action plan to execute this command safely and efficiently.
        """

        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.prompt_engineer.create_advanced_system_prompt()},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.2,  # Low temperature for consistency
            max_tokens=1500
        )

        plan_text = response.choices[0].message['content'].strip()
        return self.parse_llm_response(plan_text)

    def parse_llm_response(self, response: str) -> Dict[str, Any]:
        """
        Parse and validate the LLM response
        """
        try:
            # Find JSON in response
            json_start = response.find('{')
            json_end = response.rfind('}') + 1
            if json_start != -1 and json_end != 0:
                json_str = response[json_start:json_end]
                plan = json.loads(json_str)

                # Validate plan structure
                if 'plan' not in plan:
                    plan['plan'] = []

                return plan
            else:
                return {
                    "plan": [],
                    "confidence": 0.0,
                    "reasoning": "Could not extract plan from response",
                    "error": response
                }
        except json.JSONDecodeError:
            return {
                "plan": [],
                "confidence": 0.0,
                "reasoning": "Could not parse LLM response as JSON",
                "error": response
            }

# Example usage
if __name__ == "__main__":
    # Initialize the system
    robot = RobotInterface()
    planner = LLMRoboticPlanner(api_key="your-api-key-here")  # Replace with actual API key

    # Example command
    command = "Move to the table, pick up the red cup, and place it at the sink"

    print(f"Processing command: {command}")
    print(f"Current robot state: {robot.get_state()}")

    # Generate plan using LLM
    plan = planner.plan_for_command(command, robot.get_state())
    print(f"\nGenerated plan: {json.dumps(plan, indent=2)}")

    # In a real system, you would now validate and execute the plan
    print("\nPlan validation and execution would happen here in a real system")
```

## Integration with ROS 2

### Action Client Integration

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

class LLMPlannerROS2Node(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        # ROS 2 action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # LLM planner
        self.llm_planner = LLMRoboticPlanner(api_key="your-api-key")

        # Command subscriber
        self.command_sub = self.create_subscription(
            String,
            'voice_commands',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        """
        Process a natural language command
        """
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Get current robot state
        robot_state = self.get_robot_state()

        # Generate plan using LLM
        plan = self.llm_planner.plan_for_command(command, robot_state)

        # Execute the plan
        self.execute_plan(plan)

    def get_robot_state(self):
        """
        Get current robot state for planning context
        """
        # In practice, this would query robot state from TF, etc.
        return {
            "location": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "carrying": None,
            "objects": {}
        }

    def execute_plan(self, plan):
        """
        Execute the LLM-generated plan using ROS 2 actions
        """
        for step in plan.get("plan", []):
            action = step["action"]
            parameters = step.get("parameters", {})

            if action == "move_base":
                self.execute_navigation_action(parameters)
            elif action == "grasp":
                self.execute_grasp_action(parameters)
            # Add more action types as needed
```

## Challenges and Considerations

### Reliability
- LLMs can generate incorrect or unsafe plans
- Need robust validation and error handling
- Confidence scoring is important

### Safety
- Critical to validate all generated actions
- Need human oversight for safety-critical tasks
- Implement multiple safety layers

### Context Understanding
- LLMs may not understand robot's current state
- Need clear state representation
- Consider using vision and other sensors for grounding

## Future Directions

### Improved Safety
- Fine-tuning LLMs on robotics-specific data
- Incorporating physics simulators for validation
- Better uncertainty quantification

### Enhanced Grounding
- Combining LLMs with vision-language models
- Using real-time perception for plan adjustment
- Multi-modal understanding

## Summary

LLM-based cognitive planning provides a powerful approach to natural language understanding for robotics. By carefully engineering prompts and implementing safety validation, these systems can convert complex natural language commands into executable ROS 2 action sequences. The key is balancing the LLM's reasoning capabilities with robust safety checks and validation mechanisms.