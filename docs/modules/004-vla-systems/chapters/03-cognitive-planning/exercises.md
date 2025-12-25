# Cognitive Planning with LLMs: Hands-On Exercises

## Exercise Overview

This exercise provides hands-on experience with implementing cognitive planning systems using Large Language Models (LLMs) in Vision-Language-Action (VLA) systems. Students will build and test a complete cognitive planning pipeline that translates natural language goals into executable robot actions.

## Exercise Objectives

By completing this exercise, students will:

1. Implement an LLM-based cognitive planning system
2. Integrate the planning system with ROS 2 communication
3. Develop natural language goal parsing and action translation
4. Test and validate the cognitive planning pipeline performance

## Exercise 1: Basic LLM Integration

### Task 1.1: Set up LLM Environment

Create a Python environment with LLM integration and test basic functionality:

```python
import openai
from typing import Dict, List, Any
import os

class LLMPlanner:
    def __init__(self, api_key: str = None, model: str = "gpt-4-turbo"):
        """
        Initialize LLM-based cognitive planner
        """
        if api_key is None:
            api_key = os.getenv("OPENAI_API_KEY")

        if api_key is None:
            raise ValueError("API key must be provided or set in OPENAI_API_KEY environment variable")

        self.client = openai.OpenAI(api_key=api_key)
        self.model = model
        print(f"Initialized LLM Planner with model: {model}")

    def generate_plan(self, goal: str, context: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Generate a plan for the given goal using LLM
        """
        prompt = self._create_planning_prompt(goal, context)

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=1024
            )

            plan_text = response.choices[0].message.content
            return self._parse_plan(plan_text)

        except Exception as e:
            print(f"Error generating plan: {e}")
            return []

    def _create_planning_prompt(self, goal: str, context: Dict[str, Any]) -> str:
        """
        Create a prompt for the LLM to generate a plan
        """
        return f"""
        You are a cognitive planning system for a humanoid robot. Generate a plan to achieve the following goal:

        GOAL: {goal}

        CONTEXT:
        Robot Capabilities: {context.get('capabilities', 'Unknown')}
        Environment: {context.get('environment', 'Unknown')}
        Current State: {context.get('state', 'Unknown')}
        Available Objects: {context.get('objects', 'None')}

        Provide the plan as a sequence of actions in the following format:
        1. ACTION_NAME - PARAMETERS - DESCRIPTION
        2. ACTION_NAME - PARAMETERS - DESCRIPTION

        Each action should be executable by the robot.
        """

    def _get_system_prompt(self) -> str:
        """
        System prompt for the LLM
        """
        return """
        You are a cognitive planning system for a humanoid robot. Generate detailed, executable action plans that consider robot capabilities, environmental constraints, and safety requirements. Each action should be specific and executable through ROS 2 interfaces.
        """

    def _parse_plan(self, plan_text: str) -> List[Dict[str, Any]]:
        """
        Parse the LLM output into structured actions
        """
        actions = []
        lines = plan_text.split('\n')

        for line in lines:
            line = line.strip()
            if line and any(char.isdigit() for char in line):
                parts = line.split(' - ')
                if len(parts) >= 2:
                    action = {
                        "action_name": parts[0].split('. ', 1)[-1].strip(),
                        "parameters": parts[1] if len(parts) > 1 else "",
                        "description": parts[2] if len(parts) > 2 else ""
                    }
                    actions.append(action)

        return actions
```

## Exercise 2: Goal Parsing and Context Integration

### Task 2.1: Natural Language Goal Parser

Implement a goal parser to extract key components from natural language goals:

```python
import re
from typing import Dict, List, Optional

class GoalParser:
    def __init__(self):
        # Define action patterns
        self.action_patterns = {
            "navigation": [r"move to (.+)", r"go to (.+)", r"navigate to (.+)"],
            "manipulation": [r"pick up (.+)", r"grasp (.+)", r"take (.+)", r"bring (.+)"],
            "perception": [r"find (.+)", r"detect (.+)", r"locate (.+)"]
        }

    def parse_goal(self, goal_text: str) -> Dict[str, Any]:
        """
        Parse a natural language goal and extract components
        """
        goal_text = goal_text.lower().strip()

        # Extract action
        action_type = None
        action_target = None
        for action_type_key, patterns in self.action_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, goal_text)
                if match:
                    action_type = action_type_key
                    action_target = match.group(1) if match.groups() else None
                    break
            if action_type:
                break

        return {
            "original_text": goal_text,
            "action_type": action_type,
            "action_target": action_target,
        }
```

## Exercise 3: ROS 2 Integration

### Task 3.1: ROS 2 Cognitive Planning Node

Create a ROS 2 node that integrates the cognitive planning system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import Dict, Any

class CognitivePlanningNode(Node):
    def __init__(self):
        super().__init__('cognitive_planning_node')

        # Initialize components
        api_key = self.declare_parameter('llm_api_key', '').value
        if not api_key:
            api_key = None

        self.planner = LLMPlanner(api_key=api_key)
        self.goal_parser = GoalParser()

        # Create subscribers and publishers
        self.goal_subscriber = self.create_subscription(
            String,
            'natural_language_goal',
            self.goal_callback,
            10
        )

        self.plan_publisher = self.create_publisher(String, 'action_sequence', 10)
        self.status_publisher = self.create_publisher(String, 'planning_status', 10)

        self.get_logger().info('Cognitive Planning Node initialized')

    def goal_callback(self, msg: String):
        """
        Handle incoming natural language goals
        """
        try:
            self.get_logger().info(f'Received goal: {msg.data}')

            # Parse the goal
            parsed_goal = self.goal_parser.parse_goal(msg.data)

            # Create basic context
            context = {
                "capabilities": ["navigation", "manipulation", "perception"],
                "environment": "indoor environment",
                "state": "robot is at charging station",
                "objects": ["cup", "table", "chair"]
            }

            # Generate plan using LLM
            plan = self.planner.generate_plan(msg.data, context)

            if plan:
                # Publish the plan
                plan_msg = String()
                plan_msg.data = json.dumps(plan)
                self.plan_publisher.publish(plan_msg)

                self.get_logger().info(f'Published plan with {len(plan)} actions')

                # Publish status
                status_msg = String()
                status_msg.data = f"Plan generated successfully with {len(plan)} actions"
                self.status_publisher.publish(status_msg)
            else:
                self.get_logger().error('Failed to generate plan')

        except Exception as e:
            self.get_logger().error(f'Error processing goal: {e}')

def main(args=None):
    rclpy.init(args=args)

    node = CognitivePlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Validation of Cognitive Planning System

### Plan Decomposition Validation

To validate that the cognitive planning system correctly decomposes natural language goals into executable actions:

```python
def validate_plan_decomposition(goal: str, generated_plan: List[Dict[str, Any]], expected_actions: List[str]) -> Dict[str, Any]:
    """
    Validate that the generated plan correctly decomposes the goal
    """
    validation_result = {
        "success": True,
        "missing_actions": [],
        "extra_actions": [],
        "completeness_score": 0.0
    }

    # Extract action names from generated plan
    generated_action_names = [action["action_name"] for action in generated_plan]

    # Check for missing actions
    for expected_action in expected_actions:
        if expected_action not in generated_action_names:
            validation_result["missing_actions"].append(expected_action)
            validation_result["success"] = False

    # Check for extra/unnecessary actions
    for generated_action in generated_action_names:
        if generated_action not in expected_actions:
            validation_result["extra_actions"].append(generated_action)

    # Calculate completeness score
    if expected_actions:
        completeness_score = (len(expected_actions) - len(validation_result["missing_actions"])) / len(expected_actions)
        validation_result["completeness_score"] = completeness_score

    return validation_result
```

## Assessment Rubric

### Technical Implementation (60 points)
- [ ] LLM integration and planning system (15 points)
- [ ] Natural language goal parsing (15 points)
- [ ] Context integration system (15 points)
- [ ] ROS 2 integration (15 points)

### Functionality (25 points)
- [ ] Plan generation accuracy (10 points)
- [ ] Context integration effectiveness (10 points)
- [ ] System responsiveness (5 points)

### Code Quality (15 points)
- [ ] Code organization and structure (5 points)
- [ ] Error handling and validation (5 points)
- [ ] Documentation and comments (5 points)

## Summary

This exercise provides hands-on experience with cognitive planning systems using LLMs in VLA applications. Students implement a complete pipeline from natural language goal parsing to action sequence generation, with integration into ROS 2 systems. The validation components ensure that the cognitive planning system correctly decomposes natural language goals into executable robot actions.