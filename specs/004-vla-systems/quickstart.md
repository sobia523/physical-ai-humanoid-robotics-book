# Quickstart Guide: Vision-Language-Action (VLA) Systems

## Overview

This guide provides a rapid introduction to implementing Vision-Language-Action (VLA) systems for humanoid robots. You'll learn how to create a basic VLA pipeline that converts voice commands to robot actions through LLM-based planning and ROS 2 execution.

## Prerequisites

### Software Requirements
- ROS 2 Humble Hawksbill installed
- Python 3.8+ with pip
- Access to a Large Language Model (OpenAI GPT, Anthropic Claude, or open-source alternative)
- OpenAI Whisper for speech recognition (or similar ASR system)
- Robot platform with ROS 2 compatibility

### Hardware Requirements
- Microphone for voice input
- Camera system for vision input
- Robot platform with basic navigation and manipulation capabilities

## Installation

### 1. Set up ROS 2 Environment
```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Create a workspace for VLA components
mkdir -p ~/vla_ws/src
cd ~/vla_ws
colcon build
source install/setup.bash
```

### 2. Install Python Dependencies
```bash
pip install openai speechrecognition transformers torch rospy ros2
```

### 3. Configure LLM Access
```bash
# For OpenAI API
export OPENAI_API_KEY="your-api-key-here"

# For other LLM providers, follow their specific setup instructions
```

## Basic VLA Pipeline Implementation

### 1. Speech Recognition Component
Create a basic speech recognition node:

```python
# speech_recognition_node.py
import rospy
import speech_recognition as sr
from std_msgs.msg import String

class SpeechRecognitionNode:
    def __init__(self):
        rospy.init_node('speech_recognition_node')
        self.pub = rospy.Publisher('voice_command', String, queue_size=10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

    def listen_for_command(self):
        with self.microphone as source:
            print("Listening for voice command...")
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)

        try:
            command_text = self.recognizer.recognize_google(audio)
            rospy.loginfo(f"Heard command: {command_text}")
            self.pub.publish(command_text)
            return command_text
        except sr.UnknownValueError:
            rospy.logerr("Could not understand audio")
            return None
        except sr.RequestError as e:
            rospy.logerr(f"Error with speech recognition service: {e}")
            return None

if __name__ == '__main__':
    node = SpeechRecognitionNode()
    # In a real implementation, you'd use a ROS timer or service call
    command = node.listen_for_command()
```

### 2. LLM-Based Planning Component
Create a cognitive planning node:

```python
# cognitive_planner.py
import openai
import json
from typing import Dict, List

class CognitivePlanner:
    def __init__(self):
        self.model = "gpt-3.5-turbo"  # or your preferred model

    def plan_from_command(self, command: str, robot_capabilities: List[str]) -> Dict:
        """
        Generate a task plan from a natural language command
        """
        system_prompt = f"""
        You are a cognitive planner for a humanoid robot. The robot has the following capabilities: {', '.join(robot_capabilities)}.
        Convert natural language commands into executable action sequences.
        Respond with a JSON object containing 'subtasks' as an array of action objects.
        Each action object should have 'action_type' and 'parameters'.
        """

        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Plan the following task: {command}"}
            ]
        )

        try:
            plan = json.loads(response.choices[0].message.content)
            return plan
        except json.JSONDecodeError:
            # Handle case where response isn't valid JSON
            return {"subtasks": []}

# Example usage
planner = CognitivePlanner()
capabilities = ["navigate_to", "pick_up", "place_object", "open_door", "close_door"]
plan = planner.plan_from_command("Go to the kitchen and bring me a cup", capabilities)
print(json.dumps(plan, indent=2))
```

### 3. Vision-Language Integration Component
Create a vision-language grounding node:

```python
# vision_language_grounding.py
import cv2
import torch
from transformers import BlipProcessor, BlipForQuestionAnswering
from PIL import Image

class VisionLanguageGrounding:
    def __init__(self):
        self.processor = BlipProcessor.from_pretrained("Salesforce/blip-vqa-base")
        self.model = BlipForQuestionAnswering.from_pretrained("Salesforce/blip-vqa-base")

    def identify_object(self, image_path: str, question: str) -> str:
        """
        Answer a question about an image to identify objects
        """
        raw_image = Image.open(image_path).convert('RGB')

        inputs = self.processor(raw_image, question, return_tensors="pt")
        out = self.model(**inputs)
        answer = self.processor.decode(out.logits.argmax(dim=-1).item())

        return answer

# Example usage
grounding = VisionLanguageGrounding()
# This would typically work with real camera images
# answer = grounding.identify_object("path_to_image.jpg", "What object is on the table?")
```

### 4. ROS 2 Action Execution Component
Create an action execution orchestrator:

```python
# action_executor.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')
        self.action_client = ActionClient(self, MoveBaseAction, 'move_base')

    def execute_navigation(self, x: float, y: float, theta: float):
        """Execute a navigation action to a specific pose"""
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.target_pose.pose.orientation.z = theta  # Simplified

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)
        return future

def main():
    rclpy.init()
    executor = ActionExecutor()

    # Example: Navigate to coordinates (1.0, 2.0)
    future = executor.execute_navigation(1.0, 2.0, 0.0)

    # In a real implementation, you'd handle the future response
    rclpy.spin(executor)
    rclpy.shutdown()
```

## Complete VLA System Integration

### Main VLA Orchestrator
```python
# vla_orchestrator.py
from speech_recognition_node import SpeechRecognitionNode
from cognitive_planner import CognitivePlanner
from action_executor import ActionExecutor
import rclpy

class VLASystem:
    def __init__(self):
        # Initialize components
        self.speech_rec = SpeechRecognitionNode()
        self.planner = CognitivePlanner()
        self.executor = ActionExecutor()

        # Define robot capabilities
        self.capabilities = [
            "navigate_to", "pick_up", "place_object",
            "open_door", "close_door", "grasp_object"
        ]

    def process_voice_command(self, command: str):
        """Process a voice command through the full VLA pipeline"""
        print(f"Processing command: {command}")

        # Step 1: Plan from command
        plan = self.planner.plan_from_command(command, self.capabilities)
        print(f"Generated plan: {plan}")

        # Step 2: Execute plan (simplified)
        for subtask in plan.get('subtasks', []):
            action_type = subtask.get('action_type')
            parameters = subtask.get('parameters', {})

            print(f"Executing: {action_type} with params {parameters}")
            # In a real system, you'd route to appropriate action executors
            if action_type == "navigate_to":
                x = parameters.get('x', 0.0)
                y = parameters.get('y', 0.0)
                # self.executor.execute_navigation(x, y, 0.0)

        return plan

def main():
    vla_system = VLASystem()

    # Example: Process a sample command
    command = "Navigate to the kitchen and pick up the red cup"
    plan = vla_system.process_voice_command(command)

    print("VLA pipeline completed successfully!")

if __name__ == '__main__':
    main()
```

## Running the Example

1. **Start ROS 2 environment**:
```bash
source /opt/ros/humble/setup.bash
cd ~/vla_ws
source install/setup.bash
```

2. **Run the VLA system**:
```bash
python vla_orchestrator.py
```

3. **Speak a command** when prompted (in the actual implementation, this would be continuous listening)

## Understanding the Architecture

The VLA system follows this flow:

1. **Voice Input**: Speech recognition converts voice to text
2. **Intent Processing**: Natural language understanding extracts intent
3. **Cognitive Planning**: LLM decomposes high-level goals into action sequences
4. **Vision Integration**: Visual perception provides context and grounding
5. **Action Execution**: ROS 2 action servers execute robot behaviors
6. **Feedback Loop**: Results are reported back to the system

## Next Steps

1. Integrate with a real robot platform
2. Add vision-language grounding for object identification
3. Implement safety checks and human-in-the-loop capabilities
4. Add more sophisticated planning with context awareness
5. Test with complex multi-step commands

## Troubleshooting

- **Speech Recognition Issues**: Ensure microphone is working and environment is not too noisy
- **LLM Access Errors**: Verify API keys and network connectivity
- **ROS 2 Connection Issues**: Check that ROS 2 nodes are properly configured and communicating
- **Action Execution Failures**: Verify robot platform capabilities match those specified in planning