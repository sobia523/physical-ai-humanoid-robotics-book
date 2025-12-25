# Reusable Code Snippets: Vision-Language-Action (VLA) Systems

## Overview
This document provides reusable code snippets for Vision-Language-Action system implementations, based on the research findings from Module 4 research.md.

## Common Imports and Setup

### VLA System Base Class
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import openai
import speech_recognition as sr
import json
import time
from typing import Dict, List, Optional, Any

class VLASystemBase(Node):
    """
    Base class for Vision-Language-Action system components
    """
    def __init__(self, node_name: str):
        super().__init__(node_name)

        # QoS profile for reliable communication
        self.reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # QoS profile for best-effort communication
        self.best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.system_id = self.get_parameter_or("system_id", "vla-system-001")
        self.get_logger().info(f"VLA System {self.system_id} initialized")
```

## Speech Recognition Snippets

### Voice Command Processor
```python
class VoiceCommandProcessor:
    def __init__(self, language: str = "en-US"):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.language = language

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

    def listen_for_command(self, timeout: float = 5.0, phrase_time_limit: float = 10.0):
        """
        Listen for a voice command and return transcribed text
        """
        try:
            with self.microphone as source:
                self.get_logger().info("Listening for voice command...")
                audio = self.recognizer.listen(
                    source,
                    timeout=timeout,
                    phrase_time_limit=phrase_time_limit
                )

            # Transcribe audio to text
            transcript = self.recognizer.recognize_google(audio, language=self.language)
            self.get_logger().info(f"Heard command: {transcript}")

            return transcript

        except sr.WaitTimeoutError:
            self.get_logger().warn("No speech detected within timeout")
            return None
        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand audio")
            return None
        except sr.RequestError as e:
            self.get_logger().error(f"Error with speech recognition service: {e}")
            return None

    def preprocess_audio(self, audio_data):
        """
        Preprocess audio data for better recognition
        """
        # Apply noise reduction, normalization, etc.
        processed_audio = audio_data  # Placeholder for actual preprocessing
        return processed_audio
```

### Intent Classification
```python
def classify_intent(transcript: str) -> Dict[str, Any]:
    """
    Classify intent from transcribed voice command
    """
    # Define intent mappings
    intent_mappings = {
        'navigation': {
            'keywords': ['go to', 'navigate to', 'move to', 'walk to', 'drive to'],
            'confidence_boost': 0.9
        },
        'manipulation': {
            'keywords': ['pick up', 'grasp', 'take', 'grab', 'place', 'put down'],
            'confidence_boost': 0.9
        },
        'information': {
            'keywords': ['what is', 'tell me about', 'describe', 'find', 'locate'],
            'confidence_boost': 0.8
        },
        'communication': {
            'keywords': ['say', 'speak', 'hello', 'talk to'],
            'confidence_boost': 0.8
        }
    }

    best_match = {'type': 'unknown', 'confidence': 0.0, 'parameters': {}}

    for intent_type, config in intent_mappings.items():
        for keyword in config['keywords']:
            if keyword.lower() in transcript.lower():
                # Calculate confidence based on keyword match
                confidence = min(config['confidence_boost'], 1.0)

                if confidence > best_match['confidence']:
                    best_match = {
                        'type': intent_type,
                        'confidence': confidence,
                        'parameters': extract_parameters(transcript, keyword)
                    }

    return best_match

def extract_parameters(transcript: str, matched_keyword: str) -> Dict[str, str]:
    """
    Extract parameters from transcript based on matched keyword
    """
    # Simple parameter extraction (in practice, use more sophisticated NLP)
    remaining_text = transcript.lower().replace(matched_keyword.lower(), '').strip()

    parameters = {}

    # Extract location/object if present
    if remaining_text:
        # Remove common words
        clean_text = remaining_text.replace('the', '').replace('a', '').replace('an', '').strip()
        parameters['target'] = clean_text

    return parameters
```

## LLM Integration Snippets

### Cognitive Planner
```python
class VLACognitivePlanner:
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        openai.api_key = api_key
        self.model = model

    def plan_from_goal(self, goal_description: str, robot_capabilities: List[str]) -> Dict[str, Any]:
        """
        Generate a task plan from a natural language goal using LLM
        """
        system_prompt = f"""
        You are a cognitive planner for a humanoid robot. The robot has these capabilities: {', '.join(robot_capabilities)}.
        Convert natural language goals into executable action sequences.
        Respond with a JSON object containing 'subtasks' as an array of action objects.
        Each action object should have 'action_type' and 'parameters'.
        Keep responses concise and focused on executable actions.
        """

        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": f"Plan the following task: {goal_description}"}
                ],
                temperature=0.3,
                max_tokens=500
            )

            # Parse the response
            plan = json.loads(response.choices[0].message.content)
            return plan

        except json.JSONDecodeError:
            # Handle case where response isn't valid JSON
            self.get_logger().error("LLM response was not valid JSON")
            return {"subtasks": [], "error": "Invalid response format"}
        except Exception as e:
            self.get_logger().error(f"Error calling LLM: {e}")
            return {"subtasks": [], "error": str(e)}

    def validate_plan(self, plan: Dict[str, Any]) -> bool:
        """
        Validate that the generated plan is properly structured
        """
        if 'subtasks' not in plan:
            return False

        if not isinstance(plan['subtasks'], list):
            return False

        for subtask in plan['subtasks']:
            if not isinstance(subtask, dict):
                return False
            if 'action_type' not in subtask:
                return False

        return True
```

### LLM Safety Wrapper
```python
def safe_llm_call(planner: VLACognitivePlanner, goal: str, capabilities: List[str], max_retries: int = 3):
    """
    Safely call LLM with retry logic and safety checks
    """
    for attempt in range(max_retries):
        try:
            plan = planner.plan_from_goal(goal, capabilities)

            # Validate the plan
            if planner.validate_plan(plan):
                return plan
            else:
                raise ValueError("Generated plan failed validation")

        except Exception as e:
            if attempt == max_retries - 1:
                # Last attempt failed
                return {"subtasks": [], "error": f"Failed after {max_retries} attempts: {str(e)}"}
            else:
                # Wait before retry
                time.sleep(1)

    return {"subtasks": [], "error": "Unexpected error in LLM call"}
```

## Vision-Language Integration Snippets

### Vision-Language Grounding
```python
class VisionLanguageGrounding:
    def __init__(self):
        # Initialize vision models (placeholder)
        self.vision_model = None  # Would be initialized with actual model
        self.nlp_model = None     # Would be initialized with actual NLP model

    def detect_and_ground(self, image, language_query: str) -> List[Dict[str, Any]]:
        """
        Detect objects in an image and ground them to language query
        """
        # Step 1: Detect objects in image
        detections = self.detect_objects(image)

        # Step 2: Ground detections to language query
        grounded_objects = []
        for detection in detections:
            if self.is_relevant_to_query(detection, language_query):
                grounded_objects.append({
                    'object_id': detection['id'],
                    'object_type': detection['type'],
                    'bbox': detection['bbox'],
                    'confidence': detection['confidence'],
                    'grounding_confidence': self.calculate_grounding_confidence(
                        detection, language_query
                    )
                })

        return grounded_objects

    def detect_objects(self, image):
        """
        Detect objects in image using vision model
        """
        # Placeholder implementation
        # In practice, use YOLO, CLIP, or similar models
        detections = [
            {
                'id': 'obj_001',
                'type': 'cup',
                'bbox': {'x': 100, 'y': 200, 'width': 50, 'height': 60},
                'confidence': 0.85
            }
        ]
        return detections

    def is_relevant_to_query(self, detection: Dict, query: str) -> bool:
        """
        Check if detection is relevant to language query
        """
        # Simple keyword matching (in practice, use semantic similarity)
        object_type = detection['type'].lower()
        query_lower = query.lower()

        # Check if object type appears in query
        return object_type in query_lower

    def calculate_grounding_confidence(self, detection: Dict, query: str) -> float:
        """
        Calculate confidence in grounding between detection and query
        """
        # Placeholder implementation
        # In practice, use cross-modal similarity
        return min(detection['confidence'], 0.9)  # Cap confidence
```

## ROS 2 Integration Snippets

### Action Executor
```python
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class VLAActionExecutor(Node):
    def __init__(self):
        super().__init__('vla_action_executor')

        # Create action clients for different action types
        self.navigation_client = ActionClient(
            self,
            NavigateToPose,  # Assuming standard navigation action
            'navigate_to_pose'
        )

        self.manipulation_client = ActionClient(
            self,
            ManipulationAction,  # Custom manipulation action
            'manipulation_action'
        )

        # Create callback groups for thread safety
        self.nav_callback_group = MutuallyExclusiveCallbackGroup()
        self.manip_callback_group = MutuallyExclusiveCallbackGroup()

    def execute_navigation_action(self, x: float, y: float, theta: float = 0.0):
        """
        Execute navigation action to reach a specific pose
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta  # Simplified orientation

        # Wait for action server
        self.navigation_client.wait_for_server()

        # Send goal and return future
        future = self.navigation_client.send_goal_async(goal_msg)
        return future

    def execute_manipulation_action(self, action_type: str, parameters: Dict[str, Any]):
        """
        Execute manipulation action with specified parameters
        """
        goal_msg = ManipulationAction.Goal()
        goal_msg.action_type = action_type
        goal_msg.parameters = parameters  # Convert dict to message format

        self.manipulation_client.wait_for_server()
        future = self.manipulation_client.send_goal_async(goal_msg)
        return future

    def execute_subtask(self, subtask: Dict[str, Any]):
        """
        Execute a single subtask based on its action type
        """
        action_type = subtask.get('action_type', '')
        parameters = subtask.get('parameters', {})

        if action_type == 'navigate_to':
            x = parameters.get('x', 0.0)
            y = parameters.get('y', 0.0)
            theta = parameters.get('theta', 0.0)
            return self.execute_navigation_action(x, y, theta)

        elif action_type == 'pick_up':
            # Execute pick-up action
            return self.execute_manipulation_action('pick_up', parameters)

        elif action_type == 'place_object':
            # Execute place action
            return self.execute_manipulation_action('place', parameters)

        else:
            self.get_logger().warn(f"Unknown action type: {action_type}")
            return None
```

## System Integration Snippets

### VLA Orchestrator
```python
class VLASystemOrchestrator(VLASystemBase):
    def __init__(self):
        super().__init__('vla_system_orchestrator')

        # Initialize components
        self.speech_processor = VoiceCommandProcessor()
        self.cognitive_planner = VLACognitivePlanner(api_key=self.get_api_key())
        self.vision_grounding = VisionLanguageGrounding()
        self.action_executor = VLAActionExecutor()

        # Robot capabilities (would be loaded from config)
        self.robot_capabilities = [
            'navigate_to', 'pick_up', 'place_object',
            'speak', 'listen', 'perceive_objects'
        ]

        # Publishers and subscribers
        self.voice_cmd_pub = self.create_publisher(String, '/vla/speech/output/intent', self.reliable_qos)
        self.status_pub = self.create_publisher(String, '/vla/system/status', self.best_effort_qos)

        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)

    def process_voice_command(self, command: str):
        """
        Process a voice command through the full VLA pipeline
        """
        self.get_logger().info(f"Processing voice command: {command}")

        # Step 1: Classify intent
        intent = classify_intent(command)
        self.get_logger().info(f"Detected intent: {intent}")

        if intent['confidence'] < 0.5:
            self.get_logger().warn("Low confidence in intent classification")
            return False

        # Step 2: Generate plan using LLM
        plan = safe_llm_call(
            self.cognitive_planner,
            command,
            self.robot_capabilities
        )

        if 'error' in plan:
            self.get_logger().error(f"Plan generation failed: {plan['error']}")
            return False

        # Step 3: Execute plan
        success = self.execute_plan(plan)
        return success

    def execute_plan(self, plan: Dict[str, Any]) -> bool:
        """
        Execute a plan with multiple subtasks
        """
        self.get_logger().info(f"Executing plan with {len(plan.get('subtasks', []))} subtasks")

        for i, subtask in enumerate(plan.get('subtasks', [])):
            self.get_logger().info(f"Executing subtask {i+1}: {subtask}")

            # Execute the subtask
            future = self.action_executor.execute_subtask(subtask)

            if future:
                # Wait for completion (in practice, this would be non-blocking)
                try:
                    result = future.result()
                    self.get_logger().info(f"Subtask {i+1} completed successfully")
                except Exception as e:
                    self.get_logger().error(f"Subtask {i+1} failed: {e}")
                    return False
            else:
                self.get_logger().warn(f"Could not execute subtask {i+1}")
                return False

        self.get_logger().info("Plan executed successfully")
        return True

    def get_api_key(self) -> str:
        """
        Get API key from secure storage (implement based on your security needs)
        """
        # In practice, get from environment variables, secure storage, etc.
        return self.get_parameter_or("openai_api_key", "")

    def publish_status(self):
        """
        Publish system status periodically
        """
        status_msg = String()
        status_msg.data = f"Ready - Capabilities: {', '.join(self.robot_capabilities[:3])}..."  # Truncate for brevity
        self.status_pub.publish(status_msg)
```

## Error Handling and Validation Snippets

### Safe Execution Wrapper
```python
def safe_execute(func, *args, max_retries: int = 3, error_handler=None, **kwargs):
    """
    Safely execute a function with retry logic and error handling
    """
    for attempt in range(max_retries):
        try:
            result = func(*args, **kwargs)
            return result
        except Exception as e:
            if attempt == max_retries - 1:
                # Last attempt - call error handler or re-raise
                if error_handler:
                    return error_handler(e)
                else:
                    raise
            else:
                # Log intermediate failure and retry
                print(f"Attempt {attempt + 1} failed: {e}. Retrying...")
                time.sleep(0.5)  # Brief delay before retry
```

### Validation Utilities
```python
def validate_task_plan(plan: Dict[str, Any]) -> List[str]:
    """
    Validate a task plan and return list of validation errors
    """
    errors = []

    if not isinstance(plan, dict):
        errors.append("Plan must be a dictionary")
        return errors

    if 'subtasks' not in plan:
        errors.append("Plan must contain 'subtasks' key")

    subtasks = plan.get('subtasks', [])
    if not isinstance(subtasks, list):
        errors.append("'subtasks' must be a list")

    for i, subtask in enumerate(subtasks):
        if not isinstance(subtask, dict):
            errors.append(f"Subtask {i} must be a dictionary")
            continue

        if 'action_type' not in subtask:
            errors.append(f"Subtask {i} must have 'action_type' field")

    return errors

def validate_robot_capabilities(capabilities: List[str]) -> List[str]:
    """
    Validate robot capabilities and return list of validation errors
    """
    errors = []

    if not isinstance(capabilities, list):
        errors.append("Capabilities must be a list")
        return errors

    allowed_types = {'navigate_to', 'pick_up', 'place_object', 'speak', 'listen', 'perceive_objects'}

    for i, capability in enumerate(capabilities):
        if not isinstance(capability, str):
            errors.append(f"Capability {i} must be a string")
        elif capability not in allowed_types:
            errors.append(f"Unknown capability '{capability}', allowed types: {allowed_types}")

    return errors
```

## Utility Functions

### Configuration Loader
```python
def load_vla_config(config_path: str) -> Dict[str, Any]:
    """
    Load VLA system configuration from YAML file
    """
    import yaml

    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            return config
    except FileNotFoundError:
        print(f"Config file not found: {config_path}")
        return {}
    except yaml.YAMLError as e:
        print(f"Error parsing YAML config: {e}")
        return {}

def validate_config(config: Dict[str, Any]) -> bool:
    """
    Validate configuration structure
    """
    required_sections = ['vla_system', 'speech_recognition', 'llm_integration']

    for section in required_sections:
        if section not in config:
            print(f"Missing required config section: {section}")
            return False

    return True
```

### Performance Monitoring
```python
import time
from functools import wraps

def measure_execution_time(func):
    """
    Decorator to measure execution time of functions
    """
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()

        execution_time = end_time - start_time
        print(f"{func.__name__} executed in {execution_time:.4f} seconds")

        return result
    return wrapper

class PerformanceMonitor:
    """
    Monitor performance metrics for VLA system components
    """
    def __init__(self):
        self.metrics = {}

    def start_timer(self, metric_name: str):
        """
        Start timing for a specific metric
        """
        self.metrics[metric_name] = {
            'start_time': time.time(),
            'completed': False
        }

    def end_timer(self, metric_name: str) -> float:
        """
        End timing and return elapsed time for a metric
        """
        if metric_name in self.metrics and not self.metrics[metric_name]['completed']:
            elapsed = time.time() - self.metrics[metric_name]['start_time']
            self.metrics[metric_name]['elapsed'] = elapsed
            self.metrics[metric_name]['completed'] = True
            return elapsed

        return -1  # Error indicator
```

## Main Execution Example

### VLA System Main Loop
```python
def main(args=None):
    """
    Main function to run the VLA system
    """
    rclpy.init(args=args)

    # Create and configure the VLA orchestrator
    vla_system = VLASystemOrchestrator()

    # Example: Process a voice command
    example_command = "Navigate to the kitchen and pick up the red cup"

    try:
        success = vla_system.process_voice_command(example_command)
        if success:
            print("Command processed successfully")
        else:
            print("Command processing failed")
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error in VLA system: {e}")
    finally:
        vla_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```