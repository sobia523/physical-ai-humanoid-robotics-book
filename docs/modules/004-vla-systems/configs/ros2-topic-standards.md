# ROS 2 Message Topic Standards: Vision-Language-Action (VLA) Systems

## Overview
This document defines the ROS 2 message topic standards for Vision-Language-Action systems, following the research findings from Module 4 research.md and integrating with the existing ROS 2 nervous system from Module 1.

## Topic Naming Convention
All VLA system topics follow the convention: `/vla/[component]/[function]/[data_type]`

## Speech Recognition Topics

### Voice Command Input
- **Topic**: `/vla/speech/input/audio`
- **Message Type**: `std_msgs/msg/String` or `sensor_msgs/msg/AudioData`
- **Direction**: Publisher (from microphone/speech node)
- **Purpose**: Raw audio input or transcribed text from speech recognition
- **QoS**: Reliable, transient local for configuration

### Voice Command Output
- **Topic**: `/vla/speech/output/intent`
- **Message Type**: `std_msgs/msg/String` or custom `vla_msgs/VoiceCommandIntent`
- **Direction**: Subscriber (to intent processing node)
- **Purpose**: Processed voice command intent with parameters
- **QoS**: Reliable for command delivery

### Speech Status
- **Topic**: `/vla/speech/status`
- **Message Type**: `std_msgs/msg/String` or custom `vla_msgs/SpeechStatus`
- **Message Fields**:
  - `status` (string): Current speech recognition status (listening, processing, ready, error)
  - `confidence` (float): Confidence level of last recognition
  - `timestamp` (builtin_interfaces/msg/Time): When status was reported
- **Direction**: Publisher (from speech recognition node)
- **Purpose**: Report speech recognition system status
- **QoS**: Best effort for status updates

## Natural Language Processing Topics

### Intent Classification
- **Topic**: `/vla/nlp/intent_classification`
- **Message Type**: Custom `vla_msgs/NaturalLanguageIntent`
- **Message Fields**:
  - `intent_type` (string): Type of intent (navigation, manipulation, information, etc.)
  - `parameters` (dictionary): Structured parameters from command
  - `confidence` (float): Confidence in intent classification
  - `original_text` (string): Original transcribed text
  - `timestamp` (builtin_interfaces/msg/Time): When intent was classified
- **Direction**: Publisher (from NLP node) / Subscriber (to planning node)
- **Purpose**: Classified intent with parameters for planning
- **QoS**: Reliable for command integrity

### Context Update
- **Topic**: `/vla/nlp/context_update`
- **Message Type**: Custom `vla_msgs/ContextUpdate`
- **Message Fields**:
  - `context_id` (string): Identifier for context
  - `context_data` (dictionary): Context information
  - `update_type` (string): Type of context update
  - `timestamp` (builtin_interfaces/msg/Time): When context was updated
- **Direction**: Publisher (from various nodes) / Subscriber (to NLP node)
- **Purpose**: Share context information between components
- **QoS**: Best effort for context updates

## Cognitive Planning Topics

### Task Plan Generation
- **Topic**: `/vla/planning/request`
- **Message Type**: Custom `vla_msgs/PlanRequest`
- **Message Fields**:
  - `goal_description` (string): Natural language goal
  - `robot_capabilities` (string[]): List of robot capabilities
  - `environment_context` (string): Environmental context
  - `request_id` (string): Unique request identifier
  - `timestamp` (builtin_interfaces/msg/Time): When request was made
- **Direction**: Publisher (from intent processor) / Subscriber (to planner)
- **Purpose**: Request task plan generation from LLM
- **QoS**: Reliable for command delivery

### Task Plan Output
- **Topic**: `/vla/planning/result`
- **Message Type**: Custom `vla_msgs/TaskPlan`
- **Message Fields**:
  - `plan_id` (string): Unique identifier for the plan
  - `goal_description` (string): Original goal that was planned
  - `subtasks` (Subtask[]): Array of subtasks that make up the plan
  - `estimated_duration` (float): Estimated time to complete the plan
  - `required_resources` (string[]): List of resources needed for plan execution
  - `priority` (uint8): Priority level for plan execution
  - `status` (string): Current status of the plan
  - `timestamp` (builtin_interfaces/msg/Time): When plan was generated
- **Direction**: Publisher (from planner) / Subscriber (to action executor)
- **Purpose**: Generated task plan for execution
- **QoS**: Reliable for plan integrity

### Subtask Definition
- **Message Type**: Custom `vla_msgs/Subtask`
- **Message Fields**:
  - `task_id` (string): Unique identifier for the subtask
  - `action_type` (string): Type of action to perform
  - `parameters` (dictionary): Parameters for the action
  - `description` (string): Human-readable description of the subtask
  - `sequence_number` (uint8): Order in which the subtask should be executed
  - `preconditions` (string[]): Conditions that must be true before execution
  - `postconditions` (string[]): Expected state after execution

## Vision-Language Integration Topics

### Vision Perception Input
- **Topic**: `/vla/vision/input/image`
- **Message Type**: `sensor_msgs/msg/Image`
- **Direction**: Publisher (from camera driver) / Subscriber (to vision node)
- **Purpose**: Raw image input for vision processing
- **QoS**: Best effort for real-time processing

### Vision Perception Output
- **Topic**: `/vla/vision/output/perception`
- **Message Type**: Custom `vla_msgs/VisionPerceptionResult`
- **Message Fields**:
  - `id` (string): Unique identifier for the perception result
  - `timestamp` (builtin_interfaces/msg/Time): When the perception was performed
  - `detected_objects` (ObjectDetection[]): List of objects detected in the scene
  - `scene_description` (string): Natural language description of the scene
  - `spatial_relationships` (string[]): Relationships between detected objects
  - `confidence_map` (dictionary): Confidence levels for different aspects of perception
  - `contextual_relevance` (float): Relevance to current task or command
- **Direction**: Publisher (from vision node) / Subscriber (to grounding node)
- **Purpose**: Processed vision perception results
- **QoS**: Best effort for perception updates

### Object Detection
- **Message Type**: Custom `vla_msgs/ObjectDetection`
- **Message Fields**:
  - `id` (string): Unique identifier for the detection
  - `object_type` (string): Type/class of the detected object
  - `bounding_box` (BoundingBox): Coordinates of the bounding box
  - `confidence` (float): Confidence level of the detection
  - `properties` (dictionary): Additional properties of the object
  - `spatial_coordinates` (Point): 3D coordinates in robot's reference frame

### Bounding Box
- **Message Type**: Custom `vla_msgs/BoundingBox`
- **Message Fields**:
  - `x_offset` (int): X offset of the bounding box
  - `y_offset` (int): Y offset of the bounding box
  - `width` (int): Width of the bounding box
  - `height` (int): Height of the bounding box

### Vision-Language Grounding
- **Topic**: `/vla/vision/output/grounding`
- **Message Type**: Custom `vla_msgs/VisionLanguageMapping`
- **Message Fields**:
  - `id` (string): Unique identifier for the mapping
  - `language_reference` (string): Text reference from natural language
  - `visual_element_id` (string): ID of the corresponding visual element
  - `confidence` (float): Confidence level of the mapping
  - `grounding_method` (string): Method used for grounding (attention, context, etc.)
  - `created_at` (builtin_interfaces/msg/Time): Timestamp of mapping creation
- **Direction**: Publisher (from grounding node) / Subscriber (to planning/action nodes)
- **Purpose**: Grounding between language references and visual elements
- **QoS**: Best effort for grounding updates

## Action Execution Topics

### Action Request
- **Topic**: `/vla/action/request`
- **Message Type**: Custom `vla_msgs/ActionRequest`
- **Message Fields**:
  - `action_id` (string): Unique identifier for the action
  - `action_name` (string): Name of the action to execute
  - `parameters` (dictionary): Parameters for the action
  - `timeout` (float): Maximum time to wait for completion
  - `request_timestamp` (builtin_interfaces/msg/Time): When request was made
- **Direction**: Publisher (from planner) / Subscriber (to action servers)
- **Purpose**: Request execution of a specific action
- **QoS**: Reliable for action integrity

### Action Feedback
- **Topic**: `/vla/action/feedback`
- **Message Type**: Custom `vla_msgs/ActionFeedback`
- **Message Fields**:
  - `action_id` (string): Reference to the original action request
  - `status` (string): Current status of the action (executing, succeeded, failed)
  - `progress` (float): Progress percentage (0.0 to 1.0)
  - `feedback_message` (string): Additional feedback information
  - `feedback_timestamp` (builtin_interfaces/msg/Time): When feedback was generated
- **Direction**: Publisher (from action servers) / Subscriber (to monitoring nodes)
- **Purpose**: Provide feedback during action execution
- **QoS**: Best effort for feedback updates

### Action Result
- **Topic**: `/vla/action/result`
- **Message Type**: Custom `vla_msgs/ActionResult`
- **Message Fields**:
  - `action_id` (string): Reference to the original action request
  - `success` (bool): Whether the action was successful
  - `result_data` (dictionary): Data returned by the action server
  - `error_message` (string): Error message if action failed
  - `execution_time` (float): Time taken to execute the action
  - `completion_timestamp` (builtin_interfaces/msg/Time): When action completed
- **Direction**: Publisher (from action servers) / Subscriber (to monitoring nodes)
- **Purpose**: Report final result of action execution
- **QoS**: Reliable for result integrity

## System State Topics

### System Status
- **Topic**: `/vla/system/status`
- **Message Type**: Custom `vla_msgs/SystemStatus`
- **Message Fields**:
  - `system_id` (string): Identifier for the VLA system
  - `current_task` (string): Currently executing task or null if idle
  - `robot_status` (string): Current status of the robot
  - `last_command` (string): Most recent command processed
  - `vlc_confidence` (float): Overall confidence in VLA processing
  - `uptime` (float): System uptime in seconds
  - `timestamp` (builtin_interfaces/msg/Time): When status was reported
- **Direction**: Publisher (from system manager) / Subscriber (to monitoring nodes)
- **Purpose**: Overall system status and health monitoring
- **QoS**: Best effort for status updates

### Performance Metrics
- **Topic**: `/vla/system/performance`
- **Message Type**: Custom `vla_msgs/PerformanceMetrics`
- **Message Fields**:
  - `metric_id` (string): Unique identifier for the metrics
  - `processing_latency` (float): Processing latency in seconds
  - `throughput` (float): Requests per second
  - `cpu_usage` (float): CPU usage percentage
  - `memory_usage` (float): Memory usage percentage
  - `gpu_usage` (float): GPU usage percentage (if applicable)
  - `timestamp` (builtin_interfaces/msg/Time): When metrics were collected
- **Direction**: Publisher (from performance monitor) / Subscriber (to monitoring nodes)
- **Purpose**: Performance metrics for system optimization
- **QoS**: Best effort for metrics updates

## Service Definitions

### Plan Task Service
- **Service Type**: `vla_srvs/PlanTask`
- **Request Fields**:
  - `goal_description` (string): Natural language goal to plan
  - `robot_capabilities` (string[]): List of robot capabilities
  - `environment_context` (string): Environmental context
- **Response Fields**:
  - `success` (bool): Whether planning was successful
  - `task_plan` (TaskPlan): Generated task plan
  - `error_message` (string): Error message if planning failed

### Execute Action Service
- **Service Type**: `vla_srvs/ExecuteAction`
- **Request Fields**:
  - `action_name` (string): Name of the action to execute
  - `parameters` (dictionary): Parameters for the action
  - `timeout` (float): Maximum time to wait for completion
- **Response Fields**:
  - `success` (bool): Whether action execution was successful
  - `result_data` (dictionary): Data returned by the action
  - `execution_time` (float): Time taken to execute the action
  - `error_message` (string): Error message if execution failed

### Ground Language Service
- **Service Type**: `vla_srvs/GroundLanguage`
- **Request Fields**:
  - `language_query` (string): Language query to ground
  - `image_data` (Image): Image to ground the query to
  - `context` (string): Context for grounding
- **Response Fields**:
  - `success` (bool): Whether grounding was successful
  - `grounded_objects` (ObjectDetection[]): Grounded objects
  - `confidence` (float): Confidence in grounding
  - `error_message` (string): Error message if grounding failed

## Action Definitions

### Execute Task Action
- **Action Type**: `vla_actions/ExecuteTask`
- **Goal Fields**:
  - `task_plan` (TaskPlan): Task plan to execute
  - `timeout` (float): Maximum time to execute the plan
- **Result Fields**:
  - `success` (bool): Whether task execution was successful
  - `executed_subtasks` (uint8): Number of subtasks executed
  - `failed_subtasks` (uint8): Number of subtasks that failed
  - `total_execution_time` (float): Total time taken to execute the plan
  - `error_message` (string): Error message if execution failed
- **Feedback Fields**:
  - `current_subtask_index` (uint8): Index of currently executing subtask
  - `progress_percentage` (float): Progress percentage (0.0 to 1.0)
  - `current_status` (string): Current status of execution
  - `last_action_result` (ActionResult): Result of last action executed

## Integration with Module 1 (ROS 2 Nervous System)

### Compatibility with Existing Message Types
- VLA system messages extend and integrate with Module 1 message types
- Common message standards ensure interoperability
- Standard ROS 2 interfaces maintain compatibility with existing systems

### Topic Remapping
- VLA topics can be remapped to integrate with existing robot systems
- Configuration allows for flexible topic naming based on robot architecture
- Bridge nodes facilitate communication between different topic naming schemes

## Quality of Service (QoS) Guidelines

### Reliable Communication
- Use `Reliable` QoS for command and control messages
- Critical actions and planning results require guaranteed delivery
- Configuration and status updates may use `Transient Local` for persistence

### Best Effort Communication
- Use `Best Effort` QoS for sensor data and status updates
- Performance metrics and perception results can tolerate occasional loss
- Real-time sensor streams benefit from best-effort delivery

### Deadline and Lifespan
- Set appropriate deadlines for time-sensitive messages
- Configure lifespan for messages that become invalid after a time
- Use liveliness policies to detect node failures

## Security Considerations

### Message Authentication
- Implement message signing for critical commands
- Verify message origin for safety-critical actions
- Use ROS 2 security features for authenticated communication

### Data Privacy
- Encrypt sensitive data transmitted between nodes
- Protect LLM API keys and personal information
- Implement access controls for private topics