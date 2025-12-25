# Data Model: Module 1: The Robotic Nervous System (ROS 2)

**Feature**: 001-ros2-nervous-system
**Created**: 2025-12-19
**Status**: Complete

## Entity: ROS 2 Node

**Definition**: A process that performs computation, implementing communication with other nodes through publishers, subscribers, services, actions, and parameters.

**Attributes**:
- `node_name`: String - Unique identifier for the node within the ROS 2 system
- `node_namespace`: String - Optional namespace to organize nodes hierarchically
- `parameters`: Dictionary - Configurable values that can be set at runtime
- `publishers`: List - Collection of publishers created by this node
- `subscribers`: List - Collection of subscribers created by this node
- `services`: List - Collection of services provided by this node
- `clients`: List - Collection of service clients used by this node
- `timers`: List - Collection of timer callbacks managed by this node

**Relationships**:
- One-to-many: A node can create multiple publishers, subscribers, services, etc.
- Many-to-many: Nodes communicate with other nodes through topics and services

**Validation Rules**:
- Node name must follow ROS naming conventions (alphanumeric, underscores, starting with letter)
- Node must initialize with rclpy before creating any communication entities
- Node must properly clean up all resources on destruction

**State Transitions**:
- `Uninitialized` → `Initialized` → `Active` → `Destroyed`

## Entity: ROS 2 Topic

**Definition**: A communication channel through which ROS 2 nodes exchange messages of a specific type.

**Attributes**:
- `topic_name`: String - Unique identifier for the topic within the ROS 2 system
- `message_type`: String - Type definition of messages published/subscribed to this topic
- `qos_profile`: QoSProfile - Quality of service settings for message delivery
- `publisher_count`: Integer - Number of publishers connected to this topic
- `subscription_count`: Integer - Number of subscribers connected to this topic

**Relationships**:
- Many-to-many: Topics connect multiple publishers and subscribers
- One-to-many: Each topic has messages of a specific type

**Validation Rules**:
- Topic names must follow ROS naming conventions
- Message types must be valid ROS 2 message definitions
- QoS profiles must be compatible between publishers and subscribers

## Entity: ROS 2 Message

**Definition**: A data structure used to exchange information between ROS 2 nodes through topics.

**Attributes**:
- `message_type`: String - The specific ROS message type (e.g., std_msgs/String, sensor_msgs/JointState)
- `fields`: List - Collection of field definitions with names and data types
- `timestamp`: Time - Timestamp of message creation (if applicable)
- `header`: Header - Standard header containing frame_id and timestamp (for some message types)

**Relationships**:
- Many-to-one: Multiple messages of the same type can be published to a topic
- One-to-many: A message type can be used by multiple topics

**Validation Rules**:
- Message structure must match the defined message type specification
- Required fields must be populated
- Data types must match field definitions

## Entity: URDF Model

**Definition**: An XML format for representing robot descriptions, including links (rigid bodies), joints (kinematic and dynamic properties), and visual elements.

**Attributes**:
- `model_name`: String - Name of the robot model
- `links`: List - Collection of rigid body elements that make up the robot
- `joints`: List - Collection of joint elements connecting links
- `materials`: List - Collection of visual materials used in the model
- `gazebo_extensions`: List - Gazebo-specific extensions for simulation
- `transmissions`: List - Transmission elements for actuator control

**Relationships**:
- One-to-many: A URDF model contains multiple links and joints
- Many-to-many: Joints connect pairs of links in a kinematic chain

**Validation Rules**:
- Must have a single root link in the kinematic tree
- Joint names must be unique within the model
- Joint parent/child relationships must form a valid kinematic tree
- Inertial properties must be properly defined for dynamic simulation

## Entity: Link

**Definition**: A rigid body element in a URDF model representing a physical part of the robot.

**Attributes**:
- `name`: String - Unique identifier for the link
- `inertial`: Inertial - Mass, center of mass, and inertia matrix
- `visual`: Visual - Visual representation for graphics
- `collision`: Collision - Collision representation for physics simulation
- `origin`: Pose - Position and orientation relative to parent (for child links)

**Relationships**:
- One-to-many: A link can be connected to multiple joints
- One-to-one: Each link has one inertial, visual, and collision definition

**Validation Rules**:
- Name must be unique within the URDF model
- Inertial properties must be physically valid (positive mass, valid inertia matrix)
- Visual and collision geometries must be properly defined

## Entity: Joint

**Definition**: A connection between two links in a URDF model, defining the kinematic and dynamic relationship.

**Attributes**:
- `name`: String - Unique identifier for the joint
- `type`: String - Joint type (revolute, continuous, prismatic, fixed, etc.)
- `parent`: String - Name of the parent link
- `child`: String - Name of the child link
- `origin`: Pose - Position and orientation of the joint relative to parent
- `axis`: Vector3 - Joint axis of motion
- `limits`: JointLimits - Motion limits for revolute and prismatic joints
- `dynamics`: JointDynamics - Dynamic properties like damping and friction

**Relationships**:
- Many-to-one: Multiple joints can connect to the same parent link
- One-to-many: A joint connects exactly one parent to one child link

**Validation Rules**:
- Name must be unique within the URDF model
- Parent and child links must exist in the model
- Joint type must be valid (revolute, continuous, prismatic, fixed, floating, planar)
- Limits must be consistent with joint type

## Entity: Python Agent

**Definition**: A Python program that implements AI or control logic and integrates with ROS 2 for robot control.

**Attributes**:
- `agent_name`: String - Name of the agent
- `ros_node`: ROS2Node - The ROS 2 node instance this agent uses
- `input_handlers`: List - Functions to handle incoming ROS messages
- `output_publishers`: List - Publishers used to send commands to the robot
- `control_logic`: Function - The core AI/control algorithm
- `state_variables`: Dictionary - Internal state maintained by the agent

**Relationships**:
- One-to-one: An agent typically uses one ROS 2 node (though it can use multiple)
- Many-to-many: An agent can communicate with multiple ROS 2 topics and services

**Validation Rules**:
- Must properly initialize and clean up ROS 2 resources
- Control logic must handle errors gracefully
- Should follow ROS 2 best practices for node implementation

## Entity: Simulation Environment

**Definition**: A physics-based environment that simulates the robot and its interactions with the world.

**Attributes**:
- `simulation_name`: String - Name of the simulation environment
- `world_file`: String - Path to the world description file
- `robot_models`: List - URDF models loaded into the simulation
- `physics_engine`: String - Physics engine used (ODE, Bullet, etc.)
- `gravity`: Vector3 - Gravity vector applied to the simulation
- `real_time_factor`: Float - Ratio of simulation time to real time

**Relationships**:
- One-to-many: A simulation environment can host multiple robot models
- Many-to-one: Multiple ROS 2 nodes can interface with the same simulation

**Validation Rules**:
- URDF models must be valid and loadable
- Physics parameters must be physically reasonable
- Real-time factor should be appropriate for the use case