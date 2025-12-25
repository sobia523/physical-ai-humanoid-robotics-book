# Data Model for Module 2: The Digital Twin (Gazebo & Unity)

## Entity: PhysicsEnvironment
**Description**: Represents the physics simulation environment in Gazebo

### Fields:
- `gravity_vector` (array): 3D gravity vector [x, y, z] in m/s² (default: [0, 0, -9.8])
- `physics_engine` (string): Physics engine type (default: "ode")
- `update_rate` (float): Physics update rate in Hz (default: 1000.0)
- `real_time_factor` (float): Real-time update rate multiplier (default: 1.0)
- `erp` (float): Error Reduction Parameter (default: 0.2)
- `cfm` (float): Constraint Force Mixing (default: 1e-5)
- `max_contacts` (int): Maximum contacts per collision (default: 20)

### Validation Rules:
- `gravity_vector` must be a 3-element array
- `update_rate` must be positive (> 0)
- `real_time_factor` must be positive (> 0)
- `erp` must be between 0 and 1
- `cfm` must be positive (> 0)

## Entity: RenderingScene
**Description**: Represents the rendering environment in Unity

### Fields:
- `render_pipeline` (string): Rendering pipeline type (default: "hdrp")
- `lighting_mode` (string): Lighting configuration (default: "global_illumination")
- `anti_aliasing` (string): Anti-aliasing method (default: "taa")
- `resolution_width` (int): Rendering width in pixels (default: 1920)
- `resolution_height` (int): Rendering height in pixels (default: 1080)
- `frame_rate` (int): Target frame rate (default: 60)
- `lod_bias` (float): Level of Detail bias multiplier (default: 1.0)

### Validation Rules:
- `resolution_width` must be positive (> 0)
- `resolution_height` must be positive (> 0)
- `frame_rate` must be positive (> 0)
- `lod_bias` must be positive (> 0)

## Entity: SensorModel
**Description**: Represents a simulated sensor in the digital twin

### Fields:
- `sensor_type` (string): Type of sensor (lidar, depth_camera, imu)
- `sensor_name` (string): Unique identifier for the sensor
- `topic_name` (string): ROS 2 topic name for sensor data
- `position` (array): 3D position [x, y, z] relative to robot (default: [0, 0, 0])
- `orientation` (array): 3D orientation [roll, pitch, yaw] in radians (default: [0, 0, 0])
- `noise_model` (string): Type of noise model (default: "gaussian")
- `noise_variance` (float): Noise variance parameter (default: 0.01)

### Validation Rules:
- `sensor_type` must be one of ["lidar", "depth_camera", "imu"]
- `sensor_name` must be unique within the robot
- `topic_name` must follow ROS 2 naming conventions
- `position` must be a 3-element array
- `orientation` must be a 3-element array

## Entity: LidarSensor (extends SensorModel)
**Description**: Specialized entity for LiDAR sensor simulation

### Fields:
- `range_min` (float): Minimum detection range in meters (default: 0.1)
- `range_max` (float): Maximum detection range in meters (default: 30.0)
- `ray_count` (int): Number of laser rays (default: 720)
- `field_of_view` (float): Field of view in radians (default: 6.28319 for 360°)

### Validation Rules:
- `range_min` must be positive (> 0)
- `range_max` must be greater than `range_min`
- `ray_count` must be positive (> 0)
- `field_of_view` must be between 0 and 6.28319 (2π)

## Entity: DepthCameraSensor (extends SensorModel)
**Description**: Specialized entity for depth camera sensor simulation

### Fields:
- `resolution_width` (int): Image width in pixels (default: 640)
- `resolution_height` (int): Image height in pixels (default: 480)
- `field_of_view` (float): Camera field of view in degrees (default: 60.0)
- `depth_format` (string): Depth data format (default: "float32")

### Validation Rules:
- `resolution_width` must be positive (> 0)
- `resolution_height` must be positive (> 0)
- `field_of_view` must be between 1 and 179 degrees

## Entity: ImuSensor (extends SensorModel)
**Description**: Specialized entity for IMU sensor simulation

### Fields:
- `accel_range` (float): Accelerometer range in g (default: 16.0)
- `gyro_range` (float): Gyroscope range in °/s (default: 2000.0)
- `mag_range` (float): Magnetometer range in µT (default: 4800.0)

### Validation Rules:
- `accel_range` must be positive (> 0)
- `gyro_range` must be positive (> 0)
- `mag_range` must be positive (> 0)

## Entity: RobotModel
**Description**: Represents a humanoid robot model in the simulation

### Fields:
- `model_name` (string): Unique identifier for the robot model
- `urdf_path` (string): Path to URDF file describing the robot
- `joint_count` (int): Number of joints in the robot
- `link_count` (int): Number of links in the robot
- `mass` (float): Total mass of the robot in kg
- `base_frame` (string): Base frame ID for the robot (default: "base_link")

### Validation Rules:
- `model_name` must be unique
- `urdf_path` must be a valid file path
- `joint_count` must be positive (> 0)
- `link_count` must be positive (> 0)
- `mass` must be positive (> 0)

## Entity: SimulationConfiguration
**Description**: Overall configuration for the digital twin simulation

### Fields:
- `simulation_name` (string): Name of the simulation
- `physics_environment` (PhysicsEnvironment): Physics environment settings
- `rendering_scene` (RenderingScene): Rendering scene settings
- `robot_model` (RobotModel): Robot model to simulate
- `sensors` (array): Array of SensorModel instances
- `start_time` (float): Simulation start time in seconds (default: 0.0)
- `duration` (float): Simulation duration in seconds (default: 0.0 for indefinite)

### Validation Rules:
- `simulation_name` must be unique
- `physics_environment` must be a valid PhysicsEnvironment
- `rendering_scene` must be a valid RenderingScene
- `robot_model` must be a valid RobotModel
- `sensors` must contain only valid SensorModel instances
- `duration` must be non-negative (≥ 0)