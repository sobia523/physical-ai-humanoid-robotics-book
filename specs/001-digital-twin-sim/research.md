# Research for Module 2: The Digital Twin (Gazebo & Unity)

## Research Findings Summary

### Decision: Gazebo Physics Parameters for Humanoid Simulation
**Rationale**: Based on Gazebo documentation and robotics research, we'll use ODE (Open Dynamics Engine) physics engine with specific parameters optimized for humanoid robot simulation.
- Gravity: 9.8 m/s² (standard Earth gravity)
- Update rate: 1000 Hz for accurate physics
- ERP (Error Reduction Parameter): 0.2 for stable joints
- CFM (Constraint Force Mixing): 1e-5 for realistic contact

**Alternatives considered**: Bullet physics engine, DART physics engine - ODE chosen for its widespread use in robotics simulation and compatibility with ROS 2.

### Decision: Unity Version and Packages for Robotics
**Rationale**: Unity 2022.3 LTS (Long Term Support) with Unity Robotics Package (URP) and Visual Design Toolkit.
- Unity 2022.3 LTS for stability and long-term support
- Unity Robotics Package for ROS 2 integration
- Visual Design Toolkit for high-fidelity rendering
- Unity Simulation package for large-scale environment rendering

**Alternatives considered**: Unity 2023.x versions, Unreal Engine - Unity chosen for its robotics ecosystem and integration tools.

### Decision: ROS 2 Message Types for Sensor Simulation
**Rationale**: Standard ROS 2 message types that are widely supported and compatible with existing tools:
- LiDAR: `sensor_msgs/msg/LaserScan`
- Depth Cameras: `sensor_msgs/msg/Image` (depth format) + `sensor_msgs/msg/CameraInfo`
- IMUs: `sensor_msgs/msg/Imu`
- Joint States: `sensor_msgs/msg/JointState`

**Alternatives considered**: Custom message types - standard types chosen for compatibility with existing ROS 2 tools and frameworks.

### Decision: Simulation Performance Optimization
**Rationale**: Balanced approach between fidelity and performance:
- Physics update rate: 1000 Hz for accuracy, with variable rendering rate (30-60 FPS)
- Level of Detail (LOD) systems for complex environments
- Occlusion culling for rendering optimization
- Multi-threading for physics and rendering separation

**Alternatives considered**: Fixed performance settings - adaptive approach chosen to accommodate different hardware configurations.

## Detailed Research Outcomes

### 1. Gazebo Physics Research
Based on the official Gazebo documentation and research papers on humanoid robotics simulation:

- **ODE Physics Engine Settings**:
  - Solver type: Quick (for real-time simulation)
  - Min step size: 0.001 seconds (1000 Hz)
  - Real time update rate: 1.0 (matches real-time)
  - Max contacts: 20 (for complex contact scenarios)

- **Humanoid-Specific Parameters**:
  - Joint friction: 0.1-0.2 (realistic for servo actuators)
  - Joint damping: 0.01-0.1 (for natural movement)
  - Collision mesh simplification: 0.01m resolution

### 2. Unity Robotics Integration Research
Based on Unity Robotics documentation and best practices:

- **Required Packages**:
  - Unity Robotics Package (com.unity.robotics.urp)
  - ROS-TCP-Connector for ROS 2 communication
  - Unity Simulation for large-scale environments
  - HDRP (High Definition Render Pipeline) for high-fidelity rendering

- **Rendering Settings**:
  - Real-time Global Illumination for realistic lighting
  - Anti-aliasing: TAA (Temporal Anti-Aliasing)
  - Post-processing effects for visual enhancement

### 3. Sensor Simulation Research
Based on ROS 2 documentation and sensor simulation best practices:

- **LiDAR Simulation**:
  - Ray count: 720 rays for 360° coverage
  - Range: 0.1m to 30m
  - Noise model: Gaussian with configurable variance
  - Topic: `/sensor/lidar_scan`

- **Depth Camera Simulation**:
  - Resolution: 640x480 (configurable)
  - Field of view: 60°
  - Noise model: Gaussian depth noise
  - Topics: `/sensor/depth_camera/image_raw`, `/sensor/depth_camera/camera_info`

- **IMU Simulation**:
  - Accelerometer range: ±16g
  - Gyroscope range: ±2000°/s
  - Magnetometer range: ±4800 µT
  - Topic: `/sensor/imu/data`

### 4. Performance Optimization Research
Based on robotics simulation performance studies:

- **Hardware Requirements**:
  - Minimum: 8GB RAM, Quad-core CPU, Dedicated GPU with 2GB VRAM
  - Recommended: 16GB RAM, Hexa-core CPU, Dedicated GPU with 4GB+ VRAM

- **Optimization Techniques**:
  - Level of Detail (LOD) for distant objects
  - Occlusion culling to avoid rendering hidden objects
  - Multi-threaded physics and rendering
  - Fixed timestep for physics stability