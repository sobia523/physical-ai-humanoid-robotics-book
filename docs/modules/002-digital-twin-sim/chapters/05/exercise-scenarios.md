# Exercise Scenarios: Reproducible Digital Twin Workflows

## Overview

This chapter provides reproducible exercise scenarios for humanoid robotics digital twins with standardized expected outcomes. These scenarios are designed to be executed across different hardware configurations while producing consistent, measurable results that validate the functionality and performance of digital twin systems.

## Scenario 1: Basic Digital Twin Validation

### Objective
Validate the core functionality of a sensor-enabled digital twin system with standardized test procedures and expected outcomes.

### Prerequisites
- Gazebo Garden/Harmonic installed
- ROS 2 Humble workspace configured
- Basic humanoid robot URDF model
- LiDAR, IMU, and camera sensors configured

### Setup Requirements
```bash
# Minimum hardware requirements
- CPU: 4+ cores (Intel i5 or equivalent)
- RAM: 8GB minimum
- GPU: OpenGL 3.3+ compatible
- OS: Ubuntu 22.04 LTS or Windows 10/11

# Required packages
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-gazebo-ros-pkgs ros-humble-robot-localization \
  ros-humble-rosbridge-suite
```

### Execution Steps

1. **Launch Basic Simulation**
   ```bash
   # Source ROS 2 workspace
   source /opt/ros/humble/setup.bash
   source ~/robot_ws/install/setup.bash

   # Launch the basic digital twin
   ros2 launch digital_twin_bringup basic_simulation.launch.py
   ```

2. **Verify Sensor Data Streams**
   ```bash
   # Check available topics
   ros2 topic list | grep -E "(scan|imu|camera|odom)"

   # Verify LiDAR data
   ros2 topic echo /humanoid/lidar/scan --field ranges --field range_min --field range_max

   # Verify IMU data
   ros2 topic echo /humanoid/imu/data --field orientation --field angular_velocity --field linear_acceleration

   # Verify camera data
   ros2 topic echo /humanoid/camera/image_raw --field header --field height --field width
   ```

3. **Run Basic Navigation Test**
   ```bash
   # Send simple navigation goal
   ros2 action send_goal /humanoid/navigate_to_pose nav2_msgs/action/NavigateToPose \
     '{pose: {pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, header: {frame_id: map}}}'
   ```

4. **Monitor System Performance**
   ```bash
   # Monitor CPU and memory usage
   htop

   # Monitor simulation real-time factor
   gz stats
   ```

### Expected Outcomes
- All sensor topics publish data at expected rates (LiDAR: 10Hz, IMU: 50Hz, Camera: 30Hz)
- Robot successfully navigates to goal position within 5% tolerance
- Simulation maintains real-time factor > 0.8
- CPU usage < 70% during normal operation
- No TF transform errors reported
- Sensor data ranges within expected parameters

### Success Criteria
- Navigation completes within 30 seconds
- Robot reaches goal position within 0.1m tolerance
- All sensors publish continuous data streams
- No simulation errors or warnings
- Performance metrics remain within specified bounds

### Assessment Questions
1. What was the average real-time factor during the simulation?
2. How many sensor data messages were published during the navigation task?
3. What was the maximum deviation from the expected goal position?
4. Were any TF transform errors reported during the simulation?

## Scenario 2: Performance Optimization Exercise

### Objective
Optimize a digital twin simulation for improved performance while maintaining acceptable simulation fidelity.

### Prerequisites
- Completed Scenario 1
- Understanding of Gazebo physics parameters
- Basic knowledge of ROS 2 QoS settings

### Baseline Configuration
```yaml
# Baseline simulation parameters
simulation:
  timing:
    max_step_size: 0.001
    real_time_factor: 1.0
    max_real_time_update_rate: 1000.0
  physics:
    solver_type: "quick"
    iters: 20
    sor: 1.3
    contact_surface_layer: 0.001
    contact_max_correcting_vel: 100.0
  sensors:
    lidar:
      samples: 720
      update_rate: 10
      max_range: 30.0
    camera:
      width: 640
      height: 480
      update_rate: 30
```

### Execution Steps

1. **Establish Performance Baseline**
   ```bash
   # Run baseline simulation
   ros2 launch digital_twin_bringup performance_test.launch.py --ros-args -p simulation.timing.max_step_size:=0.001

   # Monitor performance for 2 minutes
   ros2 run top top_node > baseline_performance.log &
   gz stats > baseline_gazebo.log &

   # Run navigation task to collect data
   python3 scripts/run_navigation_test.py --duration 120
   ```

2. **Apply Optimization 1: Physics Parameters**
   ```bash
   # Reduce physics solver iterations
   ros2 launch digital_twin_bringup performance_test.launch.py --ros-args \
     -p simulation.physics.iters:=10 \
     -p simulation.timing.max_step_size:=0.002
   ```

3. **Apply Optimization 2: Sensor Parameters**
   ```bash
   # Reduce sensor update rates and resolution
   ros2 launch digital_twin_bringup performance_test.launch.py --ros-args \
     -p sensors.lidar.samples:=360 \
     -p sensors.lidar.update_rate:=5 \
     -p sensors.camera.width:=320 \
     -p sensors.camera.height:=240 \
     -p sensors.camera.update_rate:=15
   ```

4. **Apply Optimization 3: ROS 2 QoS Settings**
   ```python
   # Test with best-effort QoS for sensor data
   # In sensor processing node configuration
   sensor_qos = QoSProfile(
       depth=5,
       reliability=ReliabilityPolicy.BEST_EFFORT,
       durability=DurabilityPolicy.VOLATILE
   )
   ```

5. **Compare Performance Metrics**
   ```bash
   # Collect performance data for each optimization
   python3 scripts/compare_performance.py \
     --baseline baseline_performance.log \
     --optimized optimized_performance.log
   ```

### Expected Outcomes
- Optimization 1: 20-30% CPU usage reduction with &lt;5% accuracy impact
- Optimization 2: 15-25% CPU usage reduction with acceptable sensor data quality
- Optimization 3: Improved message throughput with minimal data loss
- Combined optimizations: Real-time factor maintained > 0.9 while reducing CPU usage by 35-50%

### Success Criteria
- Real-time factor remains > 0.9 after optimizations
- Navigation accuracy degradation &lt; 10%
- CPU usage reduction > 25%
- Sensor data quality remains within acceptable ranges
- No significant increase in message loss rates

### Assessment Questions
1. Which optimization provided the greatest performance improvement?
2. How did the optimizations affect navigation accuracy?
3. What was the trade-off between performance and simulation fidelity?
4. Which sensor parameters were most impactful for performance?

## Scenario 3: Multi-Sensor Integration and Debugging

### Objective
Integrate multiple sensors into a digital twin system and debug common integration issues using systematic approaches.

### Prerequisites
- Understanding of sensor data formats
- Knowledge of TF frame management
- Experience with ROS 2 debugging tools

### Setup Requirements
- LiDAR, IMU, camera, and depth camera sensors configured
- Robot localization system (EKF) running
- RViz2 for visualization

### Execution Steps

1. **Launch Multi-Sensor Simulation**
   ```bash
   # Launch simulation with all sensors
   ros2 launch digital_twin_bringup multi_sensor_simulation.launch.py
   ```

2. **Verify Individual Sensor Data**
   ```bash
   # Check each sensor topic individually
   ros2 topic echo /humanoid/lidar/scan --field ranges -c
   ros2 topic echo /humanoid/imu/data --field orientation -c
   ros2 topic echo /humanoid/camera/image_raw --field header -c
   ros2 topic echo /humanoid/depth_camera/depth/image_raw --field header -c
   ```

3. **Validate TF Transform Chain**
   ```bash
   # Check TF tree structure
   ros2 run tf2_tools view_frames

   # Verify specific transforms
   ros2 run tf2_ros tf2_echo base_link lidar_link
   ros2 run tf2_ros tf2_echo base_link camera_link
   ros2 run tf2_ros tf2_echo base_link imu_link
   ```

4. **Test Sensor Fusion**
   ```bash
   # Launch robot localization (EKF)
   ros2 launch robot_localization ekf.launch.py

   # Monitor fused pose
   ros2 topic echo /humanoid/odometry/filtered --field pose --field twist
   ```

5. **Introduce Common Issues and Debug**
   ```bash
   # Issue 1: Misaligned sensor frames
   # Check for transform errors
   ros2 run tf2_ros tf2_monitor

   # Issue 2: Sensor timing issues
   ros2 topic hz /humanoid/lidar/scan
   ros2 topic hz /humanoid/imu/data

   # Issue 3: Data format mismatches
   ros2 topic info /humanoid/lidar/scan
   ros2 topic info /humanoid/camera/image_raw
   ```

6. **Validate Integration**
   ```bash
   # Run integrated sensor test
   python3 scripts/test_sensor_integration.py

   # Verify data consistency across sensors
   python3 scripts/validate_sensor_data.py
   ```

### Expected Outcomes
- All sensors publish data simultaneously without conflicts
- TF transforms are available and consistent
- Sensor fusion produces reliable state estimates
- No timing or synchronization issues
- Data formats are compatible across all sensors

### Common Issues and Solutions
- **TF Missing Transform**: Verify robot description and spawn parameters
- **Sensor Data Timing**: Check update rates and buffer sizes
- **Coordinate Frame Mismatches**: Validate frame IDs and transform chains
- **Data Type Incompatibilities**: Ensure message types match expectations

### Success Criteria
- All sensors publish continuously without errors
- TF tree is complete and consistent
- Sensor fusion produces stable estimates
- No data loss or corruption observed
- Integration passes validation tests

### Assessment Questions
1. What was the most challenging integration issue encountered?
2. How did you verify the correctness of TF transforms?
3. What validation steps confirmed successful sensor fusion?
4. How would you troubleshoot timing synchronization issues?

## Scenario 4: Unity Integration Validation

### Objective
Validate the integration between Gazebo physics simulation and Unity visualization, ensuring real-time synchronization and data consistency.

### Prerequisites
- Unity 2022.3 LTS installed with Robotics Package
- ROS-TCP-Connector configured
- Completed Gazebo simulation scenarios

### Setup Requirements
```bash
# Unity Robotics Package dependencies
pip install unity-robots
# ROS-TCP-Connector setup
git clone https://github.com/Unity-Technologies/ROS-TCP-Connector.git
```

### Execution Steps

1. **Launch Gazebo Simulation**
   ```bash
   # Start physics simulation
   ros2 launch digital_twin_bringup unity_integration.launch.py
   ```

2. **Start Unity Visualization**
   ```bash
   # Launch Unity scene
   unity_robotics -s UnityDigitalTwinScene.unity
   # Or build and run the Unity application
   ```

3. **Establish ROS-TCP Connection**
   ```csharp
   // Unity side: Connect to ROS
   ROSConnection.instance.rosIPAddress = "127.0.0.1";
   ROSConnection.instance.rosPort = 10000;
   ```

4. **Monitor Synchronization**
   ```bash
   # Check for synchronization messages
   ros2 topic echo /unity_sync_status
   ros2 topic echo /simulation_timing
   ```

5. **Validate Data Consistency**
   ```bash
   # Compare sensor data from Gazebo vs Unity
   ros2 topic echo /gazebo/lidar/scan > gazebo_lidar.log &
   ros2 topic echo /unity/lidar/scan > unity_lidar.log &

   # Run comparison script
   python3 scripts/compare_gazebo_unity_data.py
   ```

6. **Test Real-time Performance**
   ```bash
   # Monitor frame rates and timing
   # Unity: Monitor FPS in console
   # Gazebo: Monitor real-time factor
   # Network: Monitor latency between systems
   ```

### Expected Outcomes
- Unity visualization updates in real-time with Gazebo physics
- Sensor data consistency between simulation and visualization
- Network latency < 50ms for acceptable synchronization
- Unity maintains > 30 FPS during simulation
- No frame drops or synchronization issues

### Success Criteria
- Physics and visualization remain synchronized
- Sensor data matches between systems
- Real-time performance maintained
- No network communication errors
- Smooth user experience in Unity

### Assessment Questions
1. What was the measured network latency between systems?
2. How did you verify data consistency between Gazebo and Unity?
3. What was the Unity application frame rate during simulation?
4. How would you optimize network communication for better performance?

## Scenario 5: Advanced Perception Pipeline

### Objective
Implement and validate an advanced perception pipeline using multiple sensors and processing algorithms in the digital twin environment.

### Prerequisites
- Multi-sensor digital twin (completed Scenario 3)
- Understanding of point cloud processing
- Experience with computer vision algorithms

### Execution Steps

1. **Launch Advanced Perception Setup**
   ```bash
   # Launch simulation with perception nodes
   ros2 launch digital_twin_bringup advanced_perception.launch.py
   ```

2. **Initialize Perception Pipeline**
   ```bash
   # Launch point cloud processing
   ros2 launch perception_pcl process.launch.py

   # Launch computer vision nodes
   ros2 launch vision_opencv image_pipeline.launch.py

   # Launch object detection
   ros2 launch object_detection detect.launch.py
   ```

3. **Process Multi-Sensor Data**
   ```bash
   # Subscribe to multiple sensor streams
   ros2 run perception_pipeline multi_sensor_fusion_node \
     --ros-args -p fusion_method:=point_cloud_projection
   ```

4. **Validate Perception Results**
   ```bash
   # Check detection accuracy
   ros2 topic echo /perception/detected_objects

   # Monitor processing performance
   ros2 topic echo /perception/performance_metrics

   # Verify 3D reconstruction quality
   ros2 topic echo /perception/point_cloud_fused
   ```

5. **Test with Complex Scenarios**
   ```bash
   # Run perception in cluttered environment
   ros2 launch digital_twin_bringup complex_environment.launch.py

   # Execute dynamic obstacle detection
   python3 scripts/test_dynamic_detection.py
   ```

6. **Analyze Results**
   ```bash
   # Generate performance report
   python3 scripts/analyze_perception_performance.py

   # Validate detection accuracy
   python3 scripts/validate_detections.py
   ```

### Expected Outcomes
- Point cloud fusion produces accurate 3D representations
- Object detection identifies > 90% of objects in static scenes
- Dynamic obstacle detection works with < 100ms latency
- Perception pipeline maintains real-time performance
- Sensor fusion improves overall detection accuracy

### Success Criteria
- Detection accuracy > 90% for known objects
- Processing latency < 100ms for real-time operation
- Point cloud fusion produces consistent results
- Pipeline handles sensor failures gracefully
- Performance metrics meet real-time requirements

### Assessment Questions
1. What was the detection accuracy achieved in the complex environment?
2. How did sensor fusion improve perception results compared to single sensors?
3. What was the maximum processing latency observed?
4. How would you optimize the perception pipeline for better performance?

## Hardware Configuration Validation

### Objective
Validate that all exercise scenarios produce consistent results across different hardware configurations.

### Test Configurations
- **Configuration A**: High-end (8+ cores, 32GB RAM, dedicated GPU)
- **Configuration B**: Mid-range (4-6 cores, 16GB RAM, integrated GPU)
- **Configuration C**: Minimum spec (4 cores, 8GB RAM, integrated GPU)

### Validation Process
1. Execute each scenario on all configurations
2. Record performance metrics and execution times
3. Compare results across configurations
4. Document any configuration-specific issues

### Expected Consistency Metrics
- Navigation accuracy should vary by < 5% across configurations
- Simulation real-time factor should maintain > 0.5 on minimum spec
- Sensor data quality should remain consistent regardless of hardware
- Exercise completion rates should be > 95% across all configurations

## Conclusion

These reproducible exercise scenarios provide standardized workflows for validating and testing digital twin systems for humanoid robotics. Each scenario includes clear objectives, execution steps, expected outcomes, and assessment criteria to ensure consistent results across different environments and hardware configurations. The scenarios build upon each other, starting with basic validation and progressing to advanced integration and optimization tasks.

By following these scenarios, developers and students can systematically validate their digital twin implementations, identify performance bottlenecks, and ensure that their systems meet the required specifications for real-time operation. The standardized approach enables comparison of results across different implementations and provides a foundation for continuous improvement of digital twin systems.