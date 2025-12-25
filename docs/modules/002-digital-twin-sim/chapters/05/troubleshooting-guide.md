# Troubleshooting Guide for Digital Twin Simulation

## Overview

This guide provides solutions for common issues encountered when working with digital twin simulations for humanoid robotics. The guide covers problems related to Gazebo physics simulation, ROS 2 communication, Unity visualization, sensor integration, and performance optimization.

## Common Issues and Solutions

### 1. Gazebo Simulation Issues

#### Robot Falls Through Ground
**Symptoms**: Robot model falls through the ground plane or other static objects
**Causes**: Insufficient friction, collision parameters, or physics settings
**Solutions**:
1. Increase friction coefficients in the URDF/SDF:
   ```xml
   <surface>
     <friction>
       <ode>
         <mu>0.8</mu>
         <mu2>0.8</mu2>
       </ode>
     </friction>
   </surface>
   ```
2. Adjust contact parameters:
   ```xml
   <surface>
     <contact>
       <ode>
         <kp>1e9</kp>  <!-- Increase stiffness -->
         <kd>1e6</kd>  <!-- Increase damping -->
       </ode>
     </contact>
   </surface>
   ```
3. Verify collision geometry is properly defined
4. Check that the robot has appropriate mass distribution

#### Unstable or Erratic Movements
**Symptoms**: Robot exhibits jittery, oscillating, or unstable behavior
**Causes**: Physics parameters too aggressive or insufficient solver iterations
**Solutions**:
1. Reduce solver iterations and adjust parameters:
   ```xml
   <physics type="ode">
     <ode>
       <solver>
         <type>quick</type>
         <iters>20</iters>  <!-- Reduce if still unstable -->
         <sor>1.3</sor>
       </solver>
       <constraints>
         <cfm>0.0</cfm>
         <erp>0.2</erp>
       </constraints>
     </ode>
   </physics>
   ```
2. Increase collision surface layer:
   ```xml
   <contact>
     <ode>
       <contact_surface_layer>0.001</contact_surface_layer>
     </ode>
   </contact>
   ```
3. Verify joint limits and damping parameters in URDF

#### Joint Limits Not Working
**Symptoms**: Robot joints exceed defined limits or behave unexpectedly
**Solutions**:
1. Verify joint limits in URDF:
   ```xml
   <joint name="joint_name" type="revolute">
     <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
     <dynamics damping="0.1" friction="0.0"/>
   </joint>
   ```
2. Check Gazebo plugin configuration for joint controllers
3. Ensure proper joint transmission setup

### 2. ROS 2 Communication Issues

#### No Sensor Data Publishing
**Symptoms**: Sensor topics show no messages or very low message rates
**Solutions**:
1. Check Gazebo sensor plugin configuration:
   ```xml
   <sensor name="lidar" type="ray">
     <ray>
       <scan>
         <horizontal>
           <samples>360</samples>
           <resolution>1</resolution>
           <min_angle>-3.14159</min_angle>
           <max_angle>3.14159</max_angle>
         </horizontal>
       </scan>
     </ray>
     <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
       <ros>
         <namespace>/humanoid</namespace>
         <remapping>~/out:=lidar/scan</remapping>
       </ros>
       <output_type>sensor_msgs/LaserScan</output_type>
     </plugin>
   </sensor>
   ```
2. Verify topic names with `ros2 topic list`
3. Check frame IDs match TF tree
4. Confirm sensor parameters in URDF/SDF

#### TF Transform Errors
**Symptoms**: Missing transforms, "Could not find a connection between..." errors
**Solutions**:
1. Check TF tree with:
   ```bash
   ros2 run tf2_tools view_frames
   ```
2. Verify robot description (URDF) has proper joint definitions
3. Ensure robot_state_publisher is running:
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="..."
   ```
4. Check joint_state_publisher is publishing joint states
5. Verify frame names are consistent across all components

#### High Message Latency
**Symptoms**: Delayed messages, poor real-time performance
**Solutions**:
1. Adjust QoS settings for sensor data:
   ```python
   sensor_qos = QoSProfile(
       depth=5,  # Reduce buffer size
       reliability=ReliabilityPolicy.BEST_EFFORT,  # Use best effort for sensors
       durability=DurabilityPolicy.VOLATILE
   )
   ```
2. Reduce sensor update rates in Gazebo configuration
3. Check network connectivity for distributed systems
4. Monitor system resources (CPU, memory usage)

### 3. Unity Integration Issues

#### Connection Failures with ROS-TCP-Connector
**Symptoms**: Unity cannot connect to ROS 2, connection timeouts
**Solutions**:
1. Verify IP address and port configuration:
   ```csharp
   // In Unity script
   ros.rosIPAddress = "127.0.0.1";
   ros.rosPort = 10000;  // Ensure this matches ROS bridge port
   ```
2. Check ROS bridge is running:
   ```bash
   ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=10000
   ```
3. Ensure firewall allows connections on the specified port
4. Verify both systems are on the same network (for distributed setups)

#### Visualization Lag or Desync
**Symptoms**: Unity visualization not synchronized with Gazebo physics
**Solutions**:
1. Reduce Unity update rate to match Gazebo physics rate
2. Implement proper time synchronization:
   ```csharp
   // In Unity, throttle updates
   void Update()
   {
       if (Time.time < nextUpdateTime)
           return;
       nextUpdateTime = Time.time + (1f / updateRate);
       // Process updates
   }
   ```
3. Check network latency between systems
4. Reduce complexity of Unity scene if performance is poor

#### Sensor Data Not Appearing in Unity
**Symptoms**: Sensor data available in ROS 2 but not visible in Unity
**Solutions**:
1. Verify Unity subscribers are properly configured
2. Check message type compatibility between ROS 2 and Unity
3. Ensure proper topic remapping
4. Test with simple messages first (e.g., Float32) to verify connection

### 4. Sensor-Specific Issues

#### LiDAR Data Problems
**Symptoms**: Empty ranges, incorrect distances, or no data
**Solutions**:
1. Check range parameters in SDF:
   ```xml
   <sensor type="ray" name="lidar">
     <ray>
       <range>
         <min>0.1</min>
         <max>30.0</max>  <!-- Ensure max range is appropriate -->
         <resolution>0.01</resolution>
       </range>
     </ray>
   </sensor>
   ```
2. Verify scan parameters (samples, resolution, angles)
3. Check for ray intersection with robot body causing self-detection
4. Adjust noise parameters if data appears too clean or too noisy

#### Camera Issues
**Symptoms**: Black images, incorrect colors, low resolution
**Solutions**:
1. Verify camera parameters:
   ```xml
   <sensor type="camera" name="camera">
     <camera>
       <horizontal_fov>1.0472</horizontal_fov>  <!-- 60 degrees -->
       <image>
         <width>640</width>
         <height>480</height>
         <format>R8G8B8</format>
       </image>
       <clip>
         <near>0.1</near>
         <far>10.0</far>
       </clip>
     </camera>
   </sensor>
   ```
2. Check lighting in Gazebo world
3. Verify camera mounting and orientation
4. Ensure proper image format compatibility

#### IMU Data Problems
**Symptoms**: Drifting values, incorrect orientation, high noise
**Solutions**:
1. Verify IMU configuration:
   ```xml
   <sensor type="imu" name="imu_sensor">
     <imu>
       <angular_velocity>
         <x>
           <noise type="gaussian">
             <mean>0.0</mean>
             <stddev>0.01</stddev>
           </noise>
         </x>
       </angular_velocity>
       <linear_acceleration>
         <z>
           <noise type="gaussian">
             <mean>0.0</mean>
             <stddev>0.017</stddev>
           </noise>
         </z>
       </linear_acceleration>
     </imu>
   </sensor>
   ```
2. Check frame alignment and orientation
3. Verify gravity is properly simulated
4. Ensure IMU is mounted in appropriate location

### 5. Performance Issues

#### Low Simulation Speed (RTF &lt; 1.0)
**Symptoms**: Simulation running slower than real-time
**Solutions**:
1. Increase physics step size:
   ```xml
   <physics>
     <max_step_size>0.01</max_step_size>  <!-- Increase from 0.001 -->
   </physics>
   ```
2. Reduce solver iterations
3. Simplify collision geometries
4. Reduce sensor update rates
5. Disable unnecessary plugins

#### High CPU Usage
**Symptoms**: CPU usage > 80%, system slowdown
**Solutions**:
1. Reduce number of active sensors
2. Lower sensor resolution and update rates
3. Simplify visual models in Gazebo
4. Use efficient algorithms in ROS 2 nodes
5. Implement multithreading where appropriate

#### Memory Leaks
**Symptoms**: Memory usage increasing over time
**Solutions**:
1. Implement proper resource cleanup in custom nodes
2. Use object pooling for frequently allocated objects
3. Monitor memory usage with tools like `htop`
4. Check for circular references in data structures
5. Profile memory usage with appropriate tools

### 6. Navigation and Control Issues

#### Robot Not Following Commands
**Symptoms**: Robot doesn't respond to velocity commands or navigation goals
**Solutions**:
1. Verify joint controller configuration:
   ```yaml
   controller_manager:
     ros__parameters:
       update_rate: 100
       use_sim_time: true

   diff_drive_controller:
     type: diff_drive_controller/DiffDriveController
     ros__parameters:
       left_wheel_names: ["left_wheel_joint"]
       right_wheel_names: ["right_wheel_joint"]
       wheel_separation: 0.5
       wheel_radius: 0.1
   ```
2. Check TF tree for proper wheel transforms
3. Verify command topic subscriptions
4. Ensure proper velocity limits and acceleration parameters

#### Poor Navigation Performance
**Symptoms**: Robot takes inefficient paths, fails to reach goals, gets stuck
**Solutions**:
1. Adjust costmap parameters:
   ```yaml
   local_costmap:
     plugins:
       - {name: obstacles, type: "nav2_costmap_2d::ObstacleLayer"}
       - {name: inflation, type: "nav2_costmap_2d::InflationLayer"}
     obstacle_range: 3.0
     raytrace_range: 3.5
     inflation_radius: 0.55
   ```
2. Verify sensor data quality and coverage
3. Check map resolution and accuracy
4. Adjust planner parameters for your specific robot

## Diagnostic Commands

### ROS 2 Diagnostics
```bash
# Check all topics and their types
ros2 topic list -t

# Monitor topic message rates
ros2 topic hz /humanoid/lidar/scan

# Echo specific topic
ros2 topic echo /humanoid/odom

# Check available services
ros2 service list

# Check active nodes
ros2 node list
```

### Gazebo Diagnostics
```bash
# Check simulation status
gz stats

# List all models in simulation
gz model -m

# Check topic information
gz topic -t /gazebo/worlds -m
```

### TF Diagnostics
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo base_link lidar_link

# Monitor TF for errors
ros2 run tf2_ros tf2_monitor
```

## Prevention Strategies

### 1. Systematic Testing
- Test individual components before integration
- Use minimal configurations to isolate issues
- Implement automated validation tests
- Monitor performance metrics continuously

### 2. Configuration Management
- Use version control for all configuration files
- Maintain separate configs for different hardware tiers
- Document all parameter changes and their effects
- Create validation scripts for configuration files

### 3. Monitoring and Logging
- Implement comprehensive logging in all components
- Monitor resource usage during simulation
- Set up alerts for performance thresholds
- Regularly review logs for potential issues

## When to Seek Additional Help

If troubleshooting steps don't resolve the issue:

1. Check the official documentation for Gazebo, ROS 2, and Unity Robotics
2. Search the ROS Answers forum and Gazebo answers
3. Review the project's GitHub issues for similar problems
4. Consider creating a minimal reproducible example for community support
5. Verify that your system meets minimum hardware requirements

## Hardware-Specific Considerations

### Low-End Systems (4 cores, 8GB RAM)
- Reduce simulation complexity
- Lower sensor resolutions and update rates
- Disable non-essential visualization features
- Use simplified collision models

### Mid-Range Systems (4-6 cores, 16GB RAM)
- Balance quality and performance
- Enable most features with moderate settings
- Monitor resource usage during operation

### High-End Systems (8+ cores, 32GB+ RAM)
- Maximize simulation quality
- Enable advanced features
- Focus on accuracy over performance

## Conclusion

This troubleshooting guide provides solutions for common issues in digital twin simulations. Remember to approach problems systematically, test components individually, and maintain detailed logs of your configurations and changes. When encountering new issues, try to isolate the problem to a specific component (Gazebo, ROS 2, Unity, or sensors) and apply targeted solutions.