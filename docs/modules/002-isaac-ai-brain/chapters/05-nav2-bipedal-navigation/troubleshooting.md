# Troubleshooting Guide for Nav2 Bipedal Navigation

## 1. Navigation Startup Issues

### Problem: Nav2 fails to launch
**Symptoms:**
- Launch command fails with errors
- Nodes don't start properly
- Configuration file errors

**Diagnosis:**
1. Check configuration file syntax:
   ```bash
   # Validate YAML syntax
   python3 -c "import yaml; print(yaml.safe_load(open('bipedal-nav2-config.yaml')))"
   ```

2. Verify all required parameters are present in the config file

3. Check file permissions and paths

**Solutions:**
- Fix any YAML syntax errors in configuration files
- Ensure all referenced files exist and have correct paths
- Verify ROS 2 environment is properly sourced
- Check that all required packages are installed

### Problem: Robot model not found
**Symptoms:**
- URDF not loaded
- TF tree incomplete
- Robot appears invisible in RViz

**Solutions:**
1. Verify URDF path and file existence
2. Check that robot_state_publisher is running
3. Validate joint states are being published
4. Ensure all required TF frames are published

## 2. Localization Issues

### Problem: Robot position inaccurate or drifting
**Symptoms:**
- Robot appears in wrong location
- Position estimate drifts over time
- Navigation fails due to poor localization

**Diagnosis:**
1. Check AMCL parameter configuration
2. Verify sensor data quality (LiDAR, IMU)
3. Examine map quality and feature density
4. Monitor particle filter performance

**Solutions:**
```yaml
# In AMCL configuration, try these adjustments:
amcl:
  ros__parameters:
    # Increase particle count for better estimation
    max_particles: 3000
    min_particles: 1000
    # Improve sensor model
    laser_likelihood_max_dist: 3.0
    # Reduce process noise
    alpha1: 0.1
    alpha2: 0.1
    alpha3: 0.1
    alpha4: 0.1
    alpha5: 0.1
    # Adjust resampling frequency
    resample_interval: 1
```

### Problem: Initial pose estimation fails
**Symptoms:**
- Robot starts with wrong position
- Long time to converge to correct pose
- Localization stuck in wrong location

**Solutions:**
1. Manually set initial pose in RViz with good accuracy
2. Use better initial particle distribution
3. Improve map quality in the starting area
4. Ensure good sensor coverage of the starting area

## 3. Path Planning Problems

### Problem: Path planner fails to find valid path
**Symptoms:**
- Navigation goal rejected
- "No valid path found" error
- Robot doesn't start moving

**Diagnosis:**
1. Check costmap inflation parameters
2. Verify map is loaded and accessible
3. Examine goal position validity
4. Check robot footprint configuration

**Solutions:**
```yaml
# Adjust global costmap parameters
global_costmap:
  global_costmap:
    ros__parameters:
      inflation_radius: 1.0  # Increase to handle narrow spaces
      cost_scaling_factor: 3.0  # Adjust for appropriate inflation
      # Ensure map is properly configured
      track_unknown_space: true
      # Increase tolerance in path planner
planner_server:
  ros__parameters:
    GridBased:
      tolerance: 1.0  # Allow path planning with some tolerance
```

### Problem: Robot gets stuck in oscillation
**Symptoms:**
- Robot moves back and forth
- Controller commands oscillate
- Navigation doesn't progress

**Solutions:**
1. Adjust controller lookahead distance
2. Modify PID controller parameters
3. Increase costmap resolution for smoother planning
4. Reduce maximum velocities for more stable control

## 4. Controller Issues

### Problem: Robot doesn't follow planned path
**Symptoms:**
- Deviation from planned path
- Erratic movement
- Poor tracking performance

**Diagnosis:**
1. Check controller parameters
2. Verify velocity limits
3. Examine transform accuracy
4. Monitor control loop frequency

**Solutions:**
```yaml
# In controller configuration
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Ensure sufficient control frequency
    # For MPPI controller, adjust parameters
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 25  # Number of steps to look ahead
      simulation_time: 1.7  # Time horizon for prediction
      # Adjust penalties for better tracking
      penalty_path_distance: 0.5
      penalty_rotation: 0.2
```

### Problem: Robot falls or loses balance during navigation
**Symptoms:**
- Robot topples over
- Excessive sway or oscillation
- Safety system activates

**Solutions:**
1. **Reduce velocity limits:**
   ```yaml
   velocity_smoother:
     ros__parameters:
       max_velocity: [0.2, 0.0, 0.2]  # Significantly reduced
       max_accel: [1.0, 0.0, 1.5]    # Reduced acceleration
   ```

2. **Increase safety margins:**
   - Use larger robot footprint in costmaps
   - Increase obstacle inflation radius
   - Reduce turning speeds

3. **Adjust controller for stability:**
   - Use slower control frequency if needed
   - Implement smoother velocity profiles
   - Add balance feedback if available

## 5. Costmap Issues

### Problem: Costmaps not updating properly
**Symptoms:**
- Obstacles not detected
- Static costmap not reflecting map
- Local costmap empty or incorrect

**Diagnosis:**
1. Check sensor data topics
2. Verify costmap parameters
3. Examine transform tree
4. Monitor costmap update frequency

**Solutions:**
```yaml
# Ensure proper costmap configuration
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      # Verify observation sources
      observation_sources: scan
      scan:
        topic: /scan  # Verify topic name matches sensor
        data_type: "LaserScan"
        clearing: True
        marking: True
        obstacle_max_range: 3.0
        raytrace_max_range: 4.0
```

### Problem: Costmap shows false obstacles
**Symptoms:**
- Navigation avoiding empty spaces
- Unnecessarily conservative paths
- Costmap shows obstacles where none exist

**Solutions:**
1. Check sensor calibration
2. Adjust sensor noise filtering
3. Verify transform accuracy between sensor and robot
4. Tune costmap obstacle detection parameters

## 6. Recovery Behavior Problems

### Problem: Recovery behaviors not activating
**Symptoms:**
- Robot gets stuck without recovery
- Navigation fails instead of trying alternatives
- No backup or spin behaviors

**Diagnosis:**
1. Check recovery server configuration
2. Verify behavior tree setup
3. Examine timeout and failure conditions

**Solutions:**
```yaml
# Ensure recovery server is properly configured
recovery_server:
  ros__parameters:
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries::Spin"
      spin_dist: 1.57
      time_allowance: 15.0
    backup:
      plugin: "nav2_recoveries::BackUp"
      backup_dist: 0.3
      backup_speed: 0.05
      time_allowance: 15.0
    wait:
      plugin: "nav2_recoveries::Wait"
      wait_duration: 5.0
```

### Problem: Recovery behaviors too aggressive
**Symptoms:**
- Recovery causes robot to fall
- Excessive spinning or backing up
- Recovery fails to solve problem

**Solutions:**
1. **Reduce recovery intensity:**
   ```yaml
   recovery_server:
     ros__parameters:
       spin:
         spin_dist: 0.785  # Reduce from 1.57 (90 degrees)
       backup:
         backup_dist: 0.15  # Reduce from 0.3
         backup_speed: 0.02  # Reduce from 0.05
   ```

## 7. Performance Issues

### Problem: High CPU or memory usage
**Symptoms:**
- System slowdown
- Node timing issues
- Resource exhaustion

**Diagnosis:**
1. Check update frequencies
2. Examine costmap resolution and size
3. Monitor particle filter usage
4. Verify sensor data rates

**Solutions:**
1. **Reduce update frequencies:**
   ```yaml
   local_costmap:
     ros__parameters:
       update_frequency: 2.0  # Reduce from 5.0
   controller_server:
     ros__parameters:
       controller_frequency: 10.0  # Reduce from 20.0
   ```

2. **Optimize costmap parameters:**
   - Lower resolution (increase cell size)
   - Reduce costmap size if possible
   - Reduce particle count in AMCL

### Problem: Navigation delays or timeouts
**Symptoms:**
- Slow path planning
- Navigation cancellation
- Timeout errors

**Solutions:**
1. Increase timeout values in configuration
2. Optimize path planning parameters
3. Check for computational bottlenecks
4. Verify network latency if using distributed system

## 8. Isaac ROS Integration Issues

### Problem: Isaac ROS perception nodes not integrating with Nav2
**Symptoms:**
- SLAM fails to work with navigation
- Sensor data not available to costmaps
- Perception nodes not publishing expected topics

**Solutions:**
1. Verify topic remapping between Isaac ROS nodes and Nav2
2. Check QoS profile compatibility
3. Ensure proper frame ID consistency
4. Validate Isaac ROS configuration files

### Problem: GPU acceleration not working with navigation
**Symptoms:**
- CPU usage high despite GPU availability
- Performance below expected levels
- CUDA errors during navigation

**Solutions:**
1. Verify NVIDIA GPU and driver installation
2. Check CUDA and TensorRT versions
3. Validate Isaac ROS GPU acceleration settings
4. Monitor GPU utilization during operation

## 9. Debugging Commands and Tools

### Useful ROS 2 Commands
```bash
# Check all running nodes
ros2 node list

# Check topic connections
ros2 topic list
ros2 topic info /scan
ros2 topic echo /tf

# Check service availability
ros2 service list

# Monitor navigation status
ros2 action list
ros2 action info /navigate_to_pose

# Check parameters
ros2 param list
ros2 param get <node_name> <parameter_name>
```

### RViz Visualization for Debugging
Add these displays to diagnose navigation issues:
- **TF:** Check transform tree integrity
- **Path:** Visualize global and local paths
- **PoseArray:** Show AMCL particles
- **Map:** Display static and costmap layers
- **LaserScan:** Verify sensor data
- **MarkerArray:** Show path planning results

### Logging and Analysis
```bash
# Enable detailed logging
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# Record navigation data for analysis
ros2 bag record -a -o navigation_debug

# Monitor performance
ros2 run top top
```

## 10. Preventive Measures

### Configuration Validation
- Regularly test configurations in simulation before deployment
- Maintain separate configs for different environments
- Document configuration changes and their effects
- Use version control for all configuration files

### System Health Monitoring
- Monitor CPU, memory, and GPU usage
- Track navigation success rates
- Log and analyze error patterns
- Set up alerts for common issues

### Testing Protocols
- Test in controlled environments before complex scenarios
- Validate robot stability under navigation loads
- Verify sensor reliability in various conditions
- Regular system checks and calibrations