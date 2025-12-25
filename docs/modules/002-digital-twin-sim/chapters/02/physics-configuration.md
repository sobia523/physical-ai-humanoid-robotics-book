# Gazebo Physics Configuration and Plugin Setup

## Introduction

This chapter provides detailed instructions for configuring physics parameters and plugins in Gazebo for realistic humanoid robot simulation. Proper configuration of physics parameters is essential for achieving stable, accurate, and realistic simulation behavior that closely matches real-world physics.

## Physics Engine Configuration

### Selecting the Appropriate Physics Engine

Gazebo supports multiple physics engines, each with different characteristics suitable for specific applications:

- **ODE (Open Dynamics Engine)**: Default and most commonly used engine, ideal for stable multi-body systems
- **Bullet**: Offers advanced collision detection capabilities
- **DART**: Provides advanced kinematic and dynamic capabilities for complex systems

For humanoid robotics applications, ODE is typically the recommended choice due to its stability with multi-link systems.

### Core Physics Parameters

The following parameters form the foundation of realistic physics simulation:

```xml
<physics name="ode_physics" type="ode">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>1e-5</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

#### Parameter Explanations

- **gravity**: Standard Earth gravity (9.8 m/s²) directed downward along the Z-axis
- **max_step_size**: 0.001 seconds (1000 Hz) for accurate physics simulation, especially important for humanoid dynamics
- **real_time_factor**: 1.0 to maintain real-time simulation (1:1 ratio with real time)
- **real_time_update_rate**: 1000.0 Hz for high-fidelity physics calculations
- **solver/type**: Quick solver optimized for real-time simulation
- **solver/iters**: 10 iterations for constraint solving (balance between accuracy and performance)
- **solver/sor**: 1.3 Successive Over-Relaxation parameter for convergence
- **constraints/cfm**: 1e-5 Constraint Force Mixing for numerical stability
- **constraints/erp**: 0.2 Error Reduction Parameter for stable joint constraints
- **contact_max_correcting_vel**: 100.0 m/s maximum contact correction velocity
- **contact_surface_layer**: 0.001 m surface penetration tolerance

### Tuning Physics Parameters for Humanoid Robots

Humanoid robots have specific requirements that may necessitate parameter adjustments:

#### For Stable Walking
- Increase ERP to 0.3-0.4 for more stable foot-ground contact
- Reduce CFM to 1e-6 for more rigid contact constraints
- Consider increasing solver iterations to 15-20 for complex multi-point contact scenarios

#### For Fast Motion
- Maintain max_step_size at 0.001 or reduce to 0.0005 for very fast movements
- Increase real_time_update_rate to 2000 Hz if computational resources allow
- Use higher solver iterations (20-30) for accuracy during high-velocity motion

#### For Computational Efficiency
- Increase max_step_size to 0.002 (500 Hz) for less demanding simulations
- Reduce solver iterations to 5-8 for faster computation
- Adjust ERP to 0.1-0.15 for less stringent constraint enforcement

## Joint Configuration Parameters

### Joint Limits and Constraints

Proper joint configuration is critical for realistic humanoid motion:

```xml
<joint name="shoulder_joint" type="revolute">
  <parent>torso</parent>
  <child>upper_arm</child>
  <axis xyz="0 1 0">
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
    <dynamics damping="0.2" friction="0.15"/>
  </axis>
</joint>
```

#### Joint Limit Configuration

- **Lower/Upper Limits**: Define the physical range of motion based on the real robot's capabilities
- **Effort Limits**: Represent maximum torque/force the actuator can provide (in N for prismatic, N-m for revolute)
- **Velocity Limits**: Represent the maximum speed of the joint (in m/s for prismatic, rad/s for revolute)

#### Joint Dynamics Parameters

- **Damping**: Energy dissipation in the joint (0.1-0.5 for large joints, 0.01-0.1 for small joints)
- **Friction**: Static and dynamic friction values (0.1-0.3 for actuated joints)

### Joint Types for Humanoid Applications

#### Revolute Joints
Most common for humanoid robots, used for rotational joints like elbows, knees, and shoulders:

```xml
<joint name="elbow_joint" type="revolute">
  <parent>upper_arm</parent>
  <child>lower_arm</child>
  <axis xyz="0 0 1"/>
  <limit lower="-2.0" upper="0.5" effort="30" velocity="3"/>
  <dynamics damping="0.15" friction="0.1"/>
</joint>
```

#### Continuous Joints
For joints with unlimited rotation (e.g., some waist joints):

```xml
<joint name="waist_rotation" type="continuous">
  <parent>torso</parent>
  <child>pelvis</child>
  <axis xyz="0 0 1"/>
  <dynamics damping="0.2" friction="0.1"/>
</joint>
```

#### Fixed Joints
For non-moving connections (e.g., sensor mounts):

```xml
<joint name="sensor_mount" type="fixed">
  <parent>head</parent>
  <child>camera_link</child>
</joint>
```

## Collision and Contact Configuration

### Surface Properties

Configure realistic interaction between objects using surface properties:

```xml
<collision name="link_collision">
  <geometry>
    <box>
      <size>0.1 0.1 0.1</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.8</mu>  <!-- Primary friction coefficient -->
        <mu2>0.8</mu2>  <!-- Secondary friction coefficient -->
        <slip1>0.0</slip1>  <!-- Slip in primary direction -->
        <slip2>0.0</slip2>  <!-- Slip in secondary direction -->
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.1</restitution_coefficient>  <!-- Bounciness -->
      <threshold>100000.0</threshold>  <!-- Velocity threshold for bouncing -->
    </bounce>
    <contact>
      <ode>
        <soft_cfm>0.000001</soft_cfm>  <!-- Soft constraint force mixing -->
        <soft_erp>0.2</soft_erp>  <!-- Soft error reduction parameter -->
        <kp>1000000000000.0</kp>  <!-- Contact stiffness -->
        <kd>1.0</kd>  <!-- Damping coefficient -->
        <max_vel>100.0</max_vel>  <!-- Maximum contact correction velocity -->
        <min_depth>0.001</min_depth>  <!-- Penetration depth threshold -->
      </ode>
    </contact>
  </surface>
</collision>
```

### Contact Parameter Tuning

#### For Humanoid Foot-Ground Interaction
- **mu**: 0.8-1.0 for good grip during walking
- **restitution_coefficient**: 0.0-0.1 for minimal bouncing
- **kp**: High values (1e12) for firm contact
- **soft_erp**: 0.2-0.4 for stable contact resolution

#### For Object Manipulation
- **mu**: 0.6-0.9 for good grip on objects
- **restitution_coefficient**: 0.1-0.3 for realistic object handling
- **min_depth**: 0.001-0.002 m for appropriate contact depth

## Plugin Configuration

### Physics-Related Plugins

Gazebo plugins extend functionality for specific simulation needs:

#### Joint Controllers
For controlling joint positions, velocities, or efforts:

```xml
<plugin name="position_controller" filename="libgazebo_ros_joint_position.so">
  <robotNamespace>/simple_humanoid</robotNamespace>
  <jointName>left_shoulder_joint</jointName>
  <topicName>/joint_position</topicName>
  <updateRate>100</updateRate>
  <robotParam>robot_description</robotParam>
  <commandTopic>command</commandTopic>
  <stateTopic>state</stateTopic>
</plugin>
```

#### Sensor Plugins
For simulating various sensors:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <pose>0 0 0 0 0 0</pose>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <robotNamespace>/simple_humanoid</robotNamespace>
    <topicName>/sensor/imu/data</topicName>
    <serviceName>/imu_service</serviceName>
    <gaussianNoise>0.001</gaussianNoise>
  </plugin>
</sensor>
```

#### Force/Torque Sensors
For measuring interaction forces:

```xml
<sensor name="ft_sensor" type="force_torque">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <force_torque>
    <frame>sensor</frame>
    <measure_direction>child_to_parent</measure_direction>
  </force_torque>
  <plugin name="ft_plugin" filename="libgazebo_ros_ft_sensor.so">
    <robotNamespace>/simple_humanoid</robotNamespace>
    <topicName>/sensor/ft_data</topicName>
  </plugin>
</sensor>
```

### Plugin Integration with Physics

When configuring plugins, consider their interaction with the physics engine:

- **Update Rates**: Ensure plugin update rates are compatible with physics update rates
- **Computational Load**: Balance plugin complexity with real-time performance requirements
- **Timing Consistency**: Maintain consistent timing between physics simulation and plugin operations

## Performance Optimization

### Physics Performance Settings

Optimize physics simulation for better performance while maintaining accuracy:

#### For Real-time Simulation
- max_step_size: 0.001 (1000 Hz) for balance of accuracy and performance
- solver/iters: 10 for reasonable constraint solving
- max_contacts: 20 per collision pair to limit computational complexity

#### For High Accuracy (Non-real-time)
- max_step_size: 0.0001 (10000 Hz) for maximum accuracy
- solver/iters: 50-100 for precise constraint solving
- real_time_factor: 0 (unlimited) to prioritize accuracy over real-time performance

### Collision Optimization

- **Simplified Collision Models**: Use simplified meshes for physics while maintaining detailed visuals
- **Collision Filtering**: Use collision masks to reduce unnecessary collision checks
- **Contact Reduction**: Limit the number of contact points for complex geometries

## Troubleshooting Common Issues

### Unstable Simulation
**Symptoms**: Robot shakes, falls over easily, joints oscillate
**Solutions**:
- Increase ERP value (try 0.3-0.5)
- Decrease CFM value (try 1e-6)
- Increase solver iterations (try 15-20)
- Verify inertial properties are correctly specified

### Performance Issues
**Symptoms**: Slow simulation, missed real-time deadlines
**Solutions**:
- Increase max_step_size (try 0.002)
- Reduce solver iterations (try 5-8)
- Simplify collision geometry
- Reduce number of contact points

### Unrealistic Motion
**Symptoms**: Motion appears too fast/slow, unnatural movement patterns
**Solutions**:
- Verify actuator limits (effort, velocity)
- Check joint ranges and types
- Review control algorithm parameters
- Validate contact properties

## Validation and Testing

### Physics Configuration Validation

Before deploying physics configurations, validate them with:

1. **Stability Testing**: Run simulation for extended periods to check for drift or instability
2. **Consistency Checks**: Ensure parameters are consistent across similar joints/models
3. **Performance Monitoring**: Monitor simulation speed and resource usage
4. **Physical Plausibility**: Verify that motions and interactions appear physically realistic

### Configuration Best Practices

1. **Document All Parameters**: Maintain detailed records of physics configuration values
2. **Version Control**: Track changes to physics configurations in version control
3. **Modular Configuration**: Organize configurations in modular, reusable components
4. **Testing Framework**: Develop automated tests for physics configurations
5. **Benchmarking**: Establish performance and accuracy benchmarks for validation

## Example Configuration Template

Here's a complete template for a humanoid robot physics configuration:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_physics_world">
    <!-- Physics engine configuration -->
    <physics name="ode_physics" type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>1e-5</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Humanoid robot model -->
    <model name="humanoid_robot">
      <!-- Model definition with proper inertial properties -->
    </model>
  </world>
</sdf>
```

## Summary

Proper physics configuration is fundamental to achieving realistic simulation in Gazebo for humanoid robotics applications. By carefully tuning parameters such as gravity, solver settings, joint constraints, and contact properties, you can create simulations that accurately reflect real-world physics while maintaining computational efficiency. Regular validation and testing of configurations ensure that simulations remain stable and physically plausible throughout development and testing phases.