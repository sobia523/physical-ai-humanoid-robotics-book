# Gazebo Physics Parameters for Humanoid Robotics

This document details the physics parameters used in Gazebo simulations for humanoid robotics applications, based on research findings and best practices.

## Physics Engine Configuration

### ODE Physics Engine Settings

Based on Gazebo documentation and robotics research, we use ODE (Open Dynamics Engine) physics engine with specific parameters optimized for humanoid robot simulation:

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

### Parameter Explanations

- **gravity**: Standard Earth gravity (9.8 m/s²) directed downward along Z-axis
- **max_step_size**: 0.001 seconds (1000 Hz) for accurate physics simulation
- **real_time_factor**: 1.0 to maintain real-time simulation
- **real_time_update_rate**: 1000.0 Hz for high-fidelity physics
- **solver/type**: Quick solver for real-time simulation
- **solver/iters**: 10 iterations for constraint solving
- **solver/sor**: 1.3 Successive Over-Relaxation parameter
- **constraints/cfm**: 1e-5 Constraint Force Mixing for stability
- **constraints/erp**: 0.2 Error Reduction Parameter for stable joints
- **contact_max_correcting_vel**: 100.0 m/s maximum contact correction velocity
- **contact_surface_layer**: 0.001 m surface penetration tolerance

## Humanoid-Specific Parameters

### Joint Configuration Parameters

For humanoid robot joints, the following parameters are recommended:

- **Joint friction**: 0.1-0.2 (realistic for servo actuators)
- **Joint damping**: 0.01-0.1 (for natural movement)
- **Collision mesh simplification**: 0.01m resolution

### Balance and Stability Parameters

For humanoid balance and stability:

- **Center of Mass**: Accurately modeled based on robot geometry
- **Inertia tensors**: Properly calculated for each link
- **Contact properties**: Appropriate friction coefficients for foot-ground interaction

## Performance Optimization Settings

### Physics Update Rate vs. Visual Quality

- **Physics update rate**: 1000 Hz for accuracy
- **Visual rendering rate**: Variable (30-60 FPS) to maintain performance
- **Max contacts**: 20 per collision for complex scenarios

### Computational Efficiency

- **Collision simplification**: Simplified collision meshes for physics while maintaining detailed visual models
- **Contact parameters**: Optimized for humanoid robot interactions
- **Solver settings**: Balanced for real-time performance

## Configuration Best Practices

### For Humanoid Robotics

1. **Use ODE physics engine** for stability with multi-link systems
2. **Set max_step_size to 0.001** for accurate humanoid dynamics
3. **Configure ERP and CFM** for stable joint constraints
4. **Validate gravity settings** match real-world conditions
5. **Test with various humanoid poses** to ensure stability

### Troubleshooting Common Issues

- **Robot shaking**: Increase ERP or decrease CFM
- **Unstable joints**: Verify inertial properties and joint limits
- **Poor contact behavior**: Adjust contact parameters (surface_layer, max_correcting_vel)
- **Performance issues**: Consider reducing physics update rate or simplifying collision models

## Alternative Physics Engine Configurations

### Bullet Physics Configuration

For scenarios requiring advanced collision detection:

```xml
<physics name="bullet_physics" type="bullet">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <bullet>
    <solver>
      <type>sequential_impulse</type>
      <iterations>10</iterations>
    </solver>
    <constraints>
      <cfm>1e-5</cfm>
      <erp>0.2</erp>
    </constraints>
  </bullet>
</physics>
```

### DART Physics Configuration

For advanced kinematic and dynamic capabilities:

```xml
<physics name="dart_physics" type="dart">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
</physics>
```

## Validation Checklist

Before using physics configurations:

- [ ] Gravity settings match Earth's gravity (9.8 m/s²)
- [ ] Physics update rate is appropriate (typically 1000 Hz)
- [ ] ERP and CFM values are optimized for stability
- [ ] Solver iterations are sufficient for accuracy
- [ ] Contact parameters are appropriate for humanoid interactions
- [ ] Inertial properties are correctly specified for all links
- [ ] Joint limits and friction are realistic
- [ ] Performance is acceptable for real-time simulation

These configurations provide a solid foundation for realistic physics simulation in humanoid robotics applications using Gazebo.