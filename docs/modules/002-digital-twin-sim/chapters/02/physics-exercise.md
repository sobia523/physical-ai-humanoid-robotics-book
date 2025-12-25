# Hands-On Exercise: Physics Simulation Setup for Humanoid Robots

## Exercise Overview

In this hands-on exercise, you will configure and implement a physics simulation environment for a humanoid robot in Gazebo. You will learn to set up realistic physics parameters, configure joint dynamics, and validate the simulation behavior through practical testing.

### Learning Objectives

By completing this exercise, you will be able to:
- Configure Gazebo physics parameters for humanoid robot simulation
- Set up realistic joint dynamics and constraints
- Validate physics simulation behavior through testing
- Troubleshoot common physics simulation issues
- Optimize simulation parameters for performance and accuracy

### Prerequisites

Before starting this exercise, ensure you have:
- Gazebo installed (Garden or Harmonic recommended)
- Basic understanding of XML configuration files
- Basic knowledge of ROS 2 (for sensor integration)
- Access to the example configuration files from previous chapters

## Exercise Tasks

### Task 1: Setting Up the Physics World (30 minutes)

In this task, you will create a basic physics world configuration for humanoid simulation.

#### Step 1: Create the World File
1. Create a new directory: `~/gazebo_exercises/physics_setup`
2. Create a world file named `humanoid_physics.world`
3. Configure the basic physics engine with ODE as the type
4. Set the gravity to standard Earth gravity (0 0 -9.8)
5. Set the maximum step size to 0.001 seconds (1000 Hz)
6. Configure the real-time factor to 1.0 for real-time simulation

#### Step 2: Configure Solver Parameters
1. Set the solver type to "quick"
2. Configure 10 solver iterations
3. Set the SOR (Successive Over-Relaxation) parameter to 1.3
4. Set the Constraint Force Mixing (CFM) to 1e-5
5. Set the Error Reduction Parameter (ERP) to 0.2

#### Step 3: Add Environment Elements
1. Include the ground plane model
2. Include the sun model for lighting
3. Save the world file

**Expected Result**: A basic world file with proper physics configuration that can be loaded in Gazebo.

### Task 2: Creating a Simple Humanoid Model (45 minutes)

In this task, you will create a simplified humanoid robot model with proper inertial properties and joint dynamics.

#### Step 1: Define the Robot Model
1. Create a URDF file named `simple_humanoid.urdf` in the exercise directory
2. Define the following links:
   - Torso (box: 0.3x0.2x0.6 m, mass: 5.0 kg)
   - Head (sphere: radius 0.1 m, mass: 1.0 kg)
   - Four limbs (upper and lower arms/legs as cylinders)

#### Step 2: Configure Inertial Properties
For each link, define:
- Mass (realistic for humanoid components)
- Inertia tensor (calculated based on geometry and mass)
- Center of mass position (if not at origin)

Example for torso link:
```xml
<link name="torso">
  <collision>
    <geometry>
      <box size="0.3 0.2 0.6"/>
    </geometry>
  </collision>
  <visual>
    <geometry>
      <box size="0.3 0.2 0.6"/>
    </geometry>
  </visual>
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
  </inertial>
</link>
```

#### Step 3: Define Joint Connections
Create joints connecting the links:
- Revolute joints for shoulders, elbows, hips, and knees
- Set appropriate limits based on human joint ranges
- Configure damping and friction values (damping: 0.1-0.3, friction: 0.1-0.2)
- Set effort and velocity limits appropriate for humanoid actuators

#### Step 4: Include Model in World File
1. Add the robot model to your world file
2. Position the robot above the ground (approximately 0.8m height)
3. Save the model and world files

**Expected Result**: A humanoid robot model with realistic inertial properties that can be loaded in Gazebo.

### Task 3: Configuring Contact Properties (30 minutes)

In this task, you will configure realistic contact properties for robot-environment interactions.

#### Step 1: Configure Foot Contact Properties
1. In the lower leg links, add surface properties for the collision elements
2. Set friction coefficients to 0.8 for good grip during walking
3. Set restitution coefficient to 0.1 for minimal bouncing
4. Configure contact parameters (soft_erp: 0.3, kp: 1e12)

#### Step 2: Add Example Objects
1. Create a simple box object (0.2x0.2x0.2 m, mass: 0.5 kg) in the world
2. Configure its contact properties with moderate friction (mu: 0.6)
3. Position it in the environment for interaction testing

#### Step 3: Test Contact Behavior
1. Load the world in Gazebo
2. Observe the initial contact behavior
3. Note any issues with unrealistic sliding or bouncing

**Expected Result**: Robot feet that make stable contact with the ground and objects that interact realistically.

### Task 4: Tuning Physics Parameters (30 minutes)

In this task, you will experiment with different physics parameters to optimize simulation behavior.

#### Step 1: Test Different ERP Values
1. Create three versions of your world file with different ERP values:
   - Version A: ERP = 0.1 (less constraint enforcement)
   - Version B: ERP = 0.2 (default)
   - Version C: ERP = 0.4 (stronger constraint enforcement)
2. Load each version in Gazebo
3. Observe the stability and constraint behavior

#### Step 2: Test Different CFM Values
1. Create three versions with different CFM values:
   - Version A: CFM = 1e-4 (less rigid constraints)
   - Version B: CFM = 1e-5 (default)
   - Version C: CFM = 1e-6 (more rigid constraints)
2. Load and observe behavior

#### Step 3: Evaluate Performance vs. Accuracy
1. Monitor simulation real-time factor for each configuration
2. Note which parameters provide the best balance of stability and performance
3. Document your findings

**Expected Result**: Understanding of how different physics parameters affect simulation behavior.

### Task 5: Adding Sensors and Validation (45 minutes)

In this task, you will add sensors to validate physics simulation and test the overall system.

#### Step 1: Add IMU Sensor
1. Add an IMU sensor to the robot's torso link
2. Configure it to publish at 100 Hz
3. Add appropriate noise values (e.g., 0.001 for realistic sensor noise)
4. Connect it to a ROS 2 topic

#### Step 2: Add Joint State Publisher
1. Configure joint state publishing for all robot joints
2. Set update rate to match physics simulation (1000 Hz)
3. Verify joint states are published correctly

#### Step 3: Validate Physics Behavior
1. Launch the simulation with your configured robot
2. Monitor joint positions and velocities through ROS 2
3. Check IMU readings for realistic acceleration and orientation changes
4. Verify that the robot maintains stable contact with the ground

#### Step 4: Test Motion Commands
1. Send simple joint position commands to move the robot
2. Observe how the physics simulation responds to commands
3. Check for any unrealistic oscillations or instabilities

**Expected Result**: A fully configured robot simulation with sensors that provides realistic physics behavior.

## Exercise Deliverables

Upon completion of this exercise, you should have:

1. **World Configuration File**: A properly configured `humanoid_physics.world` file with optimized physics parameters
2. **Robot Model**: A URDF file (`simple_humanoid.urdf`) with realistic inertial properties and joint dynamics
3. **Configuration Documentation**: A brief report documenting:
   - Physics parameters used and rationale
   - Joint limits and dynamics settings
   - Contact properties and their effects
   - Performance vs. accuracy trade-offs observed
4. **Validation Results**: Observations on simulation behavior and any issues encountered
5. **Troubleshooting Notes**: Solutions to any problems encountered during the exercise

## Assessment Criteria

Your exercise will be assessed based on:

- **Physics Configuration (25%)**: Proper setup of physics engine parameters
- **Model Accuracy (25%)**: Realistic inertial properties and joint constraints
- **Contact Behavior (20%)**: Appropriate contact properties and interactions
- **Validation (20%)**: Successful testing and validation of simulation behavior
- **Documentation (10%)**: Clear documentation of configuration choices and results

## Troubleshooting Tips

### Common Issues and Solutions

**Robot falls through the ground**:
- Check that the ground plane is properly included in the world
- Verify that the robot is positioned above the ground level
- Increase ERP value or decrease CFM for more stable contact

**Robot shakes or oscillates**:
- Reduce joint damping values
- Increase solver iterations
- Check that inertial properties are properly specified

**Simulation runs slowly**:
- Increase max_step_size to 0.002
- Reduce solver iterations to 5-8
- Simplify collision geometry

**Joints behave unrealistically**:
- Verify joint limits match human anatomical ranges
- Check that effort and velocity limits are appropriate
- Adjust damping and friction values

## Extension Activities

For advanced learners, consider these additional challenges:

1. **Walking Controller**: Implement a simple walking controller to test physics with dynamic motion
2. **Balance Testing**: Test the robot's response to external forces applied to the torso
3. **Multi-robot Simulation**: Add a second robot and test interaction physics
4. **Terrain Variations**: Test simulation with different ground types (slopes, uneven terrain)

## Summary

This exercise provided hands-on experience with configuring physics simulation parameters for humanoid robots in Gazebo. You learned to balance simulation stability, accuracy, and performance while creating realistic robot-environment interactions. The skills developed in this exercise form the foundation for more complex simulation scenarios in humanoid robotics research and development.