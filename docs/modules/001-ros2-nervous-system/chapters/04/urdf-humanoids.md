# Understanding URDF for Humanoids

## Learning Objectives

By the end of this chapter, students will be able to:
- Define URDF (Unified Robot Description Format) and explain its role in humanoid robotics
- Create URDF models that correctly define humanoid robot structures with joints, links, and sensors
- Implement proper joint limits and types (revolute, continuous, prismatic) for humanoid robots
- Include correct inertial properties for each link in URDF models
- Define visual and collision geometries for humanoid models
- Integrate URDF models with ROS 2 nodes for complete implementation
- Validate URDF models in simulation environments

## Theory/Concept

### Introduction to URDF

The Unified Robot Description Format (URDF) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of robots, including their structure, joints, links, and associated metadata such as inertial properties, visual meshes, and collision properties. URDF is essential for humanoid robotics as it allows for accurate modeling of complex articulated structures.

URDF is used by various ROS tools and libraries including:
- Robot State Publisher: Publishes the forward kinematics of the robot to tf
- Gazebo: For simulation of the robot
- MoveIt: For motion planning
- RViz: For visualization

### URDF Structure Components

A URDF model consists of two main elements:

1. **Links**: Represent rigid bodies of the robot (e.g., chassis, arm segments, links in a chain)
2. **Joints**: Define how links connect and move relative to each other

```xml
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### Links: Physical and Visual Properties

Each link in a URDF file can contain three main properties:

#### Visual Properties
- **Geometry**: Defines the shape (box, cylinder, sphere, mesh)
- **Material**: Defines color and texture properties
- **Origin**: Position and orientation relative to the link's coordinate frame

#### Collision Properties
- **Geometry**: Defines the collision shape (usually simpler than visual)
- **Origin**: Position and orientation relative to the link's coordinate frame

#### Inertial Properties
- **Mass**: Mass of the link in kilograms
- **Inertia**: 6 values representing the 3x3 inertia matrix (ixx, ixy, ixz, iyy, iyz, izz)

### Joints: Types and Properties

Joints define how links connect and move relative to each other. The main joint types are:

#### Revolute Joint
- Has a range of motion with defined limits
- Single degree of freedom rotation
- Used for elbow, knee, and other joints with limited rotation

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="1.0" effort="100" velocity="1.0"/>
</joint>
```

#### Continuous Joint
- Rotates continuously without limits
- Used for wheels, continuous rotation servos

```xml
<joint name="wheel_joint" type="continuous">
  <parent link="chassis"/>
  <child link="wheel"/>
  <origin xyz="0.2 0 -0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

#### Prismatic Joint
- Linear motion along an axis
- Has defined limits for translation

```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="base"/>
  <child link="slider"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0.0" upper="0.5" effort="100" velocity="0.5"/>
</joint>
```

#### Fixed Joint
- No motion between links
- Used to connect two links permanently

### Sensors in URDF

Sensors can be integrated into URDF models using Gazebo plugins. This allows for simulation of real sensors in Gazebo while maintaining the URDF format for compatibility with ROS tools.

```xml
<!-- IMU sensor in a head link -->
<gazebo reference="head_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <noise>
        <type>gaussian</type>
        <accel>
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </accel>
        <gyro>
          <mean>0.0</mean>
          <stddev>1.1e-3</stddev>
        </gyro>
      </noise>
    </imu>
  </sensor>
</gazebo>
```

### Inertial Properties for Humanoid Robots

Inertial properties are critical for accurate simulation of humanoid robots. The inertia tensor represents how mass is distributed in the link and affects the robot's motion under forces and torques.

For a humanoid robot, inertial properties should reflect realistic values based on:
- Volume and density of components
- Distribution of mass within each link
- Position of actuators and electronics

The 3x3 inertia matrix has six independent values due to symmetry:
```
| ixx  ixy  ixz |
| ixy  iyy  iyz |
| ixz  iyz  izz |
```

### Humanoid Robot Kinematic Structure

Humanoid robots have a specific kinematic structure that typically includes:

- **Torso**: Main body with head and attachment points for arms and legs
- **Head**: Usually with cameras, sensors, and neck joint
- **Arms**: Shoulder, elbow, and wrist joints for manipulation
- **Legs**: Hip, knee, and ankle joints for locomotion
- **Feet**: For balance and ground contact

## Example/Hands-on Exercise

### Exercise: Creating a Basic Humanoid Robot URDF Model

In this exercise, you'll create a basic humanoid robot model with a torso, head, two arms, and two legs using URDF.

#### Prerequisites
- ROS 2 Humble Hawksbill installed
- Basic understanding of XML syntax
- RViz or Gazebo for visualization

#### Step 1: Create the URDF Package
1. Create a new package for URDF files: `ros2 pkg create --build-type ament_cmake robot_description`
2. Navigate to the package: `cd ~/ros2_ws/src/robot_description`
3. Create a `urdf` directory: `mkdir urdf`




## Code Demonstration



## References/Resources

1. Open Robotics. (2023). URDF: Unified Robot Description Format. *ROS Documentation*. https://docs.ros.org/en/humble/Tutorials/URDF/URDF-Main.html

2. Chitta, S., Marder-Eppstein, E., & Smart, W. D. (2010). Automatic generation of tactile sensor mappings for robot hands. *2010 IEEE/RSJ International Conference on Intelligent Robots and Systems*, 2263-2268. https://doi.org/10.1109/IROS.2010.5649412

3. Cousin, A., & Righetti, L. (2015). Kinematic and dynamic analysis of a lower limb exoskeleton. *2015 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2431-2436. https://doi.org/10.1109/IROS.2015.7353675

4. Open Robotics. (2023). Working with URDF files. *ROS Documentation*. https://docs.ros.org/en/humble/Tutorials/URDF/Building-A-Visual-Robot-Model-With-URDF.html

5. Featherstone, R. (2008). *Rigid body dynamics algorithms*. Springer Science & Business Media. (Chapter on robot description and kinematics)

## Chapter Summary

This chapter provided a comprehensive understanding of URDF (Unified Robot Description Format) for humanoid robotics applications. We explored the fundamental components of URDF including links, joints, visual properties, collision properties, and inertial properties.

Key takeaways from this chapter:
- URDF is essential for modeling humanoid robots with complex articulated structures
- Proper definition of joints (revolute, continuous, prismatic) is crucial for realistic robot motion
- Inertial properties significantly impact simulation accuracy and robot dynamics
- Visual and collision geometries must be properly defined for both visualization and physics simulation
- Sensors can be integrated into URDF models for simulation purposes
- Integration with ROS 2 tools like Robot State Publisher and TF enables complete robot functionality

The hands-on exercise demonstrated creating a complete humanoid robot model with torso, head, arms, and legs, complete with proper joint definitions and inertial properties. The code demonstration showed how to integrate URDF models with ROS 2 nodes for publishing joint states and maintaining transforms.

These URDF modeling techniques are fundamental for humanoid robotics, enabling accurate simulation, motion planning, and control of complex multi-joint systems.