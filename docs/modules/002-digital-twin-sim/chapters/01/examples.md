# Code and Configuration Examples

## Introduction

This chapter provides practical examples of code and configuration files that demonstrate the concepts discussed in the Introduction to Digital Twins. These examples illustrate how to set up basic simulation environments, configure sensors, and establish communication between simulation platforms and ROS 2.

## Gazebo Simulation Setup Example

### Basic Robot Spawn Configuration

The following example demonstrates how to create a launch file for spawning a robot model in Gazebo:

```xml
<?xml version="1.0"?>
<launch>
  <!-- Arguments -->
  <arg name="model" default="$(find robot_description)/urdf/simple_robot.urdf"/>
  <arg name="gui" default="true"/>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="headless" default="false"/>

  <!-- Start Gazebo with the specified world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="worlds/empty.world"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="headless" value="$(arg headless)"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Load robot description parameter -->
  <param name="robot_description" command="$(find xacro)/xacro.py $(arg model)" />

  <!-- Push robot_description to factory and spawn robot in gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
        args="-param robot_description -urdf -model simple_robot"
        respawn="false" output="screen"/>
</launch>
```

**Explanation:**
- The launch file defines arguments for model path, GUI settings, and simulation parameters
- It includes the empty world launch file from gazebo_ros package
- The robot description is loaded as a parameter from the URDF file
- The spawn_model node creates the robot model in the Gazebo simulation

### Physics Configuration Example

Here's an example of how to configure physics parameters in a Gazebo world file:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="digital_twin_world">
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

    <!-- Include the robot model -->
    <include>
      <uri>model://simple_robot</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>

    <!-- Additional environment elements -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.6 0.4 -0.8</direction>
    </light>
  </world>
</sdf>
```

**Explanation:**
- The physics section configures the ODE physics engine with specific parameters
- `max_step_size` of 0.001 seconds corresponds to a 1000 Hz update rate
- `real_time_factor` of 1.0 ensures simulation runs at real-time speed
- The ODE solver configuration optimizes for real-time simulation
- Constraints parameters (CFM and ERP) balance stability and accuracy

## Unity ROS-TCP-Connector Example

### Basic ROS Connection Setup

Here's an example of setting up a basic ROS connection in Unity:

```csharp
using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class ROSConnectionSetup : MonoBehaviour
{
    [Header("ROS Connection Settings")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    private ROSConnection ros;

    void Start()
    {
        // Initialize the ROS connection
        ros = ROSConnection.instance;
        ros.Initialize(rosIPAddress, rosPort);

        Debug.Log("ROS Connection initialized to " + rosIPAddress + ":" + rosPort);
    }

    void OnDestroy()
    {
        // Clean up the ROS connection
        if (ros != null)
        {
            ros.Close();
        }
    }
}
```

**Explanation:**
- The script uses Unity.Robotics.ROSTCPConnector namespace for ROS communication
- IP address and port are configurable through the Unity inspector
- The ROSConnection instance is initialized in the Start method
- Proper cleanup is performed in OnDestroy to close the connection

### Simple Sensor Publisher Example

Here's an example of publishing sensor data from Unity to ROS:

```csharp
using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class SensorPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    [Header("Topic Settings")]
    public string topicName = "/sensor/unity_sensor_data";

    private ROSConnection ros;
    private float publishRate = 0.1f; // Publish every 0.1 seconds

    void Start()
    {
        ros = ROSConnection.instance;
        ros.Initialize(rosIPAddress, rosPort);

        // Start the publishing coroutine
        StartCoroutine(PublishSensorData());
    }

    IEnumerator PublishSensorData()
    {
        while (true)
        {
            // Create and populate the message
            Float32Msg sensorMsg = new Float32Msg();
            sensorMsg.data = Random.Range(0.0f, 100.0f); // Simulate sensor reading

            // Publish the message to the specified topic
            ros.Publish(topicName, sensorMsg);

            // Wait for the specified time
            yield return new WaitForSeconds(publishRate);
        }
    }

    void OnDestroy()
    {
        if (ros != null)
        {
            ros.Close();
        }
    }
}
```

**Explanation:**
- The script publishes Float32 messages to a specified ROS topic
- Uses a coroutine to publish data at regular intervals
- Random values simulate sensor readings for demonstration
- Proper cleanup is implemented in OnDestroy

## Configuration File Examples

### Simulation Configuration Template

Here's a YAML configuration file that demonstrates how to structure simulation parameters:

```yaml
# Digital Twin Simulation Configuration
simulation:
  name: "humanoid_robot_demo"
  description: "Basic simulation of humanoid robot with sensors"
  duration: 300  # Simulation duration in seconds (0 for indefinite)

# Physics environment configuration
physics:
  engine: "ode"
  gravity:
    x: 0.0
    y: 0.0
    z: -9.81
  update_rate: 1000  # Hz
  real_time_factor: 1.0
  erp: 0.2  # Error reduction parameter
  cfm: 1.0e-5  # Constraint force mixing

# Robot configuration
robot:
  model_file: "configs/simple_robot.urdf"
  initial_position:
    x: 0.0
    y: 0.0
    z: 0.5
  initial_orientation:
    roll: 0.0
    pitch: 0.0
    yaw: 0.0

# Sensor configuration
sensors:
  - name: "lidar_front"
    type: "lidar"
    topic: "/sensor/lidar_scan"
    position:
      x: 0.1
      y: 0.0
      z: 0.3
    parameters:
      range_min: 0.1
      range_max: 30.0
      ray_count: 720
      fov_horizontal: 360
      noise_model: "gaussian"
      noise_variance: 0.001

  - name: "imu_body"
    type: "imu"
    topic: "/sensor/imu/data"
    position:
      x: 0.0
      y: 0.0
      z: 0.1
    parameters:
      accel_range: 16.0
      gyro_range: 2000.0
      mag_range: 4800.0
      noise_model: "gaussian"
      noise_variance: 0.001

# Environment configuration
environment:
  world_file: "configs/worlds/simple_world.world"
  lighting:
    ambient_intensity: 0.8
    directional_light_enabled: true
    directional_light_direction:
      x: -0.6
      y: 0.4
      z: -0.8
  objects:
    - name: "ground_plane"
      type: "plane"
      position:
        x: 0.0
        y: 0.0
        z: 0.0
    - name: "obstacle_1"
      type: "box"
      position:
        x: 2.0
        y: 0.0
        z: 0.5
      dimensions:
        x: 1.0
        y: 1.0
        z: 1.0
```

**Explanation:**
- The configuration is structured hierarchically for clarity
- Physics parameters match those used in the Gazebo example
- Robot configuration includes position and orientation
- Sensors are defined with specific parameters and ROS topics
- Environment configuration specifies the world and objects

## ROS 2 Node Example

### Simple Sensor Data Publisher

Here's a Python example of a ROS 2 node that publishes sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math

class DigitalTwinSensorPublisher(Node):
    def __init__(self):
        super().__init__('digital_twin_sensor_publisher')

        # Create publisher for LiDAR data
        self.publisher = self.create_publisher(
            LaserScan,
            '/sensor/lidar_scan',
            10
        )

        # Timer for publishing data at regular intervals
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for simulation
        self.i = 0

        self.get_logger().info('Digital Twin Sensor Publisher initialized')

    def timer_callback(self):
        # Create LaserScan message
        msg = LaserScan()

        # Set header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'

        # Set LiDAR parameters
        msg.angle_min = -math.pi  # -180 degrees
        msg.angle_max = math.pi   # 180 degrees
        msg.angle_increment = 0.01  # ~0.57 degrees per increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 30.0

        # Calculate number of readings
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)

        # Generate simulated range data
        ranges = []
        for i in range(num_readings):
            # Simulate a circular obstacle at distance 5m in front
            angle = msg.angle_min + i * msg.angle_increment
            distance = 5.0 + 0.5 * math.sin(5 * angle + self.i * 0.1)  # Add some variation
            ranges.append(distance)

        msg.ranges = ranges
        msg.intensities = [100.0] * num_readings  # Constant intensity

        # Publish the message
        self.publisher.publish(msg)

        # Increment counter
        self.i += 1

        self.get_logger().info(f'Published LiDAR data with {len(ranges)} readings')

def main(args=None):
    rclpy.init(args=args)

    sensor_publisher = DigitalTwinSensorPublisher()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
- The node creates a publisher for LiDAR data on the `/sensor/lidar_scan` topic
- Uses a timer callback to publish data at regular intervals (10 Hz)
- Creates realistic LaserScan messages with proper header and parameters
- Simulates range data with a circular obstacle pattern
- Includes proper error handling and node cleanup

## Summary

These examples demonstrate practical implementations of the concepts covered in the Introduction to Digital Twins chapter. They show how to set up simulation environments, configure physics parameters, establish ROS communication, and create both Gazebo and Unity components for a digital twin system. The code examples are ready to use with appropriate modifications for specific use cases and include comprehensive comments explaining the functionality.

The configuration files provide templates that can be adapted for different simulation scenarios, while the ROS 2 node example shows how to generate realistic sensor data for testing and validation purposes.