# Code Snippet Templates for Simulation Examples

This document provides reusable code templates for simulation examples in the digital twin module.

## Gazebo Launch File Template

```xml
<?xml version="1.0"?>
<launch>
  <!-- Launch arguments -->
  <arg name="use_sim_time" default="true"/>
  <arg name="world_name" default="empty"/>

  <!-- Start Gazebo -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find robot_description)/worlds/$(arg world_name).world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Robot state publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false">
    <param name="publish_frequency" value="50.0"/>
    <param name="use_tf_static" value="false"/>
    <param name="ignore_timestamp" value="false"/>
  </node>

  <!-- Spawn robot in Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-file $(find robot_description)/urdf/simple_robot.urdf -urdf -model simple_robot" respawn="false" output="screen"/>
</launch>
```

## ROS 2 LiDAR Publisher Template

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class SimpleLidar(Node):
    def __init__(self):
        super().__init__('simple_lidar')
        self.publisher = self.create_publisher(LaserScan, '/sensor/lidar_scan', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 0.01
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 30.0

        # Generate sample ranges
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = [float(i % 10 + 1) for i in range(num_readings)]
        msg.intensities = [100.0] * num_readings

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    simple_lidar = SimpleLidar()
    rclpy.spin(simple_lidar)
    simple_lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS 2 Depth Camera Publisher Template

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthCameraPublisher(Node):
    def __init__(self):
        super().__init__('depth_camera_publisher')
        self.image_publisher = self.create_publisher(Image, '/sensor/depth_camera/image_raw', 10)
        self.info_publisher = self.create_publisher(CameraInfo, '/sensor/depth_camera/camera_info', 10)
        self.bridge = CvBridge()

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Create a sample depth image (grayscale)
        height, width = 480, 640
        depth_image = np.random.rand(height, width) * 30.0  # Random depth values 0-30m

        # Convert to 16-bit format for depth data
        depth_image = (depth_image * 1000).astype(np.uint16)  # Convert to mm

        # Publish depth image
        image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'camera_link'
        self.image_publisher.publish(image_msg)

        # Publish camera info
        info_msg = self.create_camera_info_msg(width, height)
        info_msg.header.stamp = image_msg.header.stamp
        info_msg.header.frame_id = 'camera_link'
        self.info_publisher.publish(info_msg)

    def create_camera_info_msg(self, width, height):
        info_msg = CameraInfo()
        info_msg.width = width
        info_msg.height = height
        info_msg.k = [500.0, 0.0, width/2.0,  # fx, 0, cx
                      0.0, 500.0, height/2.0, # 0, fy, cy
                      0.0, 0.0, 1.0]          # 0, 0, 1
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Distortion coefficients
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Rectification matrix
        info_msg.p = [500.0, 0.0, width/2.0, 0.0,  # Projection matrix
                      0.0, 500.0, height/2.0, 0.0,
                      0.0, 0.0, 1.0, 0.0]
        return info_msg

def main(args=None):
    rclpy.init(args=args)
    publisher = DepthCameraPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS 2 IMU Publisher Template

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math
import numpy as np

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher = self.create_publisher(Imu, '/sensor/imu/data', 10)
        timer_period = 0.05  # seconds (20 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle = 0.0

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulate small oscillations
        self.angle += 0.1
        roll = 0.1 * math.sin(self.angle)
        pitch = 0.05 * math.cos(self.angle)
        yaw = 0.02 * math.sin(2 * self.angle)

        # Convert to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy

        # Angular velocity (small random values)
        msg.angular_velocity = Vector3(
            x=np.random.normal(0, 0.01),
            y=np.random.normal(0, 0.01),
            z=np.random.normal(0, 0.01)
        )

        # Linear acceleration (gravity + small variations)
        msg.linear_acceleration = Vector3(
            x=np.random.normal(0, 0.1),
            y=np.random.normal(0, 0.1),
            z=9.8 + np.random.normal(0, 0.1)
        )

        # Add covariance values (indicates uncertainty)
        for i in range(9):
            msg.orientation_covariance[i] = 0.01 if i % 4 == 0 else 0.0  # Diagonal only
            msg.angular_velocity_covariance[i] = 0.01 if i % 4 == 0 else 0.0
            msg.linear_acceleration_covariance[i] = 0.01 if i % 4 == 0 else 0.0

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = IMUPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Unity ROS-TCP-Connector Template

```csharp
using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class RobotController : MonoBehaviour
{
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;
    public string lidarTopic = "/sensor/lidar_scan";

    private ROSConnection ros;
    private bool isConnected = false;

    void Start()
    {
        ros = ROSConnection.instance;
        ros.Initialize(rosIPAddress, rosPort);

        // Subscribe to LiDAR topic
        ros.Subscribe<LaserScanMsg>(lidarTopic, OnLidarDataReceived);
    }

    void OnLidarDataReceived(LaserScanMsg laserScan)
    {
        // Process LiDAR data
        Debug.Log("Received LiDAR data with " + laserScan.ranges.Length + " points");

        // Example: Use the first range value
        if (laserScan.ranges.Length > 0 && laserScan.ranges[0] > 0)
        {
            Debug.Log("Distance to obstacle: " + laserScan.ranges[0] + " meters");
        }
    }

    void Update()
    {
        // Send data to ROS if needed
        if (Input.GetKeyDown(KeyCode.Space))
        {
            SendRobotPosition();
        }
    }

    void SendRobotPosition()
    {
        // Example of sending robot position to ROS
        // Implementation would depend on your specific needs
    }
}
```

## Configuration File Template

```yaml
# Simulation Configuration Template
simulation:
  name: "digital_twin_example"
  physics:
    gravity: [0, 0, -9.81]
    engine: "ode"
    update_rate: 1000
    erp: 0.2
    cfm: 1.0e-5
  rendering:
    resolution:
      width: 1920
      height: 1080
    frame_rate: 60
    pipeline: "hdrp"
  robot:
    model_file: "configs/simple_robot.urdf"
    initial_position: [0, 0, 0.5]
    initial_orientation: [0, 0, 0]
  sensors:
    - type: "lidar"
      name: "front_lidar"
      topic: "/sensor/lidar_scan"
      position: [0.0, 0.0, 0.3]
      parameters:
        range_min: 0.1
        range_max: 30.0
        ray_count: 720
        fov: 360
    - type: "imu"
      name: "body_imu"
      topic: "/sensor/imu/data"
      position: [0.0, 0.0, 0.1]
      parameters:
        accel_range: 16.0
        gyro_range: 2000.0
```