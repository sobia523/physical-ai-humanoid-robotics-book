# ROS 2 Integration for Simulated Sensors

## Introduction

This chapter provides detailed instructions for connecting simulated sensors in Gazebo to ROS 2 topics. Proper integration ensures that sensor data from the simulation environment can be processed by ROS 2 nodes, enabling the development and testing of perception and control algorithms.

## Understanding Sensor Plugins

Gazebo uses sensor plugins to interface with ROS 2. These plugins are responsible for:
- Publishing sensor data to ROS 2 topics
- Managing sensor parameters through ROS 2 services
- Handling coordinate frame transformations
- Managing timing and synchronization

### Common Sensor Plugins

1. **`libgazebo_ros_ray_sensor.so`**: For LiDAR and other ray-based sensors
2. **`libgazebo_ros_depth_camera.so`**: For depth cameras
3. **`libgazebo_ros_imu.so`**: For IMU sensors
4. **`libgazebo_ros_camera.so`**: For RGB cameras

## Connecting LiDAR Sensors to ROS 2

### Configuration Steps

1. **Define the sensor in your URDF/SDF**:
   ```xml
   <sensor type="ray" name="humanoid_lidar">
     <ray>
       <scan>
         <horizontal>
           <samples>1080</samples>
           <resolution>1</resolution>
           <min_angle>-3.14159</min_angle>
           <max_angle>3.14159</max_angle>
         </horizontal>
       </scan>
       <range>
         <min>0.1</min>
         <max>30.0</max>
         <resolution>0.01</resolution>
       </range>
     </ray>
     <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
       <ros>
         <namespace>/humanoid</namespace>
         <remapping>~/out:=/humanoid/lidar/scan</remapping>
       </ros>
       <output_type>sensor_msgs/LaserScan</output_type>
       <frame_name>lidar_link</frame_name>
     </plugin>
   </sensor>
   ```

2. **Launch the simulation**:
   ```bash
   ros2 launch your_robot_gazebo robot_world.launch.py
   ```

3. **Verify the topic**:
   ```bash
   ros2 topic echo /humanoid/lidar/scan sensor_msgs/msg/LaserScan
   ```

### Topic Structure for LiDAR

- **Topic**: `/humanoid/lidar/scan`
- **Message Type**: `sensor_msgs/LaserScan`
- **Key Fields**:
  - `ranges[]`: Array of range measurements
  - `intensities[]`: Array of intensity measurements (optional)
  - `angle_min`, `angle_max`: Angular range
  - `angle_increment`: Angular resolution
  - `time_increment`: Time between measurements
  - `scan_time`: Time between scans
  - `range_min`, `range_max`: Valid range bounds

## Connecting Depth Camera Sensors to ROS 2

### Configuration Steps

1. **Define the depth camera in your URDF/SDF**:
   ```xml
   <sensor type="depth" name="humanoid_depth_camera">
     <camera>
       <horizontal_fov>1.0472</horizontal_fov>
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
     <plugin name="gazebo_ros_depth_camera" filename="libgazebo_ros_depth_camera.so">
       <ros>
         <namespace>/humanoid</namespace>
         <remapping>depth/image_raw:=/humanoid/depth_camera/depth/image_raw</remapping>
         <remapping>rgb/image_raw:=/humanoid/depth_camera/color/image_raw</remapping>
         <remapping>rgb/camera_info:=/humanoid/depth_camera/color/camera_info</remapping>
       </ros>
       <frame_name>depth_camera_link</frame_name>
       <baseline>0.1</baseline>
       <distortion_k1>0.0</distortion_k1>
       <distortion_k2>0.0</distortion_k2>
       <distortion_k3>0.0</distortion_k3>
       <distortion_t1>0.0</distortion_t1>
       <distortion_t2>0.0</distortion_t2>
     </plugin>
   </sensor>
   ```

2. **Launch the simulation**:
   ```bash
   ros2 launch your_robot_gazebo robot_world.launch.py
   ```

3. **Verify the topics**:
   ```bash
   ros2 topic echo /humanoid/depth_camera/depth/image_raw sensor_msgs/msg/Image
   ros2 topic echo /humanoid/depth_camera/color/image_raw sensor_msgs/msg/Image
   ros2 topic echo /humanoid/depth_camera/color/camera_info sensor_msgs/msg/CameraInfo
   ```

### Topic Structure for Depth Camera

- **Depth Image Topic**: `/humanoid/depth_camera/depth/image_raw`
  - **Message Type**: `sensor_msgs/Image`
  - **Encoding**: `32FC1` (32-bit float, 1 channel) or `16UC1` (16-bit unsigned int)

- **Color Image Topic**: `/humanoid/depth_camera/color/image_raw`
  - **Message Type**: `sensor_msgs/Image`
  - **Encoding**: `rgb8`, `bgr8`, etc.

- **Camera Info Topic**: `/humanoid/depth_camera/color/camera_info`
  - **Message Type**: `sensor_msgs/CameraInfo`
  - **Contains**: Camera intrinsics, distortion parameters, resolution

## Connecting IMU Sensors to ROS 2

### Configuration Steps

1. **Define the IMU in your URDF/SDF**:
   ```xml
   <sensor type="imu" name="humanoid_imu">
     <always_on>true</always_on>
     <update_rate>100</update_rate>
     <imu>
       <angular_velocity>
         <x>
           <noise type="gaussian">
             <mean>0.0</mean>
             <stddev>0.001</stddev>
           </noise>
         </x>
         <y>
           <noise type="gaussian">
             <mean>0.0</mean>
             <stddev>0.001</stddev>
           </noise>
         </y>
         <z>
           <noise type="gaussian">
             <mean>0.0</mean>
             <stddev>0.001</stddev>
           </noise>
         </z>
       </angular_velocity>
       <linear_acceleration>
         <x>
           <noise type="gaussian">
             <mean>0.0</mean>
             <stddev>1.7e-05</stddev>
           </noise>
         </x>
         <y>
           <noise type="gaussian">
             <mean>0.0</mean>
             <stddev>1.7e-05</stddev>
           </noise>
         </y>
         <z>
           <noise type="gaussian">
             <mean>0.0</mean>
             <stddev>1.7e-05</stddev>
           </noise>
         </z>
       </linear_acceleration>
     </imu>
     <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
       <ros>
         <namespace>/humanoid</namespace>
         <remapping>~/out:=/humanoid/imu/data</remapping>
       </ros>
       <frame_name>imu_link</frame_name>
       <topic_name>/humanoid/imu/data</topic_name>
       <serviceName>/humanoid/imu/calibrate</serviceName>
       <gaussian_noise>0.0017</gaussian_noise>
     </plugin>
   </sensor>
   ```

2. **Launch the simulation**:
   ```bash
   ros2 launch your_robot_gazebo robot_world.launch.py
   ```

3. **Verify the topic**:
   ```bash
   ros2 topic echo /humanoid/imu/data sensor_msgs/msg/Imu
   ```

### Topic Structure for IMU

- **Topic**: `/humanoid/imu/data`
- **Message Type**: `sensor_msgs/Imu`
- **Key Fields**:
  - `header`: Timestamp and frame ID
  - `orientation`: Quaternion representing orientation
  - `orientation_covariance`: Covariance matrix for orientation
  - `angular_velocity`: Angular velocity vector (x, y, z)
  - `angular_velocity_covariance`: Covariance matrix for angular velocity
  - `linear_acceleration`: Linear acceleration vector (x, y, z)
  - `linear_acceleration_covariance`: Covariance matrix for linear acceleration

## Sensor Data Processing Pipeline

### Basic Sensor Data Subscriber Example

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import numpy as np

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Initialize CvBridge for image processing
        self.bridge = CvBridge()

        # Subscribe to sensor topics
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/humanoid/lidar/scan',
            self.lidar_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid/imu/data',
            self.imu_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/humanoid/depth_camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.get_logger().info('Sensor processor node started')

    def lidar_callback(self, msg):
        # Process LiDAR data
        ranges = np.array(msg.ranges)
        # Filter out invalid ranges
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        self.get_logger().info(f'LiDAR: {len(valid_ranges)} valid readings')

    def imu_callback(self, msg):
        # Process IMU data
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        self.get_logger().info(f'IMU: Orientation - x:{orientation.x:.3f}, y:{orientation.y:.3f}')

    def depth_callback(self, msg):
        # Convert ROS Image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Process depth image
            depth_array = np.array(cv_image)
            avg_depth = np.mean(depth_array[depth_array > 0])  # Average non-zero depth values

            self.get_logger().info(f'Depth: Average distance - {avg_depth:.2f}m')
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

def main(args=None):
    rclpy.init(args=args)
    sensor_processor = SensorProcessor()

    try:
        rclpy.spin(sensor_processor)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File Configuration

Create a launch file to start your sensor processing node with the simulation:

```xml
<launch>
  <!-- Start Gazebo with your world -->
  <include file="$(find-pkg-share your_robot_gazebo)/launch/robot_world.launch.py" />

  <!-- Start your sensor processing node -->
  <node pkg="your_robot_perception" exec="sensor_processor" name="sensor_processor" output="screen">
    <param name="use_sim_time" value="true" />
  </node>
</launch>
```

## Troubleshooting Common Issues

### Sensor Data Not Publishing

1. **Check plugin loading**:
   ```bash
   # Check if the plugin loaded correctly
   grep -i "plugin" ~/.gazebo/log/*/server-*.log
   ```

2. **Verify topic existence**:
   ```bash
   ros2 topic list | grep humanoid
   ```

3. **Check for errors**:
   ```bash
   ros2 topic info /humanoid/lidar/scan
   ```

### Coordinate Frame Issues

1. **Check tf tree**:
   ```bash
   ros2 run tf2_tools view_frames
   # Or use rviz2 to visualize the robot
   ```

2. **Verify frame names**:
   - Ensure the `frame_name` in the plugin matches the link name in your URDF
   - Check for typos in frame names

### Performance Issues

1. **Reduce update rates** if experiencing performance problems
2. **Use appropriate resolutions** for your application
3. **Implement message throttling** if needed:
   ```python
   from message_filters import Subscriber, TimeSynchronizer

   # Synchronize multiple sensor topics
   lidar_sub = Subscriber(node, LaserScan, '/humanoid/lidar/scan')
   imu_sub = Subscriber(node, Imu, '/humanoid/imu/data')

   sync = TimeSynchronizer([lidar_sub, imu_sub], 10)
   sync.registerCallback(callback_function)
   ```

## Best Practices

1. **Use appropriate update rates** for each sensor type
2. **Implement proper error handling** for sensor data processing
3. **Validate sensor data** ranges and formats before processing
4. **Use simulation time** (`use_sim_time: true`) for consistent timing
5. **Configure appropriate noise models** to match real-world sensor characteristics
6. **Test with different sensor configurations** to ensure robustness

## Conclusion

Proper ROS 2 integration of simulated sensors is crucial for effective robotics development and testing. By following these instructions, you can ensure that your simulated sensors publish data to appropriate ROS 2 topics, allowing for seamless integration with your perception and control pipelines. Remember to validate your sensor data and tune parameters to match the characteristics of real-world sensors for the most effective simulation-to-reality transfer.