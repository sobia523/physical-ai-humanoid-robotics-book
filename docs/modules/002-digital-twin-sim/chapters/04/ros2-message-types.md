# ROS 2 Message Types for Sensor Data in Humanoid Robotics

## Overview

This document provides comprehensive documentation for the ROS 2 message types commonly used in humanoid robotics sensor applications. Understanding these message types is crucial for developing effective sensor processing nodes and ensuring compatibility between simulated and real sensors.

## Standard Sensor Message Types

### sensor_msgs/LaserScan

The `LaserScan` message is used for 2D LiDAR sensor data, containing a single scan of range measurements.

**Fields:**
- `std_msgs/Header header`: Timestamp and coordinate frame ID
- `float32 angle_min`: Minimum angle of the scan [rad]
- `float32 angle_max`: Maximum angle of the scan [rad]
- `float32 angle_increment`: Angular distance between measurements [rad]
- `float32 time_increment`: Time between measurements [seconds]
- `float32 scan_time`: Time between scans [seconds]
- `float32 range_min`: Minimum range value [m]
- `float32 range_max`: Maximum range value [m]
- `float32[] ranges`: Array of range measurements [m]
- `float32[] intensities`: Array of intensity measurements [optional]

**Usage Example:**
```python
from sensor_msgs.msg import LaserScan

scan_msg = LaserScan()
scan_msg.header.stamp = self.get_clock().now().to_msg()
scan_msg.header.frame_id = 'lidar_link'
scan_msg.angle_min = -3.14159
scan_msg.angle_max = 3.14159
scan_msg.angle_increment = 0.01
scan_msg.time_increment = 0.001
scan_msg.scan_time = 0.1
scan_msg.range_min = 0.1
scan_msg.range_max = 30.0
scan_msg.ranges = [1.5, 1.6, 1.7, ...]  # Array of range measurements
```

### sensor_msgs/PointCloud2

The `PointCloud2` message represents a collection of points in 3D space, commonly used for 3D LiDAR data.

**Fields:**
- `std_msgs/Header header`: Timestamp and coordinate frame ID
- `uint32 height`: Two-dimensional structure of the point cloud
- `uint32 width`: Number of points in the cloud
- `sensor_msgs/PointField[] fields`: Information about fields
- `bool is_bigendian`: Endianness of data
- `uint32 point_step`: Length of each point in bytes
- `uint32 row_step`: Length of each row in bytes
- `uint8[] data`: Actual point data
- `bool is_dense`: Whether the cloud contains invalid values

**Common Field Names:**
- `x`, `y`, `z`: Cartesian coordinates
- `intensity`: Intensity value (optional)
- `normal_x`, `normal_y`, `normal_z`: Surface normal (optional)

### sensor_msgs/Image

The `Image` message carries image data from cameras and depth sensors.

**Fields:**
- `std_msgs/Header header`: Timestamp and coordinate frame ID
- `uint32 height`: Image height in pixels
- `uint32 width`: Image width in pixels
- `string encoding`: Encoding format (e.g., "rgb8", "16UC1", "32FC1")
- `uint8 is_bigendian`: Endianness of data
- `uint32 step`: Number of bytes per row
- `uint8[] data`: Image data (size = step * height)

**Common Encodings:**
- `rgb8`: 8-bit RGB
- `bgr8`: 8-bit BGR
- `mono8`: 8-bit grayscale
- `mono16`: 16-bit grayscale
- `32FC1`: 32-bit float for depth images

### sensor_msgs/CameraInfo

The `CameraInfo` message contains calibration and metadata for camera sensors.

**Fields:**
- `std_msgs/Header header`: Timestamp and coordinate frame ID
- `uint32 height`: Image height in pixels
- `uint32 width`: Image width in pixels
- `string distortion_model`: Distortion model (e.g., "plumb_bob", "rational_polynomial")
- `float64[] D`: Distortion coefficients
- `float64[9] K`: 3x3 intrinsic camera matrix
- `float64[9] R`: 3x3 rectification matrix
- `float64[12] P`: 3x4 projection matrix
- `uint32 binning_x`, `binning_y`: Binning factors
- `sensor_msgs/RegionOfInterest roi`: Region of interest

### sensor_msgs/Imu

The `Imu` message carries data from Inertial Measurement Units.

**Fields:**
- `std_msgs/Header header`: Timestamp and coordinate frame ID
- `geometry_msgs/Quaternion orientation`: Orientation as quaternion
- `float64[9] orientation_covariance`: Covariance matrix for orientation
- `geometry_msgs/Vector3 angular_velocity`: Angular velocity [rad/s]
- `float64[9] angular_velocity_covariance`: Covariance matrix for angular velocity
- `geometry_msgs/Vector3 linear_acceleration`: Linear acceleration [m/s²]
- `float64[9] linear_acceleration_covariance`: Covariance matrix for linear acceleration

**Coordinate Frame Convention:**
- X: Forward
- Y: Left
- Z: Up (following right-hand rule)

## Sensor-Specific Message Types

### sensor_msgs/MagneticField

Used for magnetometer data, typically published alongside IMU data.

**Fields:**
- `std_msgs/Header header`: Timestamp and coordinate frame ID
- `geometry_msgs/Vector3 magnetic_field`: Magnetic field vector [Tesla]
- `float64[9] magnetic_field_covariance`: Covariance matrix

### sensor_msgs/FluidPressure

For pressure sensors that might be used in humanoid robots.

**Fields:**
- `std_msgs/Header header`: Timestamp and coordinate frame ID
- `float64 fluid_pressure`: Pressure reading [Pascals]
- `float64 variance`: Measurement variance

### sensor_msgs/Temperature

For temperature sensors that might be used for thermal monitoring.

**Fields:**
- `std_msgs/Header header`: Timestamp and coordinate frame ID
- `float64 temperature`: Temperature reading [Celsius]
- `float64 variance`: Measurement variance

## Message Construction Best Practices

### Header Management

The header is crucial for proper message handling:

```python
from std_msgs.msg import Header

# Always set the header with appropriate timestamp and frame ID
header = Header()
header.stamp = self.get_clock().now().to_msg()
header.frame_id = 'sensor_link'
```

### Timestamp Synchronization

For multi-sensor systems, proper timestamp management is essential:

```python
# Use the same timestamp for related sensor readings when possible
timestamp = self.get_clock().now().to_msg()

lidar_msg.header.stamp = timestamp
imu_msg.header.stamp = timestamp  # If readings are time-synchronized
```

### Frame ID Conventions

Follow consistent naming conventions for frame IDs:

- Use descriptive names: `lidar_link`, `camera_depth_optical_frame`, `imu_link`
- Follow the robot's URDF frame hierarchy
- Use underscores instead of hyphens in frame names
- Keep names concise but descriptive

## Performance Considerations

### Message Size Optimization

Large messages can impact network performance:

- For point clouds, consider downsampling when full resolution isn't needed
- Use appropriate data types (float32 instead of float64 when precision allows)
- Compress images when possible using image_transport packages

### Quality of Service (QoS) Settings

Configure appropriate QoS settings for different sensor types:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For LiDAR data (real-time, loss-tolerant)
lidar_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# For IMU data (real-time, important)
imu_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)
```

## Validation and Debugging

### Message Validation

Always validate message contents before processing:

```python
def validate_laser_scan(self, msg):
    if len(msg.ranges) == 0:
        self.get_logger().warn("Empty laser scan received")
        return False

    if msg.range_min >= msg.range_max:
        self.get_logger().warn("Invalid range parameters")
        return False

    return True
```

### Common Issues and Solutions

1. **NaN values in sensor data**: Check sensor configuration and physical setup
2. **Timestamp issues**: Verify clock synchronization between simulation and nodes
3. **Frame ID mismatches**: Check TF tree and coordinate frame definitions
4. **Message size errors**: Implement message throttling or downsampling

## Integration with Simulation

### Gazebo Plugin Configuration

Ensure plugin configuration matches message type expectations:

```xml
<plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
  <ros>
    <namespace>/humanoid</namespace>
    <remapping>~/out:=/humanoid/imu/data</remapping>
  </ros>
  <frame_name>imu_link</frame_name>
  <topic_name>/humanoid/imu/data</topic_name>
</plugin>
```

### Simulation-Specific Considerations

- Simulation time vs. real time
- Fixed frame relationships in simulation
- Coordinate frame alignment between simulated and real sensors

## Conclusion

Understanding ROS 2 sensor message types is fundamental to developing effective humanoid robotics applications. Proper use of these standardized messages ensures compatibility between different sensor systems, simulation environments, and processing algorithms. Always validate message contents, follow best practices for performance optimization, and maintain consistency in frame ID conventions to ensure robust sensor integration in your humanoid robotics projects.