# ROS 2 Message Topic Standards for Sensor Simulation

This document defines the standard ROS 2 message topics and types used in the digital twin simulation module.

## Sensor Data Topics

### LiDAR
- **Topic**: `/sensor/lidar_scan`
- **Message Type**: `sensor_msgs/msg/LaserScan`
- **Description**: 2D laser scan data from simulated LiDAR sensor

### Depth Camera
- **Topic**: `/sensor/depth_camera/image_raw`
- **Message Type**: `sensor_msgs/msg/Image`
- **Description**: Raw depth image data
- **Topic**: `/sensor/depth_camera/camera_info`
- **Message Type**: `sensor_msgs/msg/CameraInfo`
- **Description**: Camera calibration and intrinsic parameters

### IMU
- **Topic**: `/sensor/imu/data`
- **Message Type**: `sensor_msgs/msg/Imu`
- **Description**: Inertial measurement unit data with orientation, angular velocity, and linear acceleration

### Joint States
- **Topic**: `/joint_states`
- **Message Type**: `sensor_msgs/msg/JointState`
- **Description**: Joint position, velocity, and effort information for all robot joints

### Control Commands
- **Topic**: `/cmd_vel`
- **Message Type**: `geometry_msgs/msg/Twist`
- **Description**: Velocity commands for robot movement

## Topic Naming Convention
- Sensor topics follow the pattern: `/sensor/[sensor_name]/[data_type]`
- All topics use lowercase with underscores
- Sensor names are descriptive and unique per robot instance