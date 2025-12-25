#!/usr/bin/env python3
"""
Sensor Fusion and Analysis Script

This script demonstrates how to fuse data from multiple sensors (LiDAR, IMU)
to improve perception and localization in humanoid robotics applications.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Pose, Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from collections import deque
import statistics

class SensorFusionAnalyzer(Node):
    def __init__(self):
        super().__init__('sensor_fusion_analyzer')

        # Sensor data storage
        self.lidar_data = None
        self.imu_data = None
        self.odom_data = None

        # Data buffers for temporal analysis
        self.imu_buffer = deque(maxlen=10)  # Store last 10 IMU readings
        self.lidar_buffer = deque(maxlen=5)  # Store last 5 LiDAR readings

        # State estimation variables
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z
        self.velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vz
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w (quaternion)
        self.linear_acceleration = np.array([0.0, 0.0, 0.0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])

        # Covariance matrices for uncertainty tracking
        self.position_covariance = np.eye(3) * 0.1
        self.orientation_covariance = np.eye(4) * 0.01

        # Subscribers
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

        # Publisher for fused state
        self.odom_pub = self.create_publisher(Odometry, '/humanoid/odom_fused', 10)

        # Timer for fusion processing
        self.fusion_timer = self.create_timer(0.05, self.perform_sensor_fusion)  # 20 Hz

        self.get_logger().info('Sensor fusion analyzer node initialized')

    def lidar_callback(self, msg):
        """Process incoming LiDAR data."""
        self.lidar_data = {
            'header': msg.header,
            'ranges': np.array(msg.ranges),
            'intensities': np.array(msg.intensities) if len(msg.intensities) > 0 else None,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }

        # Add to buffer
        self.lidar_buffer.append(self.lidar_data)

    def imu_callback(self, msg):
        """Process incoming IMU data."""
        self.imu_data = {
            'header': msg.header,
            'orientation': np.array([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]),
            'angular_velocity': np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ]),
            'linear_acceleration': np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])
        }

        # Add to buffer
        self.imu_buffer.append(self.imu_data)

        # Update state variables
        self.orientation = self.imu_data['orientation']
        self.angular_velocity = self.imu_data['angular_velocity']
        self.linear_acceleration = self.imu_data['linear_acceleration']

    def integrate_imu_data(self, dt):
        """Integrate IMU data to estimate position and velocity."""
        if len(self.imu_buffer) < 2:
            return

        # Get the last two IMU readings
        prev_imu = self.imu_buffer[-2]
        curr_imu = self.imu_buffer[-1]

        # Calculate time difference
        dt = (curr_imu['header'].stamp.sec - prev_imu['header'].stamp.sec) + \
             (curr_imu['header'].stamp.nanosec - prev_imu['header'].stamp.nanosec) * 1e-9

        if dt <= 0:
            dt = 0.05  # Default to 20Hz if timing info unavailable

        # Get current acceleration in world frame
        rotation = R.from_quat(self.orientation)
        world_acceleration = rotation.apply(self.linear_acceleration)

        # Update velocity (integrate acceleration)
        self.velocity += world_acceleration * dt

        # Update position (integrate velocity)
        self.position += self.velocity * dt

        # Update orientation (integrate angular velocity)
        # Convert angular velocity to quaternion derivative
        omega_quat = np.array([0, self.angular_velocity[0], self.angular_velocity[1], self.angular_velocity[2]])
        orientation_derivative = 0.5 * self.quaternion_multiply(omega_quat, self.orientation)
        self.orientation += orientation_derivative * dt
        # Normalize quaternion
        self.orientation = self.orientation / np.linalg.norm(self.orientation)

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return np.array([w, x, y, z])

    def process_lidar_for_position_correction(self):
        """Use LiDAR data to correct position estimates."""
        if self.lidar_data is None:
            return

        # Simple approach: detect obstacles and use them for position correction
        # In a real system, this would involve feature matching, SLAM, etc.

        # Calculate average distance in front of robot (simplified)
        front_sector = self.lidar_data['ranges'][len(self.lidar_data['ranges'])//2 - 50:len(self.lidar_data['ranges'])//2 + 50]
        valid_distances = front_sector[np.isfinite(front_sector) & (front_sector > 0)]

        if len(valid_distances) > 0:
            avg_front_distance = np.mean(valid_distances)
            self.get_logger().info(f'Average front distance: {avg_front_distance:.2f}m')

            # This is a simplified correction - in reality, you'd use more sophisticated methods
            # like matching features to a map or using ICP (Iterative Closest Point)
            return avg_front_distance
        else:
            return None

    def perform_sensor_fusion(self):
        """Perform sensor fusion using IMU and LiDAR data."""
        if self.imu_data is None:
            return

        # Get time difference for integration
        if len(self.imu_buffer) >= 2:
            dt = (self.imu_data['header'].stamp.nanosec - self.imu_buffer[-2]['header'].stamp.nanosec) * 1e-9
            if dt <= 0:
                dt = 0.05  # Default to 20Hz
        else:
            dt = 0.05  # Default to 20Hz

        # Integrate IMU data for position/velocity estimation
        self.integrate_imu_data(dt)

        # Use LiDAR data for position correction (simplified approach)
        lidar_correction = self.process_lidar_for_position_correction()

        # Publish fused odometry
        self.publish_fused_odometry()

        # Log state
        self.get_logger().info(
            f'Fused State - Pos: ({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}), '
            f'Vel: ({self.velocity[0]:.2f}, {self.velocity[1]:.2f}, {self.velocity[2]:.2f})'
        )

    def publish_fused_odometry(self):
        """Publish the fused odometry message."""
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        odom_msg.pose.pose.position.x = float(self.position[0])
        odom_msg.pose.pose.position.y = float(self.position[1])
        odom_msg.pose.pose.position.z = float(self.position[2])

        # Set orientation
        odom_msg.pose.pose.orientation.x = float(self.orientation[0])
        odom_msg.pose.pose.orientation.y = float(self.orientation[1])
        odom_msg.pose.pose.orientation.z = float(self.orientation[2])
        odom_msg.pose.pose.orientation.w = float(self.orientation[3])

        # Set position covariance (simplified)
        odom_msg.pose.covariance[0] = float(self.position_covariance[0, 0])  # XX
        odom_msg.pose.covariance[7] = float(self.position_covariance[1, 1])  # YY
        odom_msg.pose.covariance[14] = float(self.position_covariance[2, 2])  # ZZ

        # Set velocity
        odom_msg.twist.twist.linear.x = float(self.velocity[0])
        odom_msg.twist.twist.linear.y = float(self.velocity[1])
        odom_msg.twist.twist.linear.z = float(self.velocity[2])
        odom_msg.twist.twist.angular.x = float(self.angular_velocity[0])
        odom_msg.twist.twist.angular.y = float(self.angular_velocity[1])
        odom_msg.twist.twist.angular.z = float(self.angular_velocity[2])

        # Publish the message
        self.odom_pub.publish(odom_msg)

    def calculate_environment_features(self):
        """Extract features from sensor data for environment understanding."""
        if self.lidar_data is None:
            return None

        # Calculate some basic features from LiDAR data
        ranges = self.lidar_data['ranges']
        valid_ranges = ranges[np.isfinite(ranges) & (ranges > 0)]

        if len(valid_ranges) == 0:
            return None

        features = {
            'min_distance': float(np.min(valid_ranges)),
            'max_distance': float(np.max(valid_ranges)),
            'avg_distance': float(np.mean(valid_ranges)),
            'num_valid_readings': len(valid_ranges),
            'free_space_ratio': len(valid_ranges) / len(ranges),
            'obstacle_density': 1.0 - (len(valid_ranges) / len(ranges))
        }

        # Detect obstacles in different sectors
        num_sectors = 8
        sector_size = len(ranges) // num_sectors
        sectors = []

        for i in range(num_sectors):
            start_idx = i * sector_size
            end_idx = min((i + 1) * sector_size, len(ranges))
            sector_ranges = ranges[start_idx:end_idx]
            valid_sector = sector_ranges[np.isfinite(sector_ranges) & (sector_ranges > 0)]

            if len(valid_sector) > 0:
                sectors.append(float(np.min(valid_sector)))
            else:
                sectors.append(float('inf'))

        features['sectors'] = sectors
        return features

    def analyze_sensor_data_quality(self):
        """Analyze the quality and reliability of sensor data."""
        quality_metrics = {}

        # IMU quality
        if self.imu_data is not None:
            # Check for reasonable acceleration (should be around 9.81 m/s² when stationary)
            acc_magnitude = np.linalg.norm(self.linear_acceleration)
            quality_metrics['imu_acceleration_magnitude'] = acc_magnitude
            quality_metrics['imu_acceleration_reasonable'] = 8.0 < acc_magnitude < 12.0

            # Check angular velocity for reasonable values
            ang_vel_magnitude = np.linalg.norm(self.angular_velocity)
            quality_metrics['imu_angular_velocity_magnitude'] = ang_vel_magnitude
            quality_metrics['imu_angular_velocity_reasonable'] = ang_vel_magnitude < 10.0  # rad/s

        # LiDAR quality
        if self.lidar_data is not None:
            valid_ranges = self.lidar_data['ranges'][np.isfinite(self.lidar_data['ranges']) & (self.lidar_data['ranges'] > 0)]
            quality_metrics['lidar_valid_readings_ratio'] = len(valid_ranges) / len(self.lidar_data['ranges'])
            quality_metrics['lidar_data_available'] = len(valid_ranges) > 0

        return quality_metrics


def main(args=None):
    rclpy.init(args=args)

    analyzer = SensorFusionAnalyzer()

    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        analyzer.get_logger().info('Interrupted, shutting down...')
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()