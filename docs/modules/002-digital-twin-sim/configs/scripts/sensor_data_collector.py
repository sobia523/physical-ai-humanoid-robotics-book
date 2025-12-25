#!/usr/bin/env python3
"""
Sensor Data Collection and Processing Script

This script demonstrates how to collect and process sensor data from simulated
LiDAR, Depth Camera, and IMU sensors in a Gazebo simulation environment.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2, CameraInfo
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import csv
from datetime import datetime
import yaml
from pathlib import Path

class SensorDataCollector(Node):
    def __init__(self):
        super().__init__('sensor_data_collector')

        # Initialize CvBridge for image processing
        self.bridge = CvBridge()

        # Storage for sensor data
        self.lidar_data = None
        self.depth_image = None
        self.rgb_image = None
        self.imu_data = None
        self.camera_info = None

        # Data collection parameters
        self.collection_rate = 1.0  # Hz
        self.collection_enabled = False
        self.data_buffer = []
        self.collection_count = 0

        # Directory for saving data
        self.data_dir = Path.home() / "sensor_data_collection"
        self.data_dir.mkdir(exist_ok=True)

        # Subscribe to sensor topics
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/humanoid/lidar/scan',
            self.lidar_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/humanoid/depth_camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.rgb_sub = self.create_subscription(
            Image,
            '/humanoid/depth_camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid/imu/data',
            self.imu_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/humanoid/depth_camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Timer for data processing
        self.timer = self.create_timer(1.0 / self.collection_rate, self.process_and_save_data)

        self.get_logger().info('Sensor data collector node initialized')
        self.get_logger().info('Waiting for sensor data...')

    def lidar_callback(self, msg):
        """Process incoming LiDAR data."""
        self.lidar_data = {
            'header': msg.header,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'time_increment': msg.time_increment,
            'scan_time': msg.scan_time,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'ranges': np.array(msg.ranges),
            'intensities': np.array(msg.intensities) if msg.intensities else None
        }

    def depth_callback(self, msg):
        """Process incoming depth image data."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')

    def rgb_callback(self, msg):
        """Process incoming RGB image data."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.rgb_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting RGB image: {e}')

    def imu_callback(self, msg):
        """Process incoming IMU data."""
        self.imu_data = {
            'header': msg.header,
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }

    def camera_info_callback(self, msg):
        """Process incoming camera info data."""
        self.camera_info = {
            'header': msg.header,
            'width': msg.width,
            'height': msg.height,
            'distortion_model': msg.distortion_model,
            'd': list(msg.d),
            'k': list(msg.k),
            'r': list(msg.r),
            'p': list(msg.p)
        }

    def process_lidar_data(self, lidar_data):
        """Process LiDAR data to extract useful information."""
        if lidar_data is None:
            return None

        # Filter out invalid range values
        valid_ranges = lidar_data['ranges'][(lidar_data['ranges'] >= lidar_data['range_min']) &
                                           (lidar_data['ranges'] <= lidar_data['range_max'])]

        # Calculate statistics
        if len(valid_ranges) > 0:
            stats = {
                'min_distance': float(np.min(valid_ranges)),
                'max_distance': float(np.max(valid_ranges)),
                'avg_distance': float(np.mean(valid_ranges)),
                'num_valid_readings': len(valid_ranges),
                'num_total_readings': len(lidar_data['ranges'])
            }
        else:
            stats = {
                'min_distance': float('inf'),
                'max_distance': 0.0,
                'avg_distance': 0.0,
                'num_valid_readings': 0,
                'num_total_readings': len(lidar_data['ranges'])
            }

        return stats

    def process_depth_image(self, depth_image):
        """Process depth image to extract useful information."""
        if depth_image is None:
            return None

        # Calculate depth statistics
        valid_depths = depth_image[depth_image > 0]  # Filter out invalid depths (0 or negative)

        if len(valid_depths) > 0:
            stats = {
                'min_depth': float(np.min(valid_depths)),
                'max_depth': float(np.max(valid_depths)),
                'avg_depth': float(np.mean(valid_depths)),
                'median_depth': float(np.median(valid_depths)),
                'num_valid_pixels': len(valid_depths),
                'total_pixels': depth_image.size
            }
        else:
            stats = {
                'min_depth': float('inf'),
                'max_depth': 0.0,
                'avg_depth': 0.0,
                'median_depth': 0.0,
                'num_valid_pixels': 0,
                'total_pixels': depth_image.size
            }

        return stats

    def process_imu_data(self, imu_data):
        """Process IMU data to extract useful information."""
        if imu_data is None:
            return None

        # Calculate magnitude of vectors
        angular_velocity_magnitude = np.sqrt(
            imu_data['angular_velocity']['x']**2 +
            imu_data['angular_velocity']['y']**2 +
            imu_data['angular_velocity']['z']**2
        )

        linear_acceleration_magnitude = np.sqrt(
            imu_data['linear_acceleration']['x']**2 +
            imu_data['linear_acceleration']['y']**2 +
            imu_data['linear_acceleration']['z']**2
        )

        return {
            'angular_velocity_magnitude': angular_velocity_magnitude,
            'linear_acceleration_magnitude': linear_acceleration_magnitude,
            'orientation': imu_data['orientation']
        }

    def save_images(self, rgb_image, depth_image, timestamp):
        """Save RGB and depth images to disk."""
        if rgb_image is not None:
            rgb_path = self.data_dir / f"rgb_{timestamp}.jpg"
            cv2.imwrite(str(rgb_path), rgb_image)

        if depth_image is not None:
            depth_path = self.data_dir / f"depth_{timestamp}.png"
            cv2.imwrite(str(depth_path), depth_image)

    def save_lidar_scan(self, lidar_data, timestamp):
        """Save LiDAR scan data to CSV file."""
        if lidar_data is None:
            return

        csv_path = self.data_dir / f"lidar_{timestamp}.csv"

        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['angle', 'range', 'intensity'])

            num_points = len(lidar_data['ranges'])
            for i in range(num_points):
                angle = lidar_data['angle_min'] + i * lidar_data['angle_increment']
                range_val = lidar_data['ranges'][i]
                intensity = lidar_data['intensities'][i] if lidar_data['intensities'] is not None else 0.0
                writer.writerow([angle, range_val, intensity])

    def process_and_save_data(self):
        """Process and save sensor data at regular intervals."""
        if not self.collection_enabled:
            return

        # Get current timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

        # Process each sensor type
        lidar_stats = self.process_lidar_data(self.lidar_data)
        depth_stats = self.process_depth_image(self.depth_image)
        imu_processed = self.process_imu_data(self.imu_data)

        # Create data record
        data_record = {
            'timestamp': timestamp,
            'lidar_stats': lidar_stats,
            'depth_stats': depth_stats,
            'imu_data': imu_processed
        }

        # Save images
        if self.rgb_image is not None or self.depth_image is not None:
            self.save_images(self.rgb_image, self.depth_image, timestamp)

        # Save LiDAR scan
        if self.lidar_data is not None:
            self.save_lidar_scan(self.lidar_data, timestamp)

        # Save to buffer
        self.data_buffer.append(data_record)
        self.collection_count += 1

        # Log progress
        self.get_logger().info(f'Data collection progress: {self.collection_count} samples collected')

        # Save summary data to YAML file periodically
        if self.collection_count % 10 == 0:  # Every 10 samples
            self.save_summary_data()

    def save_summary_data(self):
        """Save summary data to a YAML file."""
        if not self.data_buffer:
            return

        summary_path = self.data_dir / "sensor_summary.yaml"

        # Prepare summary data
        summary_data = {
            'collection_info': {
                'total_samples': self.collection_count,
                'collection_start_time': str(self.get_clock().now().to_msg()),
                'data_directory': str(self.data_dir)
            },
            'recent_samples': self.data_buffer[-5:]  # Last 5 samples
        }

        with open(summary_path, 'w') as yaml_file:
            yaml.dump(summary_data, yaml_file, default_flow_style=False)

        self.get_logger().info(f'Summary data saved to {summary_path}')

    def start_collection(self):
        """Start data collection."""
        self.collection_enabled = True
        self.get_logger().info('Data collection started')

    def stop_collection(self):
        """Stop data collection."""
        self.collection_enabled = False
        self.get_logger().info('Data collection stopped')

        # Save final data
        if self.data_buffer:
            final_path = self.data_dir / f"final_sensor_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.yaml"
            with open(final_path, 'w') as yaml_file:
                yaml.dump({
                    'collection_summary': {
                        'total_samples': self.collection_count,
                        'data_directory': str(self.data_dir)
                    },
                    'all_samples': self.data_buffer
                }, yaml_file, default_flow_style=False)

            self.get_logger().info(f'Final data saved to {final_path}')


def main(args=None):
    rclpy.init(args=args)

    collector = SensorDataCollector()

    # Start collection after a short delay to allow connections
    collector.get_logger().info('Starting data collection in 5 seconds...')
    collector.create_timer(5.0, collector.start_collection)

    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.get_logger().info('Interrupted, stopping data collection...')
        collector.stop_collection()
    finally:
        collector.stop_collection()
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()