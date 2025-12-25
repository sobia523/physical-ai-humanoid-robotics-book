#!/usr/bin/env python3
"""
Sensor Data Validation Checker

This script validates that sensor data streams from simulation are realistic
and compatible with ROS 2. It checks data ranges, formats, timing, and
physical plausibility.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import math
from datetime import datetime
import statistics

class SensorValidationChecker(Node):
    def __init__(self):
        super().__init__('sensor_validation_checker')

        # Initialize CvBridge for image processing
        self.bridge = CvBridge()

        # Data validation statistics
        self.validation_stats = {
            'lidar': {'valid_samples': 0, 'invalid_samples': 0, 'data_points': []},
            'imu': {'valid_samples': 0, 'invalid_samples': 0, 'data_points': []},
            'depth': {'valid_samples': 0, 'invalid_samples': 0, 'data_points': []},
            'rgb': {'valid_samples': 0, 'invalid_samples': 0, 'data_points': []}
        }

        # Timing validation
        self.last_lidar_time = None
        self.last_imu_time = None
        self.last_depth_time = None

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

        self.rgb_sub = self.create_subscription(
            Image,
            '/humanoid/depth_camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/humanoid/depth_camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Timer for periodic validation reports
        self.report_timer = self.create_timer(10.0, self.report_validation_status)

        self.get_logger().info('Sensor validation checker node initialized')
        self.get_logger().info('Monitoring sensor data streams for realism and ROS 2 compatibility...')

    def lidar_callback(self, msg):
        """Validate LiDAR data."""
        is_valid = True
        errors = []

        # Check header
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            errors.append("Invalid timestamp")
            is_valid = False

        # Check angle parameters
        if msg.angle_min >= msg.angle_max:
            errors.append("Invalid angle range: min >= max")
            is_valid = False

        if msg.angle_increment <= 0:
            errors.append("Invalid angle increment")
            is_valid = False

        # Check range parameters
        if msg.range_min >= msg.range_max:
            errors.append("Invalid range: min >= max")
            is_valid = False

        # Check data dimensions
        expected_samples = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        if len(msg.ranges) != expected_samples:
            errors.append(f"Mismatched ranges array length: expected {expected_samples}, got {len(msg.ranges)}")
            is_valid = False

        # Validate range values
        invalid_ranges = 0
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max) and not math.isnan(r) and r != float('inf'):
                invalid_ranges += 1

        if invalid_ranges > len(msg.ranges) * 0.5:  # More than 50% invalid
            errors.append(f"Too many invalid range values: {invalid_ranges}/{len(msg.ranges)}")
            is_valid = False

        # Check timing consistency
        if self.last_lidar_time is not None:
            time_diff = (msg.header.stamp.sec - self.last_lidar_time.sec) + \
                       (msg.header.stamp.nanosec - self.last_lidar_time.nanosec) * 1e-9
            expected_freq = 1.0 / (msg.scan_time if msg.scan_time > 0 else 0.1)
            if abs(time_diff - 1.0/expected_freq) > 0.1:  # 10% tolerance
                errors.append(f"Timing inconsistency: expected ~{1.0/expected_freq:.2f}s, got {time_diff:.2f}s")
                is_valid = False

        self.last_lidar_time = msg.header.stamp

        # Log validation results
        if is_valid:
            self.validation_stats['lidar']['valid_samples'] += 1
            self.get_logger().debug(f'LiDAR: Valid sample received ({len(msg.ranges)} points)')
        else:
            self.validation_stats['lidar']['invalid_samples'] += 1
            self.get_logger().warn(f'LiDAR: Invalid sample - {", ".join(errors)}')

        # Store data for statistics
        self.validation_stats['lidar']['data_points'].append({
            'timestamp': self.get_clock().now(),
            'num_points': len(msg.ranges),
            'valid_points_ratio': 1.0 - (invalid_ranges / len(msg.ranges)) if len(msg.ranges) > 0 else 0.0,
            'errors': errors
        })

        # Keep only recent data (last 100 samples)
        if len(self.validation_stats['lidar']['data_points']) > 100:
            self.validation_stats['lidar']['data_points'] = self.validation_stats['lidar']['data_points'][-100:]

    def imu_callback(self, msg):
        """Validate IMU data."""
        is_valid = True
        errors = []

        # Check header
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            errors.append("Invalid timestamp")
            is_valid = False

        # Check orientation quaternion normalization
        norm = math.sqrt(msg.orientation.x**2 + msg.orientation.y**2 +
                        msg.orientation.z**2 + msg.orientation.w**2)
        if abs(norm - 1.0) > 0.01:  # 1% tolerance
            errors.append(f"Quaternion not normalized: {norm:.3f}")
            is_valid = False

        # Check for reasonable acceleration (when robot is stationary, should be ~9.81 m/s²)
        acc_magnitude = math.sqrt(msg.linear_acceleration.x**2 +
                                 msg.linear_acceleration.y**2 +
                                 msg.linear_acceleration.z**2)
        if acc_magnitude < 5.0 or acc_magnitude > 15.0:
            # This might be valid if the robot is moving, so just log for review
            self.get_logger().debug(f'IMU: Acceleration magnitude {acc_magnitude:.2f} m/s² (might be during motion)')

        # Check for reasonable angular velocities
        ang_vel_magnitude = math.sqrt(msg.angular_velocity.x**2 +
                                     msg.angular_velocity.y**2 +
                                     msg.angular_velocity.z**2)
        if ang_vel_magnitude > 10.0:  # 10 rad/s is very fast
            errors.append(f"High angular velocity: {ang_vel_magnitude:.2f} rad/s")
            is_valid = False

        # Check covariance matrices
        if any(np.isnan(cov) for cov in msg.orientation_covariance):
            errors.append("NaN values in orientation covariance")
            is_valid = False

        if any(np.isnan(cov) for cov in msg.angular_velocity_covariance):
            errors.append("NaN values in angular velocity covariance")
            is_valid = False

        if any(np.isnan(cov) for cov in msg.linear_acceleration_covariance):
            errors.append("NaN values in linear acceleration covariance")
            is_valid = False

        # Check timing consistency
        if self.last_imu_time is not None:
            time_diff = (msg.header.stamp.sec - self.last_imu_time.sec) + \
                       (msg.header.stamp.nanosec - self.last_imu_time.nanosec) * 1e-9
            if time_diff <= 0:
                errors.append("Non-increasing timestamps")
                is_valid = False

        self.last_imu_time = msg.header.stamp

        # Log validation results
        if is_valid:
            self.validation_stats['imu']['valid_samples'] += 1
            self.get_logger().debug(f'IMU: Valid sample received')
        else:
            self.validation_stats['imu']['invalid_samples'] += 1
            self.get_logger().warn(f'IMU: Invalid sample - {", ".join(errors)}')

        # Store data for statistics
        self.validation_stats['imu']['data_points'].append({
            'timestamp': self.get_clock().now(),
            'acceleration_magnitude': acc_magnitude,
            'angular_velocity_magnitude': ang_vel_magnitude,
            'errors': errors
        })

        # Keep only recent data (last 100 samples)
        if len(self.validation_stats['imu']['data_points']) > 100:
            self.validation_stats['imu']['data_points'] = self.validation_stats['imu']['data_points'][-100:]

    def depth_callback(self, msg):
        """Validate depth image data."""
        is_valid = True
        errors = []

        # Check header
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            errors.append("Invalid timestamp")
            is_valid = False

        # Check image dimensions
        if msg.width <= 0 or msg.height <= 0:
            errors.append("Invalid image dimensions")
            is_valid = False

        # Check encoding
        if msg.encoding not in ['32FC1', '16UC1', 'mono16']:
            errors.append(f"Unexpected depth encoding: {msg.encoding}")
            is_valid = False

        # Convert to OpenCV format and validate
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if cv_image is None:
                errors.append("Failed to convert depth image")
                is_valid = False
            else:
                # Check depth values
                valid_depths = cv_image[cv_image > 0]
                if len(valid_depths) > 0:
                    if np.min(valid_depths) < 0.05 or np.max(valid_depths) > 100.0:  # Reasonable limits
                        self.get_logger().debug(f'Depth: Values outside typical range: {np.min(valid_depths):.2f} - {np.max(valid_depths):.2f}m')

                # Check for excessive invalid (zero) pixels
                invalid_pixels = np.sum(cv_image == 0)
                total_pixels = cv_image.size
                invalid_ratio = invalid_pixels / total_pixels
                if invalid_ratio > 0.9:  # More than 90% invalid
                    errors.append(f"Too many invalid depth pixels: {invalid_ratio:.1%}")
                    is_valid = False
        except Exception as e:
            errors.append(f"Error processing depth image: {str(e)}")
            is_valid = False

        # Check timing consistency
        if self.last_depth_time is not None:
            time_diff = (msg.header.stamp.sec - self.last_depth_time.sec) + \
                       (msg.header.stamp.nanosec - self.last_depth_time.nanosec) * 1e-9
            if time_diff <= 0:
                errors.append("Non-increasing timestamps")
                is_valid = False

        self.last_depth_time = msg.header.stamp

        # Log validation results
        if is_valid:
            self.validation_stats['depth']['valid_samples'] += 1
            self.get_logger().debug(f'Depth: Valid sample received ({msg.width}x{msg.height})')
        else:
            self.validation_stats['depth']['invalid_samples'] += 1
            self.get_logger().warn(f'Depth: Invalid sample - {", ".join(errors)}')

        # Store data for statistics
        self.validation_stats['depth']['data_points'].append({
            'timestamp': self.get_clock().now(),
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'errors': errors
        })

        # Keep only recent data (last 100 samples)
        if len(self.validation_stats['depth']['data_points']) > 100:
            self.validation_stats['depth']['data_points'] = self.validation_stats['depth']['data_points'][-100:]

    def rgb_callback(self, msg):
        """Validate RGB image data."""
        is_valid = True
        errors = []

        # Check header
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            errors.append("Invalid timestamp")
            is_valid = False

        # Check image dimensions
        if msg.width <= 0 or msg.height <= 0:
            errors.append("Invalid image dimensions")
            is_valid = False

        # Check encoding
        if msg.encoding not in ['rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8', 'mono16']:
            errors.append(f"Unexpected RGB encoding: {msg.encoding}")
            is_valid = False

        # Convert to OpenCV format and validate
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if cv_image is None:
                errors.append("Failed to convert RGB image")
                is_valid = False
            else:
                if cv_image.shape[0] != msg.height or cv_image.shape[1] != msg.width:
                    errors.append("Image dimensions mismatch")
                    is_valid = False
        except Exception as e:
            errors.append(f"Error processing RGB image: {str(e)}")
            is_valid = False

        # Log validation results
        if is_valid:
            self.validation_stats['rgb']['valid_samples'] += 1
            self.get_logger().debug(f'RGB: Valid sample received ({msg.width}x{msg.height})')
        else:
            self.validation_stats['rgb']['invalid_samples'] += 1
            self.get_logger().warn(f'RGB: Invalid sample - {", ".join(errors)}')

        # Store data for statistics
        self.validation_stats['rgb']['data_points'].append({
            'timestamp': self.get_clock().now(),
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'errors': errors
        })

        # Keep only recent data (last 100 samples)
        if len(self.validation_stats['rgb']['data_points']) > 100:
            self.validation_stats['rgb']['data_points'] = self.validation_stats['rgb']['data_points'][-100:]

    def camera_info_callback(self, msg):
        """Validate camera info data."""
        is_valid = True
        errors = []

        # Check header
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            errors.append("Invalid timestamp")
            is_valid = False

        # Check dimensions
        if msg.width <= 0 or msg.height <= 0:
            errors.append("Invalid camera dimensions")
            is_valid = False

        # Check intrinsic matrix
        if len(msg.k) != 9:
            errors.append("Invalid K matrix size")
            is_valid = False
        else:
            # Check that focal lengths are positive
            if msg.k[0] <= 0 or msg.k[4] <= 0:  # fx and fy
                errors.append("Invalid focal lengths in K matrix")
                is_valid = False

        # Check projection matrix
        if len(msg.p) != 12:
            errors.append("Invalid P matrix size")
            is_valid = False

        # Log validation results
        if is_valid:
            self.get_logger().debug('CameraInfo: Valid sample received')
        else:
            self.get_logger().warn(f'CameraInfo: Invalid sample - {", ".join(errors)}')

    def report_validation_status(self):
        """Report validation statistics."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("SENSOR VALIDATION REPORT")
        self.get_logger().info("=" * 60)

        for sensor_type, stats in self.validation_stats.items():
            total_samples = stats['valid_samples'] + stats['invalid_samples']
            if total_samples > 0:
                valid_ratio = stats['valid_samples'] / total_samples
                self.get_logger().info(f"{sensor_type.upper()}:")
                self.get_logger().info(f"  Valid samples: {stats['valid_samples']}")
                self.get_logger().info(f"  Invalid samples: {stats['invalid_samples']}")
                self.get_logger().info(f"  Success rate: {valid_ratio:.1%}")

                if sensor_type == 'lidar' and stats['data_points']:
                    avg_valid_points = statistics.mean([d['valid_points_ratio'] for d in stats['data_points']]) if stats['data_points'] else 0
                    self.get_logger().info(f"  Average valid points ratio: {avg_valid_points:.1%}")
            else:
                self.get_logger().info(f"{sensor_type.upper()}: No data received yet")

        # Overall assessment
        all_success_rates = []
        for stats in self.validation_stats.values():
            total = stats['valid_samples'] + stats['invalid_samples']
            if total > 0:
                all_success_rates.append(stats['valid_samples'] / total)

        if all_success_rates:
            overall_success_rate = statistics.mean(all_success_rates)
            self.get_logger().info(f"\nOverall validation success rate: {overall_success_rate:.1%}")

            if overall_success_rate >= 0.95:
                self.get_logger().info("✅ EXCELLENT: Sensor data streams are highly realistic and ROS 2 compatible")
            elif overall_success_rate >= 0.80:
                self.get_logger().info("⚠️  GOOD: Sensor data streams are mostly realistic with minor issues")
            else:
                self.get_logger().info("❌ POOR: Sensor data streams have significant issues that need attention")

        self.get_logger().info("=" * 60)

    def run_final_validation(self):
        """Run final validation and return comprehensive assessment."""
        self.get_logger().info("Running final validation assessment...")

        results = {}
        for sensor_type, stats in self.validation_stats.items():
            total_samples = stats['valid_samples'] + stats['invalid_samples']
            if total_samples > 0:
                valid_ratio = stats['valid_samples'] / total_samples
                results[sensor_type] = {
                    'valid_ratio': valid_ratio,
                    'total_samples': total_samples,
                    'valid_samples': stats['valid_samples'],
                    'invalid_samples': stats['invalid_samples']
                }

        return results


def main(args=None):
    rclpy.init(args=args)

    validator = SensorValidationChecker()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Validation interrupted by user')
        results = validator.run_final_validation()

        # Print final summary
        validator.get_logger().info("\nFINAL VALIDATION SUMMARY:")
        for sensor, data in results.items():
            validator.get_logger().info(f"{sensor.upper()}: {data['valid_ratio']:.1%} valid ({data['valid_samples']}/{data['total_samples']})")

    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()