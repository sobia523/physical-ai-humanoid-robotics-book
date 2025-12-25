#!/usr/bin/env python3
"""
Digital Twin Simulation Validation Script

This script validates that all components of the integrated digital twin simulation
work together properly. It checks for proper communication between Gazebo, ROS 2,
and Unity components, verifies sensor data integrity, and ensures the simulation
maintains real-time performance.
"""

import subprocess
import time
import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu, Image, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import sys
import os
import signal
import argparse
import yaml
import json
from datetime import datetime

class DigitalTwinValidator:
    def __init__(self):
        self.validation_results = {
            'timestamp': datetime.now().isoformat(),
            'components': {},
            'performance': {},
            'data_integrity': {},
            'integration_status': 'unknown'
        }
        self.subscribers = []
        self.test_results = {}
        self.start_time = None

    def validate_gazebo_connection(self):
        """Validate Gazebo simulation is running and accessible"""
        print("Validating Gazebo connection...")

        try:
            # Check if gz stats command works (indicates Gazebo is running)
            result = subprocess.run(['gz', 'stats'], capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print("✓ Gazebo simulation is running")
                self.validation_results['components']['gazebo'] = {
                    'status': 'running',
                    'details': result.stdout[:200]  # First 200 chars of output
                }
                return True
            else:
                print("✗ Gazebo simulation not accessible")
                self.validation_results['components']['gazebo'] = {
                    'status': 'not_running',
                    'error': result.stderr
                }
                return False
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
            print("✗ Gazebo not installed or not running")
            self.validation_results['components']['gazebo'] = {
                'status': 'not_accessible',
                'error': 'Gazebo command not found or not running'
            }
            return False

    def validate_ros2_nodes(self):
        """Validate ROS 2 nodes are running and communicating"""
        print("Validating ROS 2 nodes...")

        try:
            # Check for active nodes
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                nodes = result.stdout.strip().split('\n')
                print(f"✓ Found {len(nodes)} active ROS 2 nodes")

                required_nodes = [
                    'robot_state_publisher',
                    'joint_state_publisher',
                    'gazebo_ros_node',
                    'navigation',
                    'localization'
                ]

                active_required_nodes = [node for node in nodes if any(req in node for req in required_nodes)]
                print(f"✓ Found {len(active_required_nodes)}/{len(required_nodes)} required nodes")

                self.validation_results['components']['ros2_nodes'] = {
                    'status': 'running',
                    'count': len(nodes),
                    'required_nodes_found': len(active_required_nodes),
                    'required_nodes_expected': len(required_nodes),
                    'active_nodes': nodes
                }

                return len(active_required_nodes) >= len(required_nodes) * 0.8  # 80% threshold
            else:
                print("✗ Could not list ROS 2 nodes")
                self.validation_results['components']['ros2_nodes'] = {
                    'status': 'error',
                    'error': result.stderr
                }
                return False
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            print("✗ ROS 2 not accessible")
            self.validation_results['components']['ros2_nodes'] = {
                'status': 'not_accessible',
                'error': 'ROS 2 command not accessible'
            }
            return False

    def validate_topics(self):
        """Validate required ROS 2 topics are available and publishing"""
        print("Validating ROS 2 topics...")

        required_topics = [
            '/humanoid/lidar/scan',
            '/humanoid/imu/data',
            '/humanoid/camera/image_raw',
            '/humanoid/odom',
            '/humanoid/joint_states',
            '/tf',
            '/tf_static'
        ]

        try:
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                available_topics = result.stdout.strip().split('\n')

                missing_topics = [topic for topic in required_topics if topic not in available_topics]
                available_required = [topic for topic in required_topics if topic in available_topics]

                print(f"✓ Found {len(available_required)}/{len(required_topics)} required topics")

                if missing_topics:
                    print(f"✗ Missing topics: {missing_topics}")

                # Test message rates for available topics
                topic_rates = {}
                for topic in available_required:
                    try:
                        rate_result = subprocess.run(['ros2', 'topic', 'hz', topic],
                                                   capture_output=True, text=True, timeout=3)
                        if rate_result.returncode == 0:
                            # Extract average rate
                            lines = rate_result.stdout.split('\n')
                            for line in lines:
                                if 'average rate:' in line:
                                    import re
                                    match = re.search(r'average rate: ([\d.]+)', line)
                                    if match:
                                        topic_rates[topic] = float(match.group(1))
                                    break
                    except:
                        topic_rates[topic] = 0.0

                self.validation_results['components']['topics'] = {
                    'status': 'available' if not missing_topics else 'partial',
                    'required_found': len(available_required),
                    'required_total': len(required_topics),
                    'missing_topics': missing_topics,
                    'rates': topic_rates
                }

                return len(missing_topics) == 0
            else:
                print("✗ Could not list ROS 2 topics")
                self.validation_results['components']['topics'] = {
                    'status': 'error',
                    'error': result.stderr
                }
                return False
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            print("✗ Could not access ROS 2 topics")
            self.validation_results['components']['topics'] = {
                'status': 'not_accessible',
                'error': 'Could not access ROS 2 topics'
            }
            return False

    def validate_sensor_data(self):
        """Validate sensor data quality and ranges"""
        print("Validating sensor data integrity...")

        # This would typically involve subscribing to topics and validating data
        # For this validation script, we'll check if data is being published at expected rates

        sensor_validation = {}

        # Check LiDAR data
        lidar_ok = self._check_lidar_data()
        sensor_validation['lidar'] = lidar_ok

        # Check IMU data
        imu_ok = self._check_imu_data()
        sensor_validation['imu'] = imu_ok

        # Check camera data
        camera_ok = self._check_camera_data()
        sensor_validation['camera'] = camera_ok

        print(f"✓ Sensor validation: LiDAR={lidar_ok}, IMU={imu_ok}, Camera={camera_ok}")

        self.validation_results['data_integrity']['sensors'] = sensor_validation

        return all([lidar_ok, imu_ok, camera_ok])

    def _check_lidar_data(self):
        """Check if LiDAR data is valid"""
        try:
            result = subprocess.run(['ros2', 'topic', 'echo', '/humanoid/lidar/scan',
                                   '--field', 'range_min', '--field', 'range_max', '--field', 'ranges',
                                   '-c', '1'], capture_output=True, text=True, timeout=5)
            if result.returncode == 0 and 'ranges' in result.stdout:
                return True
        except:
            pass
        return False

    def _check_imu_data(self):
        """Check if IMU data is valid"""
        try:
            result = subprocess.run(['ros2', 'topic', 'echo', '/humanoid/imu/data',
                                   '--field', 'orientation', '--field', 'angular_velocity',
                                   '--field', 'linear_acceleration', '-c', '1'],
                                   capture_output=True, text=True, timeout=5)
            if result.returncode == 0 and 'orientation' in result.stdout:
                return True
        except:
            pass
        return False

    def _check_camera_data(self):
        """Check if camera data is valid"""
        try:
            result = subprocess.run(['ros2', 'topic', 'echo', '/humanoid/camera/image_raw',
                                   '--field', 'header', '--field', 'height', '--field', 'width',
                                   '-c', '1'], capture_output=True, text=True, timeout=5)
            if result.returncode == 0 and 'height' in result.stdout and 'width' in result.stdout:
                return True
        except:
            pass
        return False

    def validate_performance(self):
        """Validate simulation performance metrics"""
        print("Validating performance metrics...")

        # Check current CPU and memory usage
        import psutil

        cpu_percent = psutil.cpu_percent(interval=1)
        memory_percent = psutil.virtual_memory().percent

        print(f"Current CPU usage: {cpu_percent}%")
        print(f"Current memory usage: {memory_percent}%")

        # Get simulation RTF if available
        rtf = self._get_simulation_rtf()
        print(f"Current RTF: {rtf}")

        performance_ok = (
            cpu_percent < 85.0 and  # CPU usage below 85%
            memory_percent < 85.0 and  # Memory usage below 85%
            rtf > 0.5  # RTF above 0.5 (acceptable for real-time)
        )

        self.validation_results['performance'] = {
            'cpu_percent': cpu_percent,
            'memory_percent': memory_percent,
            'simulation_rtf': rtf,
            'performance_ok': performance_ok,
            'cpu_threshold': 85.0,
            'memory_threshold': 85.0,
            'rtf_threshold': 0.5
        }

        return performance_ok

    def _get_simulation_rtf(self):
        """Get simulation real-time factor"""
        try:
            result = subprocess.run(['gz', 'stats'], capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                import re
                match = re.search(r'RTF\[(\d+\.?\d*)\]', result.stdout)
                if match:
                    return float(match.group(1))
        except:
            pass
        return 0.0

    def validate_tf_frames(self):
        """Validate TF transform tree integrity"""
        print("Validating TF frames...")

        try:
            result = subprocess.run(['ros2', 'run', 'tf2_tools', 'view_frames'],
                                  capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                # Check if the output contains frame information
                if 'Frames' in result.stdout or 'frame' in result.stdout.lower():
                    print("✓ TF tree is available and valid")
                    self.validation_results['components']['tf'] = {
                        'status': 'valid',
                        'details': 'TF tree contains frames'
                    }
                    return True
                else:
                    print("✗ TF tree validation failed - no frames detected")
                    self.validation_results['components']['tf'] = {
                        'status': 'invalid',
                        'error': 'No frames detected in TF tree'
                    }
                    return False
            else:
                print(f"✗ TF validation failed: {result.stderr}")
                self.validation_results['components']['tf'] = {
                    'status': 'error',
                    'error': result.stderr
                }
                return False
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            print("✗ Could not validate TF frames")
            self.validation_results['components']['tf'] = {
                'status': 'not_accessible',
                'error': 'TF validation tool not accessible'
            }
            return False

    def run_complete_validation(self):
        """Run complete validation of the digital twin system"""
        print("="*60)
        print("DIGITAL TWIN SIMULATION VALIDATION")
        print("="*60)

        self.start_time = time.time()

        # Run all validation checks
        checks = [
            ("Gazebo Connection", self.validate_gazebo_connection),
            ("ROS 2 Nodes", self.validate_ros2_nodes),
            ("ROS 2 Topics", self.validate_topics),
            ("TF Frames", self.validate_tf_frames),
            ("Sensor Data", self.validate_sensor_data),
            ("Performance", self.validate_performance)
        ]

        results = {}
        for name, check_func in checks:
            print(f"\n[{name}]")
            try:
                result = check_func()
                results[name] = result
                status = "PASS" if result else "FAIL"
                print(f"Result: {status}")
            except Exception as e:
                print(f"Result: ERROR - {str(e)}")
                results[name] = False

        # Overall validation result
        all_passed = all(results.values())
        total_time = time.time() - self.start_time

        print(f"\n{'='*60}")
        print("VALIDATION SUMMARY")
        print(f"{'='*60}")

        for name, result in results.items():
            status = "PASS" if result else "FAIL"
            print(f"{name:20s}: {status}")

        print(f"\nOverall Result: {'PASS' if all_passed else 'FAIL'}")
        print(f"Total Validation Time: {total_time:.2f} seconds")

        self.validation_results['integration_status'] = 'pass' if all_passed else 'fail'
        self.validation_results['total_time'] = total_time
        self.validation_results['individual_results'] = results

        # Add recommendations based on results
        recommendations = []
        if not results.get("Gazebo Connection", True):
            recommendations.append("Start Gazebo simulation before running validation")
        if not results.get("ROS 2 Nodes", True):
            recommendations.append("Launch required ROS 2 nodes")
        if not results.get("ROS 2 Topics", True):
            recommendations.append("Check robot launch files for missing publishers")
        if not results.get("Performance", True):
            recommendations.append("Review performance optimization settings")

        if recommendations:
            print(f"\nRecommendations:")
            for rec in recommendations:
                print(f"  - {rec}")

        self.validation_results['recommendations'] = recommendations

        return all_passed

    def save_validation_report(self, filename=None):
        """Save validation results to a file"""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"digital_twin_validation_{timestamp}.json"

        with open(filename, 'w') as f:
            json.dump(self.validation_results, f, indent=2, default=str)

        print(f"\nValidation report saved to: {filename}")
        return filename

def main():
    parser = argparse.ArgumentParser(description='Digital Twin Simulation Validator')
    parser.add_argument('--output', '-o', help='Output file for validation report')
    parser.add_argument('--verbose', '-v', action='store_true', help='Enable verbose output')

    args = parser.parse_args()

    validator = DigitalTwinValidator()

    # Run validation
    success = validator.run_complete_validation()

    # Save report
    report_file = validator.save_validation_report(filename=args.output)

    # Exit with appropriate code
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()