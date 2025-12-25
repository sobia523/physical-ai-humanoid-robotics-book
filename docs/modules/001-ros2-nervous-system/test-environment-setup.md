# Test Environment Setup for Code Examples with Gazebo Simulation

## Purpose

This document outlines the setup and configuration of the test environment for validating code examples in the ROS 2 Nervous System module. The environment includes Gazebo simulation capabilities for testing robot control examples.

## Test Environment Components

### 1. Core Components
- **ROS 2 Humble Hawksbill**: The core middleware framework
- **Gazebo Classic**: Physics-based simulation environment
- **Python 3.8+**: Runtime for Python examples
- **Standard ROS 2 packages**: rclpy, std_msgs, sensor_msgs, geometry_msgs, etc.

### 2. Testing Tools
- **Colcon**: Build tool for ROS 2 packages
- **pytest**: Python testing framework
- **launch files**: For testing complex scenarios
- **rqt tools**: For visualization and debugging

## System Requirements

### Minimum Requirements
- **OS**: Ubuntu 22.04 LTS
- **RAM**: 8GB or more (for simulation)
- **CPU**: Multi-core processor
- **Graphics**: OpenGL 2.1+ capable GPU for Gazebo rendering

### Recommended Configuration
- **OS**: Ubuntu 22.04 LTS with latest updates
- **RAM**: 16GB or more
- **CPU**: 4+ cores with good single-thread performance
- **Storage**: SSD with 50GB+ free space
- **Network**: Internet access for package installation and updates

## Environment Setup

### 1. Verify Core Installation

Ensure ROS 2 Humble and Gazebo are properly installed:

```bash
# Verify ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 --version

# Verify Gazebo installation
gz --version  # or gazebo for older versions

# Verify required packages
dpkg -l | grep ros-humble-desktop-full
dpkg -l | grep ros-humble-gazebo-*
```

### 2. Set Up Workspace for Testing

Create a dedicated workspace for testing:

```bash
mkdir -p ~/ros2_test_ws/src
cd ~/ros2_test_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Install Testing Dependencies

Install additional packages needed for testing:

```bash
sudo apt update
sudo apt install python3-pytest python3-pytest-cov python3-pytest-mock
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins
sudo apt install ros-humble-tf2-tools ros-humble-ros2-control*
sudo apt install ros-humble-gazebo-ros2-control ros-humble-gazebo-ros-pkgs
```

## Test Environment Configuration

### 1. Environment Variables

Add these to your `~/.bashrc` for consistent testing:

```bash
# ROS 2 configuration
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Gazebo configuration
export GAZEBO_MODEL_PATH=~/ros2_test_ws/src/models:~/.gazebo/models
export GAZEBO_RESOURCE_PATH=~/ros2_test_ws/src/worlds:~/.gazebo/worlds

# Testing configuration
export PYTHONPATH=$PYTHONPATH:~/ros2_test_ws/install/lib/python3.10/site-packages
```

### 2. Test Launch Configuration

Create a test launch configuration file:

```bash
mkdir -p ~/ros2_test_ws/src/test_launch
```

Create `~/ros2_test_ws/src/test_launch/test_environment.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    """Launch file for setting up the test environment."""

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo server node
    gazebo_server = ExecuteProcess(
        cmd=['gz sim -s'],
        output='screen'
    )

    # Gazebo client node (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gz sim gui'],
        output='screen'
    )

    # Example test node (to be created later)
    test_node = Node(
        package='ros2_nervous_system_examples',
        executable='simple_publisher',
        name='test_publisher',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        gazebo_server,
        gazebo_client,
        test_node
    ])
```

## Validation Process

### 1. Basic Environment Validation

Run these commands to verify the test environment:

```bash
# Test basic ROS 2 functionality
source /opt/ros/humble/setup.bash
ros2 topic list

# Test Python import capabilities
python3 -c "import rclpy; print('rclpy import successful')"

# Test Gazebo (in separate terminals)
gz sim --version
```

### 2. Simple Test Node Validation

Create a simple test to validate the environment:

```bash
# Create test directory
mkdir -p ~/ros2_test_ws/src/test_validation
cd ~/ros2_test_ws/src/test_validation

# Create test package.xml
cat > package.xml << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>test_validation</name>
  <version>0.1.0</version>
  <description>Test validation for ROS 2 environment</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# Create setup files
cat > setup.py << EOF
from setuptools import setup

setup(
    name='test_validation',
    version='0.1.0',
    packages=[],
    py_modules=['test_publisher', 'test_subscriber'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/test_validation']),
        ('share/test_validation', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Test',
    maintainer_email='test@example.com',
    description='Test validation for ROS 2 environment',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'test_publisher = test_publisher:main',
            'test_subscriber = test_subscriber:main',
        ],
    },
)
EOF

cat > setup.cfg << EOF
[develop]
script-dir=$base/lib/test_validation
[install]
install-scripts=$base/lib/test_validation
EOF

# Create test publisher
cat > test_publisher.py << EOF
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Test message {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestPublisher()

    try:
        rclpy.spin(test_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        test_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

# Create test subscriber
cat > test_subscriber.py << EOF
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(
            String,
            'test_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    test_subscriber = TestSubscriber()

    try:
        rclpy.spin(test_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        test_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

### 3. Build and Run Validation

```bash
# Build the test package
cd ~/ros2_test_ws
colcon build --packages-select test_validation
source install/setup.bash

# Test publisher and subscriber in separate terminals
# Terminal 1:
ros2 run test_validation test_publisher

# Terminal 2:
ros2 run test_validation test_subscriber
```

## Gazebo Simulation Validation

### 1. Test Gazebo Integration

```bash
# Create a simple URDF test model
mkdir -p ~/ros2_test_ws/src/models/test_robot
cat > ~/ros2_test_ws/src/models/test_robot/model.urdf << EOF
<?xml version="1.0"?>
<robot name="test_robot">
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
</robot>
EOF

# Test launching Gazebo with the model
gz sim -r ~/ros2_test_ws/src/models/test_robot/model.urdf
```

### 2. Validate ROS 2 - Gazebo Integration

Create a joint state publisher test:

```bash
cat > ~/ros2_test_ws/src/test_validation/joint_state_publisher.py << EOF
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['test_joint']
        msg.position = [math.sin(self.i * 0.1)]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()

    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

# Add to setup.py entry points:
# 'test_joint_state_publisher = joint_state_publisher:main',

## Automated Testing Setup

### 1. Create Test Scripts

Create automated test validation scripts:

```bash
# Create test script directory
mkdir -p ~/ros2_test_ws/src/test_scripts

# Create environment validation script
cat > ~/ros2_test_ws/src/test_scripts/validate_environment.sh << 'EOF'
#!/bin/bash

echo "Validating ROS 2 Nervous System Test Environment..."

# Check if ROS 2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS 2 is not installed or not in PATH"
    exit 1
else
    echo "✅ ROS 2 installation found: $(ros2 --version)"
fi

# Check if Gazebo is installed
if ! command -v gz &> /dev/null && ! command -v gazebo &> /dev/null; then
    echo "❌ Gazebo is not installed"
    exit 1
else
    if command -v gz &> /dev/null; then
        echo "✅ Gazebo installation found: $(gz --version)"
    else
        echo "✅ Gazebo installation found"
    fi
fi

# Check Python modules
python3 -c "import rclpy" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ rclpy module available"
else
    echo "❌ rclpy module not available"
    exit 1
fi

# Check required ROS packages
if ros2 pkg list | grep -q "std_msgs"; then
    echo "✅ std_msgs package available"
else
    echo "❌ std_msgs package not available"
    exit 1
fi

if ros2 pkg list | grep -q "sensor_msgs"; then
    echo "✅ sensor_msgs package available"
else
    echo "❌ sensor_msgs package not available"
    exit 1
fi

echo "✅ All basic environment checks passed!"
echo "Environment validation complete."
EOF

chmod +x ~/ros2_test_ws/src/test_scripts/validate_environment.sh
```

### 2. Unit Test Framework

Create a basic unit test framework:

```bash
# Create test directory structure
mkdir -p ~/ros2_test_ws/src/ros2_nervous_system_examples/test

# Create test configuration
cat > ~/ros2_test_ws/src/ros2_nervous_system_examples/test/test_config.py << EOF
"""
Test configuration for ROS 2 Nervous System examples.
"""
import os

# Test parameters
TEST_TIMEOUT = 10.0  # seconds
TEST_NODE_NAME = "test_node"
TEST_TOPIC_NAME = "test_topic"
TEST_MESSAGE_TYPE = "std_msgs/msg/String"

# Environment settings
USE_SIM_TIME = True
SIMULATION_SPEED = 1.0  # real-time
EOF
```

## Continuous Integration Testing

### 1. Test Workflow

Set up a basic test workflow for validating examples:

```bash
# Create test workflow directory
mkdir -p ~/ros2_test_ws/src/test_workflows

# Create test runner script
cat > ~/ros2_test_ws/src/test_workflows/test_runner.py << 'EOF'
#!/usr/bin/env python3
"""
Test runner for ROS 2 Nervous System examples.
"""
import subprocess
import sys
import time
import signal
import os

def run_test(test_name, test_command, timeout=30):
    """Run a single test and return results."""
    print(f"Running test: {test_name}")

    try:
        # Start the test process
        process = subprocess.Popen(
            test_command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # Wait for completion or timeout
        stdout, stderr = process.communicate(timeout=timeout)

        if process.returncode == 0:
            print(f"✅ {test_name} PASSED")
            return True
        else:
            print(f"❌ {test_name} FAILED")
            print(f"Error: {stderr.decode()}")
            return False

    except subprocess.TimeoutExpired:
        process.kill()
        print(f"❌ {test_name} TIMED OUT")
        return False
    except Exception as e:
        print(f"❌ {test_name} ERROR: {str(e)}")
        return False

def main():
    """Run all tests in the test suite."""
    tests = [
        {
            "name": "Basic Publisher Test",
            "command": "timeout 5 ros2 run test_validation test_publisher"
        },
        {
            "name": "Basic Subscriber Test",
            "command": "timeout 5 ros2 run test_validation test_subscriber"
        }
    ]

    passed = 0
    total = len(tests)

    for test in tests:
        if run_test(test["name"], test["command"]):
            passed += 1

    print(f"\nTest Results: {passed}/{total} tests passed")

    if passed == total:
        print("🎉 All tests passed!")
        return 0
    else:
        print("💥 Some tests failed!")
        return 1

if __name__ == "__main__":
    sys.exit(main())
EOF
```

## Final Validation

### 1. Complete Environment Check

Run the complete validation:

```bash
# Make sure we're in the right directory and environment is sourced
cd ~/ros2_test_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run the validation script
~/ros2_test_ws/src/test_scripts/validate_environment.sh

# Build all test packages
colcon build --packages-select test_validation
source install/setup.bash

# Run basic communication test
# Terminal 1 (keep running):
ros2 run test_validation test_publisher &

# Terminal 2:
sleep 2  # Wait for publisher to start
ros2 run test_validation test_subscriber --once

# Kill publisher
kill %1
```

### 2. Simulation Validation

Test the Gazebo integration:

```bash
# Test that Gazebo can be launched
gz sim --help >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ Gazebo can be launched"
else
    echo "❌ Gazebo launch failed"
fi

# Test ROS 2 - Gazebo bridge (if installed)
if ros2 pkg list | grep -q "gazebo_ros_pkgs"; then
    echo "✅ Gazebo ROS packages available"
else
    echo "⚠️ Gazebo ROS packages not found (this may be OK depending on requirements)"
fi
```

## Troubleshooting Common Issues

### 1. Environment Variables Not Set

If tests fail due to environment issues:

```bash
# Ensure environment is properly sourced
source /opt/ros/humble/setup.bash
source ~/ros2_test_ws/install/setup.bash

# Verify environment variables
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
```

### 2. Permission Issues

If Gazebo fails to launch:

```bash
# Check graphics permissions
ls -la /tmp/.X11-unix/
xhost +local:docker  # if using Docker
```

### 3. Package Build Issues

If packages fail to build:

```bash
# Clean build
rm -rf ~/ros2_test_ws/build ~/ros2_test_ws/install ~/ros2_test_ws/log
colcon build --packages-select test_validation
```

## Summary

This test environment provides:

- ✅ Basic ROS 2 functionality validation
- ✅ Gazebo simulation integration
- ✅ Automated testing framework
- ✅ Continuous integration workflow
- ✅ Troubleshooting guidelines

The environment is now ready for validating all code examples in the ROS 2 Nervous System module.
```

Update setup.py to include the new entry point:
