# Quickstart Guide: ROS 2 Development Environment Setup

**Feature**: 001-ros2-nervous-system
**Created**: 2025-12-19
**Status**: Complete

## Overview

This guide provides step-by-step instructions for setting up a ROS 2 development environment suitable for the "Robotic Nervous System" module. The setup includes ROS 2 Humble Hawksbill, development tools, and simulation environment.

## Prerequisites

Before starting, ensure your system meets these requirements:

### System Requirements
- Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- At least 4GB RAM (8GB recommended)
- At least 20GB free disk space
- Internet connection for package installation

### Alternative Systems
- For macOS: Use Docker with Ubuntu container
- For Windows: Use WSL2 with Ubuntu 22.04 or Docker Desktop

## Installation Steps

### Step 1: Update System Packages

```bash
sudo apt update && sudo apt upgrade -y
```

### Step 2: Set Locale

```bash
locale  # Check for UTF-8 support
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 3: Add ROS 2 Repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 4: Install ROS 2 Humble

```bash
sudo apt update
sudo apt install ros-humble-desktop-full
```

### Step 5: Install Additional Dependencies

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

### Step 6: Setup Environment

Add ROS 2 to your bash profile:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 7: Install Gazebo Classic (Simulation)

```bash
sudo apt install ros-humble-gazebo-*
sudo apt install gazebo
```

### Step 8: Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Verification

### Test Basic ROS 2 Installation

```bash
# Open a new terminal
source /opt/ros/humble/setup.bash

# Check ROS 2 version
ros2 --version

# Run a simple test
ros2 topic list
```

### Test rclpy (Python Client Library)

Create a simple test file:

```bash
cd ~/ros2_ws/src
mkdir -p my_robot_tutorials
cd my_robot_tutorials
mkdir -p my_robot_tutorials/my_robot_tutorials
touch my_robot_tutorials/__init__.py
```

Create a simple publisher test file `my_robot_tutorials/my_robot_tutorials/simple_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Make the file executable and test:

```bash
chmod +x my_robot_tutorials/my_robot_tutorials/simple_publisher.py

# In one terminal
cd ~/ros2_ws
source install/setup.bash
python3 src/my_robot_tutorials/my_robot_tutorials/simple_publisher.py

# In another terminal
source /opt/ros/humble/setup.bash
ros2 topic echo /topic std_msgs/msg/String
```

## Python Development Setup

### Install Python Virtual Environment

```bash
sudo apt install python3-venv
python3 -m venv ~/ros2_env
source ~/ros2_env/bin/activate
pip install --upgrade pip
```

### Install Additional Python Dependencies

```bash
pip install rclpy
# Note: rclpy is typically installed with ROS 2, but this ensures it's available in the virtual environment
```

## Simulation Environment Test

### Launch Gazebo with Empty World

```bash
source /opt/ros/humble/setup.bash
gz sim (or gazebo depending on version)
```

### Test with Simple Robot Model

```bash
# Download a simple robot model
mkdir -p ~/ros2_ws/src/models
cd ~/ros2_ws/src/models
wget https://github.com/ros-simulation/gazebo_ros_pkgs/raw/ros2/gazebo_ros/test/worlds/simple_arm.world

# Launch Gazebo with the world
gz sim -r simple_arm.world
```

## Troubleshooting

### Common Issues and Solutions

1. **ROS 2 commands not found**:
   - Ensure you sourced the setup.bash file: `source /opt/ros/humble/setup.bash`
   - Check your bashrc file: `cat ~/.bashrc | grep ros`

2. **Python import errors**:
   - Make sure you're in the correct terminal session with ROS 2 sourced
   - Check that rclpy is installed: `python3 -c "import rclpy"`

3. **Gazebo fails to launch**:
   - Check graphics drivers: `nvidia-smi` (if using NVIDIA)
   - Try running with software rendering: `export LIBGL_ALWAYS_SOFTWARE=1`

4. **Workspace build fails**:
   - Ensure all dependencies are installed: `rosdep install --from-paths src --ignore-src -r -y`
   - Clean build: `rm -rf build install log && colcon build`

### Environment Variables

If needed, set these environment variables:

```bash
# Add to ~/.bashrc for persistent settings
export ROS_DOMAIN_ID=0  # Default domain
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # DDS implementation
export GAZEBO_MODEL_PATH=~/ros2_ws/src/models:$GAZEBO_MODEL_PATH
```

## Next Steps

With your environment set up, you're ready to:

1. Follow the ROS 2 tutorials in the upcoming chapters
2. Create your first ROS 2 nodes using rclpy
3. Work with URDF models for humanoid robots
4. Integrate Python agents with ROS 2 systems

## Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Gazebo Documentation](http://gazebosim.org/tutorials)