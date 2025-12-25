# ROS 2 Humble Hawksbill Installation and Setup Guide for Students

## Overview

This guide provides step-by-step instructions for installing and setting up ROS 2 Humble Hawksbill, which is the recommended ROS 2 distribution for this module. ROS 2 Humble is an LTS (Long Term Support) version with support until May 2027, making it ideal for educational purposes.

## System Requirements

### Minimum Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- **RAM**: At least 4GB (8GB recommended)
- **Disk Space**: At least 20GB free space
- **Internet Connection**: Required for package installation

### Alternative Systems
- **macOS**: Use Docker with Ubuntu container
- **Windows**: Use WSL2 with Ubuntu 22.04 or Docker Desktop

## Installation Steps

### Step 1: Update System Packages

First, ensure your system is up to date:

```bash
sudo apt update && sudo apt upgrade -y
```

### Step 2: Set Locale

Make sure your locale is set to UTF-8, which is required by ROS 2:

```bash
locale  # Check for UTF-8 support
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 3: Add ROS 2 Repository

Add the ROS 2 repository to your system:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 4: Install ROS 2 Humble Desktop Full

Install the complete ROS 2 Humble desktop package:

```bash
sudo apt update
sudo apt install ros-humble-desktop-full
```

### Step 5: Install Additional Dependencies

Install additional tools needed for development:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

### Step 6: Setup Environment

Add ROS 2 to your bash profile to make it available in new terminals:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 7: Install Gazebo for Simulation

Install Gazebo Classic (legacy) for simulation examples:

```bash
sudo apt install ros-humble-gazebo-*
sudo apt install gazebo
```

### Step 8: Create ROS 2 Workspace

Create a workspace for your ROS 2 projects:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Verification

### Test Basic ROS 2 Installation

Open a new terminal and run these commands to verify the installation:

```bash
# Source the ROS 2 setup
source /opt/ros/humble/setup.bash

# Check ROS 2 version
ros2 --version

# Verify ROS 2 can list topics (should return empty list, not error)
ros2 topic list
```

### Test rclpy (Python Client Library)

Create a simple test to verify Python integration:

1. Create a test package directory:
```bash
cd ~/ros2_ws/src
mkdir -p my_robot_tutorials
cd my_robot_tutorials
mkdir -p my_robot_tutorials/my_robot_tutorials
touch my_robot_tutorials/__init__.py
```

2. Create a simple publisher test file `my_robot_tutorials/my_robot_tutorials/simple_publisher.py`:

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

3. Make the file executable and test:
```bash
chmod +x my_robot_tutorials/my_robot_tutorials/simple_publisher.py

# In one terminal
cd ~/ros2_ws
source install/setup.bash
python3 src/my_robot_tutorials/my_robot_tutorials/simple_publisher.py

# In another terminal (while the first is running)
source /opt/ros/humble/setup.bash
ros2 topic echo /topic std_msgs/msg/String
```

If you see "Hello World" messages appearing in the second terminal, your ROS 2 installation is working correctly!

## Python Development Setup

### Install Python Virtual Environment

For better package management:

```bash
sudo apt install python3-venv
python3 -m venv ~/ros2_env
source ~/ros2_env/bin/activate
pip install --upgrade pip
```

Note: rclpy is typically installed with ROS 2, but you can ensure it's available in your virtual environment if needed.

## Simulation Environment Test

### Launch Gazebo with Empty World

```bash
source /opt/ros/humble/setup.bash
gz sim (or gazebo depending on version)
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

If needed, set these environment variables by adding to your `~/.bashrc`:

```bash
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