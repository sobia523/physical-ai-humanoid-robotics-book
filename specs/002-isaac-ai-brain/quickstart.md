# Quickstart Guide: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)

## Overview
This quickstart guide provides the essential setup and initial steps to understand and implement the AI-Robot Brain concepts using NVIDIA Isaac technologies. This guide is designed for advanced students and professionals in AI and robotics who are familiar with ROS 2.

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **GPU**: NVIDIA GPU with CUDA support (RTX series recommended)
- **RAM**: 16GB minimum, 32GB recommended
- **CPU**: Multi-core processor (Intel i5 or AMD Ryzen 5 equivalent)
- **Storage**: 50GB free space for Isaac Sim and dependencies

### Software Dependencies
- ROS 2 Humble Hawksbill
- NVIDIA Isaac Sim
- Isaac ROS packages
- CUDA Toolkit 11.8 or later
- Docker (for containerized deployments)

## Installation Steps

### 1. Install ROS 2 Humble
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### 2. Install NVIDIA Isaac Sim
```bash
# Download Isaac Sim from NVIDIA Developer website
# Follow the installation guide at: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html

# Verify installation
cd ~/isaac-sim
./isaac-sim-headless.sh --version
```

### 3. Install Isaac ROS Packages
```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws

# Install Isaac ROS dependencies
sudo apt update
sudo apt install -y ros-humble-isaac-ros-perception ros-humble-isaac-ros-stereo-depth ros-humble-isaac-ros-visual- slam ros-humble-isaac-ros-message-filters

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### 4. Install Navigation2
```bash
# Install Nav2 packages
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
```

## Basic Setup and Verification

### 1. Verify Isaac Sim Installation
```bash
# Launch Isaac Sim in headless mode to test
cd ~/isaac-sim
./isaac-sim-headless.sh --/headlessRendering/enabled=True --/renderer/enableHDRISky=False
```

### 2. Test Perception Pipeline
```bash
# Source your workspace
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Run a basic stereo depth estimation pipeline
ros2 launch isaac_ros_stereo_image_proc isaac_ros_stereo_image_proc.launch.py
```

### 3. Test VSLAM
```bash
# Launch a simple VSLAM example
ros2 launch isaac_ros_visual_slam visual_slam_node.launch.py
```

## Running Your First AI-Robot Brain Example

### 1. Launch Isaac Sim Environment
```bash
# Navigate to Isaac Sim examples
cd ~/isaac-sim
./isaac-sim.sh

# In Isaac Sim, load the humanoid robot example environment
# Use the "Create -> Robot -> Mobile Manipulator" template
```

### 2. Launch Perception Pipeline
```bash
# In a new terminal
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Launch perception pipeline
ros2 launch your_perception_package humanoid_perception.launch.py
```

### 3. Launch Navigation Stack
```bash
# Launch Nav2 for the humanoid robot
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=~/isaac_ros_ws/src/your_robot_config/nav2_params.yaml
```

## Key Configuration Files

### Perception Pipeline Configuration
Create `~/isaac_ros_ws/src/your_robot_config/config/perception_pipeline.yaml`:

```yaml
perception_pipeline:
  object_detection:
    model_path: "/path/to/detection/model"
    confidence_threshold: 0.7
    max_objects: 50
  depth_estimation:
    min_depth: 0.1
    max_depth: 20.0
    resolution: [640, 480]
  sensor_fusion:
    camera_topic: "/humanoid/camera/image_raw"
    lidar_topic: "/humanoid/lidar/scan"
    imu_topic: "/humanoid/imu/data"
```

### VSLAM Configuration
Create `~/isaac_ros_ws/src/your_robot_config/config/vslam_config.yaml`:

```yaml
visual_slam:
  tracking:
    enable_visual_odometry: true
    enable_loop_closure: true
    enable_map_localization: true
  mapping:
    map_resolution: 0.05
    map_size: [20.0, 20.0]
    max_keyframes: 1000
  sensor_params:
    camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    distortion_coeffs: [k1, k2, p1, p2, k3]
    imu_rate: 100
```

## Troubleshooting Common Issues

### Isaac Sim Not Launching
- Verify GPU drivers are up to date
- Check CUDA installation: `nvidia-smi` and `nvcc --version`
- Ensure sufficient system resources are available

### Perception Pipeline Not Processing Data
- Verify sensor topics are publishing: `ros2 topic list`
- Check sensor calibration parameters
- Confirm GPU acceleration is enabled

### Navigation Not Working
- Verify TF tree is complete: `ros2 run tf2_tools view_frames`
- Check costmap configuration
- Ensure proper localization (VSLAM) is running

## Next Steps

1. Complete Chapter 1: Introduction to AI-Robot Brain to understand core concepts
2. Explore Isaac Sim examples for synthetic data generation
3. Experiment with different perception pipelines
4. Configure VSLAM for your specific humanoid robot model
5. Implement bipedal-specific navigation constraints in Nav2

## Resources

- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/released/index.html)
- [ROS Navigation2 Documentation](https://navigation.ros.org/)
- [NVIDIA Developer Portal](https://developer.nvidia.com/)

## Verification Checklist

- [ ] ROS 2 Humble installed and working
- [ ] Isaac Sim installed and launching
- [ ] Isaac ROS packages installed
- [ ] Basic perception pipeline running
- [ ] Navigation2 installed
- [ ] GPU acceleration confirmed
- [ ] Sensor simulation working