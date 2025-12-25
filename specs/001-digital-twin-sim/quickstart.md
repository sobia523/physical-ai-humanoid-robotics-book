# Quickstart Guide for Module 2: The Digital Twin (Gazebo & Unity)

## Overview
This guide provides a quick setup for the digital twin simulation environment covering Gazebo physics simulation, Unity rendering, and ROS 2 integration.

## Prerequisites
- Ubuntu 22.04 LTS or Windows 10/11 with WSL2
- ROS 2 Humble Hawksbill installed
- Unity Hub with Unity 2022.3 LTS
- Gazebo Garden or Harmonic
- Git and basic development tools

## Environment Setup

### 1. ROS 2 Workspace Setup
```bash
# Create workspace
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build
source install/setup.bash
```

### 2. Gazebo Installation
```bash
# Install Gazebo Garden
sudo apt install ros-humble-gazebo-*
sudo apt install gazebo

# Verify installation
gazebo --version
```

### 3. Unity Setup
1. Install Unity Hub from unity.com
2. Install Unity 2022.3 LTS through Unity Hub
3. Install required packages:
   - Unity Robotics Package (URP)
   - ROS-TCP-Connector
   - HDRP (High Definition Render Pipeline)

## Basic Simulation Structure

### 1. Create a Simple Robot Model
Create a basic URDF file in `~/digital_twin_ws/src/robot_description/urdf/simple_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
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

  <joint name="base_to_lidar" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.0 0.0 0.2" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### 2. Gazebo Launch File
Create `~/digital_twin_ws/src/robot_description/launch/robot_sim.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot state publisher
    robot_description_path = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'simple_robot.urdf'
    )

    with open(robot_description_path, 'r') as infp:
        robot_description = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_state_publisher)

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch',
            '/gazebo.launch.py'
        ])
    )
    ld.add_action(gazebo)

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'simple_robot'],
        output='screen'
    )
    ld.add_action(spawn_entity)

    return ld
```

### 3. Run the Simulation
```bash
# Build the workspace
cd ~/digital_twin_ws
colcon build
source install/setup.bash

# Launch the simulation
ros2 launch robot_description robot_sim.launch.py
```

## Unity Integration Setup

### 1. Create ROS-TCP-Connector Connection
In Unity, create a new script `ROSConnection.cs`:

```csharp
using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class ROSConnection : MonoBehaviour
{
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    void Start()
    {
        ROSConnection.instance = ROSConnection.CreateFromGameObject(this);
        ROSConnection.instance.Initialize(rosIPAddress, rosPort);
    }
}
```

### 2. Create a Simple Sensor Publisher
Create a ROS 2 node to publish sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class SimpleLidar(Node):
    def __init__(self):
        super().__init__('simple_lidar')
        self.publisher = self.create_publisher(LaserScan, '/sensor/lidar_scan', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 0.01
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 30.0

        # Generate sample ranges
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = [float(i % 10 + 1) for i in range(num_readings)]
        msg.intensities = [100.0] * num_readings

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    simple_lidar = SimpleLidar()
    rclpy.spin(simple_lidar)
    simple_lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Complete Simulation

1. Start ROS 2 daemon:
```bash
# Terminal 1
source ~/digital_twin_ws/install/setup.bash
ros2 daemon start
```

2. Launch Gazebo simulation:
```bash
# Terminal 2
source ~/digital_twin_ws/install/setup.bash
ros2 launch robot_description robot_sim.launch.py
```

3. Run the sensor publisher:
```bash
# Terminal 3
source ~/digital_twin_ws/install/setup.bash
python3 simple_lidar.py
```

4. Start Unity project with ROS-TCP-Connector configured

## Validation Steps

1. Verify Gazebo shows the robot model
2. Check ROS 2 topics are being published:
   ```bash
   ros2 topic list
   ros2 topic echo /sensor/lidar_scan
   ```
3. Confirm Unity connects to ROS network
4. Validate sensor data flows between systems

## Troubleshooting

- If Gazebo doesn't start: Check if X11 forwarding is enabled on WSL2
- If ROS topics don't appear: Verify ROS_DOMAIN_ID is consistent
- If Unity can't connect: Check firewall settings and ROS-TCP-Connector configuration