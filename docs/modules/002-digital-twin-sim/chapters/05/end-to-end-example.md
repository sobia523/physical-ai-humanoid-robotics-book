# End-to-End Example: Humanoid Robot Navigation in Simulated Environment

## Overview

This chapter provides a comprehensive end-to-end example of a humanoid robot navigating a simulated environment using integrated sensor and simulation components. The example demonstrates the practical application of all the concepts covered in previous chapters, including physics simulation, rendering, sensor simulation, and ROS 2 integration.

## Scenario Description

Our scenario involves a humanoid robot that must navigate through a structured indoor environment containing obstacles, dynamic elements, and navigation goals. The robot is equipped with:
- A 2D LiDAR sensor for obstacle detection
- A depth camera for environmental mapping
- An IMU for orientation and motion tracking
- Wheel-based or bipedal locomotion system

## Environment Setup

### World Definition

The simulated environment consists of:

```
┌─────────────────────────────────────────┐
│  Start  │    │              │  Goal    │
│    ●    │    │              │    ■     │
│         │    │    Table     │          │
│         │    │    [===]     │          │
│         │    │              │          │
│  ┌──────┼────┼──────────────┼──────────┤
│  │      │    │              │          │
│  │ Obstacle │              │          │
│  │  [||]   │    Chair      │          │
│  │         │    [###]      │          │
│  └─────────┼───────────────┼──────────┤
│            │               │          │
│   Door     │    Clear      │   Door   │
│   [---]    │   Space       │  [---]   │
│            │               │          │
└─────────────────────────────────────────┘
```

### Gazebo World Configuration

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_navigation">
    <!-- Include the default outdoor environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add static obstacles -->
    <model name="table">
      <pose>5 0 0 0 0 0</pose>
      <link name="table_base">
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 0.8 0.8</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="chair">
      <pose>2 -2 0 0 0 0</pose>
      <link name="chair_base">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.6 0.6 0.8</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.6 0.6 0.8</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Add a dynamic object that moves around -->
    <model name="moving_object">
      <pose>-3 2 0 0 0 0</pose>
      <link name="object_link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.5</length>
            </cylinder>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.5</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
      <!-- Add a simple plugin to make the object move -->
      <plugin name="object_mover" filename="libgazebo_ros_pubslish_odometry.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>1.0</updateRate>
      </plugin>
    </model>

    <!-- Define navigation goals -->
    <state world_name="humanoid_navigation">
      <model name="navigation_goal">
        <pose>8 3 0 0 0 0</pose>
        <link name="goal_visual">
          <visual name="goal_visual">
            <geometry>
              <box>
                <size>0.3 0.3 0.3</size>
              </box>
            </geometry>
            <material>
              <ambient>1 0 0 1</ambient>
              <diffuse>1 0 0 1</diffuse>
            </material>
          </visual>
        </link>
      </model>
    </state>
  </world>
</sdf>
```

## Robot Configuration

### URDF Model with Sensors

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.3" length="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.3" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="50"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="20"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head with sensors -->
  <joint name="head_joint" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- LiDAR sensor on head -->
  <joint name="lidar_joint" type="fixed">
    <parent link="head"/>
    <child link="lidar_link"/>
    <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
  </link>

  <!-- IMU sensor -->
  <joint name="imu_joint" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
    </visual>
  </link>

  <!-- Gazebo plugins -->
  <gazebo reference="base_link">
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <material>Gazebo/Blue</material>
  </gazebo>

  <!-- LiDAR plugin -->
  <gazebo reference="lidar_link">
    <sensor type="ray" name="humanoid_lidar">
      <pose>0 0 0 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>~/out:=/humanoid/lidar/scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU plugin -->
  <gazebo reference="imu_link">
    <sensor type="imu" name="humanoid_imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-05</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-05</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-05</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>~/out:=/humanoid/imu/data</remapping>
        </ros>
        <frame_name>imu_link</frame_name>
        <topic_name>/humanoid/imu/data</topic_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Motor controller for locomotion -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid</robotNamespace>
    </plugin>
  </gazebo>
</robot>
```

## Navigation Algorithm Implementation

### Path Planning and Obstacle Avoidance Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import math
from scipy.spatial import distance

class HumanoidNavigation(Node):
    def __init__(self):
        super().__init__('humanoid_navigation')

        # Robot state
        self.current_pose = np.array([0.0, 0.0])
        self.current_orientation = 0.0
        self.target_pose = np.array([8.0, 3.0])  # Goal position
        self.lidar_data = None

        # Navigation parameters
        self.linear_speed = 0.5
        self.angular_speed = 0.5
        self.min_obstacle_distance = 0.5  # meters
        self.arrival_threshold = 0.3      # meters

        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/humanoid/lidar/scan',
            self.lidar_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/humanoid/odom',
            self.odom_callback,
            10
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/humanoid/cmd_vel', 10)

        # Timer for navigation control
        self.nav_timer = self.create_timer(0.1, self.navigation_control)

        self.get_logger().info('Humanoid Navigation node initialized')

    def lidar_callback(self, msg):
        """Process LiDAR data for obstacle detection."""
        self.lidar_data = {
            'ranges': np.array(msg.ranges),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }

    def odom_callback(self, msg):
        """Update robot's current pose from odometry."""
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y

        # Extract orientation from quaternion
        q = msg.pose.pose.orientation
        self.current_orientation = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def detect_obstacles(self):
        """Detect obstacles from LiDAR data."""
        if self.lidar_data is None:
            return False, 0.0

        # Filter valid range values
        valid_ranges = self.lidar_data['ranges'][
            (self.lidar_data['ranges'] > 0) &
            (self.lidar_data['ranges'] < 10.0)  # Only consider close obstacles
        ]

        if len(valid_ranges) == 0:
            return False, float('inf')

        min_distance = np.min(valid_ranges)
        return min_distance < self.min_obstacle_distance, min_distance

    def calculate_desired_direction(self):
        """Calculate desired direction to target."""
        direction_vector = self.target_pose - self.current_pose
        distance_to_target = np.linalg.norm(direction_vector)

        if distance_to_target < self.arrival_threshold:
            return np.array([0.0, 0.0]), True  # Reached target

        desired_direction = direction_vector / distance_to_target
        return desired_direction, False

    def obstacle_avoidance_direction(self):
        """Calculate direction to avoid obstacles."""
        if self.lidar_data is None:
            return np.array([1.0, 0.0])  # Default forward

        # Find the clearest direction (longest range reading)
        ranges = self.lidar_data['ranges']
        angle_increment = self.lidar_data['angle_increment']

        # Split ranges into sectors
        num_sectors = 8
        sector_size = len(ranges) // num_sectors
        sector_averages = []

        for i in range(num_sectors):
            start_idx = i * sector_size
            end_idx = min((i + 1) * sector_size, len(ranges))
            sector_range = ranges[start_idx:end_idx]
            valid_sector = sector_range[np.isfinite(sector_range) & (sector_range > 0)]

            if len(valid_sector) > 0:
                avg_range = np.mean(valid_sector)
            else:
                avg_range = float('inf')

            sector_averages.append(avg_range)

        # Find sector with maximum average range (clearest path)
        best_sector = np.argmax(sector_averages)
        best_angle = self.lidar_data['angle_min'] + (best_sector * sector_size + sector_size//2) * angle_increment

        # Convert to 2D direction vector
        avoidance_direction = np.array([
            math.cos(best_angle),
            math.sin(best_angle)
        ])

        return avoidance_direction

    def navigation_control(self):
        """Main navigation control loop."""
        if self.lidar_data is None:
            return

        # Calculate desired direction to target
        desired_direction, target_reached = self.calculate_desired_direction()

        if target_reached:
            self.get_logger().info('Target reached! Stopping robot.')
            self.stop_robot()
            return

        # Check for obstacles
        obstacle_detected, min_distance = self.detect_obstacles()

        cmd_vel = Twist()

        if obstacle_detected:
            # Obstacle avoidance mode
            self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m, avoiding...')

            # Combine desired direction with obstacle avoidance
            avoidance_direction = self.obstacle_avoidance_direction()

            # Weighted combination of goal-seeking and obstacle-avoiding
            combined_direction = 0.3 * desired_direction + 0.7 * avoidance_direction
            combined_direction = combined_direction / np.linalg.norm(combined_direction)

            # Calculate required rotation to face the combined direction
            current_angle = self.current_orientation
            desired_angle = math.atan2(combined_direction[1], combined_direction[0])
            angle_diff = math.atan2(
                math.sin(desired_angle - current_angle),
                math.cos(desired_angle - current_angle)
            )

            # Set velocities
            cmd_vel.linear.x = self.linear_speed * 0.5  # Slower when avoiding obstacles
            cmd_vel.angular.z = self.angular_speed * angle_diff
        else:
            # Normal navigation toward target
            current_angle = self.current_orientation
            desired_angle = math.atan2(desired_direction[1], desired_direction[0])
            angle_diff = math.atan2(
                math.sin(desired_angle - current_angle),
                math.cos(desired_angle - current_angle)
            )

            # Set velocities
            cmd_vel.linear.x = self.linear_speed
            cmd_vel.angular.z = self.angular_speed * angle_diff

        # Ensure velocities are within bounds
        cmd_vel.linear.x = max(0.0, min(cmd_vel.linear.x, self.linear_speed))
        cmd_vel.angular.z = max(-self.angular_speed, min(cmd_vel.angular.z, self.angular_speed))

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

    def stop_robot(self):
        """Stop the robot."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def get_robot_state(self):
        """Get current robot state."""
        obstacle_detected, distance = self.detect_obstacles()
        return {
            'position': self.current_pose,
            'orientation': self.current_orientation,
            'target': self.target_pose,
            'distance_to_target': np.linalg.norm(self.target_pose - self.current_pose),
            'obstacle_detected': obstacle_detected,
            'obstacle_distance': distance
        }

def main(args=None):
    rclpy.init(args=args)
    navigator = HumanoidNavigation()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation interrupted by user')
    finally:
        navigator.stop_robot()
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Configuration

### Complete Launch File

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='humanoid_navigation.world')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch',
            '/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': [get_package_share_directory('your_robot_gazebo'), '/worlds/', world_file],
            'verbose': 'true'
        }.items()
    )

    # Robot state publisher
    robot_description_path = os.path.join(
        get_package_share_directory('your_robot_description'),
        'urdf',
        'humanoid_robot.urdf.xacro'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': open(robot_description_path).read()}
        ]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'  # Slightly above ground
        ],
        output='screen'
    )

    # Joint state publisher (for simulation)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Navigation node
    navigation_node = Node(
        package='your_robot_navigation',
        executable='humanoid_navigation',
        name='humanoid_navigation',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # TF publisher (for coordinate transforms)
    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # Create launch description with proper sequencing
    ld = LaunchDescription()

    # Add Gazebo first
    ld.add_action(gazebo)

    # Add robot state publisher
    ld.add_action(robot_state_publisher)

    # Add joint state publisher
    ld.add_action(joint_state_publisher)

    # Wait for Gazebo to start before spawning robot
    spawn_timer = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )
    ld.add_action(spawn_timer)

    # Wait more before starting navigation
    nav_timer = TimerAction(
        period=10.0,
        actions=[navigation_node]
    )
    ld.add_action(nav_timer)

    # Add TF publisher
    ld.add_action(tf_publisher)

    return ld
```

## Unity Integration (Optional)

### High-Fidelity Visualization Setup

For high-fidelity rendering in Unity, the robot model and environment would be exported using tools like the Unity Robotics Simulation package. The key steps would include:

1. **Model Export**: Export robot URDF to FBX format with proper joint hierarchies
2. **Environment Mapping**: Create Unity equivalent of the Gazebo environment
3. **Sensor Visualization**: Implement visualization for LiDAR point clouds and depth camera data
4. **Synchronization**: Sync Unity visualization with Gazebo simulation in real-time

### Unity Scene Setup

```csharp
using UnityEngine;
using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSLib.Sensor_msgs;

public class HumanoidRobotController : MonoBehaviour
{
    public GameObject robotModel;
    public Transform lidarPointcloudParent;
    public Material lidarPointMaterial;

    private ROSConnection ros;
    private string lidarTopic = "/humanoid/lidar/scan";
    private string cmdVelTopic = "/humanoid/cmd_vel";

    void Start()
    {
        ros = ROSConnection.instance;
        ros.Subscribe<LaserScanMsg>(lidarTopic, OnLidarReceived);
    }

    void OnLidarReceived(LaserScanMsg scan)
    {
        // Update LiDAR visualization in Unity
        UpdateLidarVisualization(scan);
    }

    void UpdateLidarVisualization(LaserScanMsg scan)
    {
        // Clear previous point cloud
        foreach (Transform child in lidarPointcloudParent)
        {
            Destroy(child.gameObject);
        }

        // Create new point cloud based on scan data
        for (int i = 0; i < scan.ranges.Length; i++)
        {
            if (scan.ranges[i] > scan.range_min && scan.ranges[i] < scan.range_max)
            {
                float angle = scan.angle_min + i * scan.angle_increment;
                Vector3 pointPos = new Vector3(
                    scan.ranges[i] * Mathf.Cos(angle),
                    0.1f, // Height offset
                    scan.ranges[i] * Mathf.Sin(angle)
                );

                GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                point.transform.SetParent(lidarPointcloudParent);
                point.transform.localPosition = pointPos;
                point.transform.localScale = Vector3.one * 0.05f;
                point.GetComponent<Renderer>().material = lidarPointMaterial;
                Destroy(point.GetComponent<Collider>()); // Remove collider
            }
        }
    }

    public void SendVelocityCommand(float linear, float angular)
    {
        // Send velocity command to ROS
        var twist = new Unity.Robotics.ROSTCPConnector.ROSLib.Geometry_msgs.TwistMsg();
        twist.linear = new Unity.Robotics.ROSTCPConnector.ROSLib.Geometry_msgs.Vector3Msg(linear, 0, 0);
        twist.angular = new Unity.Robotics.ROSTCPConnector.ROSLib.Geometry_msgs.Vector3Msg(0, 0, angular);

        ros.Send<Unity.Robotics.ROSTCPConnector.ROSLib.Geometry_msgs.TwistMsg>(cmdVelTopic, twist);
    }
}
```

## Running the Complete Example

### Step-by-Step Execution

1. **Environment Setup**
   ```bash
   # Source ROS 2 environment
   source /opt/ros/humble/setup.bash
   source ~/your_robot_ws/install/setup.bash

   # Build workspace
   cd ~/your_robot_ws
   colcon build --packages-select your_robot_description your_robot_gazebo your_robot_navigation
   source install/setup.bash
   ```

2. **Launch Simulation**
   ```bash
   # Launch the complete simulation
   ros2 launch your_robot_bringup humanoid_navigation.launch.py
   ```

3. **Monitor Progress**
   ```bash
   # View sensor data
   ros2 topic echo /humanoid/lidar/scan sensor_msgs/msg/LaserScan

   # Monitor robot state
   ros2 topic echo /humanoid/odom nav_msgs/msg/Odometry

   # Check navigation commands
   ros2 topic echo /humanoid/cmd_vel geometry_msgs/msg/Twist
   ```

4. **Visualization**
   ```bash
   # Launch RViz for visualization
   ros2 run rviz2 rviz2 -d ~/your_robot_description/rviz/navigation.rviz
   ```

## Expected Behavior and Outcomes

The end-to-end example demonstrates:

1. **Physics Simulation**: The humanoid robot moves realistically through the environment
2. **Sensor Integration**: LiDAR and IMU data are properly integrated
3. **Navigation Algorithm**: The robot successfully navigates around obstacles to reach its goal
4. **ROS 2 Communication**: All components communicate effectively through ROS 2 topics
5. **Environment Interaction**: The robot responds appropriately to static and dynamic obstacles

## Performance Analysis

### Key Metrics to Monitor

1. **Navigation Success Rate**: Percentage of successful navigation attempts
2. **Path Efficiency**: Ratio of actual path length to optimal path length
3. **Obstacle Avoidance**: Time spent avoiding obstacles vs. direct navigation
4. **Computational Performance**: CPU and memory usage during simulation
5. **Sensor Data Quality**: Accuracy and reliability of sensor readings

### Troubleshooting Common Issues

- **Robot gets stuck**: Check obstacle detection parameters and navigation thresholds
- **Oscillating behavior**: Adjust control gains and smoothing parameters
- **High CPU usage**: Reduce simulation update rates or simplify sensor models
- **Coordination problems**: Verify coordinate frame alignments and TF transforms

## Conclusion

This end-to-end example demonstrates the complete integration of physics simulation, sensor simulation, and navigation control for humanoid robotics. It showcases how all the individual components covered in previous chapters work together to enable complex robotic behaviors in a simulated environment. Students can use this example as a foundation for developing their own humanoid robot applications and experiments.