---
title: Connecting Isaac ROS VSLAM to ROS 2 Navigation
sidebar_position: 3
---

# Connecting Isaac ROS VSLAM to ROS 2 Navigation

## Overview

This chapter provides comprehensive instructions for integrating Isaac ROS Visual SLAM (VSLAM) systems with ROS 2 navigation stack. The integration enables humanoid robots to use VSLAM-based localization and mapping for autonomous navigation in unknown environments. This integration is critical for achieving robust autonomous navigation without relying on GPS or pre-built maps.

## Understanding the Integration Architecture

### The Navigation Stack Components

The ROS 2 navigation stack consists of several key components that work together:

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   VSLAM         │───▶│  Navigation      │───▶│  Robot          │
│   Localization  │    │  System         │    │  Controller     │
│                 │    │                 │    │                 │
│ • Pose          │    │ • Global        │    │ • Local         │
│   Estimation    │    │   Planner       │    │   Planner       │
│ • Map Building  │    │ • Local Planner │    │ • Trajectory    │
│ • Loop Closure  │    │ • Controller    │    │   Tracker       │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                        │                       │
         ▼                        ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   TF Tree       │    │  Costmaps        │    │  Movement       │
│                 │    │                  │    │  Commands       │
│ • map → odom    │    │ • Global        │    │                 │
│ • odom → base   │    │   Costmap       │    │ • cmd_vel       │
│   _link         │    │ • Local         │    │ • joint         │
└─────────────────┘    │   Costmap       │    │   Commands      │
                       └──────────────────┘    └─────────────────┘
```

### VSLAM to Navigation Data Flow

The integration involves connecting VSLAM outputs to navigation inputs:

1. **Pose Estimation**: VSLAM provides robot pose estimates
2. **Map Building**: VSLAM generates occupancy grids for navigation
3. **Localization**: VSLAM maintains robot position in the map

## Setting Up the Integration

### Prerequisites

Before integrating VSLAM with navigation, ensure you have:

1. **Isaac ROS VSLAM Package** installed and tested
2. **ROS 2 Navigation Stack** installed (Nav2)
3. **Robot Description** (URDF) with proper sensor configurations
4. **TF Tree** with proper transformations
5. **Basic Navigation** working with alternative localization (e.g., AMCL)

### Installing Required Packages

```bash
# Install Nav2 packages
sudo apt update
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-controller
sudo apt install ros-humble-nav2-planners
sudo apt install ros-humble-nav2-behaviors

# Install Isaac ROS VSLAM packages
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-isaac-ros-common
```

### Basic Integration Configuration

#### 1. TF Tree Setup

Create proper TF relationships between VSLAM and navigation:

```xml
<!-- tf_setup.launch.py -->
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Static transforms for VSLAM integration
    tf_static_transforms = [
        # Map frame to VSLAM world frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_vslam_world',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'vslam_world']
        ),

        # VSLAM camera frame to robot base
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base',
            arguments=['0.1', '0.0', '1.5', '0', '0', '0', 'base_link', 'camera_link']
        )
    ]

    return LaunchDescription(tf_static_transforms)
```

#### 2. VSLAM Configuration

Configure VSLAM to output compatible data for navigation:

```yaml
# vslam_for_navigation.yaml
/**:
  ros__parameters:
    # VSLAM node parameters
    use_sim_time: false
    publish_pose: true
    publish_map: true

    # Output topics for navigation
    pose_topic: "/vslam/pose"
    map_topic: "/vslam/occupancy_grid"
    pointcloud_topic: "/vslam/pointcloud"

    # Processing parameters
    enable_loop_closure: true
    enable_bundle_adjustment: true
    max_keyframes: 20
    min_triangulation_angle: 5.0  # degrees
    min_feature_parallax: 2.0     # pixels

    # Hardware acceleration
    use_gpu: true
    gpu_id: 0
    tensorrt_precision: "fp16"

    # Performance settings
    target_fps: 10
    max_processing_time: 0.1  # seconds
```

#### 3. Navigation Configuration for VSLAM

Modify the navigation configuration to work with VSLAM:

```yaml
# vslam_navigation.yaml
amcl:
  ros__parameters:
    # Disable AMCL since using VSLAM for localization
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    likelihood_max_dist: 2.0
    set_initial_pose: false  # VSLAM provides initial pose
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
    use_initial_pose: false
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: "navigate_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_consistent_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # Controller parameters
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      rotation_shim:
        plugin: "nav2_controller::SimpleProgressChecker"
        min_rotational_vel: 0.4
        max_rotational_vel: 1.0
        rotational_acc_lim: 3.2

      # Pure pursuit parameters
      nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController:
        plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        desired_linear_vel: 0.5
        lookahead_dist: 0.6
        min_lookahead_dist: 0.3
        max_lookahead_dist: 0.9
        lookahead_time: 1.5
        rotate_to_heading_angular_vel: 1.8
        max_angular_accel: 3.2
        goal_dist_tol: 0.25
        use_velocity_scaled_lookahead_dist: false
        min_approach_linear_velocity: 0.05
        approach_velocity_scaling_dist: 0.6
        use_approach_velocity_scaling: true
        use_regulated_linear_velocity_scaling: true
        use_cost_regulated_linear_velocity_scaling: true
        regulated_linear_scaling_min_radius: 0.9
        regulated_linear_scaling_min_speed: 0.25
        use_rotate_to_heading: false
        rotate_to_heading_min_angle: 0.785
        max_angular_vel: 1.0
        min_angular_vel: 0.5
        angular_acc_lim: 3.2

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "map"  # VSLAM provides map frame
      robot_base_frame: "base_link"
      use_sim_time: false
      rolling_window: true
      width: 30
      height: 30
      resolution: 0.05  # Match VSLAM resolution
      origin_x: -15.0
      origin_y: -15.0
      footprint: "[ [0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3] ]"
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        voxel_size: 0.05
        observation_sources: pointcloud_marking
        pointcloud_marking:
          topic: "/vslam/pointcloud"  # Use VSLAM point cloud
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          obstacle_range: 2.5
          raytrace_range: 3.0
          clearing: true
          marking: true
          data_type: "PointCloud2"
          queue_size: 10
          transform_tolerance: 0.2
          observation_persistence: 0.0
          max_z_voxels: 10
          unknown_threshold: 15
          mark_threshold: 0
          update_frequence: 5.0
          publish_voxel_map: false
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: "map"  # VSLAM provides map frame
      robot_base_frame: "base_link"
      use_sim_time: false
      footprint: "[ [0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3] ]"
      resolution: 0.05  # Match VSLAM resolution
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: pointcloud_obstacles
        pointcloud_obstacles:
          topic: "/vslam/pointcloud"  # Use VSLAM point cloud
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          obstacle_range: 2.5
          raytrace_range: 3.0
          clearing: true
          marking: true
          data_type: "PointCloud2"
          queue_size: 10
          transform_tolerance: 0.2
          observation_persistence: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: true

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

## Launch Files for Integration

### 1. VSLAM and Navigation Integration Launch

Create a launch file that brings up both systems:

```python
# vslam_navigation_integration.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('your_robot_bringup'),
                                   'config', 'vslam_navigation.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    # VSLAM container
    vslam_container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # VSLAM visual odometry node
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam_node',
                parameters=[
                    os.path.join(get_package_share_directory('your_robot_bringup'),
                                 'config', 'vslam_config.yaml')
                ],
                remappings=[
                    ('/visual_slam/image0', '/front_camera/image_rect_color'),
                    ('/visual_slam/camera_info0', '/front_camera/camera_info'),
                    ('/visual_slam/pose', '/vslam/pose'),
                    ('/visual_slam/map', '/vslam/map')
                ]
            )
        ],
        output='screen'
    )

    # Navigation container (using composition if specified)
    start_composed_container = ComposableNodeContainer(
        condition=IfCondition(use_composition),
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[params_file]),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[params_file]),
            ComposableNode(
                package='nav2_recoveries',
                plugin='nav2_recoveries::RecoveryServer',
                name='recoveries_server',
                parameters=[params_file]),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[params_file]),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[params_file]),
        ],
        output='screen',
    )

    # Alternative: Non-composed launch of navigation
    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': use_composition,
            'container_name': container_name
        }.items(),
    )

    # TF broadcaster to connect VSLAM pose to navigation
    tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='vslam_to_map_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'vslam_world'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Pose converter node to convert VSLAM pose to navigation format
    pose_converter = Node(
        package='your_robot_bringup',
        executable='vslam_pose_converter',
        name='vslam_pose_converter',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('vslam_pose', '/vslam/pose'),
            ('nav_pose', '/amcl_pose'),  # Route VSLAM pose to navigation
            ('initial_pose', '/initialpose'),
            ('goal_pose', '/goal_pose')
        ]
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)

    # Add VSLAM container
    ld.add_action(vslam_container)

    # Add pose converter
    ld.add_action(pose_converter)

    # Add TF broadcaster
    ld.add_action(tf_broadcaster)

    # Add navigation (either composed or standard)
    ld.add_action(start_composed_container)
    ld.add_action(start_navigation)

    return ld
```

### 2. Custom Pose Converter Node

Create a custom node to convert VSLAM poses to navigation format:

```python
# vslam_pose_converter.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf2_ros
import tf2_geometry_msgs
from builtin_interfaces.msg import Time
import numpy as np

class VSLAMPoseConverter(Node):
    def __init__(self):
        super().__init__('vslam_pose_converter')

        # Create subscribers
        self.vslam_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'vslam_pose',
            self.vslam_pose_callback,
            10
        )

        # Create publishers
        self.nav_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'nav_pose',
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            'vslam_odom',
            10
        )

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Store last pose for TF broadcasting
        self.last_pose = None

        # Timer for TF publishing
        self.timer = self.create_timer(0.1, self.publish_tf)  # 10 Hz

        self.get_logger().info('VSLAM Pose Converter Node Started')

    def vslam_pose_callback(self, msg):
        """Convert VSLAM pose to navigation format"""
        try:
            # Create navigation pose message
            nav_pose = PoseWithCovarianceStamped()
            nav_pose.header = msg.header
            nav_pose.header.frame_id = 'map'  # Navigation expects map frame
            nav_pose.pose = msg.pose

            # Publish converted pose
            self.nav_pose_pub.publish(nav_pose)

            # Create and publish odometry
            odom_msg = Odometry()
            odom_msg.header = msg.header
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose = msg.pose

            # Publish odometry
            self.odom_pub.publish(odom_msg)

            # Store for TF broadcasting
            self.last_pose = msg

        except Exception as e:
            self.get_logger().error(f'Error converting VSLAM pose: {e}')

    def publish_tf(self):
        """Publish TF transforms for navigation"""
        if self.last_pose is not None:
            try:
                # Create transform from map to base_link
                t = TransformStamped()

                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'map'
                t.child_frame_id = 'base_link'

                # Set translation
                t.transform.translation.x = self.last_pose.pose.pose.position.x
                t.transform.translation.y = self.last_pose.pose.pose.position.y
                t.transform.translation.z = self.last_pose.pose.pose.position.z

                # Set rotation
                t.transform.rotation = self.last_pose.pose.pose.orientation

                # Send transform
                self.tf_broadcaster.sendTransform(t)

            except Exception as e:
                self.get_logger().error(f'Error publishing TF: {e}')


def main(args=None):
    rclpy.init(args=args)

    converter = VSLAMPoseConverter()

    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Advanced Integration Techniques

### 1. Multi-Sensor Fusion with VSLAM

For improved robustness, combine VSLAM with other sensors:

```yaml
# multi_sensor_fusion.yaml
sensor_fusion:
  ros__parameters:
    # Input sources
    vslam_pose_topic: "/vslam/pose"
    imu_topic: "/imu/data"
    wheel_odom_topic: "/odom"

    # Output topic
    fused_pose_topic: "/fused_pose"

    # Fusion parameters
    vslam_weight: 0.6
    imu_weight: 0.3
    wheel_odom_weight: 0.1

    # Covariance parameters
    vslam_position_variance: 0.1
    vslam_orientation_variance: 0.05
    imu_orientation_variance: 0.01
    wheel_odom_variance: 0.05

    # Update frequencies
    vslam_update_rate: 10.0  # Hz
    imu_update_rate: 100.0   # Hz
    wheel_odom_rate: 50.0    # Hz
```

### 2. Map Conversion for Navigation

Convert VSLAM maps to navigation-compatible formats:

```python
# map_converter.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial import KDTree
import struct

class VSLAMMapConverter(Node):
    def __init__(self):
        super().__init__('vslam_map_converter')

        # Subscriber for VSLAM point cloud
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/vslam/pointcloud',
            self.pointcloud_callback,
            10
        )

        # Publisher for navigation occupancy grid
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/vslam/occupancy_grid',
            10
        )

        # Map parameters
        self.map_resolution = 0.05  # meters per cell
        self.map_width = 200  # cells (10m x 10m map)
        self.map_height = 200
        self.map_origin_x = -10.0  # meters
        self.map_origin_y = -10.0

        # Initialize empty map
        self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)

        self.get_logger().info('VSLAM Map Converter Initialized')

    def pointcloud_callback(self, msg):
        """Convert point cloud to occupancy grid"""
        try:
            # Parse point cloud data
            points = self.parse_pointcloud(msg)

            # Create new occupancy grid
            new_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)

            # Populate grid based on point cloud
            for point in points:
                x, y, z = point

                # Convert world coordinates to grid indices
                grid_x = int((x - self.map_origin_x) / self.map_resolution)
                grid_y = int((y - self.map_origin_y) / self.map_resolution)

                # Check bounds
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    # Mark as occupied (100) if point exists
                    new_grid[grid_y, grid_x] = 100

            # Apply smoothing/filtering to create navigation-compatible map
            smoothed_grid = self.smooth_occupancy_grid(new_grid)

            # Publish occupancy grid
            self.publish_occupancy_grid(smoothed_grid)

        except Exception as e:
            self.get_logger().error(f'Error converting map: {e}')

    def parse_pointcloud(self, pc2_msg):
        """Parse PointCloud2 message to numpy array"""
        points = []

        # Get field offsets
        offset_x = -1
        offset_y = -1
        offset_z = -1

        for field in pc2_msg.fields:
            if field.name == 'x':
                offset_x = field.offset
            elif field.name == 'y':
                offset_y = field.offset
            elif field.name == 'z':
                offset_z = field.offset

        # Parse data
        fmt = '<fff'  # 3 floats (x,y,z)
        step = 12  # Size of 3 floats

        for i in range(0, len(pc2_msg.data), step):
            x, y, z = struct.unpack(fmt, pc2_msg.data[i:i+step])
            points.append([x, y, z])

        return np.array(points)

    def smooth_occupancy_grid(self, grid):
        """Apply smoothing to create navigation-compatible map"""
        # Create a copy of the grid
        smoothed = grid.copy()

        # Apply morphological operations to clean up the map
        # This helps fill in small gaps and smooth edges
        from scipy import ndimage

        # Create structuring element for morphological operations
        structure = np.ones((3, 3), dtype=bool)

        # Apply closing operation to fill small holes
        closed = ndimage.binary_closing(grid > 50, structure=structure)

        # Apply opening operation to remove small obstacles
        opened = ndimage.binary_opening(closed, structure=structure)

        # Convert back to occupancy grid format
        smoothed = np.where(opened, 100, np.where(grid == -1, -1, 0))

        return smoothed.astype(np.int8)

    def publish_occupancy_grid(self, grid_data):
        """Publish occupancy grid message"""
        msg = OccupancyGrid()

        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Info
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Data (flatten the 2D array)
        msg.data = grid_data.flatten().tolist()

        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    converter = VSLAMMapConverter()

    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Testing and Validation

### 1. Basic Integration Test

Test the basic integration with these commands:

```bash
# Terminal 1: Launch VSLAM and navigation
ros2 launch your_robot_bringup vslam_navigation_integration.launch.py

# Terminal 2: Send a navigation goal
ros2 run nav2_msgs action_client.py NavigateToPose
# Or use RViz2 to send goals

# Terminal 3: Monitor the system
ros2 topic echo /vslam/pose
ros2 topic echo /amcl_pose
ros2 run tf2_tools view_frames
```

### 2. Performance Monitoring

Monitor system performance during operation:

```bash
# Monitor CPU and memory usage
htop

# Monitor GPU usage
nvidia-smi -l 1

# Monitor ROS 2 topics
ros2 topic hz /vslam/pose
ros2 topic hz /amcl_pose

# Monitor TF tree
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_monitor
```

### 3. Validation Checks

Perform these validation checks:

1. **TF Tree Validation**: Verify all frames are properly connected
2. **Pose Consistency**: Check that VSLAM and navigation poses align
3. **Map Quality**: Ensure VSLAM-generated maps are suitable for navigation
4. **Localization Stability**: Verify consistent localization performance
5. **Navigation Performance**: Test successful path planning and execution

## Troubleshooting Common Issues

### 1. TF Frame Issues

**Problem**: Navigation doesn't receive proper transforms from VSLAM
**Solution**:
- Check that all required TF frames exist
- Verify static transforms are published
- Use `ros2 run tf2_tools view_frames` to inspect the tree

### 2. Timing Issues

**Problem**: Navigation receives outdated poses
**Solution**:
- Check transform tolerance parameters
- Ensure proper timestamp synchronization
- Verify QoS settings match between nodes

### 3. Map Resolution Mismatch

**Problem**: VSLAM map resolution doesn't match navigation expectations
**Solution**:
- Adjust VSLAM map resolution to match navigation config
- Use map converter nodes to handle resolution differences
- Ensure consistent coordinate frames

## Best Practices

### 1. System Design

- Use composable nodes for better performance
- Implement proper error handling and recovery
- Monitor system health continuously
- Use appropriate QoS settings for real-time performance

### 2. Performance Optimization

- Optimize VSLAM for target frame rate (typically 10-20 Hz for navigation)
- Use GPU acceleration where possible
- Implement proper resource management
- Apply adaptive processing based on system load

### 3. Safety Considerations

- Implement fallback localization if VSLAM fails
- Monitor tracking quality and alert on degradation
- Use multi-sensor fusion for robustness
- Implement safe stop procedures

## Conclusion

Connecting Isaac ROS VSLAM to ROS 2 navigation creates a powerful autonomous navigation system for humanoid robots. The integration requires careful attention to TF trees, message formats, and timing considerations. With proper configuration and validation, this system enables robots to navigate in unknown environments without relying on GPS or pre-built maps.

The key to successful integration lies in understanding the data flow between systems, ensuring proper coordinate frame relationships, and implementing appropriate performance monitoring and safety measures.