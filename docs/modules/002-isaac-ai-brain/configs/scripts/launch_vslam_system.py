#!/usr/bin/env python3
"""
VSLAM System Launch Script

This script provides a comprehensive launch configuration for the Isaac ROS VSLAM system.
It includes all necessary components for a complete VSLAM pipeline with mapping and navigation integration.

Author: Isaac ROS Developer
License: Apache 2.0
"""

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import yaml


def generate_launch_description():
    """
    Generate launch description for the complete VSLAM system.

    This includes:
    - Camera drivers and preprocessing
    - VSLAM pipeline
    - Mapping utilities
    - Navigation integration
    - Visualization tools
    """

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    camera_namespace = LaunchConfiguration('camera_namespace')
    enable_visualization = LaunchConfiguration('enable_visualization')
    enable_gpu_processing = LaunchConfiguration('enable_gpu_processing')

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
                                   'config', 'vslam_system.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='vslam_container',
        description='the name of container that nodes will load in if use composition')

    declare_camera_namespace_cmd = DeclareLaunchArgument(
        'camera_namespace',
        default_value='/front_camera',
        description='Namespace for camera topics')

    declare_enable_visualization_cmd = DeclareLaunchArgument(
        'enable_visualization',
        default_value='True',
        description='Enable visualization nodes')

    declare_enable_gpu_processing_cmd = DeclareLaunchArgument(
        'enable_gpu_processing',
        default_value='True',
        description='Enable GPU processing for VSLAM')

    # Create VSLAM container with all necessary components
    vslam_container = ComposableNodeContainer(
        condition=IfCondition(use_composition),
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # 1. Camera preprocessing node
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectificationNode',
                name='camera_rectification',
                parameters=[{
                    'input_width': 1280,
                    'input_height': 720,
                    'output_width': 1280,
                    'output_height': 720,
                    'use_gpu': enable_gpu_processing,
                    'gpu_id': 0,
                    'camera_info_url': 'file:///tmp/camera_calibration.yaml'
                }],
                remappings=[
                    ('image_raw', [camera_namespace, '/image_raw']),
                    ('camera_info', [camera_namespace, '/camera_info']),
                    ('image_rect', [camera_namespace, '/image_rect'])
                ]
            ),

            # 2. Feature detection node
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::FeatureTrackerNode',
                name='feature_tracker',
                parameters=[{
                    'use_gpu': enable_gpu_processing,
                    'gpu_id': 0,
                    'max_features': 1000,
                    'min_eigenvalue_threshold': 0.001,
                    'min_displacement_threshold': 0.5,
                    'track_lifetime_thresh': 5,
                    'loss_detection_enabled': True
                }],
                remappings=[
                    ('/visual_slam/image0', [camera_namespace, '/image_rect']),
                    ('/visual_slam/camera_info0', [camera_namespace, '/camera_info']),
                    ('/visual_slam/tracked/features', '/vslam/tracked_features')
                ]
            ),

            # 3. Visual SLAM node
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[
                    os.path.join(get_package_share_directory('your_robot_bringup'),
                                 'config', 'vslam_config.yaml')
                ],
                remappings=[
                    ('/visual_slam/image0', [camera_namespace, '/image_rect']),
                    ('/visual_slam/camera_info0', [camera_namespace, '/camera_info']),
                    ('/visual_slam/pose', '/vslam/pose'),
                    ('/visual_slam/imu', '/imu/data'),
                    ('/visual_slam/feature0', '/vslam/tracked_features')
                ]
            ),

            # 4. Loop closure detection
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::LoopClosureNode',
                name='loop_closure',
                parameters=[{
                    'enable_loop_closure': True,
                    'min_loop_closure_score': 0.8,
                    'max_num_images_in_database': 1000,
                    'use_gpu': enable_gpu_processing
                }],
                remappings=[
                    ('/visual_slam/loop_closure', '/vslam/loop_closure_detected'),
                    ('/visual_slam/keyframes', '/vslam/keyframes')
                ]
            )
        ],
        output='screen',
    )

    # Create mapping container
    mapping_container = ComposableNodeContainer(
        condition=IfCondition(use_composition),
        name='mapping_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # 1. Occupancy grid mapping
            ComposableNode(
                package='costmap_2d',
                plugin='costmap_2d::InflationLayer',
                name='inflation_layer',
                parameters=[{
                    'inflation_radius': 0.55,
                    'cost_scaling_factor': 3.0
                }]
            ),

            # 2. Point cloud to laser scan (for navigation)
            ComposableNode(
                package='pointcloud_to_laserscan',
                plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                name='pointcloud_to_laserscan',
                parameters=[{
                    'target_frame': 'base_link',
                    'transform_tolerance': 0.2,
                    'min_height': 0.0,
                    'max_height': 1.0,
                    'angle_min': -3.14,
                    'angle_max': 3.14,
                    'angle_increment': 0.0087,
                    'scan_time': 0.3333,
                    'range_min': 0.45,
                    'range_max': 4.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0
                }],
                remappings=[
                    ('cloud_in', '/vslam/pointcloud'),
                    ('scan', '/vslam/scan')
                ]
            )
        ],
        output='screen',
    )

    # Non-composed launch of navigation stack
    from launch.actions import GroupAction
    from launch_ros.actions import PushRosNamespace

    navigation_group = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            PushRosNamespace(
                condition=IfCondition(LaunchConfiguration('use_namespace')),
                namespace=LaunchConfiguration('namespace')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file
                }.items(),
            ),
        ]
    )

    # RViz2 visualization node
    rviz_config_file = os.path.join(
        get_package_share_directory('your_robot_bringup'),
        'rviz',
        'vslam_navigation.rviz'
    )

    rviz_node = Node(
        condition=IfCondition(PythonExpression(['"', enable_visualization, '" == "True"'])),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # TF broadcasters for proper coordinate frame relationships
    tf_broadcasters = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_broadcaster',
            arguments=['0.1', '0.0', '1.5', '0', '0', '0', 'base_link', 'camera_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ]

    # Create launch description
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_camera_namespace_cmd)
    ld.add_action(declare_enable_visualization_cmd)
    ld.add_action(declare_enable_gpu_processing_cmd)

    # Add containers
    ld.add_action(vslam_container)
    ld.add_action(mapping_container)

    # Add navigation group
    ld.add_action(navigation_group)

    # Add visualization
    ld.add_action(rviz_node)

    # Add TF broadcasters
    for broadcaster in tf_broadcasters:
        ld.add_action(broadcaster)

    # Add a timer to ensure all nodes start properly
    startup_timer = TimerAction(
        period=5.0,
        actions=[
            # Additional startup actions can be added here
        ]
    )
    ld.add_action(startup_timer)

    return ld


# Additional utility functions for VSLAM system configuration
def create_vslam_config_file(config_path: str = '/tmp/vslam_config.yaml'):
    """
    Create a default VSLAM configuration file with recommended parameters.

    Args:
        config_path: Path where the config file should be created
    """
    config = {
        '/**': {
            'ros__parameters': {
                # VSLAM node parameters
                'use_sim_time': False,
                'publish_pose': True,
                'publish_map': True,

                # Processing parameters
                'enable_loop_closure': True,
                'enable_bundle_adjustment': True,
                'max_keyframes': 20,
                'min_triangulation_angle': 5.0,  # degrees
                'min_feature_parallax': 2.0,     # pixels

                # Hardware acceleration
                'use_gpu': True,
                'gpu_id': 0,
                'tensorrt_precision': 'fp16',

                # Performance settings
                'target_fps': 10,
                'max_processing_time': 0.1,  # seconds

                # Feature detection
                'max_features': 1000,
                'min_eigenvalue_threshold': 0.001,
                'min_displacement_threshold': 0.5,

                # Tracking
                'track_lifetime_thresh': 5,
                'loss_detection_enabled': True,

                # Loop closure
                'loop_closure_min_score': 0.8,
                'max_num_images_in_database': 1000,

                # Bundle adjustment
                'enable_local_ba': True,
                'enable_global_ba': True,
                'ba_max_iterations': 100,
                'ba_convergence_threshold': 1e-6
            }
        }
    }

    with open(config_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)

    print(f'VSLAM configuration file created at: {config_path}')


def create_rviz_config_file(config_path: str = '/tmp/vslam_navigation.rviz'):
    """
    Create a default RViz configuration file for VSLAM visualization.

    Args:
        config_path: Path where the config file should be created
    """
    rviz_config = """Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /Grid1
        - /TF1
        - /Odometry1
        - /Map1
        - /PointCloud21
        - /PoseWithCovariance1
      Splitter Ratio: 0.5
    Tree Height: 775
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 50
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        {}
      Update Interval: 0
      Value: true
    - Angle Tolerance: 0.10000000149011612
      Class: rviz_default_plugins/Odometry
      Covariance:
        Orientation:
          Alpha: 0.5
          Color: 255; 25; 0
          Color Style: Unique
          Frame: Local
          Offset: 1
          Scale: 1
          Value: true
        Position:
          Alpha: 0.30000001192092896
          Color: 255; 255; 127
          Scale: 1
          Value: true
        Value: true
      Enabled: true
      Keep: 100
      Name: Odometry
      Position Tolerance: 0.10000000149011612
      Shape:
        Alpha: 1
        Color: 255; 25; 0
        Head Length: 0.30000001192092896
        Head Radius: 0.10000000149011612
        Shaft Radius: 0.05000000074505806
        Value: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /vslam/odometry
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Transient Local
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /vslam/occupancy_grid
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /vslam/occupancy_grid_updates
      Use Timestamp: false
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: RGB8
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: PointCloud2
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /vslam/pointcloud
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Class: rviz_default_plugins/PoseWithCovariance
      Color: 255; 25; 0
      Covariance:
        Orientation:
          Alpha: 0.5
          Color: 255; 25; 0
          Color Style: Unique
          Frame: Local
          Offset: 1
          Scale: 1
          Value: true
        Position:
          Alpha: 0.30000001192092896
          Color: 255; 255; 127
          Scale: 1
          Value: true
        Value: true
      Enabled: true
      Name: PoseWithCovariance
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /vslam/pose
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.5
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1025
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd0000000400000000000001560000039bfc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d0000039b000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f0000039bfc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d0000039b000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d00650100000000000004500000000000000000000004d80000039b00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Width: 1853
  X: 67
  Y: 27
"""

    with open(config_path, 'w') as f:
        f.write(rviz_config)

    print(f'RViz configuration file created at: {config_path}')


def main():
    """Main function to create supporting configuration files"""
    print("Creating VSLAM system configuration files...")

    # Create default config files if they don't exist
    config_dir = '/tmp/'

    vslam_config_path = os.path.join(config_dir, 'vslam_config.yaml')
    if not os.path.exists(vslam_config_path):
        create_vslam_config_file(vslam_config_path)

    rviz_config_path = os.path.join(config_dir, 'vslam_navigation.rviz')
    if not os.path.exists(rviz_config_path):
        create_rviz_config_file(rviz_config_path)

    print("VSLAM system launch script ready!")
    print("To launch the system, run:")
    print("ros2 launch launch_vslam_system.py")


if __name__ == '__main__':
    main()