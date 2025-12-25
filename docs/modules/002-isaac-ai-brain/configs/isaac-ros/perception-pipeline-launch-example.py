# Isaac ROS Complete Perception Pipeline Launch Example
# Demonstrates how to launch the hardware-accelerated perception pipeline

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    hardware_profile_arg = DeclareLaunchArgument(
        'hardware_profile',
        default_value='desktop_rtx_3080',
        description='Hardware profile to use: jetson_xavier_nx, jetson_agx_orin, or desktop_rtx_3080'
    )

    enable_profiling_arg = DeclareLaunchArgument(
        'enable_profiling',
        default_value='false',
        description='Enable performance profiling'
    )

    # Get launch configuration
    hardware_profile = LaunchConfiguration('hardware_profile')
    enable_profiling = LaunchConfiguration('enable_profiling')

    # Create a composable node container for the perception pipeline
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Multi-threaded container for better performance
        parameters=[
            # Global parameters for the container
            {
                'use_intra_process_comms': True,  # Enable zero-copy intra-process communication
                'enable_gpu_memory_pooling': True,  # Enable GPU memory pooling
                'gpu_memory_pool_size': '4096MB'  # GPU memory pool size
            }
        ],
        composable_node_descriptions=[
            # 1. Camera image preprocessing node
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectificationNode',
                name='camera_rectification',
                parameters=[
                    {
                        'input_width': 1280,
                        'input_height': 720,
                        'output_width': 1280,
                        'output_height': 720,
                        'use_gpu': True,
                        'gpu_id': 0,
                        'camera_info_url': 'file:///tmp/camera_calibration.yaml',
                        'input_qos_history': 'keep_last',
                        'input_qos_depth': 10,
                        'input_qos_reliability': 'reliable',
                        'output_qos_history': 'keep_last',
                        'output_qos_depth': 10,
                        'output_qos_reliability': 'reliable'
                    }
                ],
                remappings=[
                    ('image_raw', '/front_camera/image_raw'),
                    ('camera_info', '/front_camera/camera_info'),
                    ('image_rect', '/front_camera/image_rect')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # 2. GPU-accelerated object detection node
            ComposableNode(
                package='isaac_ros_detectnet',
                plugin='nvidia::isaac_ros::detectnet::DetectNetNode',
                name='object_detection',
                parameters=[
                    {
                        'input_width': 1280,
                        'input_height': 720,
                        'model_path': '/models/resnet18_detector.plan',
                        'class_labels_path': '/models/coco_labels.txt',
                        'confidence_threshold': 0.7,
                        'use_gpu': True,
                        'gpu_id': 0,
                        'tensorrt_precision': 'fp16',
                        'tensorrt_engine_cache': '/tmp/tensorrt_cache',
                        'input_qos_history': 'keep_last',
                        'input_qos_depth': 10,
                        'input_qos_reliability': 'reliable',
                        'output_qos_history': 'keep_last',
                        'output_qos_depth': 10,
                        'output_qos_reliability': 'reliable'
                    }
                ],
                remappings=[
                    ('image', '/front_camera/image_rect'),
                    ('detections', '/perception/detections')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # 3. GPU-accelerated semantic segmentation node
            ComposableNode(
                package='isaac_ros_segformer',
                plugin='nvidia::isaac_ros::segformer::SegformerNode',
                name='semantic_segmentation',
                parameters=[
                    {
                        'input_width': 640,
                        'input_height': 480,
                        'model_path': '/models/segformer_model.plan',
                        'class_labels_path': '/models/cityscapes_labels.txt',
                        'use_gpu': True,
                        'gpu_id': 0,
                        'tensorrt_precision': 'fp16',
                        'tensorrt_engine_cache': '/tmp/tensorrt_cache',
                        'input_qos_history': 'keep_last',
                        'input_qos_depth': 10,
                        'input_qos_reliability': 'reliable',
                        'output_qos_history': 'keep_last',
                        'output_qos_depth': 10,
                        'output_qos_reliability': 'reliable'
                    }
                ],
                remappings=[
                    ('image', '/front_camera/image_rect'),
                    ('segmentation', '/perception/segmentation')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # 4. LiDAR preprocessing node
            ComposableNode(
                package='isaac_ros_pointcloud_utils',
                plugin='nvidia::isaac_ros::pointcloud_utils::PreprocessorNode',
                name='lidar_preprocessing',
                parameters=[
                    {
                        'use_gpu': True,
                        'gpu_id': 0,
                        'min_range': 0.5,
                        'max_range': 100.0,
                        'min_height': -2.0,
                        'max_height': 5.0,
                        'remove_ground': True,
                        'ground_estimator_points': 100,
                        'ground_estimator_max_iterations': 100,
                        'ground_estimator_distance_threshold': 0.2,
                        'input_qos_history': 'keep_last',
                        'input_qos_depth': 5,
                        'input_qos_reliability': 'reliable',
                        'output_qos_history': 'keep_last',
                        'output_qos_depth': 5,
                        'output_qos_reliability': 'reliable'
                    }
                ],
                remappings=[
                    ('pointcloud', '/lidar/points'),
                    ('pointcloud_filtered', '/lidar/points_processed')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # 5. GPU-accelerated clustering node
            ComposableNode(
                package='isaac_ros_cluster_segmentation',
                plugin='nvidia::isaac_ros::cluster_segmentation::ClusterSegmentationNode',
                name='lidar_clustering',
                parameters=[
                    {
                        'use_gpu': True,
                        'gpu_id': 0,
                        'cluster_tolerance': 0.5,
                        'min_cluster_size': 10,
                        'max_cluster_size': 25000,
                        'input_qos_history': 'keep_last',
                        'input_qos_depth': 5,
                        'input_qos_reliability': 'reliable',
                        'output_qos_history': 'keep_last',
                        'output_qos_depth': 10,
                        'output_qos_reliability': 'reliable'
                    }
                ],
                remappings=[
                    ('input', '/lidar/points_processed'),
                    ('cluster_list', '/lidar/clusters')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # 6. GPU-accelerated 3D object detection node
            ComposableNode(
                package='isaac_ros_detectnet_3d',
                plugin='nvidia::isaac_ros::detectnet_3d::LidarDetectNetNode',
                name='lidar_object_detection',
                parameters=[
                    {
                        'use_gpu': True,
                        'gpu_id': 0,
                        'model_path': '/models/lidar_detectnet_model.plan',
                        'class_labels_path': '/models/lidar_class_labels.txt',
                        'tensorrt_precision': 'fp16',
                        'tensorrt_engine_cache': '/tmp/tensorrt_cache',
                        'min_detection_score': 0.5,
                        'max_detections': 100,
                        'input_qos_history': 'keep_last',
                        'input_qos_depth': 5,
                        'input_qos_reliability': 'reliable',
                        'output_qos_history': 'keep_last',
                        'output_qos_depth': 10,
                        'output_qos_reliability': 'reliable'
                    }
                ],
                remappings=[
                    ('input', '/lidar/points_processed'),
                    ('detections', '/lidar/detections_3d')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # 7. Multi-sensor fusion node
            ComposableNode(
                package='isaac_ros_fusion',
                plugin='nvidia::isaac_ros::fusion::FusionNode',
                name='sensor_fusion',
                parameters=[
                    {
                        'use_gpu': True,
                        'gpu_id': 0,
                        'fusion_method': 'probabilistic',
                        'max_fusion_distance': 2.0,
                        'temporal_window': 0.1,
                        'confidence_threshold': 0.6,
                        'enable_motion_compensation': True,
                        'input_qos_history': 'keep_last',
                        'input_qos_depth': 10,
                        'input_qos_reliability': 'reliable',
                        'output_qos_history': 'keep_last',
                        'output_qos_depth': 10,
                        'output_qos_reliability': 'reliable'
                    }
                ],
                remappings=[
                    ('camera_detections', '/perception/detections'),
                    ('lidar_detections', '/lidar/detections_3d'),
                    ('imu', '/imu/data'),
                    ('odom', '/odom'),
                    ('fused_detections', '/perception/fused_detections')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # 8. GPU-accelerated tracking node
            ComposableNode(
                package='isaac_ros_tracking',
                plugin='nvidia::isaac_ros::tracking::TrackingNode',
                name='object_tracking',
                parameters=[
                    {
                        'use_gpu': True,
                        'gpu_id': 0,
                        'tracker_type': 'kalman_filter',
                        'max_association_distance': 3.0,
                        'max_track_age': 10.0,
                        'min_track_age': 0.5,
                        'max_num_tracks': 100,
                        'enable_reidentification': True,
                        'input_qos_history': 'keep_last',
                        'input_qos_depth': 10,
                        'input_qos_reliability': 'reliable',
                        'output_qos_history': 'keep_last',
                        'output_qos_depth': 10,
                        'output_qos_reliability': 'reliable'
                    }
                ],
                remappings=[
                    ('input_detections', '/perception/fused_detections'),
                    ('output_tracks', '/perception/tracks')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen'
    )

    # Conditional nodes based on launch parameters
    profiling_nodes = [
        ComposableNode(
            package='isaac_ros_profiler',
            plugin='nvidia::isaac_ros::profiler::ProfilerNode',
            name='profiler_node',
            parameters=[
                {
                    'enable_profiling': enable_profiling,
                    'profile_all_nodes': True,
                    'output_file': '/tmp/perception_profile.json'
                }
            ],
            condition=IfCondition(enable_profiling)
        )
    ]

    # Add profiling nodes to container if enabled
    if profiling_nodes:
        # Note: In a real implementation, we would conditionally add these
        # For this example, we're just showing the structure
        pass

    return LaunchDescription([
        hardware_profile_arg,
        enable_profiling_arg,
        perception_container
    ])


# Additional helper functions for the launch file

def get_hardware_config(hardware_profile):
    """
    Get hardware-specific configuration based on the profile

    Args:
        hardware_profile (str): Hardware profile name

    Returns:
        dict: Hardware-specific configuration parameters
    """
    configs = {
        'jetson_xavier_nx': {
            'gpu_id': 0,
            'memory_pool_size': '2048MB',
            'tensorrt_cache_size': '512MB',
            'target_frame_rate': 15,
            'max_num_tracks': 50,
            'object_detection': {
                'input_width': 640,
                'input_height': 480
            },
            'semantic_segmentation': {
                'input_width': 320,
                'input_height': 240
            }
        },
        'jetson_agx_orin': {
            'gpu_id': 0,
            'memory_pool_size': '4096MB',
            'tensorrt_cache_size': '1024MB',
            'target_frame_rate': 30,
            'max_num_tracks': 100,
            'object_detection': {
                'input_width': 1280,
                'input_height': 720
            },
            'semantic_segmentation': {
                'input_width': 640,
                'input_height': 480
            }
        },
        'desktop_rtx_3080': {
            'gpu_id': 0,
            'memory_pool_size': '8192MB',
            'tensorrt_cache_size': '2048MB',
            'target_frame_rate': 60,
            'max_num_tracks': 200,
            'object_detection': {
                'input_width': 1920,
                'input_height': 1080
            },
            'semantic_segmentation': {
                'input_width': 1280,
                'input_height': 720
            }
        }
    }

    return configs.get(hardware_profile, configs['desktop_rtx_3080'])


def create_hardware_specific_launch_description(hardware_profile):
    """
    Create a launch description with hardware-specific parameters

    Args:
        hardware_profile (str): Hardware profile to use

    Returns:
        LaunchDescription: Configured launch description
    """
    # Get hardware configuration
    hw_config = get_hardware_config(hardware_profile)

    # Create container with hardware-specific parameters
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[
            {
                'use_intra_process_comms': True,
                'enable_gpu_memory_pooling': True,
                'gpu_memory_pool_size': hw_config['memory_pool_size']
            }
        ],
        # ... (similar to above but with hardware-specific parameters)
    )

    return LaunchDescription([container])


# Example usage commands as comments:
"""
Example Usage:

1. Launch with default configuration:
   ros2 launch perception_pipeline_launch_example.py

2. Launch with specific hardware profile:
   ros2 launch perception_pipeline_launch_example.py hardware_profile:=jetson_agx_orin

3. Launch with profiling enabled:
   ros2 launch perception_pipeline_launch_example.py enable_profiling:=true

4. Monitor the pipeline:
   ros2 topic list
   ros2 topic echo /perception/fused_detections
   ros2 run rqt_graph rqt_graph

5. Check GPU utilization:
   watch -n 1 nvidia-smi

6. Visualize results:
   ros2 run rviz2 rviz2 -d perception_visualization.rviz
"""