#!/usr/bin/env python3
"""
Launch Script for Sensor Processing Nodes

This script demonstrates how to launch sensor processing nodes for the humanoid robot
simulation. It includes parameters for configuring the sensor processing behavior.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description for sensor processing nodes."""

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    sensor_processing_namespace = LaunchConfiguration('sensor_processing_namespace')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_namespace = DeclareLaunchArgument(
        'sensor_processing_namespace',
        default_value='humanoid',
        description='Namespace for sensor processing nodes'
    )

    # Sensor data collector node
    sensor_data_collector_node = Node(
        package='your_robot_perception',  # Replace with actual package name
        executable='sensor_data_collector',
        name='sensor_data_collector',
        namespace=sensor_processing_namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/humanoid/lidar/scan', [sensor_processing_namespace, '/lidar/scan']),
            ('/humanoid/imu/data', [sensor_processing_namespace, '/imu/data']),
            ('/humanoid/depth_camera/depth/image_raw', [sensor_processing_namespace, '/depth_camera/depth/image_raw']),
            ('/humanoid/depth_camera/color/image_raw', [sensor_processing_namespace, '/depth_camera/color/image_raw']),
            ('/humanoid/depth_camera/color/camera_info', [sensor_processing_namespace, '/depth_camera/color/camera_info'])
        ],
        output='screen'
    )

    # Sensor fusion analyzer node
    sensor_fusion_analyzer_node = Node(
        package='your_robot_perception',  # Replace with actual package name
        executable='sensor_fusion_analyzer',
        name='sensor_fusion_analyzer',
        namespace=sensor_processing_namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/humanoid/lidar/scan', [sensor_processing_namespace, '/lidar/scan']),
            ('/humanoid/imu/data', [sensor_processing_namespace, '/imu/data']),
            ('/humanoid/odom_fused', [sensor_processing_namespace, '/odom_fused'])
        ],
        output='screen'
    )

    # RViz2 node for visualization (optional)
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('your_robot_description'),  # Replace with actual package
        'rviz',
        'sensor_visualization.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='sensor_rviz',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_namespace)

    # Add nodes
    ld.add_action(sensor_data_collector_node)
    ld.add_action(sensor_fusion_analyzer_node)
    # ld.add_action(rviz_node)  # Uncomment if RViz is available

    return ld


"""
Alternative shell script version for demonstration purposes:

#!/bin/bash

# Sensor Processing Launch Script
# This script launches the sensor processing nodes for the humanoid robot simulation

# Set default values
USE_SIM_TIME=${1:-true}
NAMESPACE=${2:-"humanoid"}

echo "Launching sensor processing nodes with namespace: $NAMESPACE"

# Launch the sensor data collector
echo "Starting sensor data collector..."
ros2 run your_robot_perception sensor_data_collector \
    --ros-args \
    -p use_sim_time:=$USE_SIM_TIME \
    -r /humanoid/lidar/scan:=$NAMESPACE/lidar/scan \
    -r /humanoid/imu/data:=$NAMESPACE/imu/data \
    -r /humanoid/depth_camera/depth/image_raw:=$NAMESPACE/depth_camera/depth/image_raw \
    -r /humanoid/depth_camera/color/image_raw:=$NAMESPACE/depth_camera/color/image_raw \
    -r /humanoid/depth_camera/color/camera_info:=$NAMESPACE/depth_camera/color/camera_info &

# Launch the sensor fusion analyzer
echo "Starting sensor fusion analyzer..."
ros2 run your_robot_perception sensor_fusion_analyzer \
    --ros-args \
    -p use_sim_time:=$USE_SIM_TIME \
    -r /humanoid/lidar/scan:=$NAMESPACE/lidar/scan \
    -r /humanoid/imu/data:=$NAMESPACE/imu/data \
    -r /humanoid/odom_fused:=$NAMESPACE/odom_fused &

echo "Sensor processing nodes launched successfully!"
echo "Press Ctrl+C to stop all nodes."

# Wait for all background processes
wait
"""