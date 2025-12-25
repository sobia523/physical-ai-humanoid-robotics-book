#!/usr/bin/env python3

"""
Complete Simulation Launch File

This launch file orchestrates the complete integrated simulation environment
including Gazebo, robot model, sensors, navigation, and visualization components.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='humanoid_navigation.world')
    robot_name = LaunchConfiguration('robot_name', default='humanoid_robot')
    headless = LaunchConfiguration('headless', default='false')

    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('your_robot_description')
    pkg_robot_gazebo = get_package_share_directory('your_robot_gazebo')
    pkg_robot_navigation = get_package_share_directory('your_robot_navigation')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('your_robot_gazebo'),
                'worlds',
                world_file
            ]),
            'verbose': 'true',
            'headless': headless,
            'gui': 'true'
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': open(
                PathJoinSubstitution([
                    FindPackageShare('your_robot_description'),
                    'urdf',
                    'humanoid_robot.urdf.xacro'
                ])
            ).read()}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Joint state publisher (for simulation)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn robot in Gazebo after Gazebo starts
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', [robot_name],
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',  # Slightly above ground
            '-R', '0.0',  # Roll
            '-P', '0.0',  # Pitch
            '-Y', '0.0'   # Yaw
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Navigation node
    navigation_node = Node(
        package='your_robot_navigation',
        executable='humanoid_navigation',
        name='humanoid_navigation',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        respawn=True
    )

    # Sensor fusion node
    sensor_fusion_node = Node(
        package='your_robot_perception',
        executable='sensor_fusion_analyzer',
        name='sensor_fusion_analyzer',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        respawn=True
    )

    # TF broadcaster for odom -> base_link
    tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_base_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Robot localization node (for odometry)
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('your_robot_localization'),
                'config',
                'ekf.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', 'odom'),
            ('/imu/data', '/humanoid/imu/data')
        ]
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    ))

    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value='humanoid_navigation.world',
        description='Choose one of the world files from `/your_robot_gazebo/worlds`'
    ))

    ld.add_action(DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_robot',
        description='Name of the robot to spawn'
    ))

    ld.add_action(DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without GUI'
    ))

    # Add actions in proper order with timers for synchronization
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)

    # Use timers to ensure proper startup sequence
    spawn_timer = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )
    ld.add_action(spawn_timer)

    # Add navigation and sensor fusion after robot is spawned
    nav_timer = TimerAction(
        period=10.0,
        actions=[navigation_node, sensor_fusion_node, tf_broadcaster, robot_localization]
    )
    ld.add_action(nav_timer)

    return ld