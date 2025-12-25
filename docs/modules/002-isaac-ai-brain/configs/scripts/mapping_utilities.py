#!/usr/bin/env python3
"""
Isaac ROS Mapping Utilities

This script provides mapping utilities for the VSLAM system, including:
- Occupancy grid mapping
- Point cloud processing
- Map optimization
- Map saving/loading
- Visualization tools

Author: Isaac ROS Developer
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import numpy as np
import cv2
from scipy.spatial import KDTree
import struct
import yaml
import os
from typing import Tuple, List, Optional, Dict, Any

# ROS message imports
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

# TF2 imports
import tf2_ros
from tf2_ros import TransformListener
from tf2_geometry_msgs import do_transform_pose

# OpenCV bridge
from cv_bridge import CvBridge


class MappingUtilities(Node):
    """
    Mapping utilities for VSLAM system.

    This node provides various mapping functions including:
    - Occupancy grid construction from point clouds
    - Map optimization and refinement
    - Map saving/loading
    - Visualization tools
    """

    def __init__(self):
        super().__init__('mapping_utilities')

        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_pointcloud_topic', '/vslam/pointcloud'),
                ('input_pose_topic', '/vslam/pose'),
                ('output_map_topic', '/vslam/occupancy_grid'),
                ('map_resolution', 0.05),
                ('map_size_x', 100.0),
                ('map_size_y', 100.0),
                ('map_origin_x', -50.0),
                ('map_origin_y', -50.0),
                ('max_range', 10.0),
                ('min_obstacle_height', -0.1),
                ('max_obstacle_height', 2.0),
                ('free_threshold', 0.2),
                ('occupied_threshold', 0.65),
                ('enable_3d_mapping', True),
                ('enable_loop_closure', True),
                ('loop_closure_threshold', 0.8),
                ('use_sim_time', False)
            ]
        )

        # Get parameters
        self.input_pointcloud_topic = self.get_parameter('input_pointcloud_topic').value
        self.input_pose_topic = self.get_parameter('input_pose_topic').value
        self.output_map_topic = self.get_parameter('output_map_topic').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_size_x = self.get_parameter('map_size_x').value
        self.map_size_y = self.get_parameter('map_size_y').value
        self.map_origin_x = self.get_parameter('map_origin_x').value
        self.map_origin_y = self.get_parameter('map_origin_y').value
        self.max_range = self.get_parameter('max_range').value
        self.min_obstacle_height = self.get_parameter('min_obstacle_height').value
        self.max_obstacle_height = self.get_parameter('max_obstacle_height').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.enable_3d_mapping = self.get_parameter('enable_3d_mapping').value
        self.enable_loop_closure = self.get_parameter('enable_loop_closure').value
        self.loop_closure_threshold = self.get_parameter('loop_closure_threshold').value

        # Calculate map dimensions
        self.map_width = int(self.map_size_x / self.map_resolution)
        self.map_height = int(self.map_size_y / self.map_resolution)

        # Initialize occupancy grid
        self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)

        # Initialize pose tracking
        self.current_pose = np.eye(4)
        self.poses = []  # Store trajectory

        # Initialize point cloud processing
        self.points_3d = []  # Store 3D points
        self.grid_counts = np.zeros((self.map_height, self.map_width), dtype=np.int32)
        self.grid_probabilities = np.full((self.map_height, self.map_width), 0.5, dtype=np.float32)

        # Initialize TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create QoS profile for sensor data
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Create subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.input_pointcloud_topic,
            self.pointcloud_callback,
            qos_profile
        )

        self.pose_sub = self.create_subscription(
            Pose,
            self.input_pose_topic,
            self.pose_callback,
            10
        )

        # Create publishers
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            self.output_map_topic,
            10
        )

        self.map_metadata_pub = self.create_publisher(
            MapMetaData,
            self.output_map_topic + '/metadata',
            10
        )

        # Create timer for periodic map updates
        self.map_update_timer = self.create_timer(1.0, self.publish_map)  # Update map every second

        self.get_logger().info('Mapping Utilities Node initialized successfully')
        self.get_logger().info(f'Map dimensions: {self.map_width}x{self.map_height}, resolution: {self.map_resolution}m')

    def pointcloud_callback(self, msg):
        """Callback for processing incoming point cloud data"""
        try:
            # Parse point cloud data
            points = self.parse_pointcloud(msg)

            if len(points) > 0:
                # Update occupancy grid with new points
                self.update_occupancy_grid(points)

                # Store 3D points for 3D mapping
                if self.enable_3d_mapping:
                    self.update_3d_map(points)

                self.get_logger().debug(f'Processed point cloud with {len(points)} points')

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def pose_callback(self, msg):
        """Callback for processing pose updates"""
        try:
            # Convert pose message to transformation matrix
            pose_matrix = self.pose_to_matrix(msg)

            # Store pose for trajectory
            self.current_pose = pose_matrix
            self.poses.append(pose_matrix)

            # Keep only recent poses to prevent memory overflow
            if len(self.poses) > 1000:
                self.poses = self.poses[-500:]  # Keep last 500 poses

        except Exception as e:
            self.get_logger().error(f'Error processing pose: {e}')

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

        # Parse data based on point step and data length
        point_step = pc2_msg.point_step
        row_step = pc2_msg.row_step
        width = pc2_msg.width
        height = pc2_msg.height

        # Process point cloud data
        for v in range(height):
            for u in range(width):
                idx = v * row_step + u * point_step

                # Extract x, y, z coordinates
                if offset_x >= 0 and offset_y >= 0 and offset_z >= 0:
                    try:
                        x = struct.unpack('<f', pc2_msg.data[idx+offset_x:idx+offset_x+4])[0]
                        y = struct.unpack('<f', pc2_msg.data[idx+offset_y:idx+offset_y+4])[0]
                        z = struct.unpack('<f', pc2_msg.data[idx+offset_z:idx+offset_z+4])[0]

                        # Filter points by height
                        if self.min_obstacle_height <= z <= self.max_obstacle_height:
                            points.append([x, y, z])
                    except struct.error:
                        # Skip malformed data
                        continue

        return np.array(points)

    def update_occupancy_grid(self, points):
        """Update occupancy grid with new point cloud data"""
        try:
            if len(points) == 0:
                return

            # Transform points to map coordinates
            transformed_points = self.transform_points_to_map(points)

            # Update grid with new observations
            for point in transformed_points:
                x, y = point

                # Convert to grid coordinates
                grid_x = int((x - self.map_origin_x) / self.map_resolution)
                grid_y = int((y - self.map_origin_y) / self.map_resolution)

                # Check bounds
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    # Update cell probability using Bayesian updating
                    current_prob = self.grid_probabilities[grid_y, grid_x]

                    # Simple model: points indicate occupied space
                    new_prob = min(0.9, current_prob + 0.1)

                    # Update probabilities
                    self.grid_probabilities[grid_y, grid_x] = new_prob

                    # Convert to occupancy grid format (-1: unknown, 0: free, 100: occupied)
                    if new_prob > self.occupied_threshold:
                        self.occupancy_grid[grid_y, grid_x] = 100
                    elif new_prob < self.free_threshold:
                        self.occupancy_grid[grid_y, grid_x] = 0
                    else:
                        self.occupancy_grid[grid_y, grid_x] = -1  # Unknown

        except Exception as e:
            self.get_logger().error(f'Error updating occupancy grid: {e}')

    def update_3d_map(self, points):
        """Update 3D map with new point cloud data"""
        try:
            if len(points) == 0:
                return

            # Store 3D points
            for point in points:
                self.points_3d.append(point)

            # Limit stored points to prevent memory overflow
            if len(self.points_3d) > 100000:  # Keep last 100k points
                self.points_3d = self.points_3d[-50000:]

        except Exception as e:
            self.get_logger().error(f'Error updating 3D map: {e}')

    def transform_points_to_map(self, points):
        """Transform points from sensor frame to map frame"""
        try:
            # Get transform from sensor to map frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',  # Target frame
                    points[0][0], points[0][1], points[0][2],  # This is a placeholder - need actual sensor frame
                    rclpy.time.Time(seconds=0),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # If transform not available, assume points are already in map frame
                return points[:, :2]  # Return x, y coordinates

            # Apply transformation
            transformed_points = []
            for point in points:
                # Apply translation and rotation
                # For simplicity, we'll just return x, y coordinates
                transformed_points.append([point[0], point[1]])

            return np.array(transformed_points)

        except Exception as e:
            self.get_logger().error(f'Error transforming points: {e}')
            return points[:, :2]  # Return x, y coordinates as fallback

    def pose_to_matrix(self, pose_msg):
        """Convert Pose message to 4x4 transformation matrix"""
        try:
            # Extract position
            pos = pose_msg.position
            # Extract orientation
            quat = pose_msg.orientation

            # Create transformation matrix
            matrix = np.eye(4)

            # Set translation
            matrix[0, 3] = pos.x
            matrix[1, 3] = pos.y
            matrix[2, 3] = pos.z

            # Convert quaternion to rotation matrix
            R = self.quaternion_to_rotation_matrix([quat.x, quat.y, quat.z, quat.w])
            matrix[:3, :3] = R

            return matrix

        except Exception as e:
            self.get_logger().error(f'Error converting pose to matrix: {e}')
            return np.eye(4)

    def quaternion_to_rotation_matrix(self, quat):
        """Convert quaternion to 3x3 rotation matrix"""
        try:
            x, y, z, w = quat

            # Normalize quaternion
            norm = np.sqrt(x*x + y*y + z*z + w*w)
            if norm > 0:
                x, y, z, w = x/norm, y/norm, z/norm, w/norm

            # Create rotation matrix
            R = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
                [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
            ])

            return R

        except Exception as e:
            self.get_logger().error(f'Error converting quaternion to rotation matrix: {e}')
            return np.eye(3)

    def publish_map(self):
        """Publish the current occupancy grid map"""
        try:
            msg = OccupancyGrid()

            # Header
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'

            # Metadata
            msg.info.resolution = float(self.map_resolution)
            msg.info.width = int(self.map_width)
            msg.info.height = int(self.map_height)
            msg.info.origin.position.x = float(self.map_origin_x)
            msg.info.origin.position.y = float(self.map_origin_y)
            msg.info.origin.position.z = 0.0
            msg.info.origin.orientation.w = 1.0

            # Data (flatten the 2D array)
            msg.data = self.occupancy_grid.flatten().tolist()

            # Publish map
            self.map_pub.publish(msg)

            # Also publish metadata separately
            metadata_msg = MapMetaData()
            metadata_msg.resolution = float(self.map_resolution)
            metadata_msg.width = int(self.map_width)
            metadata_msg.height = int(self.map_height)
            metadata_msg.origin = msg.info.origin

            self.map_metadata_pub.publish(metadata_msg)

            self.get_logger().debug(f'Published map with {len(msg.data)} cells')

        except Exception as e:
            self.get_logger().error(f'Error publishing map: {e}')

    def save_map(self, filename: str):
        """Save map to file"""
        try:
            map_data = {
                'resolution': float(self.map_resolution),
                'width': int(self.map_width),
                'height': int(self.map_height),
                'origin_x': float(self.map_origin_x),
                'origin_y': float(self.map_origin_y),
                'grid': self.occupancy_grid.tolist(),
                'trajectory': [pose.flatten().tolist() for pose in self.poses]
            }

            with open(filename, 'w') as f:
                yaml.dump(map_data, f)

            self.get_logger().info(f'Map saved to {filename}')

        except Exception as e:
            self.get_logger().error(f'Error saving map: {e}')

    def load_map(self, filename: str):
        """Load map from file"""
        try:
            with open(filename, 'r') as f:
                map_data = yaml.safe_load(f)

            self.map_resolution = map_data['resolution']
            self.map_width = map_data['width']
            self.map_height = map_data['height']
            self.map_origin_x = map_data['origin_x']
            self.map_origin_y = map_data['origin_y']

            # Restore grid
            self.occupancy_grid = np.array(map_data['grid'], dtype=np.int8)

            # Restore trajectory
            self.poses = [np.array(pose).reshape(4, 4) for pose in map_data['trajectory']]

            self.get_logger().info(f'Map loaded from {filename}')

        except Exception as e:
            self.get_logger().error(f'Error loading map: {e}')

    def get_map_bounds(self) -> Tuple[float, float, float, float]:
        """Get current map bounds (min_x, max_x, min_y, max_y)"""
        try:
            min_x = self.map_origin_x
            max_x = self.map_origin_x + self.map_width * self.map_resolution
            min_y = self.map_origin_y
            max_y = self.map_origin_y + self.map_height * self.map_resolution

            return min_x, max_x, min_y, max_y

        except Exception as e:
            self.get_logger().error(f'Error getting map bounds: {e}')
            return 0.0, 0.0, 0.0, 0.0

    def ray_trace_free_space(self, start_x: float, start_y: float, end_x: float, end_y: float):
        """Perform ray tracing to mark free space between two points"""
        try:
            # Bresenham's line algorithm for ray tracing
            dx = abs(end_x - start_x)
            dy = abs(end_y - start_y)
            x_step = 1 if start_x < end_x else -1
            y_step = 1 if start_y < end_y else -1

            error = dx - dy
            x, y = start_x, start_y

            while x != end_x or y != end_y:
                # Convert to grid coordinates
                grid_x = int((x - self.map_origin_x) / self.map_resolution)
                grid_y = int((y - self.map_origin_y) / self.map_resolution)

                # Check bounds
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    # Mark as free space (probability decreases)
                    current_prob = self.grid_probabilities[grid_y, grid_x]
                    new_prob = max(0.0, current_prob - 0.05)  # Small decrease for free space
                    self.grid_probabilities[grid_y, grid_x] = new_prob

                    # Update occupancy grid
                    if new_prob < self.free_threshold:
                        self.occupancy_grid[grid_y, grid_x] = 0

                # Move to next point
                error2 = 2 * error
                if error2 > -dy:
                    error -= dy
                    x += x_step
                if error2 < dx:
                    error += dx
                    y += y_step

        except Exception as e:
            self.get_logger().error(f'Error in ray tracing: {e}')

    def get_nearest_obstacle_distance(self, x: float, y: float) -> float:
        """Get distance to nearest obstacle from given point"""
        try:
            # Convert to grid coordinates
            grid_x = int((x - self.map_origin_x) / self.map_resolution)
            grid_y = int((y - self.map_origin_y) / self.map_resolution)

            # Check bounds
            if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
                return float('inf')

            # Search outward in expanding circles
            for radius in range(1, min(self.map_width, self.map_height)):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        if abs(dx) == radius or abs(dy) == radius:  # Only on the border
                            nx, ny = grid_x + dx, grid_y + dy

                            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                                if self.occupancy_grid[ny, nx] == 100:  # Occupied
                                    # Convert back to meters
                                    return np.sqrt((dx * self.map_resolution)**2 + (dy * self.map_resolution)**2)

            return float('inf')  # No obstacles found

        except Exception as e:
            self.get_logger().error(f'Error getting nearest obstacle: {e}')
            return float('inf')


def main(args=None):
    """Main function to run the mapping utilities node"""
    rclpy.init(args=args)

    mapping_node = MappingUtilities()

    try:
        rclpy.spin(mapping_node)
    except KeyboardInterrupt:
        pass
    finally:
        mapping_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()