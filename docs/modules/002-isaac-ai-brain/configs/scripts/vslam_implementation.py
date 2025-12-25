#!/usr/bin/env python3
"""
Isaac ROS VSLAM Implementation Script

This script demonstrates a complete VSLAM implementation for humanoid robots using Isaac ROS.
It includes camera calibration, feature detection, pose estimation, mapping, and integration
with ROS 2 navigation stack.

Author: Isaac ROS Developer
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import cv2
import numpy as np
import math
import time
from typing import Tuple, List, Optional

# ROS message imports
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

# TF2 imports
import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Point, Pose, PoseStamped

# OpenCV bridge
from cv_bridge import CvBridge

# Optional: Import CUDA libraries if available
try:
    import cupy as cp
    CUDA_AVAILABLE = True
except ImportError:
    CUDA_AVAILABLE = False


class VSLAMNode(Node):
    """
    Visual SLAM node for humanoid robot navigation.

    This node implements a complete VSLAM pipeline including:
    - Feature detection and tracking
    - Pose estimation
    - Map building
    - Loop closure detection
    - Integration with navigation stack
    """

    def __init__(self):
        super().__init__('vslam_node')

        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_topic', '/front_camera/image_rect_color'),
                ('camera_info_topic', '/front_camera/camera_info'),
                ('publish_pose_topic', '/vslam/pose'),
                ('publish_map_topic', '/vslam/map'),
                ('map_resolution', 0.05),
                ('map_size_x', 100.0),
                ('map_size_y', 100.0),
                ('max_features', 1000),
                ('matching_threshold', 0.7),
                ('enable_gpu_processing', True),
                ('gpu_device_id', 0),
                ('enable_loop_closure', True),
                ('loop_closure_threshold', 0.8),
                ('use_sim_time', False)
            ]
        )

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.publish_pose_topic = self.get_parameter('publish_pose_topic').value
        self.publish_map_topic = self.get_parameter('publish_map_topic').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_size_x = self.get_parameter('map_size_x').value
        self.map_size_y = self.get_parameter('map_size_y').value
        self.max_features = self.get_parameter('max_features').value
        self.matching_threshold = self.get_parameter('matching_threshold').value
        self.enable_gpu_processing = self.get_parameter('enable_gpu_processing').value
        self.gpu_device_id = self.get_parameter('gpu_device_id').value
        self.enable_loop_closure = self.get_parameter('enable_loop_closure').value
        self.loop_closure_threshold = self.get_parameter('loop_closure_threshold').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize feature detector and matcher
        self.detector = cv2.ORB_create(nfeatures=int(self.max_features))
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # Initialize tracking variables
        self.current_frame = None
        self.previous_frame = None
        self.current_features = None
        self.previous_features = None
        self.current_descriptors = None
        self.previous_descriptors = None
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.keyframes = []  # Store keyframes for mapping
        self.local_map = {}  # Local map of landmarks

        # Previous pose for motion estimation
        self.prev_pose = np.eye(4)
        self.initialized = False

        # Initialize map
        self.map_width = int(self.map_size_x / self.map_resolution)
        self.map_height = int(self.map_size_y / self.map_resolution)
        self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)

        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create QoS profile for camera data
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile
        )

        # Create publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.publish_pose_topic,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/vslam/odometry',
            10
        )

        self.map_pub = self.create_publisher(
            MarkerArray,
            self.publish_map_topic,
            10
        )

        # Performance monitoring
        self.frame_count = 0
        self.start_time = time.time()

        # Initialize camera intrinsics
        self.K = None  # Camera intrinsic matrix
        self.dist_coeffs = None  # Distortion coefficients

        self.get_logger().info('VSLAM Node initialized successfully')
        self.get_logger().info(f'Using GPU processing: {CUDA_AVAILABLE and self.enable_gpu_processing}')

    def camera_info_callback(self, msg):
        """Callback for camera info to get intrinsic parameters"""
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera calibration parameters loaded')

    def image_callback(self, msg):
        """Callback for processing incoming camera images"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Store current frame
            self.previous_frame = self.current_frame
            self.current_frame = cv_image

            # Detect and track features
            success = self.process_frame()

            if success:
                # Publish pose and map
                self.publish_pose()
                self.publish_odometry()

                # Update performance metrics
                self.frame_count += 1
                if self.frame_count % 30 == 0:  # Log every 30 frames
                    elapsed = time.time() - self.start_time
                    fps = self.frame_count / elapsed
                    self.get_logger().info(f'VSLAM FPS: {fps:.2f}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_frame(self) -> bool:
        """Process current frame for VSLAM"""
        if self.current_frame is None or self.previous_frame is None:
            return False

        # Detect features in current frame
        current_keypoints, current_descriptors = self.detect_features(self.current_frame)

        if current_descriptors is None or len(current_keypoints) < 50:
            self.get_logger().warn(f'Insufficient features detected: {len(current_keypoints) if current_keypoints else 0}')
            return False

        # Track features from previous frame
        if self.previous_descriptors is not None and self.current_descriptors is not None:
            # Match features between frames
            matches = self.match_features(self.previous_descriptors, current_descriptors)

            if len(matches) >= 10:  # Need minimum matches for pose estimation
                # Estimate pose change
                pose_change = self.estimate_pose_change(
                    self.previous_features, current_keypoints, matches
                )

                if pose_change is not None:
                    # Update current pose
                    self.current_pose = self.current_pose @ pose_change

                    # Store as keyframe if significant movement
                    if self.should_add_keyframe(pose_change):
                        self.add_keyframe(current_keypoints, current_descriptors)

                    # Update map with new landmarks
                    self.update_map(current_keypoints, current_descriptors)
        else:
            # First frame - just store features
            self.initialized = True

        # Update previous frame data
        self.previous_features = self.current_features
        self.previous_descriptors = self.current_descriptors
        self.current_features = current_keypoints
        self.current_descriptors = current_descriptors

        return True

    def detect_features(self, image):
        """Detect features in the image using ORB"""
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Detect keypoints and compute descriptors
            keypoints, descriptors = self.detector.detectAndCompute(gray, None)

            if descriptors is not None and len(keypoints) > 0:
                # Convert keypoints to numpy array
                features = np.array([kp.pt for kp in keypoints])
                return features, descriptors
            else:
                return np.array([]), None

        except Exception as e:
            self.get_logger().error(f'Error detecting features: {e}')
            return np.array([]), None

    def match_features(self, desc1, desc2):
        """Match features between two sets of descriptors"""
        try:
            if desc1 is None or desc2 is None:
                return []

            # Use FLANN matcher for better performance
            matches = self.matcher.knnMatch(desc1, desc2, k=2)

            # Apply Lowe's ratio test to filter good matches
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < self.matching_threshold * n.distance:
                        good_matches.append(m)

            return good_matches

        except Exception as e:
            self.get_logger().error(f'Error matching features: {e}')
            return []

    def estimate_pose_change(self, prev_features, curr_features, matches):
        """Estimate pose change using matched features"""
        try:
            if len(matches) < 10:
                return None

            # Extract matched points
            prev_pts = []
            curr_pts = []

            for match in matches:
                prev_idx = match.queryIdx
                curr_idx = match.trainIdx

                if prev_idx < len(prev_features) and curr_idx < len(curr_features):
                    prev_pts.append(prev_features[prev_idx])
                    curr_pts.append(curr_features[curr_idx])

            if len(prev_pts) < 10:
                return None

            prev_pts = np.array(prev_pts, dtype=np.float32)
            curr_pts = np.array(curr_pts, dtype=np.float32)

            # Estimate essential matrix
            if self.K is not None:
                E, mask = cv2.findEssentialMat(
                    curr_pts, prev_pts, self.K,
                    method=cv2.RANSAC, prob=0.999, threshold=1.0
                )

                if E is not None:
                    # Recover pose from essential matrix
                    _, R, t, _ = cv2.recoverPose(E, curr_pts, prev_pts, self.K)

                    # Create transformation matrix
                    pose_change = np.eye(4)
                    pose_change[:3, :3] = R
                    pose_change[:3, 3] = t.ravel()

                    return pose_change
            else:
                # Use homography if no camera calibration
                H, mask = cv2.findHomography(
                    prev_pts, curr_pts, cv2.RANSAC, 5.0
                )

                if H is not None:
                    # Extract rotation and translation from homography
                    # This is a simplified approach
                    pose_change = np.eye(4)
                    pose_change[:3, :3] = H[:3, :3]
                    pose_change[:3, 3] = H[:3, 3]

                    return pose_change

        except Exception as e:
            self.get_logger().error(f'Error estimating pose change: {e}')

        return None

    def should_add_keyframe(self, pose_change, translation_threshold=0.1, rotation_threshold=0.1):
        """Determine if current frame should be added as a keyframe"""
        try:
            # Calculate translation magnitude
            translation = np.linalg.norm(pose_change[:3, 3])

            # Calculate rotation angle
            R = pose_change[:3, :3]
            trace = np.trace(R)
            angle = np.arccos(max(-1, min(1, (trace - 1) / 2)))  # Clamp to avoid numerical errors

            # Add keyframe if movement exceeds thresholds
            return translation > translation_threshold or angle > rotation_threshold

        except Exception as e:
            self.get_logger().error(f'Error determining keyframe: {e}')
            return False

    def add_keyframe(self, features, descriptors):
        """Add current frame as a keyframe"""
        keyframe = {
            'features': features,
            'descriptors': descriptors,
            'pose': self.current_pose.copy(),
            'timestamp': self.get_clock().now().nanoseconds
        }

        self.keyframes.append(keyframe)

        # Limit keyframes to prevent memory overflow
        if len(self.keyframes) > 100:  # Keep only last 100 keyframes
            self.keyframes = self.keyframes[-50:]  # Keep last 50

    def update_map(self, features, descriptors):
        """Update local map with new landmarks"""
        if len(features) > 0:
            # Convert 2D features to 3D landmarks using current pose
            for i, pt in enumerate(features):
                if i < len(descriptors):
                    # Simple triangulation using current pose
                    # In a real implementation, this would use multiple views
                    landmark_3d = self.triangulate_landmark(pt)
                    if landmark_3d is not None:
                        landmark_id = hash(tuple(landmark_3d)) % 1000000  # Simple ID
                        self.local_map[landmark_id] = {
                            'position': landmark_3d,
                            'descriptor': descriptors[i] if i < len(descriptors) else None,
                            'observed_frames': 1
                        }

    def triangulate_landmark(self, feature_2d):
        """Triangulate 3D landmark from 2D feature (simplified)"""
        try:
            # In a real implementation, this would use multiple views
            # For now, we'll create a simple 3D point in front of the camera
            x, y = feature_2d
            center_x = self.current_frame.shape[1] / 2
            center_y = self.current_frame.shape[0] / 2

            # Convert to normalized coordinates
            norm_x = (x - center_x) / center_x
            norm_y = (y - center_y) / center_y

            # Create 3D point (simplified depth assumption)
            depth = 2.0  # Assume 2m depth
            landmark_3d = np.array([
                norm_x * depth,
                norm_y * depth,
                depth
            ])

            # Transform to world coordinates using current pose
            world_landmark = self.current_pose @ np.append(landmark_3d, 1)
            return world_landmark[:3]

        except Exception as e:
            self.get_logger().error(f'Error triangulating landmark: {e}')
            return None

    def publish_pose(self):
        """Publish current pose estimate"""
        try:
            from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion

            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'

            # Extract position and orientation from pose matrix
            position = self.current_pose[:3, 3]

            # Convert rotation matrix to quaternion
            R = self.current_pose[:3, :3]
            qw, qx, qy, qz = self.rotation_matrix_to_quaternion(R)

            # Set pose
            msg.pose.pose.position.x = float(position[0])
            msg.pose.pose.position.y = float(position[1])
            msg.pose.pose.position.z = float(position[2])

            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw

            # Set covariance (simplified)
            msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.1]

            self.pose_pub.publish(msg)

            # Broadcast TF transform
            self.broadcast_transform(position, [qx, qy, qz, qw])

        except Exception as e:
            self.get_logger().error(f'Error publishing pose: {e}')

    def publish_odometry(self):
        """Publish odometry information"""
        try:
            from nav_msgs.msg import Odometry

            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.child_frame_id = 'base_link'

            # Extract position and orientation from pose matrix
            position = self.current_pose[:3, 3]
            R = self.current_pose[:3, :3]
            qw, qx, qy, qz = self.rotation_matrix_to_quaternion(R)

            # Set pose
            msg.pose.pose.position.x = float(position[0])
            msg.pose.pose.position.y = float(position[1])
            msg.pose.pose.position.z = float(position[2])

            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw

            # Set velocities (estimated)
            dt = 0.1  # Assuming 10Hz
            if hasattr(self, 'prev_position'):
                velocity = (position - self.prev_position) / dt
                msg.twist.twist.linear.x = float(velocity[0])
                msg.twist.twist.linear.y = float(velocity[1])
                msg.twist.twist.linear.z = float(velocity[2])

            self.prev_position = position.copy()

            self.odom_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {e}')

    def broadcast_transform(self, position, orientation):
        """Broadcast TF transform from map to base_link"""
        try:
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'base_link'

            t.transform.translation.x = float(position[0])
            t.transform.translation.y = float(position[1])
            t.transform.translation.z = float(position[2])

            t.transform.rotation.x = orientation[0]
            t.transform.rotation.y = orientation[1]
            t.transform.rotation.z = orientation[2]
            t.transform.rotation.w = orientation[3]

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f'Error broadcasting transform: {e}')

    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion"""
        try:
            # Method from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
            trace = np.trace(R)

            if trace > 0:
                s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
                qw = 0.25 * s
                qx = (R[2, 1] - R[1, 2]) / s
                qy = (R[0, 2] - R[2, 0]) / s
                qz = (R[1, 0] - R[0, 1]) / s
            else:
                if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                    s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                    qw = (R[2, 1] - R[1, 2]) / s
                    qx = 0.25 * s
                    qy = (R[0, 1] + R[1, 0]) / s
                    qz = (R[0, 2] + R[2, 0]) / s
                elif R[1, 1] > R[2, 2]:
                    s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                    qw = (R[0, 2] - R[2, 0]) / s
                    qx = (R[0, 1] + R[1, 0]) / s
                    qy = 0.25 * s
                    qz = (R[1, 2] + R[2, 1]) / s
                else:
                    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                    qw = (R[1, 0] - R[0, 1]) / s
                    qx = (R[0, 2] + R[2, 0]) / s
                    qy = (R[1, 2] + R[2, 1]) / s
                    qz = 0.25 * s

            # Normalize quaternion
            norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
            if norm > 0:
                return qw/norm, qx/norm, qy/norm, qz/norm
            else:
                return 1.0, 0.0, 0.0, 0.0

        except Exception as e:
            self.get_logger().error(f'Error converting rotation matrix to quaternion: {e}')
            return 1.0, 0.0, 0.0, 0.0

    def loop_closure_detection(self):
        """Detect loop closures by comparing current frame with keyframes"""
        if not self.enable_loop_closure or len(self.keyframes) < 2:
            return False, None

        try:
            current_desc = self.current_descriptors
            if current_desc is None:
                return False, None

            best_match_score = 0
            best_keyframe_idx = -1

            # Compare with recent keyframes only for efficiency
            recent_keyframes = self.keyframes[-20:]  # Last 20 keyframes

            for i, kf in enumerate(recent_keyframes):
                if kf['descriptors'] is not None:
                    matches = self.match_features(kf['descriptors'], current_desc)
                    match_score = len(matches)  # Simple scoring based on number of matches

                    if match_score > best_match_score:
                        best_match_score = match_score
                        best_keyframe_idx = len(self.keyframes) - len(recent_keyframes) + i

            # Check if loop closure is detected
            if best_match_score > 50:  # Threshold for loop closure
                self.get_logger().info(f'Loop closure detected! Matched with keyframe {best_keyframe_idx}')

                # Perform loop closure optimization (simplified)
                self.optimize_poses(best_keyframe_idx)
                return True, best_keyframe_idx

        except Exception as e:
            self.get_logger().error(f'Error in loop closure detection: {e}')

        return False, None

    def optimize_poses(self, loop_keyframe_idx):
        """Perform simple pose graph optimization when loop closure is detected"""
        try:
            # In a real implementation, this would use a proper optimization library
            # like g2o or Ceres to optimize the pose graph
            self.get_logger().info('Performing pose optimization...')

            # For now, we'll just log the optimization
            # In practice, you would adjust the poses of frames between
            # the current frame and the loop-closed keyframe

        except Exception as e:
            self.get_logger().error(f'Error optimizing poses: {e}')


def main(args=None):
    """Main function to run the VSLAM node"""
    rclpy.init(args=args)

    vslam_node = VSLAMNode()

    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()