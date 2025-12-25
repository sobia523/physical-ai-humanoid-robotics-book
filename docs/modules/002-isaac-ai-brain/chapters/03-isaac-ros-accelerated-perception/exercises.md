---
title: Isaac ROS Perception Pipeline Exercises
sidebar_position: 4
---

# Isaac ROS Perception Pipeline Exercises

## Exercise 1: Basic Camera Perception Pipeline

### Objective
Implement a basic GPU-accelerated camera perception pipeline using Isaac ROS that performs image rectification and object detection.

### Prerequisites
- NVIDIA GPU with CUDA support
- ROS 2 Humble Hawksbill
- Isaac ROS packages installed
- Camera calibrated with appropriate calibration file

### Instructions

#### Step 1: Set up the Development Environment
1. Verify your GPU is properly detected:
   ```bash
   nvidia-smi
   ```

2. Install Isaac ROS camera packages:
   ```bash
   sudo apt update
   sudo apt install ros-humble-isaac-ros-image-pipeline
   sudo apt install ros-humble-isaac-ros-detectnet
   ```

3. Verify Isaac ROS installation:
   ```bash
   ros2 pkg list | grep isaac
   ```

#### Step 2: Create the Camera Pipeline Launch File
Create a launch file named `camera_perception_pipeline.launch.py`:

```python
import launch
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Create a composable node container for the camera pipeline
    camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Camera rectification node
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectificationNode',
                name='camera_rectification',
                parameters=[{
                    'input_width': 1280,
                    'input_height': 720,
                    'output_width': 1280,
                    'output_height': 720,
                    'use_gpu': True,
                    'gpu_id': 0,
                    'camera_info_url': 'file:///path/to/your/camera_calibration.yaml'
                }],
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/camera/image_rect')
                ]
            ),

            # Object detection node
            ComposableNode(
                package='isaac_ros_detectnet',
                plugin='nvidia::isaac_ros::detectnet::DetectNetNode',
                name='object_detection',
                parameters=[{
                    'input_width': 1280,
                    'input_height': 720,
                    'model_path': '/models/resnet18_detector.plan',
                    'class_labels_path': '/models/coco_labels.txt',
                    'confidence_threshold': 0.7,
                    'use_gpu': True,
                    'gpu_id': 0,
                    'tensorrt_precision': 'fp16',
                    'tensorrt_engine_cache': '/tmp/tensorrt_cache'
                }],
                remappings=[
                    ('image', '/camera/image_rect'),
                    ('detections', '/camera/detections')
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([camera_container])
```

#### Step 3: Test the Pipeline
1. Launch your camera driver (e.g., for a USB camera):
   ```bash
   # For USB camera
   ros2 launch usb_cam usb_cam-test.launch.py
   ```

2. Launch your camera perception pipeline:
   ```bash
   ros2 launch camera_perception_pipeline.launch.py
   ```

3. Verify the pipeline is running:
   ```bash
   ros2 topic list | grep camera
   ros2 topic echo /camera/detections
   ```

#### Step 4: Visualize Results
1. Install and launch RViz2:
   ```bash
   ros2 run rviz2 rviz2
   ```

2. Add a Camera display and subscribe to `/camera/image_rect`
3. Add a Detection2DArray display and subscribe to `/camera/detections`

### Deliverables
- Working camera perception pipeline launch file
- Screenshot showing object detections overlaid on camera image
- Performance metrics (frame rate, GPU utilization)
- Brief report on any issues encountered and how they were resolved

---

## Exercise 2: Multi-Sensor Fusion Pipeline

### Objective
Create a more complex perception pipeline that fuses data from multiple sensors (camera and LiDAR) to improve object detection accuracy.

### Prerequisites
- Camera and LiDAR sensors available
- Both sensors calibrated and properly positioned
- TF transforms between sensors established

### Instructions

#### Step 1: Set up Multi-Sensor Configuration
Create a configuration file `multi_sensor_config.yaml`:

```yaml
# Multi-sensor fusion configuration
pipeline:
  name: "multi_sensor_fusion"
  description: "Fusion of camera and LiDAR data for improved perception"

nodes:
  camera_processing:
    package: "isaac_ros_image_proc"
    executable: "rectification_node"
    parameters:
      input_width: 1280
      input_height: 720
      use_gpu: true
      gpu_id: 0

  lidar_processing:
    package: "isaac_ros_pointcloud_utils"
    executable: "preprocessor_node"
    parameters:
      use_gpu: true
      gpu_id: 0
      min_range: 0.5
      max_range: 100.0

  sensor_fusion:
    package: "isaac_ros_fusion"
    executable: "fusion_node"
    parameters:
      use_gpu: true
      gpu_id: 0
      fusion_method: "probabilistic"
      max_fusion_distance: 2.0
```

#### Step 2: Implement the Fusion Pipeline
Create `multi_sensor_fusion_pipeline.launch.py`:

```python
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Declare launch arguments
    enable_profiling = DeclareLaunchArgument(
        'enable_profiling',
        default_value='false',
        description='Enable performance profiling'
    )

    # Get launch configuration
    enable_profiling_config = LaunchConfiguration('enable_profiling')

    # Multi-sensor fusion container
    fusion_container = ComposableNodeContainer(
        name='fusion_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Camera processing
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectificationNode',
                name='camera_rectification',
                parameters=[{
                    'input_width': 1280,
                    'input_height': 720,
                    'output_width': 1280,
                    'output_height': 720,
                    'use_gpu': True,
                    'gpu_id': 0,
                    'camera_info_url': 'file:///path/to/camera_calibration.yaml'
                }],
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/camera/image_rect')
                ]
            ),

            # LiDAR processing
            ComposableNode(
                package='isaac_ros_pointcloud_utils',
                plugin='nvidia::isaac_ros::pointcloud_utils::PreprocessorNode',
                name='lidar_preprocessing',
                parameters=[{
                    'use_gpu': True,
                    'gpu_id': 0,
                    'min_range': 0.5,
                    'max_range': 100.0,
                    'remove_ground': True
                }],
                remappings=[
                    ('pointcloud', '/lidar/points'),
                    ('pointcloud_filtered', '/lidar/points_processed')
                ]
            ),

            # Camera object detection
            ComposableNode(
                package='isaac_ros_detectnet',
                plugin='nvidia::isaac_ros::detectnet::DetectNetNode',
                name='camera_detection',
                parameters=[{
                    'input_width': 1280,
                    'input_height': 720,
                    'model_path': '/models/resnet18_detector.plan',
                    'class_labels_path': '/models/coco_labels.txt',
                    'confidence_threshold': 0.7,
                    'use_gpu': True,
                    'gpu_id': 0,
                    'tensorrt_precision': 'fp16'
                }],
                remappings=[
                    ('image', '/camera/image_rect'),
                    ('detections', '/camera/detections')
                ]
            ),

            # LiDAR object detection
            ComposableNode(
                package='isaac_ros_detectnet_3d',
                plugin='nvidia::isaac_ros::detectnet_3d::LidarDetectNetNode',
                name='lidar_detection',
                parameters=[{
                    'model_path': '/models/lidar_detectnet_model.plan',
                    'class_labels_path': '/models/lidar_class_labels.txt',
                    'use_gpu': True,
                    'gpu_id': 0,
                    'tensorrt_precision': 'fp16',
                    'min_detection_score': 0.5
                }],
                remappings=[
                    ('input', '/lidar/points_processed'),
                    ('detections', '/lidar/detections_3d')
                ]
            ),

            # Sensor fusion node
            ComposableNode(
                package='isaac_ros_fusion',
                plugin='nvidia::isaac_ros::fusion::FusionNode',
                name='sensor_fusion',
                parameters=[{
                    'use_gpu': True,
                    'gpu_id': 0,
                    'fusion_method': 'probabilistic',
                    'max_fusion_distance': 2.0,
                    'temporal_window': 0.1,
                    'confidence_threshold': 0.6
                }],
                remappings=[
                    ('camera_detections', '/camera/detections'),
                    ('lidar_detections', '/lidar/detections_3d'),
                    ('fused_detections', '/fused_detections')
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        enable_profiling,
        fusion_container
    ])
```

#### Step 3: Launch and Test the Pipeline
1. Launch both sensors:
   ```bash
   # Launch camera
   ros2 launch usb_cam usb_cam-test.launch.py

   # Launch LiDAR (example for Velodyne)
   ros2 launch velodyne_driver velodyne_driver_node-VLP16.launch.py
   ros2 launch velodyne_pointcloud velodyne_cloud_node-VLP16.launch.py
   ```

2. Launch the fusion pipeline:
   ```bash
   ros2 launch multi_sensor_fusion_pipeline.launch.py
   ```

3. Monitor the fusion results:
   ```bash
   ros2 topic echo /fused_detections
   ```

### Deliverables
- Complete multi-sensor fusion launch file
- Configuration file for pipeline parameters
- Performance comparison between single-sensor and fused detection
- Analysis of fusion accuracy improvements
- Troubleshooting report for any synchronization issues

---

## Exercise 3: Performance Optimization Challenge

### Objective
Optimize an existing perception pipeline for maximum performance while maintaining accuracy requirements.

### Prerequisites
- Working perception pipeline from previous exercises
- Profiling tools installed (nvtop, ROS 2 tools)

### Instructions

#### Step 1: Baseline Performance Measurement
1. Measure current pipeline performance:
   ```bash
   # Monitor GPU utilization
   nvtop

   # Monitor ROS 2 topics
   ros2 topic hz /camera/detections
   ```

2. Record baseline metrics:
   - Frame rate (FPS)
   - GPU utilization (%)
   - Memory usage (MB)
   - Processing latency (ms)

#### Step 2: Apply Optimization Techniques
Implement the following optimizations:

1. **TensorRT Precision Optimization**:
   ```yaml
   # Change from fp32 to fp16 precision
   tensorrt_precision: "fp16"
   ```

2. **Memory Pooling**:
   ```python
   # Enable memory pooling in node parameters
   enable_memory_pooling: true
   ```

3. **Batch Processing**:
   ```yaml
   # Increase batch size for inference (if supported)
   tensorrt_max_batch_size: 2
   ```

4. **Intra-process Communication**:
   ```python
   # Enable in launch file
   extra_arguments=[{'use_intra_process_comms': True}]
   ```

#### Step 3: Performance Comparison
1. Compare optimized vs. baseline performance
2. Document performance gains
3. Verify accuracy is maintained

### Deliverables
- Baseline performance metrics
- Optimized pipeline configuration
- Performance comparison table
- Optimization techniques applied and their impact
- Final performance report with recommendations

---

## Exercise 4: Custom Perception Node Development

### Objective
Develop a custom Isaac ROS perception node that performs a specific task using GPU acceleration.

### Prerequisites
- Basic understanding of CUDA programming
- ROS 2 package development experience
- Isaac ROS development tools

### Instructions

#### Step 1: Define the Custom Node
Create a new ROS 2 package for your custom perception node:
```bash
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
ros2 pkg create --library-name custom_perception --dependencies rclcpp rclcpp_components sensor_msgs nvidia_isaac_ros_common custom_perception
```

#### Step 2: Implement GPU-Accelerated Processing
Create a custom node that uses GPU acceleration for a specific task (e.g., custom image filtering):

```cpp
// custom_perception/src/custom_perception_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>

namespace custom_perception
{
class CustomPerceptionNode : public rclcpp::Node
{
public:
    CustomPerceptionNode()
    : Node("custom_perception_node")
    {
        // Create subscription
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "input_image", 10,
            std::bind(&CustomPerceptionNode::imageCallback, this, std::placeholders::_1));

        // Create publisher
        result_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "output_image", 10);

        RCLCPP_INFO(this->get_logger(), "Custom Perception Node initialized");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS Image to OpenCV
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

        // Allocate GPU memory
        cv::cuda::GpuMat gpu_src, gpu_dst;
        gpu_src.upload(cv_ptr->image);

        // Perform GPU-accelerated processing (example: Gaussian blur)
        cv::cuda::bilateralFilter(gpu_src, gpu_dst, 15, 80, 80);

        // Download result
        cv::Mat result;
        gpu_dst.download(result);

        // Convert back to ROS message
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = result;

        // Publish result
        result_pub_->publish(out_msg.toImageMsg());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_pub_;
};

} // namespace custom_perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(custom_perception::CustomPerceptionNode)
```

#### Step 3: Build and Test
1. Build the package:
   ```bash
   cd ~/isaac_ros_ws
   colcon build --packages-select custom_perception
   source install/setup.bash
   ```

2. Create a launch file for your custom node:
   ```python
   # custom_perception/launch/custom_perception.launch.py
   import launch
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       custom_node = Node(
           package='custom_perception',
           executable='custom_perception_node',
           name='custom_perception_node',
           parameters=[
               {'use_gpu': True}
           ],
           remappings=[
               ('input_image', '/camera/image_rect'),
               ('output_image', '/custom_output')
           ]
       )

       return LaunchDescription([custom_node])
   ```

3. Test the custom node:
   ```bash
   ros2 launch custom_perception custom_perception.launch.py
   ```

### Deliverables
- Complete custom perception node implementation
- Launch file for the custom node
- Test results demonstrating GPU acceleration
- Performance comparison with CPU-only implementation
- Documentation of the custom algorithm implemented

---

## Exercise 5: Pipeline Validation and Testing

### Objective
Validate that your perception pipeline produces accurate results and handles edge cases appropriately.

### Instructions

#### Step 1: Create Test Scenarios
1. Prepare test data with known ground truth
2. Create scenarios with different lighting conditions
3. Prepare challenging cases (occlusions, low contrast, etc.)

#### Step 2: Implement Validation Tools
Create a validation node that compares perception results with ground truth:

```python
# validation_node.py
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32

class PerceptionValidator(Node):
    def __init__(self):
        super().__init__('perception_validator')

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/perception/detections',
            self.detection_callback,
            10
        )

        self.accuracy_pub = self.create_publisher(
            Float32,
            '/perception/accuracy',
            10
        )

    def detection_callback(self, msg):
        # Compare with ground truth (simplified example)
        accuracy = self.calculate_accuracy(msg)

        accuracy_msg = Float32()
        accuracy_msg.data = accuracy
        self.accuracy_pub.publish(accuracy_msg)

    def calculate_accuracy(self, detections):
        # Implement accuracy calculation logic
        # This would compare detections with ground truth data
        return 0.95  # Placeholder value
```

#### Step 3: Run Validation Tests
1. Execute your pipeline on test scenarios
2. Monitor accuracy metrics
3. Document performance under various conditions

### Deliverables
- Validation node implementation
- Test results with accuracy metrics
- Edge case analysis
- Recommendations for improving pipeline robustness
- Validation report with pass/fail criteria

---

## Submission Guidelines

For each exercise, please submit:
1. All source code and configuration files
2. Screenshots of working pipeline
3. Performance metrics and analysis
4. Troubleshooting notes and solutions
5. A brief report (200-500 words) summarizing your approach and findings

## Evaluation Criteria

- **Technical Implementation (40%)**: Correctness and completeness of code/configuration
- **Performance (25%)**: Efficiency and optimization of the pipeline
- **Analysis and Documentation (20%)**: Quality of analysis and documentation
- **Problem-Solving (15%)**: Demonstration of understanding and troubleshooting skills

## Resources and References

- NVIDIA Isaac ROS Documentation
- ROS 2 Humble Hawksbill Tutorials
- CUDA Programming Guide
- TensorRT Documentation
- Isaac ROS GitHub Repository Examples