# Hands-On Exercise: Sensor Simulation Setup for Humanoid Robotics

## Exercise Overview

In this hands-on exercise, you will configure and integrate simulated sensors (LiDAR, Depth Camera, and IMU) on a humanoid robot model in Gazebo and connect them to ROS 2 topics. You will then create a simple sensor processing node to validate the sensor data streams.

**Estimated Completion Time**: 90 minutes

## Prerequisites

Before starting this exercise, ensure you have:

1. Completed the previous chapters on Gazebo physics simulation and Unity rendering
2. Basic understanding of ROS 2 concepts (nodes, topics, messages)
3. A working Gazebo and ROS 2 Humble installation
4. Basic Python programming skills
5. The humanoid robot URDF model from previous exercises

## Exercise Objectives

By the end of this exercise, you will be able to:

1. Configure LiDAR, Depth Camera, and IMU sensors in a Gazebo simulation
2. Connect simulated sensors to appropriate ROS 2 topics
3. Validate sensor data streams using ROS 2 tools
4. Create a basic sensor processing node
5. Implement simple sensor fusion techniques

## Part 1: LiDAR Sensor Configuration (20 minutes)

### Step 1.1: Create the LiDAR Sensor Definition

1. Navigate to your robot's URDF directory:
   ```bash
   cd ~/your_robot_description/urdf/sensors/
   ```

2. Create a new file called `lidar.gazebo.xacro`:
   ```xml
   <?xml version="1.0"?>
   <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
     <xacro:macro name="lidar_sensor" params="prefix parent *origin">
       <!-- LiDAR Link -->
       <link name="${prefix}_link">
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
         <xacro:insert_block name="origin"/>
       </link>

       <!-- LiDAR Joint -->
       <joint name="${prefix}_joint" type="fixed">
         <xacro:insert_block name="origin"/>
         <parent link="${parent}"/>
         <child link="${prefix}_link"/>
       </joint>

       <!-- Gazebo Plugin for LiDAR -->
       <gazebo reference="${prefix}_link">
         <sensor type="ray" name="${prefix}_sensor">
           <pose>0 0 0 0 0 0</pose>
           <ray>
             <scan>
               <horizontal>
                 <samples>720</samples>
                 <resolution>1</resolution>
                 <min_angle>-1.570796</min_angle> <!-- -π/2 -->
                 <max_angle>1.570796</max_angle>   <!-- π/2 -->
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
             <frame_name>${prefix}_link</frame_name>
             <min_range>0.1</min_range>
             <max_range>30.0</max_range>
             <gaussian_noise>0.01</gaussian_noise>
           </plugin>
         </sensor>
       </gazebo>
     </xacro:macro>
   </robot>
   ```

### Step 1.2: Include the LiDAR in Your Robot Model

1. Open your main robot URDF file (e.g., `robot.gazebo.xacro`)

2. Include the LiDAR definition:
   ```xml
   <xacro:include filename="$(find your_robot_description)/urdf/sensors/lidar.gazebo.xacro"/>
   ```

3. Add the LiDAR to your robot (typically on the head or torso):
   ```xml
   <xacro:lidar_sensor prefix="lidar" parent="base_link">
     <origin xyz="0.0 0.0 1.0" rpy="0 0 0"/>
   </xacro:lidar_sensor>
   ```

## Part 2: Depth Camera Configuration (20 minutes)

### Step 2.1: Create the Depth Camera Definition

1. Create a new file called `depth_camera.gazebo.xacro` in the sensors directory:
   ```xml
   <?xml version="1.0"?>
   <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
     <xacro:macro name="depth_camera" params="prefix parent *origin">
       <!-- Depth Camera Link -->
       <link name="${prefix}_link">
         <visual>
           <geometry>
             <box size="0.02 0.08 0.04"/>
           </geometry>
           <material name="black">
             <color rgba="0.1 0.1 0.1 1.0"/>
           </material>
         </visual>
         <collision>
           <geometry>
             <box size="0.02 0.08 0.04"/>
           </geometry>
         </collision>
         <xacro:insert_block name="origin"/>
       </link>

       <!-- Depth Camera Joint -->
       <joint name="${prefix}_joint" type="fixed">
         <xacro:insert_block name="origin"/>
         <parent link="${parent}"/>
         <child link="${prefix}_link"/>
       </joint>

       <!-- Gazebo Plugin for Depth Camera -->
       <gazebo reference="${prefix}_link">
         <sensor type="depth" name="${prefix}_sensor">
           <always_on>true</always_on>
           <update_rate>30</update_rate>
           <camera>
             <horizontal_fov>1.0472</horizontal_fov> <!-- 60 degrees -->
             <image>
               <format>R8G8B8</format>
               <width>640</width>
               <height>480</height>
             </image>
             <clip>
               <near>0.1</near>
               <far>10.0</far>
             </clip>
           </camera>
           <plugin name="gazebo_ros_depth_camera" filename="libgazebo_ros_depth_camera.so">
             <ros>
               <namespace>/humanoid</namespace>
               <remapping>depth/image_raw:=/humanoid/depth_camera/depth/image_raw</remapping>
               <remapping>rgb/image_raw:=/humanoid/depth_camera/color/image_raw</remapping>
               <remapping>rgb/camera_info:=/humanoid/depth_camera/color/camera_info</remapping>
             </ros>
             <frame_name>${prefix}_link</frame_name>
             <baseline>0.1</baseline>
             <distortion_k1>0.0</distortion_k1>
             <distortion_k2>0.0</distortion_k2>
             <distortion_k3>0.0</distortion_k3>
             <distortion_t1>0.0</distortion_t1>
             <distortion_t2>0.0</distortion_t2>
           </plugin>
         </sensor>
       </gazebo>
     </xacro:macro>
   </robot>
   ```

### Step 2.2: Include the Depth Camera in Your Robot Model

1. Include the depth camera definition in your main URDF:
   ```xml
   <xacro:include filename="$(find your_robot_description)/urdf/sensors/depth_camera.gazebo.xacro"/>
   ```

2. Add the depth camera to your robot (typically on the head):
   ```xml
   <xacro:depth_camera prefix="depth_camera" parent="head_link">
     <origin xyz="0.05 0.0 0.0" rpy="0 0 0"/>
   </xacro:depth_camera>
   ```

## Part 3: IMU Configuration (15 minutes)

### Step 3.1: Create the IMU Definition

1. Create a new file called `imu.gazebo.xacro` in the sensors directory:
   ```xml
   <?xml version="1.0"?>
   <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
     <xacro:macro name="imu_sensor" params="prefix parent *origin">
       <!-- IMU Link -->
       <link name="${prefix}_link">
         <visual>
           <geometry>
             <box size="0.01 0.01 0.01"/>
           </geometry>
           <material name="red">
             <color rgba="1.0 0.0 0.0 1.0"/>
           </material>
         </visual>
         <collision>
           <geometry>
             <box size="0.01 0.01 0.01"/>
           </geometry>
         </collision>
         <xacro:insert_block name="origin"/>
       </link>

       <!-- IMU Joint -->
       <joint name="${prefix}_joint" type="fixed">
         <xacro:insert_block name="origin"/>
         <parent link="${parent}"/>
         <child link="${prefix}_link"/>
       </joint>

       <!-- Gazebo Plugin for IMU -->
       <gazebo reference="${prefix}_link">
         <sensor type="imu" name="${prefix}_sensor">
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
             <frame_name>${prefix}_link</frame_name>
             <topic_name>/humanoid/imu/data</topic_name>
             <serviceName>/humanoid/imu/calibrate</serviceName>
             <gaussian_noise>0.0017</gaussian_noise>
           </plugin>
         </sensor>
       </gazebo>
     </xacro:macro>
   </robot>
   ```

### Step 3.2: Include the IMU in Your Robot Model

1. Include the IMU definition in your main URDF:
   ```xml
   <xacro:include filename="$(find your_robot_description)/urdf/sensors/imu.gazebo.xacro"/>
   ```

2. Add the IMU to your robot (typically in the torso):
   ```xml
   <xacro:imu_sensor prefix="imu" parent="torso_link">
     <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
   </xacro:imu_sensor>
   ```

## Part 4: Launch and Test the Configuration (20 minutes)

### Step 4.1: Create a Launch File

1. Create a launch directory in your robot package:
   ```bash
   mkdir -p ~/your_robot_bringup/launch
   ```

2. Create a launch file `sensor_simulation.launch.py`:
   ```python
   import os
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
   from launch.substitutions import LaunchConfiguration
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory

   def generate_launch_description():
       # Launch arguments
       use_sim_time = LaunchConfiguration('use_sim_time', default='true')

       # Robot state publisher
       robot_description_path = os.path.join(
           get_package_share_directory('your_robot_description'),
           'urdf',
           'robot.urdf.xacro'
       )

       robot_state_publisher_node = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           name='robot_state_publisher',
           output='screen',
           parameters=[{'use_sim_time': use_sim_time, 'robot_description': open(robot_description_path).read()}]
       )

       # Gazebo launch
       gazebo = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               get_package_share_directory('gazebo_ros'),
               '/launch',
               '/gazebo.launch.py'
           ])
       )

       # Spawn robot in Gazebo
       spawn_entity = Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=['-topic', 'robot_description', '-entity', 'humanoid_robot'],
           output='screen'
       )

       return LaunchDescription([
           DeclareLaunchArgument(
               'use_sim_time',
               default_value='true',
               description='Use simulation clock if true'
           ),
           gazebo,
           robot_state_publisher_node,
           spawn_entity
       ])
   ```

### Step 4.2: Build and Launch

1. Build your packages:
   ```bash
   cd ~/your_robot_ws
   colcon build --packages-select your_robot_description your_robot_bringup
   source install/setup.bash
   ```

2. Launch the simulation:
   ```bash
   ros2 launch your_robot_bringup sensor_simulation.launch.py
   ```

### Step 4.3: Verify Sensor Topics

1. In a new terminal, check available topics:
   ```bash
   ros2 topic list | grep humanoid
   ```

2. You should see topics like:
   - `/humanoid/lidar/scan`
   - `/humanoid/depth_camera/depth/image_raw`
   - `/humanoid/depth_camera/color/image_raw`
   - `/humanoid/depth_camera/color/camera_info`
   - `/humanoid/imu/data`

3. Echo one of the topics to verify data:
   ```bash
   ros2 topic echo /humanoid/lidar/scan sensor_msgs/msg/LaserScan
   ```

## Part 5: Create a Sensor Validation Node (15 minutes)

### Step 5.1: Create the Validation Node

1. Create a Python script `sensor_validator.py` in your robot package:
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import LaserScan, Image, Imu
   from cv_bridge import CvBridge
   import numpy as np

   class SensorValidator(Node):
       def __init__(self):
           super().__init__('sensor_validator')

           # Initialize CvBridge
           self.bridge = CvBridge()

           # Create subscribers for each sensor
           self.lidar_sub = self.create_subscription(
               LaserScan, '/humanoid/lidar/scan', self.lidar_callback, 10)
           self.imu_sub = self.create_subscription(
               Imu, '/humanoid/imu/data', self.imu_callback, 10)
           self.depth_sub = self.create_subscription(
               Image, '/humanoid/depth_camera/depth/image_raw', self.depth_callback, 10)

           # Statistics tracking
           self.lidar_count = 0
           self.imu_count = 0
           self.depth_count = 0

           # Timers for periodic reporting
           self.timer = self.create_timer(5.0, self.report_status)

           self.get_logger().info('Sensor validator node started')

       def lidar_callback(self, msg):
           self.lidar_count += 1
           # Validate that ranges are within expected bounds
           valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
           if len(valid_ranges) == 0:
               self.get_logger().warning('LiDAR: No valid ranges detected')

       def imu_callback(self, msg):
           self.imu_count += 1
           # Validate IMU data
           acc_norm = np.sqrt(msg.linear_acceleration.x**2 +
                             msg.linear_acceleration.y**2 +
                             msg.linear_acceleration.z**2)
           if not (5.0 < acc_norm < 15.0):  # Reasonable range for acceleration
               self.get_logger().warning(f'IMU: Acceleration magnitude seems off: {acc_norm}')

       def depth_callback(self, msg):
           self.depth_count += 1
           # Convert and check depth image
           try:
               cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
               if cv_image is not None:
                   valid_depths = cv_image[cv_image > 0]
                   if len(valid_depths) == 0:
                       self.get_logger().warning('Depth: No valid depth values in image')
           except Exception as e:
               self.get_logger().error(f'Depth: Error processing image: {e}')

       def report_status(self):
           self.get_logger().info(
               f'Sensor data received - LiDAR: {self.lidar_count}, '
               f'IMU: {self.imu_count}, Depth: {self.depth_count}'
           )

   def main(args=None):
       rclpy.init(args=args)
       validator = SensorValidator()

       try:
           rclpy.spin(validator)
       except KeyboardInterrupt:
           pass
       finally:
           validator.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Step 5.2: Test the Validation Node

1. Make the script executable:
   ```bash
   chmod +x sensor_validator.py
   ```

2. Run the validation node (in a new terminal while simulation is running):
   ```bash
   ros2 run your_robot_perception sensor_validator
   ```

3. Observe the output to verify that sensor data is being received correctly.

## Exercise Deliverables

For this exercise, you should have:

1. **URDF Configuration Files**: Three separate files for LiDAR, Depth Camera, and IMU sensors
2. **Integration**: All sensors properly integrated into your robot URDF
3. **Launch File**: A launch file that starts the simulation with all sensors
4. **Validation Node**: A Python node that validates sensor data streams
5. **Verification**: Confirmed that all sensor topics are publishing data

## Assessment Questions

1. What are the key parameters that define a LiDAR sensor's performance?
2. How does the field of view affect depth camera performance?
3. What are the main sources of error in IMU measurements?
4. Why is it important to consider noise models in sensor simulation?
5. How would you modify the sensor configuration for outdoor vs. indoor environments?

## Troubleshooting Tips

- If sensor topics don't appear, check that plugin names are correct and that the Gazebo ROS packages are installed
- If you see "plugin not found" errors, verify that the plugin library exists and is properly named
- For IMU data that doesn't match expectations, check the coordinate frame alignment
- If simulation runs slowly, consider reducing sensor update rates or resolution

## Next Steps

After completing this exercise, you should proceed to:
1. Implement sensor fusion algorithms combining data from multiple sensors
2. Create perception nodes that process sensor data for navigation or mapping
3. Validate your sensor setup against real-world robot sensor characteristics