# Python Agents Bridging to ROS 2

## Learning Objectives

By the end of this chapter, professionals will be able to:
- Connect Python-based AI agents to ROS controllers using rclpy
- Implement autonomous robot actions through Python scripts
- Design node-based architectures for AI agent integration
- Use publisher/subscriber patterns for data exchange between agents and ROS
- Create service clients for request/response interactions with ROS services
- Implement action clients for goal-oriented robot behaviors

## Theory/Concept

### Introduction to Python Agents in ROS 2

Python agents are intelligent software components that can perceive their environment, make decisions, and take actions to achieve specific goals. In the context of ROS 2, Python agents serve as the bridge between high-level AI algorithms and the ROS ecosystem, enabling autonomous robot control and decision-making capabilities.

The integration of Python agents with ROS 2 allows for:

- **High-level decision making**: Python's rich ecosystem of AI libraries (TensorFlow, PyTorch, scikit-learn) can be used to create intelligent behaviors
- **Simplified integration**: Python's ease of use makes it ideal for rapid prototyping and development of agent-based systems
- **Flexibility**: Agents can be designed to work with various robot platforms and sensor configurations
- **Real-time capabilities**: ROS 2's real-time features enable responsive agent behaviors

### Agent Architecture in ROS 2

Python agents in ROS 2 typically follow a node-based architecture where the agent is implemented as a ROS node. This node can:

- Subscribe to sensor data topics to perceive the environment
- Publish commands to control topics to act on the environment
- Use services for synchronous interactions (e.g., requesting robot pose)
- Use actions for goal-oriented behaviors (e.g., navigation goals)

The agent's decision-making process typically involves:
1. Perception: Gathering information from sensors and other sources
2. Reasoning: Processing information to make decisions
3. Action: Sending commands to effectors or other ROS nodes

### Connecting AI Agents to ROS Controllers

The connection between Python agents and ROS controllers involves several key components:

#### Communication Patterns
- **Publisher/Subscriber**: For continuous data exchange (sensor readings, status updates)
- **Services**: For request/response interactions (querying robot state, triggering specific operations)
- **Actions**: For goal-oriented behaviors with feedback (navigation, manipulation tasks)

#### Message Types and Interfaces
Python agents must use appropriate ROS message types for communication:
- `sensor_msgs`: For sensor data (laser scans, images, IMU data)
- `geometry_msgs`: For pose, position, and velocity information
- `nav_msgs`: For navigation-related messages (path planning, odometry)
- Custom message types for domain-specific data


### Publisher/Subscriber Pattern for Agent-ROS Communication

The publisher/subscriber pattern is essential for continuous data exchange between agents and ROS. This pattern allows agents to:

- **Subscribe to sensor data**: Listen to topics like `/laser_scan`, `/camera/image_raw`, `/imu/data`
- **Publish control commands**: Send messages to topics like `/cmd_vel`, `/joint_commands`
- **Exchange state information**: Share agent status with other nodes

For example, an agent might subscribe to sensor data to make decisions and publish velocity commands to control a robot's movement.



## Example/Hands-on Exercise

### Exercise: Creating an AI Agent for Robot Navigation

In this exercise, you'll create an AI agent that can navigate a robot through a simple environment using ROS 2 and rclpy.

#### Prerequisites
- ROS 2 Humble Hawksbill installed
- Basic Python knowledge
- Understanding of ROS 2 fundamentals (topics, services, actions)

#### Step 1: Create the AI Agent Package
1. Create a new workspace: `mkdir -p ~/ros2_ws/src`
2. Navigate to the source directory: `cd ~/ros2_ws/src`
3. Create a new package: `ros2 pkg create --build-type ament_python ai_robot_agent`

#### Step 2: Implement the AI Agent Node
Create the main agent file in `ai_robot_agent/ai_robot_agent/navigation_agent.py` with the following content:

```python
#!/usr/bin/env python3
"""
Navigation Agent for ROS 2
This agent demonstrates autonomous navigation using sensors and actions.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose


class NavigationAgent(Node):
    """
    NavigationAgent - An AI agent that navigates autonomously.
    """

    def __init__(self):
        """
        Initialize the navigation agent.
        """
        super().__init__('navigation_agent')

        # Subscribe to laser scan data for obstacle detection
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Publisher for emergency stop commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Agent state variables
        self.obstacle_detected = False
        self.agent_mode = 'IDLE'  # IDLE, NAVIGATING, AVOIDING
        self.safety_distance = 0.6  # meters

        # Timer for decision making
        self.timer = self.create_timer(0.2, self.agent_decision_loop)

        self.get_logger().info('Navigation Agent initialized')

    def laser_callback(self, msg):
        """
        Process laser scan data for obstacle detection.
        """
        if len(msg.ranges) > 0:
            # Check for obstacles in the front 90-degree field
            front_ranges = msg.ranges[:len(msg.ranges)//8] + msg.ranges[-len(msg.ranges)//8:]
            min_distance = min([r for r in front_ranges if r > 0 and not r > 10], default=10)

            self.obstacle_detected = min_distance < self.safety_distance

    def agent_decision_loop(self):
        """
        Main decision-making loop for the agent.
        """
        if self.agent_mode == 'IDLE':
            # Example: Navigate to a fixed goal when idle
            self.navigate_to_goal(2.0, 2.0, 0.0)
        elif self.obstacle_detected and self.agent_mode == 'NAVIGATING':
            # Switch to avoiding mode
            self.agent_mode = 'AVOIDING'
            self.emergency_stop()
            self.get_logger().warn('Obstacle detected, stopping navigation')
        elif not self.obstacle_detected and self.agent_mode == 'AVOIDING':
            # Resume navigation if obstacle is cleared
            self.agent_mode = 'NAVIGATING'

    def navigate_to_goal(self, x, y, theta):
        """
        Navigate to a goal position using the navigation action.
        """
        self.nav_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta

        self.agent_mode = 'NAVIGATING'

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback
        )

        future.add_done_callback(self.navigation_result)

    def navigation_feedback(self, feedback_msg):
        """
        Handle navigation feedback.
        """
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance to goal: {distance:.2f}m')

    def navigation_result(self, future):
        """
        Handle navigation result.
        """
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Navigation goal accepted')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.navigation_completed)

    def navigation_completed(self, future):
        """
        Handle completion of navigation task.
        """
        result = future.result().result
        self.agent_mode = 'IDLE'
        self.get_logger().info('Navigation completed')

    def emergency_stop(self):
        """
        Send emergency stop command.
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    """
    Main function to run the navigation agent.
    """
    rclpy.init(args=args)
    agent = NavigationAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### Step 3: Update Package Configuration
1. Add the navigation agent script to your `setup.py` file in the `entry_points` section
2. Build your package: `cd ~/ros2_ws && colcon build --packages-select ai_robot_agent`
3. Source the setup: `source install/setup.bash`

#### Step 4: Run the Agent
1. Make sure you have a robot simulation running with navigation capabilities
2. Run the agent: `ros2 run ai_robot_agent navigation_agent`

## Code Demonstration



## References/Resources

1. Open Robotics. (2023). Using Python with ROS 2. *ROS Documentation*. https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Python-With-ROS2.html

2. Quigley, M., Gerkey, B., & Smart, W. (2015). *Programming robots with ROS: A practical introduction to the development of autonomous robots*. O'Reilly Media. (Chapter on Python integration)

3. Russell, S., & Norvig, P. (2020). *Artificial intelligence: A modern approach* (4th ed.). Pearson. (Chapter on agent architectures)

4. Kortenkamp, D., Bonasso, R. P., & Murphy, R. (Eds.). (1998). *AI-based mobile robots: Case studies of successful robot systems*. MIT Press. (Chapter on agent-based architectures)

5. Ferrein, A., & Lakemeyer, G. (2008). Using grounded cognitive simulation for building robot controllers. *KI-Künstliche Intelligenz*, 22(2), 109-114. https://doi.org/10.1007/s13218-008-0018-7

## Chapter Summary

This chapter explored the integration of Python-based AI agents with ROS 2 systems. We covered the fundamental concepts of connecting AI agents to ROS controllers, including node-based architectures, publisher/subscriber patterns for continuous data exchange, service clients for request/response interactions, and action clients for goal-oriented behaviors.

Key takeaways from this chapter:
- Python agents can serve as intelligent intermediaries between AI algorithms and ROS systems
- Node-based architecture allows agents to integrate seamlessly with ROS communication patterns
- Service clients enable synchronous interactions with ROS services for specific operations
- Action clients provide a robust mechanism for implementing goal-oriented behaviors with feedback
- Proper agent design involves state management, decision-making loops, and safety considerations

The hands-on exercise demonstrated creating a navigation agent that can perceive its environment, make decisions, and execute actions through ROS 2. The code examples showed practical implementations of the concepts discussed, providing a foundation for developing more sophisticated AI agents for robotic applications.

These agent integration techniques form the basis for advanced robotic systems where AI capabilities are seamlessly integrated with ROS 2's communication and control infrastructure.