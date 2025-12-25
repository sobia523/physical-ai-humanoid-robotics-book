# ROS 2 Fundamentals

## Learning Objectives

By the end of this chapter, students will be able to:
- Define ROS 2 nodes and topics
- Explain the publisher/subscriber communication model
- Create basic ROS 2 nodes using rclpy
- Implement simple publisher and subscriber nodes

## Theory/Concept

### Introduction to ROS 2

The Robot Operating System 2 (ROS 2) is a flexible framework for writing robotic software, designed for production environments with improved security, real-time capabilities, and cross-platform support. ROS 2 uses the Data Distribution Service (DDS) as its communication middleware, providing a standardized publish-subscribe model for real-time, high-performance communication between distributed applications.

### Nodes and Topics

#### Nodes

A node is the fundamental building block of a ROS 2 program. It's a process that performs computation in the ROS environment. Nodes are organized into packages to form functional units. In ROS 2, nodes are implemented as objects that inherit from the `Node` class provided by the client library (rclpy for Python).

Key characteristics of nodes:
- Each node runs in its own process
- Nodes communicate with each other through messages
- Nodes can be written in different programming languages (Python, C++, etc.)
- Nodes can be launched and managed independently
- Each node has a unique name within the ROS 2 domain

The node lifecycle includes initialization, active execution, and cleanup. During initialization, nodes set up publishers, subscribers, services, and other communication interfaces. During active execution, nodes process messages and perform computations. During cleanup, nodes properly shut down and release resources.

#### Topics

Topics are communication channels through which ROS 2 nodes exchange messages of a specific type. The publish-subscribe pattern is used for asynchronous message passing. Publishers send messages to topics without knowledge of subscribers, and subscribers receive messages from topics without knowledge of publishers.

Topic characteristics:
- Unidirectional data flow from publishers to subscribers
- Asynchronous communication model
- Multiple publishers and subscribers can connect to the same topic
- Message types must be consistent across all publishers and subscribers
- Quality of Service (QoS) settings can be configured for reliability, durability, and other properties


### Publisher/Subscriber Model

The publisher-subscriber model is the core communication pattern in ROS 2. It enables decoupled, asynchronous communication between nodes. Publishers send messages to topics without knowing who will receive them, and subscribers receive messages from topics without knowing who sent them.

This decoupling provides benefits like flexibility, scalability, robustness, and reusability. The communication flow involves publishers sending messages to topics and subscribers receiving messages from topics.

### Hands-on Examples with rclpy

The Python client library for ROS 2 (rclpy) provides the interface between Python programs and the ROS 2 system. Key components include:
- `rclpy.init()`: Initializes the ROS 2 client library
- `Node`: Base class for creating ROS 2 nodes
- `create_publisher()`: Creates a publisher for sending messages
- `create_subscription()`: Creates a subscriber for receiving messages
- `rclpy.spin()`: Processes callbacks and keeps the node running
- `rclpy.shutdown()`: Cleans up and shuts down the client library

## Example/Hands-on Exercise

### Exercise: Creating Your First ROS 2 Publisher and Subscriber

In this exercise, you'll create a simple publisher that sends "Hello, World!" messages and a subscriber that receives and displays these messages.

#### Prerequisites
- ROS 2 Humble Hawksbill installed
- Basic Python knowledge
- Terminal/command line familiarity

#### Step 1: Create a Workspace and Package
1. Create a new workspace directory: `mkdir -p ~/ros2_ws/src`
2. Navigate to the source directory: `cd ~/ros2_ws/src`
3. Create a new package: `ros2 pkg create --build-type ament_python my_robot_examples`

#### Step 2: Create the Publisher Node
1. Navigate to your package directory: `cd my_robot_examples`
2. Create a new Python file: `touch my_robot_examples/hello_publisher.py`
3. Add the publisher code (as shown in the Code Demonstration section)

#### Step 3: Create the Subscriber Node
1. Create another Python file: `touch my_robot_examples/hello_subscriber.py`
2. Add the subscriber code (as shown in the Code Demonstration section)

#### Step 4: Update Setup Configuration
1. Add the new scripts to your setup.py file in the entry_points section
2. Build your package: `cd ~/ros2_ws && colcon build --packages-select my_robot_examples`
3. Source the setup: `source install/setup.bash`

#### Step 5: Run the Nodes
1. Open a terminal and run the publisher: `ros2 run my_robot_examples hello_publisher`
2. Open another terminal and run the subscriber: `ros2 run my_robot_examples hello_subscriber`
3. Observe the communication between the nodes

## Code Demonstration

### Publisher Node Example

```python
#!/usr/bin/env python3
"""
Publisher node example for ROS 2 fundamentals chapter.
This node publishes "Hello, World!" messages to a topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloPublisher(Node):
    """
    HelloPublisher - A node that publishes simple string messages.

    This node demonstrates the basic structure of a ROS 2 publisher,
    including node initialization, publisher creation, and timer-based
    message publishing.
    """

    def __init__(self):
        """
        Initialize the HelloPublisher node.

        Sets up the publisher and timer for periodic message publishing.
        """
        # Initialize the parent Node class with the node name
        super().__init__('hello_publisher')

        # Create a publisher that sends String messages to the 'hello_topic' topic
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)

        # Set up a timer to call the timer_callback method every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track message number
        self.i = 0

        # Log initialization
        self.get_logger().info('Hello Publisher node initialized')

    def timer_callback(self):
        """
        Callback function for the timer.

        This function is called periodically by the timer and publishes
        a "Hello, World!" message with a counter.
        """
        # Create a String message
        msg = String()
        msg.data = f'Hello, World! {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function to run the HelloPublisher node.

    Initializes ROS 2, creates the node, and spins to keep it running.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the publisher node
    hello_publisher = HelloPublisher()

    try:
        # Keep the node running and processing callbacks
        rclpy.spin(hello_publisher)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up resources
        hello_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Subscriber Node Example

```python
#!/usr/bin/env python3
"""
Subscriber node example for ROS 2 fundamentals chapter.
This node subscribes to messages from the publisher node.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloSubscriber(Node):
    """
    HelloSubscriber - A node that subscribes to string messages.

    This node demonstrates the basic structure of a ROS 2 subscriber,
    including node initialization, subscription creation, and message
    callback handling.
    """

    def __init__(self):
        """
        Initialize the HelloSubscriber node.

        Sets up the subscription to receive messages from the publisher.
        """
        # Initialize the parent Node class with the node name
        super().__init__('hello_subscriber')

        # Create a subscription that receives String messages from 'hello_topic'
        # The listener_callback function will be called when a message is received
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.listener_callback,
            10)  # QoS history depth

        # Prevent unused variable warning
        self.subscription

        # Log initialization
        self.get_logger().info('Hello Subscriber node initialized')

    def listener_callback(self, msg):
        """
        Callback function for handling incoming messages.

        This function is called whenever a message is received on the
        subscribed topic.

        Args:
            msg: The received String message
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to run the HelloSubscriber node.

    Initializes ROS 2, creates the node, and spins to keep it running.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the subscriber node
    hello_subscriber = HelloSubscriber()

    try:
        # Keep the node running and processing callbacks
        rclpy.spin(hello_subscriber)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up resources
        hello_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```


## References/Resources

1. Open Robotics. (2023). ROS 2 humble hawksbill documentation. *ROS Documentation*. https://docs.ros.org/en/humble/

2. Open Robotics. (2023). rclpy: Python Client Library for ROS 2. *ROS Documentation*. https://docs.ros.org/en/humble/p/rclpy/

3. Open Robotics. (2023). ROS 2 tutorials: Nodes and topics. *ROS Documentation*. https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html

4. Quigley, M., Gerkey, B., & Smart, W. (2015). *Programming robots with ROS: A practical introduction to the development of autonomous robots*. O'Reilly Media.

5. Nahidi, A., Sharma, S. K., & Xiao, J. (2018). ROS 2: Towards trustworthy robotic computing. In *2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)* (pp. 7294-7301). IEEE. https://doi.org/10.1109/IROS.2018.8593545

## Chapter Summary

This chapter introduced the fundamental concepts of ROS 2, including nodes and topics. We explored the publisher-subscriber communication model, which enables decoupled, asynchronous communication between nodes. The hands-on exercise demonstrated creating and running basic publisher and subscriber nodes using the rclpy Python client library.

Key takeaways from this chapter:
- Nodes are the fundamental building blocks of ROS 2 programs
- Topics enable asynchronous, one-way communication using the publish-subscribe pattern
- The rclpy library provides the interface between Python and ROS 2
- Quality of Service (QoS) settings allow customization of communication behavior

These fundamentals form the foundation for more advanced ROS 2 concepts covered in subsequent chapters, including Python agent integration and URDF modeling for humanoid robots.