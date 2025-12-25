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