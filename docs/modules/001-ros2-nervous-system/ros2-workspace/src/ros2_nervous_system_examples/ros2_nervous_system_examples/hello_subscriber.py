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