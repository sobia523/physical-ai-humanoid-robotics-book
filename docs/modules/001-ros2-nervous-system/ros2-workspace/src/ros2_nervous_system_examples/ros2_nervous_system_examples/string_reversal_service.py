#!/usr/bin/env python3
"""
Service server example for ROS 2 fundamentals chapter.
This node provides a simple string reversal service.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger


class StringReversalService(Node):
    """
    StringReversalService - A service that reverses input strings.

    This node demonstrates the basic structure of a ROS 2 service server,
    including service creation and request handling.
    """

    def __init__(self):
        """
        Initialize the StringReversalService node.

        Sets up the service server to handle string reversal requests.
        """
        super().__init__('string_reversal_service')

        # Create a service that uses the Trigger service type
        # The callback function will be called when a request is received
        self.srv = self.create_service(
            Trigger,
            'reverse_string',
            self.reversal_callback
        )

        # Log initialization
        self.get_logger().info('String Reversal Service initialized')

    def reversal_callback(self, request, response):
        """
        Callback function for handling service requests.

        Args:
            request: The service request object
            response: The service response object to be filled

        Returns:
            The response object with the result
        """
        # For this example, we'll just send a confirmation
        # In a real implementation, we would reverse a string
        response.success = True
        response.message = f'Service called successfully at {self.get_clock().now()}'

        # Log the service call
        self.get_logger().info('String reversal service called')

        return response


def main(args=None):
    """
    Main function to run the StringReversalService node.

    Initializes ROS 2, creates the node, and spins to keep it running.
    """
    rclpy.init(args=args)

    string_reversal_service = StringReversalService()

    try:
        rclpy.spin(string_reversal_service)
    except KeyboardInterrupt:
        pass
    finally:
        string_reversal_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()