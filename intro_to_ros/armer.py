#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool

class Armer(Node):
    """
    Node for arming and disarming the robot using the MAVROS arming service.
    """

    def __init__(self):
        """
        Initialize the Armer node, create a client for the arming service, and log startup.
        """
        super().__init__('armer')
        self.cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli = self.create_client(CommandBool, '/mavros/cmd/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        self.get_logger().info('Arming service is ready.')

    def send_request(self, arm: bool):
        """
        Send a request to arm or disarm the robot.

        Parameters:
        arm (bool): True to arm the robot, False to disarm.
        """
        req = CommandBool.Request()
        req.value = arm
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Request successful: {future.result().success}")
        else:
            self.get_logger().error(f"Request failed: {future.exception()}")

def main(args=None):
    """
    Main function to initialize and run the Armer node.
    """
    rclpy.init(args=args)
    node = Armer()
    node.send_request(True)  # Arm the robot

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally:
        node.send_request(False)  # Disarm the robot
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
