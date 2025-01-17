#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class BlueROV2Controls(Node):
    def __init__(self):
        super().__init__('bluerov2_controls')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.publisher = self.create_publisher(
            OverrideRCIn,
            "/mavros/rc/override",
            qos_profile
        )

        # Subscription to the IMU topic
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile
        )

        self.cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        self.get_logger().info('Arming service is ready.')

        self.publisher_timer = self.create_timer(1.0, self.run_sequence)

    def run_sequence(self):
        rc_msg = OverrideRCIn()

        # Step 3: Rotate 180 degrees
        while True:
            rc_msg.channels = [1500, 1500, 1500, 1500, 1700, 1500, 1500, 1500, 1500, 1100, 1500, 0, 0, 0, 0, 0, 0, 0]
            self.publisher.publish(rc_msg)
            self.get_logger().info(f"Step 3: Rotate 180 - {rc_msg.channels}")

        self.destroy_node()

    def send_request(self, arm: bool):
        req = CommandBool.Request()
        req.value = arm
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Request successful: {future.result().success}")
        else:
            self.get_logger().error(f"Request failed: {future.exception()}")

    def imu_callback(self, msg: Imu):
        """
        Callback function for IMU messages. Logs IMU data (orientation, angular velocity, linear acceleration).

        Parameters:
        msg (Imu)
        """
        self.imu = msg
        # self.get_logger().info(f"IMU (Orientation, Angular Velocity, Linear Acceleration): {msg.orientation}    {msg.angular_velocity}   {msg.linear_acceleration}")
        # self.get_logger().info(f"IMU Orientation: {msg.orientation}")
        # self.get_logger().info(f"IMU Angular Velocity: {msg.angular_velocity}")
        self.get_logger().info(f"IMU Linear Acceleration: {msg.linear_acceleration}")

def main(args=None):
    rclpy.init(args=args)
    node = BlueROV2Controls()
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
