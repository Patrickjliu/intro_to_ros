#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pid import PID
from sensor_msgs.msg import FluidPressure
import numpy as np

class DepthController(Node):
    """
    ROS2 Node for controlling depth using PID controller and MAVLink.
    """

    def __init__(self):
        """
        Initialize the DepthController node, set up MAVLink connection, and configure subscriptions and mode.
        """
        super().__init__('depth_controller')

        # Initialize PID controller with placeholder values
        self.pid = PID(kp=1.0, ki=0.1, kd=0.05, setpoint=0.5)

        # Subscription to pressure topics
        self.diff_pressure_subscriber = self.create_subscription(
            FluidPressure,
            '/mavros/imu/diff_pressure',
            self.diff_pressure_callback,
            10
        )
        self.static_pressure_subscriber = self.create_subscription(
            FluidPressure,
            '/mavros/imu/static_pressure',
            self.static_pressure_callback,
            10
        )

        self.last_time = self.get_clock().now()

        # Initialize pressure variables
        self.static_pressure = None
        self.diff_pressure = None

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        """
        Set the PWM value for a specific RC channel.

        Args:
            channel_id (int): The RC channel ID (1-18).
            pwm (int): The PWM value (1100-1900).
        """
        if channel_id < 1 or channel_id > 8:
            self.get_logger().error("Channel DNE.")
            return

        rc_channel_values = [65535] * 8
        rc_channel_values[channel_id - 1] = pwm
        self.mav.mav.rc_channels_override_send(
            self.mav.target_system,
            self.mav.target_component,
            *rc_channel_values
        )

    def set_vertical_power(self, power=0):
        """
        Set the vertical power for the thruster.

        Args:
            power (int): Vertical power value (-100 to 100).
        """
        if power < -100 or power > 100:
            self.get_logger().warn("Power value out of range. Clipping...")
            power = np.clip(power, -100, 100)

        power = int(power)
        self.set_rc_channel_pwm(3, 1500 + power * 5)

    def static_pressure_callback(self, msg: FluidPressure):
        """
        Callback function for static pressure messages. Updates static pressure.

        Args:
            msg (FluidPressure): Static pressure message.
        """
        self.static_pressure = msg
        self.compute_and_control_depth()

    def diff_pressure_callback(self, msg: FluidPressure):
        """
        Callback function for differential pressure messages. Updates differential pressure.

        Args:
            msg (FluidPressure): Differential pressure message.
        """
        self.diff_pressure = msg
        self.compute_and_control_depth()

    def compute_and_control_depth(self):
        """
        Compute the depth from pressure readings and adjust the vertical power using PID control.
        """
        if self.static_pressure is None or self.diff_pressure is None:
            self.get_logger().warn("Static or differential pressure data not available.")
            return

        # Calculate depth from pressures
        atm = 101325  # Atmospheric pressure in Pascals
        water_density = 1000  # kg/m^3 for freshwater
        gravity = 9.81  # m/s^2
        static_pressure = self.static_pressure.fluid_pressure

        depth = (static_pressure - atm) / (water_density * gravity)
        self.get_logger().info(f"Static Pressure: {static_pressure} Pa, Calculated Depth: {depth:.2f} meters")

        # Calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            self.get_logger().warn("Time difference is zero or negative. Skipping update.")
            return

        # Get control signal from PID
        control_signal = self.pid.update(depth, dt)

        # Set vertical power
        self.set_vertical_power(-control_signal)  # Negative because of thruster direction

        self.get_logger().info(f"Depth: {depth}, Control: {control_signal}")

def main(args=None):
    """
    Main function to initialize and spin the DepthController node.
    """
    rclpy.init(args=args)
    depth_controller = DepthController()

    try:
        rclpy.spin(depth_controller)
    except KeyboardInterrupt:
        depth_controller.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally:
        depth_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()