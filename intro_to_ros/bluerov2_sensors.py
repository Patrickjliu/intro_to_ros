#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, BatteryState, FluidPressure
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class BlueROV2Sensors(Node):
    """
    Node for subscribing to BlueROV2 sensor topics and monitoring battery voltage, IMU data, and pressure.
    """

    def __init__(self):
        """
        Initialize the BlueROV2Sensors node, create subscribers, and set up a timer.
        """
        super().__init__('bluerov2_sensors')
        self.battery = None
        self.imu = None
        self.diff_pressure = None
        self.static_pressure = None
        self.water_density = 1000 # kg/m^3 for freshwater
        self.gravity = 9.81 # m/s^2

        qos_profile=QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE
            )

        # Subscription to the battery topic
        self.battery_subscriber = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            qos_profile
        )

        # Subscription to the IMU topic
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile
        )
        # Subscription to the IMU static pressure topic
        self.static_pressure_subscriber = self.create_subscription(
            FluidPressure,
            '/mavros/imu/static_pressure',
            self.static_pressure_callback,
            qos_profile
        )

        # Subscription to the IMU diff pressure topic
        self.diff_pressure_subscriber = self.create_subscription(
            FluidPressure,
            '/mavros/imu/diff_pressure',
            self.diff_pressure_callback,
            qos_profile
        )

        # Timer to periodically check battery voltage
        self.timer = self.create_timer(5.0, self.check_battery_voltage)

    def static_pressure_callback(self, msg: FluidPressure):
        """
        Callback function for static pressure messages. Calculates and prints the depth.

        Parameters:
        msg (FluidPressure)
        """
        self.static_pressure = msg
        depth = (self.static_pressure.fluid_pressure - 101325) / (self.water_density * self.gravity)  # Subtracting atmospheric pressure (Pa)
        self.get_logger().info(f"Static Pressure: {msg.fluid_pressure} Pa, Calculated Depth: {depth:.2f} meters")

    def diff_pressure_callback(self, msg: FluidPressure):
        """
        Callback function for differential pressure messages. Logs the differential pressure.

        Parameters:
        msg (FluidPressure)
        """
        self.diff_pressure = msg
        self.get_logger().info(f"Diff Pressure: {msg.fluid_pressure} Pa")

    def battery_callback(self, msg: BatteryState):
        """
        Callback function for battery state messages. Logs the battery voltage.

        Parameters:
        msg (BatteryState)
        """
        self.battery = msg
        self.get_logger().info(f"Battery Voltage: {msg.voltage} V")

    def imu_callback(self, msg: Imu):
        """
        Callback function for IMU messages. Logs IMU data (orientation, angular velocity, linear acceleration).

        Parameters:
        msg (Imu)
        """
        self.imu = msg
        # self.get_logger().info(f"IMU (Orientation, Angular Velocity, Linear Acceleration): {msg.orientation}    {msg.angular_velocity}   {msg.linear_acceleration}")
        self.get_logger().info(f"IMU Orientation: {msg.orientation}")
        self.get_logger().info(f"IMU Angular Velocity: {msg.angular_velocity}")
        self.get_logger().info(f"IMU Linear Acceleration: {msg.linear_acceleration}")

    def check_battery_voltage(self):
        """
        Check the battery voltage and log a warning if the voltage is below 12V.
        """
        if self.battery is not None and self.battery.voltage < 12:
            self.get_logger().warn(f'Battery voltage is low: {self.battery.voltage}V')

def main():
    """
    Main function to initialize and spin the BlueROV2Sensors node.
    """
    rclpy.init()
    node = BlueROV2Sensors()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
