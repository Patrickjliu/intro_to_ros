#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import numpy as np
import matplotlib.pyplot as plt
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

# Import necessary functions from the previous problem set
from intro_to_ros.underwater_physics import simulate_auv2_motion, plot_auv2_motion

class PhysicsSim(Node):
    """
    Node for simulating AUV motion based on Pose2D messages.
    """

    def __init__(self):
        super().__init__('physics_sim')
        
        # Subscription to the Pose2D topic
        self.pose2d_subscriber = self.create_subscription(
            Pose2D,
            '/physics/pose2d',
            self.pose2d_callback,
            qos_profile=QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE
            )
        )

    def pose2d_callback(self, msg: Pose2D):
        """
        Callback function for Pose2D messages. Runs the simulate_auv2_motion function.
        
        Parameters:
        msg (Pose2D): The received Pose2D message
        """
        x0, y0, theta0 = msg.x, msg.y, msg.theta
        T = np.array([2.0, 1.0, 2.0, 1.0])
        alpha = np.pi / 4
        L = 0.3
        l = 0.2
        mass = 1.0
        dt = 0.1
        t_final = 10.0

        t, x, y, theta, v, omega, a = simulate_auv2_motion(
            T, alpha, L, l, mass, mass, dt, t_final, x0, y0, theta0
        )

        plot_auv2_motion(t, x, y, theta, v, omega, a)
        plt.savefig('physics_sim.png')
        self.get_logger().info('Plot saved as physics_sim.png')

def main(args=None):
    rclpy.init(args=args)
    node = PhysicsSim()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()