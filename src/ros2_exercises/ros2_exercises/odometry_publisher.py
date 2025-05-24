#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


# Code inserted from further
class PathVisualizer:
    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()

        # Lock the aspect ratio and set equal scaling for x and y
        self.ax.set_aspect('equal', adjustable='box')

        self.path_x, self.path_y = [], []

    def visualize(self, x, y):
        # Store position history
        self.path_x.append(x)
        self.path_y.append(y)

        # Plot path
        self.ax.clear()
        self.ax.plot(self.path_x, self.path_y, 'b-', label='Path')
        self.ax.plot(x, y, 'ro', label='Current Position')  # Mark current position
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Traversed Path')
        self.ax.legend()
        plt.draw()
        plt.pause(0.001)  # Short pause to update plot


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__("odometry_publisher")
        # Use these wheel parameters for odometry calculation on Andino
        self.wheel_separation = 0.137
        self.wheel_radius = 0.033

        # Initializing the robot position and time for robot odometry calculation
        self.x = 0.0  # Robot's position on x-axis
        self.y = 0.0  # Robot's position on y-axis
        self.theta = 0.0  # Robot's orientation angle
        self.last_time = self.get_clock().now()

        # TODO: Subscribe to /joint_states topic to listen to data from left and right wheels.
        #  The message type is JointState and quality of service can be set to 10.
        #  Use self.joint_states_callback as the subscription callback.

        self.joint_states_subscriber_ = self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)

        # TODO: Create odometry publisher. Message type is Odometry, topic can be set to
        #  /robot_odometry (not to clash with the existing Andino odometry topic) and qos to 10.
        self.odom_publisher_ = self.create_publisher(Odometry, "/robot_odometry", 10)

        self.path_visualiser_ = PathVisualizer()


def main(args=None):
    rclpy.init(args=args)

    odometry_publisher = OdometryPublisher()

    try:
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        pass

    odometry_publisher.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()