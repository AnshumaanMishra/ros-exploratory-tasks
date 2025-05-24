#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


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
        self.joint_su

        # TODO: Create odometry publisher. Message type is Odometry, topic can be set to
        #  /robot_odometry (not to clash with the existing Andino odometry topic) and qos to 10.

    def joint_states_callback(self, joint_states):
        # TODO: Read left and right wheel velocities from the JointState message

        # TODO: Get the elapsed time since last odometry calculation, delta time (dt), in seconds.
        #  Save current time to self.last_time

        # TODO: The wheel velocities we read from the joint_states message are angular
        #  joint velocities (rad/s). Convert them to linear velocities for each wheel by multiplying
        #  them with a wheel radius. Then calculate the robot's linear and angular velocities
        #  with the following formulas:
        #  linear velocity = (vel right + vel left) / 2.0
        #  angular velocity = (vel right - vel left) / wheel separation

        # TODO: Now that we know how much time has elapsed since the last calculation,
        #  what was robot's previous orientation angle (theta) and with what speed the
        #  robot has moved, we can calculate the new position for the robot. Find out how to
        #  calculate this for x-axis, y-axis and robot's orientation theta, and
        #  update the values in self.x, self.y and self.theta.

        # TODO: Create new Odometry message and populate stamp and frame IDs. The parent frame
        #  ID is "odom" and child frame ID is "base_link".

        # TODO: Add the updated robot's position and orientation to the Odometry message. 
        # Be careful, the Odometry message accepts the orientation in Quaternion notation!

        # TODO: Add the updated linear and angular velocities in the Odometry message

        # TODO: Publish the odometry message
        pass


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
