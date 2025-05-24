#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle")

        self.get_logger().info("draw_circle_node has been started")

        # Creating the publisher:
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Create a timer callback
        self.timer_ = self.create_timer(timer_period_sec=0.5, callback=self.send_velocity_command)
        
    def send_velocity_command(self):

        # Create the Twist message
        message = Twist()

        # Set the attributes
        message.linear.x = 2.0
        message.angular.z = 1.0

        # Publish the message
        self.cmd_vel_pub_.publish(message)



def main(args=None):
    rclpy.init(args=args)

    node = DrawCircleNode()
    rclpy.spin(node)

    rclpy.shutdown()

