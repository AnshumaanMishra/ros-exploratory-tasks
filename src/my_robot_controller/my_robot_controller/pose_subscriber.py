#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__("pose_subscriber")

        self.get_logger().info("draw_circle_node has been started")

        # Creating the subscriber:
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        
    def pose_callback(self, message: Pose):
        print(f"X = {message.x}, Y = {message.y}, theta = {message.theta}\n")

def main(args=None):
    rclpy.init(args=args)

    node = PoseSubscriberNode()
    rclpy.spin(node)

    rclpy.shutdown()