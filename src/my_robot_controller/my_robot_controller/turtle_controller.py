#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller_node")
        self.toggle = False 

        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        
        self.get_logger().info("turtle_controller_node has been started")

    def pose_callback(self, pose: Pose):
        command = Twist()
        if(pose.x > 9 or pose.x < 2 or pose.y > 9 or pose.y < 2):
            command.linear.x = 1.0
            command.angular.z = 1.0
        else:
            command.linear.x = 5.0
            command.angular.z = 0.0

        if(pose.x > 5.5 and not self.toggle):
            self.call_set_pen_service(255, 0, 0, 5, 0)
            self.toggle = True
        elif(pose.x < 5.5 and self.toggle):
            self.call_set_pen_service(0, 0, 255, 5, 0)
            self.toggle = False
        self.cmd_vel_publisher_.publish(command)

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1):
            self.get_logger().warn("Warning: Service not running")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request=request)
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))



def main(args=None):
    rclpy.init(args=args)

    node = TurtleControllerNode()
    rclpy.spin(node)

    rclpy.shutdown()