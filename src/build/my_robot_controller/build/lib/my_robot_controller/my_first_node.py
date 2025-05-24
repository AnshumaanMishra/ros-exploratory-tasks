#!/usr/bin/env python3
# Import the main dependeny, rclpy
import rclpy

# Import the parent class for nodes
from rclpy.node import Node

# Node must inherit from class Node
class MyNode(Node):
    # Constructor:
    def __init__(self):
        # Initialise superclass with Node Name, 
        super().__init__("first_node")

        # Log some text onto the output console
        self.get_logger().info("Hello From ROS2")

def main(args = None):
    # Initialise rclpy communications
    rclpy.init(args=args)

    node = MyNode()

    # Shut Down rclpy communications
    rclpy.shutdown()


if __name__== "__main__":
    main()

