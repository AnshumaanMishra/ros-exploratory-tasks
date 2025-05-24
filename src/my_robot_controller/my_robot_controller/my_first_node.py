#!/usr/bin/env python3
# Import the main dependeny, rclpy
import rclpy

# Import the parent class for nodes
from rclpy.node import Node

# Helper Functions
def generator():
    i = 0
    while 1:
        i += 1
        yield i

gen1 = generator()
# Node must inherit from class Node
class MyNode(Node):
    # Constructor:
    def __init__(self):
        # Initialise superclass with Node Name, 
        super().__init__("first_node")

        # Log some text onto the output console
        self.get_logger().info("Hello From ROS2")

        # Timer in ROS2 that executes the callback after the specified time. 
        self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)
    
    # To display content, say, every second:
    def timer_callback(self):
        self.get_logger().info(f"Hello, {next(gen1)}")

def main(args = None):
    # Initialise rclpy communications
    rclpy.init(args=args)

    node = MyNode()


    # At this point, the node extis right after executing the logging
    # To make the node continue forever to enavle all callbacks be run
    rclpy.spin(node) 

    # Shut Down rclpy communications
    rclpy.shutdown()

# Only needed if we need to run it as a python file without using ros2 run
if __name__== "__main__":
    main()

