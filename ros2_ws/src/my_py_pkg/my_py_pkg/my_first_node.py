#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")
        self.counter_ = 0
        self.get_logger().info("Hello ROS2!!!!!")
        self.create_timer(0.5, self.timer_callback) # The time is in seconds

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info(f"Hello {str(self.counter_)}")

def main(args=None):
    # Initialize communication
    """
        This is always necessary to use the other functionality
    """
    rclpy.init(args=args) 

    node = MyNode()

    # Keep it alive
    rclpy.spin(node)

    # Stop communication
    """
        All your nodes are destroyed automatically
    """
    rclpy.shutdown()

if __name__ == "__main__":
    main()