#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random

from example_interfaces.msg import Int64

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.name_ = 'ONE'
        self.publisher_ = self.create_publisher(Int64, 'number', 10)

        self.timer_ = self.create_timer(0.5, self.publishNumber)
        self.get_logger().info(f"Number publisher node {self.name_} running!")


    def publishNumber(self):
        msg = Int64()
        msg.data = random.randint(1, 100)
        self.publisher_.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
