#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.name_ = "ONE"
        self.counter_ = 0

        self.subscriber_ = self.create_subscription(
            Int64, "number", self.callback_number, 10
        )

        self.publisher_ = self.create_publisher(Int64, "number_count", 10)

        self.get_logger().info(f"Number counter {self.name_} has been started!")

    def callback_number(self, msg):
        self.get_logger().info("Number received: " +  str(msg.data))

        self.counter_ += msg.data

        self.publish_number_count()

    def publish_number_count(self):
        msg = Int64()
        msg.data = self.counter_
        self.publisher_.publish(msg)
        self.get_logger().info("Counter: " + str(self.counter_))


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
