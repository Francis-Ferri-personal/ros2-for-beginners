#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.call_add_two_ints_servers(6, 7)

    def call_add_two_ints_servers(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(
            1.0
        ):  # The 1.0 is to wait for service (1 second)
            self.get_logger().warn("Waiting for Server Add Two Ints...")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)  # This is kind a promise in Python
        future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))

    def callback_call_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(f"{str(a)} + {str(b)} = {str(response.sum)}")
        except Exception as e:
            # El marcador %r se utiliza para representar el valor de una variable o expresión utilizando su representación con repr().
            # Si solo pusieras (e) sin la coma, Python lo interpretaría como una expresión entre paréntesis y no como una tupla.
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
