# OOP is for Object Oriented Programming.

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_no_opp")
    # We do not need the spin
    client = node.create_client(AddTwoInts, "add_two_ints")
    while not client.wait_for_service(1.0): # The 1.0 is to wait for service (1 second)
        node.get_logger().warn("Waiting for Server Add Two Ints..")

    request = AddTwoInts.Request()
    request.a = 3
    request.b = 8

    future = client.call_async(request) # This is kind a promise in Python
    rclpy.spin_until_future_complete(node, future)
    
    try:
        response = future.result()
        node.get_logger().info(f"{str(request.a)} + {str(request.b)} = {str(response.sum)}")
    except Exception as e:
        # El marcador %r se utiliza para representar el valor de una variable o expresión utilizando su representación con repr().
        # Si solo pusieras (e) sin la coma, Python lo interpretaría como una expresión entre paréntesis y no como una tupla.
        node.get_logger().error("Service call failed %r" % (e,))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
