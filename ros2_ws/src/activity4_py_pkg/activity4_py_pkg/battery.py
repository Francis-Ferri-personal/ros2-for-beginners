#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.task import Future

import time
import random
from functools import partial

from my_robot_interfaces.srv import SetLed



class BattryNode(Node):
    def __init__(self):
        super().__init__("battery")
        self.battery_state_ = 100
        self.timer_empty = self.create_timer(4.0, self.drain_battery)
        self.client_ = self.create_client(SetLed, "set_led")

        while not self.client_.wait_for_service(
            1.0
        ):  # The 1.0 is to wait for service (1 second)
            self.get_logger().warn("Waiting for Server Add Two Ints...")

    def drain_battery(self):
        self.battery_state_ = 0
        # Send request to turn on led
        led_number = random.choice([0, 1, 2])
        state = random.choice([True, False])

        request = SetLed.Request()
        request.led_number = led_number
        request.state = state

        future = self.client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_set_led, led_number=led_number, state=state))

        time.sleep(6)
        self.battery_state_ = 100

    def callback_call_set_led(self, future: Future, led_number, state):
        try:
            response: SetLed.Response = future.result()
            success = response.success
            if success:
                self.get_logger().info(f"Led {led_number} turned {state}")
            else:
                self.get_logger().info(f"Problem to change state from {led_number}")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = BattryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
