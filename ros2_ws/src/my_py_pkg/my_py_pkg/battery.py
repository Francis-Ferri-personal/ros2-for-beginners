#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from functools import partial

from my_robot_interfaces.srv import SetLed

class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery")
        self.battery_state_ = "full"
        self.last_time_battery_state_changed_ = self.get_current_time_seconds()
        self.battery_timer_ = self.create_timer(0.1, self.check_battery_state)
        self.get_logger().info("Battery node has been staretd.")

    def get_current_time_seconds(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs / 1000000000.0

    def check_battery_state(self):
        time_now = self.get_current_time_seconds()
        if self.battery_state_ == "full":
            if time_now - self.last_time_battery_state_changed_ > 4.0:
                self.battery_state_ = "empty"
                self.get_logger().info("Battery is empty! Charging battery...")
                self.last_time_battery_state_changed_ = time_now
                self.call_set_led_servers(3, 1)
        else:
            if time_now - self.last_time_battery_state_changed_ > 6.0:
                self.battery_state_ = "full"
                self.get_logger().info("Battery is now full again.")
                self.last_time_battery_state_changed_ = time_now
                self.call_set_led_servers(3, 0)

    def call_set_led_servers(self, led_number, state):
        client = self.create_client(SetLed, "set_led")
        while not client.wait_for_service(
            1.0
        ):  # The 1.0 is to wait for service (1 second)
            self.get_logger().warn("Waiting for Server Set Led...")

        request = SetLed.Request()
        request.led_number = led_number
        request.state = state

        future = client.call_async(request)  # This is kind a promise in Python
        future.add_done_callback(partial(self.callback_call_set_number, led_number=led_number, state=state))

    def callback_call_set_number(self, future:Future, led_number, state):
        try:
            response: SetLed.Response  = future.result()
            self.get_logger().info(str(response.success))
        except Exception as e:
            # El marcador %r se utiliza para representar el valor de una variable o expresión utilizando su representación con repr().
            # Si solo pusieras (e) sin la coma, Python lo interpretaría como una expresión entre paréntesis y no como una tupla.
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
