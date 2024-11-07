#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedState


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")
        self.led_panel = [False,False,False]
        self.server_ = self.create_service(SetLed, 'set_led', self.callback_set_led)
        self.led_states_publisher_ = self.create_publisher(LedState, 'led_states', 10)
        self.get_logger().info("Led Panel Node running...")

    def callback_set_led(self, request: SetLed.Request, response: SetLed.Response):
        led_number = request.led_number
        state = request.state
        self.led_panel[led_number] = state


        self.get_logger().info(f"Led {led_number} changed to {'on' if state else 'off'}")

        msg = LedState()
        msg.led1, msg.led2, msg.led3 = self.led_panel
        self.led_states_publisher_.publish(msg)

        response.success = True
        return response



def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
