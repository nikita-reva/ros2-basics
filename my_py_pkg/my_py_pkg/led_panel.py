#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedStates


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")

        self.declare_parameter("led_states", [False, False, False])
        self.leds = self.get_parameter("led_states").value

        self.server_ = self.create_service(
            SetLed, "set_led", self.callback_set_led)
        self.publisher_ = self.create_publisher(LedStates, "led_states", 10)
        self.timer = self.create_timer(1.0, self.publish_led_states)
        self.get_logger().info("Led Panel has been started.")

    def callback_set_led(self, request, response):
        if request.state and not self.leds[request.led_number - 1]:
            self.leds[request.led_number - 1] = True
            self.get_logger().info("LED " + str(request.led_number) + " has been turned on.")
            response.success = True
            response.debug_message = "LED " + \
                str(request.led_number) + " has been turned on."
        elif not request.state and self.leds[request.led_number - 1]:
            self.leds[request.led_number - 1] = False
            self.get_logger().info("LED " + str(request.led_number) + " has been turned off.")
            response.success = True
            response.debug_message = "LED " + \
                str(request.led_number) + " has been turned off."
        elif request.state and self.leds[request.led_number - 1]:
            self.get_logger().info("LED " + str(request.led_number) + " is already on.")
            response.success = False
            response.debug_message = "LED " + \
                str(request.led_number) + " is already on."
        elif not request.state and not self.leds[request.led_number - 1]:
            self.get_logger().info("LED " + str(request.led_number) + " is already off.")
            response.success = False
            response.debug_message = "LED " + \
                str(request.led_number) + " is already off."

        return response

    def publish_led_states(self):
        msg = LedStates()
        msg.leds = self.leds
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
