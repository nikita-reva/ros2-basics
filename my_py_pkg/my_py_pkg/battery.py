#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from functools import partial


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery")
        self.client = self.create_client(SetLed, "set_led")
        self.battery_full_ = True
        self.last_time_battery_state_changed_ = self.get_current_time_seconds()
        self.battery_timer_ = self.create_timer(0.1, self.check_battery_state)

    def check_battery_state(self):
        time_now = self.get_current_time_seconds()
        if self.battery_full_:
            if time_now - self.last_time_battery_state_changed_ > 4.0:
                self.get_logger().warn("Battery is empty.")
                self.call_set_led_server(3, True)
                self.last_time_battery_state_changed_ = time_now
                self.battery_full_ = False

        else:
            if time_now - self.last_time_battery_state_changed_ > 6.0:
                self.get_logger().warn("Battery is full.")
                self.call_set_led_server(3, False)
                self.last_time_battery_state_changed_ = time_now
                self.battery_full_ = True

    def get_current_time_seconds(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs / 1000000000.0

    def call_set_led_server(self, led_number, state):
        while not self.client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Set LEDs Server...")

        request = SetLed.Request()
        request.led_number = led_number
        request.state = state

        future = self.client.call_async(request)

        future.add_done_callback(
            partial(self.callback_call_set_led, led_number=led_number, state=state))

    def callback_call_set_led(self, future, led_number, state):
        try:
            response = future.result()
            self.get_logger().info(str(response.debug_message) + "\n")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
