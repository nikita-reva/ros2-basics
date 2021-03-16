#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from functools import partial


class CounterResetClientNode(Node):
    def __init__(self):
        super().__init__("counter_reset")
        self.call_reset_counter_server(True)
        self.call_reset_counter_server(True)
        self.call_reset_counter_server(False)

    def call_reset_counter_server(self, data):
        client = self.create_client(SetBool, "reset_counter")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the server to be up...")

        request = SetBool.Request()
        request.data = data

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_reset_counter, data=data))

    def callback_reset_counter(self, future, data):
        try:
            response = future.result()
            self.get_logger().info(str(response.message))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = CounterResetClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
