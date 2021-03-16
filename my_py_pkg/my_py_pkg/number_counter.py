#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")

        self.counter_ = 0
        self.subscriber_ = self.create_subscription(
            Int64, "number", self.callback_number_received, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.server_ = self.create_service(
            SetBool, "reset_counter", self.callback_reset_counter)
        self.get_logger().info("Number Counter has been started")

    def callback_number_received(self, msg):
        number_received = msg.data
        self.get_logger().info("Received: " + str(number_received))
        self.counter_ += number_received
        self.publish_count(self.counter_)

    def callback_reset_counter(self, request, response):
        if request.data:
            self.counter_ = 0
            response.success = True
            response.message = "The counter has been reset."
            self.get_logger().info("The counter has been reset.")
        else:
            response.success = False
            response.message = "The counter has not been reset."
            self.get_logger().info("The counter has not been reset.")

        return response

    def publish_count(self, num):
        msg = Int64()
        msg.data = num
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
