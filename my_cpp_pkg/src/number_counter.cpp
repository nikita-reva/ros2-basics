#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10,
            std::bind(&NumberCounterNode::callbackNumberCounter, this, std::placeholders::_1));

        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

        server_ = this->create_service<example_interfaces::srv::SetBool>("reset_counter",
                                                                         std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Number Counter has been started.");
    }

private:
    void callbackNumberCounter(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: %d", msg->data);
        counter_ += msg->data;
        RCLCPP_INFO(this->get_logger(), "Total: %d", counter_);
        publishNumber(counter_);
    }

    void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                              example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data)
        {
            counter_ = 0;
            response->success = true;
            response->message = "The counter has been reset.";
            RCLCPP_INFO(this->get_logger(), "The counter has been reset. Counter: %d", counter_);
        }
        else
        {
            response->success = false;
            response->message = "The counter has not been reset.";
            RCLCPP_INFO(this->get_logger(), "The counter has not been reset. Counter: %d", counter_);
        }
    }

    void publishNumber(int64_t counter)
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = std::int64_t(counter);
        publisher_->publish(msg);
    }

    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    int64_t counter_;

    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;

    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}