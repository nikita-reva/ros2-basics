#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class CounterResetClientNode : public rclcpp::Node
{
public:
    CounterResetClientNode() : Node("counter_reset")
    {
        threads_.push_back(std::thread(std::bind(&CounterResetClientNode::callCounterResetService, this, true)));
        threads_.push_back(std::thread(std::bind(&CounterResetClientNode::callCounterResetService, this, false)));
        threads_.push_back(std::thread(std::bind(&CounterResetClientNode::callCounterResetService, this, true)));
        threads_.push_back(std::thread(std::bind(&CounterResetClientNode::callCounterResetService, this, true)));
        threads_.push_back(std::thread(std::bind(&CounterResetClientNode::callCounterResetService, this, false)));
    }

    void callCounterResetService(bool data)
    {
        auto client = this->create_client<example_interfaces::srv::SetBool>("reset_counter");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waitung for the server to be up...");
        }

        auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
        request->data = data;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

private:
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CounterResetClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}