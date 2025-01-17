#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/trigger.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("number_counter")
    {
        counter_ = 0;
        // Alwyas use ("") and not ('')
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10, std::bind(&BatteryNode::callbackNumber, this, std::placeholders::_1));

        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

        RCLCPP_INFO(this->get_logger(), "Number counter has been started");

        service_ = this->create_service<example_interfaces::srv::Trigger>("reset_counter", std::bind(&BatteryNode::callbackResetCounter, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Reset counter service has been started");
    }

private:
    std::string name_;
    std::int64_t counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr service_;

    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: %ld", msg->data);
        counter_ += msg->data;
        publishNumberCount();
        RCLCPP_INFO(this->get_logger(), "Counter: %ld", counter_);
    }

    void publishNumberCount()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = counter_;
        publisher_->publish(msg);
    }

    // Note: If we do not define the name of the first parameter we ignore it.
    void callbackResetCounter(const example_interfaces::srv::Trigger::Request::SharedPtr, const example_interfaces::srv::Trigger::Response::SharedPtr response)
    {
        this->counter_ = 0;

        response->success = true;
        response->message = "Counter reseted!";

        RCLCPP_INFO(this->get_logger(), "Counter reseted!");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
