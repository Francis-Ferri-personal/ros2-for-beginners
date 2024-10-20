#include "rclcpp/rclcpp.hpp"
#include <random>
#include "example_interfaces/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        // Define the range for random integers
        std::uniform_int_distribution<> distrib(1, 100);

        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NumberPublisherNode::publishNumber, this));

        RCLCPP_INFO(this -> get_logger(), "Number publisher has been started.");
    }

private:
    std::mt19937 gen; // Motor de generación de números aleatorios
    std::uniform_int_distribution<> distrib;
    std::string name_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = distrib(gen);
        publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
