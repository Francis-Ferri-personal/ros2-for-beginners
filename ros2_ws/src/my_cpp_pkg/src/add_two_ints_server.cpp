#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class HardwareStatusPublisherNode : public rclcpp::Node
{
public:
    HardwareStatusPublisherNode() : Node("add_two_ints_server")
    {
        server = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", std::bind(&HardwareStatusPublisherNode::callbackAddTwoInts, this, _1, _2));

        RCLCPP_INFO(this -> get_logger(), "Service server has been started");
    }

private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server;

    void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                            const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);
        // You do not need to return the response
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
