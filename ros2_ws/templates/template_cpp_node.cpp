#include "rclcpp/rclcpp.hpp"

class HardwareStatusPublisherNode : public rclcpp::Node // MODIFY NAME
{
public:
    HardwareStatusPublisherNode() : Node("node_name") // MODIFY NAME
    {
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
