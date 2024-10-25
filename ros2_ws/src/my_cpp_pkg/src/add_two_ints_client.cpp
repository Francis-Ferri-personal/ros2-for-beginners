#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("add_two_ints_client")
    {
        // You can improve this using a thread pool
        // thread_ = std::thread(std::bind(&AddTwoIntsClientNode::callbackAddTwoIntsService, this, 1, 4));
        threads_.push_back(std::thread(std::bind(&NumberCounterNode::callbackAddTwoIntsService, this, 1, 4)));
        threads_.push_back(std::thread(std::bind(&NumberCounterNode::callbackAddTwoIntsService, this, 2, 6)));
    }

private:
    // Create a thread to aboud stopping the loop
    // std::thread thread_;
    std::vector<std::thread> threads_;

    void callbackAddTwoIntsService(int a, int b)
    {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "waiting for the server to be up...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request -> a=a;
        request -> b=b;

        auto future = client -> async_send_request(request);

        // It will block until we have a response or exception
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this -> get_logger(), "%d + %d = %ld", a,b, response -> sum);

        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this -> get_logger(), "Service call feiled");
        }
        

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
