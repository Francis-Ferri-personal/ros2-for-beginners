#include "rclcpp/rclcpp.hpp"
#include <random>
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery")
    {
        std::uniform_int_distribution<int64_t> distrib_led(0, 2);
        std::uniform_int_distribution<> distrib_bool(0, 1);
        battery_state_ = 100;
        timer_ = this->create_wall_timer(std::chrono::seconds(4), std::bind(&BatteryNode::callbackWaitFourSeconds, this));
        RCLCPP_INFO(this->get_logger(), "Battery node  is running");
    }

private:
    int battery_state_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread thread_;

    // Random
    std::mt19937 gen;
    std::uniform_int_distribution<> distrib_led;
    std::uniform_int_distribution<> distrib_bool;

    void callbackWaitFourSeconds()
    {
        battery_state_ = 0;

        // Send the request
        thread_ = std::thread(std::bind(&BatteryNode::callbackSetLed, this));

        // Wait for 6 seconds
        std::this_thread::sleep_for(std::chrono::seconds(6));
    }

    void callbackSetLed()
    {
        auto client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "waiting for the server to be up...");
        }
        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();

        int led_number = distrib_led(gen);
        bool state = (distrib_bool(gen) == 1);

        request->led_number = led_number;
        request->state = state;

        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            bool success = response->success;
            if (success)
            {
                std::string status = state ? "on" : "off";
                RCLCPP_INFO(this->get_logger(), "Led %d changed to %s", led_number, status.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "It was not posiible to change the state of %d", led_number);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call feiled");
        }
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
