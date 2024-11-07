#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_state.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel")
    {
        pub_ = this->create_publisher<my_robot_interfaces::msg::LedState>("led_states", 10);
        server_ = this->create_service<my_robot_interfaces::srv::SetLed>("set_led", std::bind(&LedPanelNode::callbackSetLed, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Led node is running...");
    }

private:
    bool led_panel_[3] = {false, false, false};

    rclcpp::Publisher<my_robot_interfaces::msg::LedState>::SharedPtr pub_;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;

    void callbackSetLed(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                    const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
{
    if (!request || !response) {
        RCLCPP_ERROR(this->get_logger(), "Invalid request or response pointer");
        return; // Exit if pointers are null
    }

    int64_t led_number = request->led_number; // Get the requested LED number
    bool state = request->state; // Get the requested state (on/off)

    RCLCPP_INFO(this->get_logger(), "Received request to change LED %ld to %s", led_number, state ? "on" : "off");

    // Validate the LED number
    if (led_number < 0 || led_number >= 3) { // Change this if you have a different number of LEDs
        RCLCPP_ERROR(this->get_logger(), "Invalid LED number: %ld", led_number);
        response->success = false; // Set success to false if there’s an error
        return; // Exit if led_number is invalid
    }

    // Update the LED state
    led_panel_[led_number] = state;

    // Log the change
    RCLCPP_INFO(this->get_logger(), "LED %ld changed to %s", led_number, state ? "on" : "off");

    // Prepare the message to publish the LED states
    auto msg = my_robot_interfaces::msg::LedState();
    msg.led1 = led_panel_[0];
    msg.led2 = led_panel_[1];
    msg.led3 = led_panel_[2];

    // Check if the publisher is initialized and publish the message
    if (pub_) {
        pub_->publish(msg);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Publisher is not initialized");
    }

    response->success = true; // Set success to true if everything went well
}

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
