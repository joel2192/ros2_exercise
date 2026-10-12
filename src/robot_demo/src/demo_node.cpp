#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class DemoNode : public rclcpp::Node
{
public:
    DemoNode()
    : Node("demo_node"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "chatter",
            10
        );

        timer_ = this->create_wall_timer(
            1s,
            std::bind(&DemoNode::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();

        message.data = "Hello ROS2: " + std::to_string(count_);

        publisher_->publish(message);

        RCLCPP_INFO(
            this->get_logger(),
            "Published: '%s'",
            message.data.c_str()
        );

        count_++;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    int count_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DemoNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}