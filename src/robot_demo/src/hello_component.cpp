
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace robot_demo
{

class HelloComponent : public rclcpp::Node
{
public:
    explicit HelloComponent(
        const rclcpp::NodeOptions & options)
    : Node("hello_component", options)
    {
        publisher_ = create_publisher<std_msgs::msg::String>(
            "component_chatter", 10);

        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                std_msgs::msg::String msg;
                msg.data = "Hello from Component";

                publisher_->publish(msg);
                RCLCPP_INFO(get_logger(), "Published: %s",
                            msg.data.c_str());
            });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_demo

RCLCPP_COMPONENTS_REGISTER_NODE(robot_demo::HelloComponent)