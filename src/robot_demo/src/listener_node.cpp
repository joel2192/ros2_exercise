#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ListenerNode : public rclcpp::Node
{
public:
  ListenerNode()
      : Node("listener_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "chatter",
        rclcpp::SensorDataQoS(),
        [this](std_msgs::msg::String::ConstSharedPtr msg)
        {
          RCLCPP_INFO(this->get_logger(), "Received: '%s'",
                      msg->data.c_str());
        });
  }

private:

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ListenerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}