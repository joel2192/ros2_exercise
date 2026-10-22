
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/string.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleDemo : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifecycleDemo()
    : LifecycleNode("lifecycle_demo")
    {
    }

protected:
    CallbackReturn on_configure(
        const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "Configuring");

        publisher_ = create_publisher<std_msgs::msg::String>(
            "lifecycle_chatter", 10);

        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                if (!publisher_ || !publisher_->is_activated()) {
                    return;
                }

                std_msgs::msg::String msg;
                msg.data = "Hello from Lifecycle Node";

                publisher_->publish(msg);
                RCLCPP_INFO(get_logger(), "Published: %s",
                            msg.data.c_str());
            });

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(
        const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "Activating");
        publisher_->on_activate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "Deactivating");
        publisher_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "Cleaning up");
        timer_.reset();
        publisher_.reset();
        return CallbackReturn::SUCCESS;
    }

private:
    rclcpp_lifecycle::LifecyclePublisher<
        std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleDemo>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}