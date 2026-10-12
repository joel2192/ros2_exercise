#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_demo/action/count_until.hpp"

using namespace std::chrono_literals;

class CountClient : public rclcpp::Node
{
public:
    using CountUntil = robot_demo::action::CountUntil;
    using GoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;

    CountClient()
    : Node("count_client")
    {
        client_ =
            rclcpp_action::create_client<CountUntil>(
                this,
                "count_until");
    }

    void send_goal(int target)
    {
        // 等待 Action Server
        if (!client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Action server not available");

            return;
        }

        auto goal_msg = CountUntil::Goal();
        goal_msg.target = target;

        RCLCPP_INFO(
            this->get_logger(),
            "Sending goal: %d",
            target);

        rclcpp_action::Client<CountUntil>::SendGoalOptions options;

        // 收到 Feedback
        options.feedback_callback =
            [this](
                GoalHandle::SharedPtr,
                const std::shared_ptr<const CountUntil::Feedback>
                    feedback)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Feedback: %d",
                    feedback->progress);
            };

        // 收到 Result
        options.result_callback =
            [this](
                const GoalHandle::WrappedResult & result)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Result: %d",
                    result.result->result);

                rclcpp::shutdown();
            };

        client_->async_send_goal(
            goal_msg,
            options);
    }

private:
    rclcpp_action::Client<CountUntil>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CountClient>();

    node->send_goal(10);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}