
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
    client_ = rclcpp_action::create_client<CountUntil>(
        this,
        "count_until");
  }

  void send_goal(int target)
  {
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

    options.goal_response_callback =
        [this](const GoalHandle::SharedPtr goal_handle)
    {
      if (!goal_handle)
      {
        RCLCPP_ERROR(
            this->get_logger(),
            "Goal rejected");
        rclcpp::shutdown();
        return;
      }

      RCLCPP_INFO(
          this->get_logger(),
          "Goal accepted: EXECUTING");

      // 收到 Goal 接受通知後，3 秒後要求取消
      cancel_timer_ = this->create_wall_timer(
          3s,
          [this, goal_handle]()
          {
            RCLCPP_INFO(
                this->get_logger(),
                "Sending cancel request");
            client_->async_cancel_goal(
                goal_handle,
                [this](
                    rclcpp_action::Client<CountUntil>::CancelResponse::SharedPtr response)
                {
                  RCLCPP_INFO(
                      this->get_logger(),
                      "Goals accepted for cancellation: %zu",
                      response->goals_canceling.size());
                });
            cancel_timer_->cancel();
          });
    };

    options.feedback_callback =
        [this](
            GoalHandle::SharedPtr,
            const std::shared_ptr<const CountUntil::Feedback> feedback)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Feedback: %d",
          feedback->progress);
    };

    options.result_callback =
        [this](const GoalHandle::WrappedResult &result)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Final result: %d",
          result.result->result);

      switch (result.code)
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(
            this->get_logger(),
            "Final state: SUCCEEDED");
        break;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(
            this->get_logger(),
            "Final state: CANCELED");
        break;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(
            this->get_logger(),
            "Final state: ABORTED");
        break;

      default:
        RCLCPP_INFO(
            this->get_logger(),
            "Final state: UNKNOWN");
        break;
      }

      rclcpp::shutdown();
    };

    client_->async_send_goal(goal_msg, options);
  }

private:
  rclcpp_action::Client<CountUntil>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CountClient>();
  node->send_goal(10);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}