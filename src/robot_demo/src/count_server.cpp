#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_demo/action/count_until.hpp"

using namespace std::chrono_literals;

class CountServer : public rclcpp::Node
{
public:
  using CountUntil = robot_demo::action::CountUntil;
  using GoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;

  CountServer()
      : Node("count_server")
  {
    this->declare_parameter<double>("count_delay", 1.0);

    // 原本的 Action Server 建立程式碼接在這裡

    action_server_ = rclcpp_action::create_server<CountUntil>(
        this,
        "count_until",

        // 收到 Goal
        [this](
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const CountUntil::Goal> goal)
        {
          (void)uuid;

          RCLCPP_INFO(
              this->get_logger(),
              "Received goal: %d",
              goal->target);

          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },

        // Client 要取消
        [this](
            const std::shared_ptr<GoalHandle> goal_handle)
        {
          RCLCPP_INFO(
              this->get_logger(),
              "Cancel request received");

          return rclcpp_action::CancelResponse::ACCEPT;
        },

        // 開始執行 Goal
        [this](
            const std::shared_ptr<GoalHandle> goal_handle)
        {
          std::thread{
              std::bind(
                  &CountServer::execute,
                  this,
                  std::placeholders::_1),
              goal_handle}
              .detach();
        });
  }

private:
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(
        this->get_logger(),
        "Executing goal...");

    const auto goal = goal_handle->get_goal();

    auto feedback =
        std::make_shared<CountUntil::Feedback>();

    auto result =
        std::make_shared<CountUntil::Result>();

    int current = 0;
    double delay = this->get_parameter("count_delay").as_double();

    
    while (current < goal->target)
    {
      // 檢查是否收到取消請求
      if (goal_handle->is_canceling())
      {
        result->result = current;
        goal_handle->canceled(result);

        RCLCPP_INFO(
            this->get_logger(),
            "Goal canceled at: %d",
            current);

        return;
      }

      current++;

      feedback->progress = current;
      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(
          this->get_logger(),
          "Progress: %d / %d",
          current,
          goal->target);

      std::this_thread::sleep_for(
          std::chrono::duration<double>(delay));
    }
    result->result = current;

    goal_handle->succeed(result);

    RCLCPP_INFO(
        this->get_logger(),
        "Goal completed: %d",
        result->result);
  }

  rclcpp_action::Server<CountUntil>::SharedPtr action_server_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CountServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}