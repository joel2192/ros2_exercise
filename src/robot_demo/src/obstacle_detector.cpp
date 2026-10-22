
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "robot_demo/obstacle_utils.hpp"

using std::placeholders::_1;

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector()
      : Node("obstacle_detector")
  {
    declare_parameter<double>("warning_distance", 1.0);

    distance_pub_ =
        create_publisher<std_msgs::msg::Float32>(
            "/obstacle_distance", 10);

    alert_pub_ =
        create_publisher<std_msgs::msg::Bool>(
            "/obstacle_alert", 10);

    scan_sub_ =
        create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(
                &ObstacleDetector::scan_callback,
                this, _1));

    RCLCPP_INFO(
        get_logger(), "Obstacle detector started");
  }

private:
  void scan_callback(
      sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    float min_distance = 0.0f;

    if (!robot_demo::find_min_valid_range(*scan, min_distance))
    {
      RCLCPP_WARN(
          get_logger(), "No valid LiDAR measurements");
      return;
    }

    if (!std::isfinite(min_distance))
    {
      RCLCPP_WARN(
          get_logger(), "No valid LiDAR measurements");
      return;
    }

    const double threshold =
        get_parameter("warning_distance").as_double();

    std_msgs::msg::Float32 distance_msg;
    distance_msg.data = min_distance;
    distance_pub_->publish(distance_msg);

    std_msgs::msg::Bool alert_msg;
    alert_msg.data = min_distance < threshold;
    alert_pub_->publish(alert_msg);

    RCLCPP_INFO(
        get_logger(),
        "Nearest obstacle: %.2f m | Alert: %s",
        min_distance,
        alert_msg.data ? "YES" : "NO");
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_sub_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      distance_pub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      alert_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
