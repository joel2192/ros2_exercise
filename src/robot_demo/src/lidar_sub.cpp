
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarSub : public rclcpp::Node
{
public:
    LidarSub() : Node("lidar_sub")
    {
        // 測試時可將這行改成 rclcpp::QoS(10).reliable()
        auto qos = rclcpp::SensorDataQoS();

        subscription_ =
            create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", qos,
                [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
                    RCLCPP_INFO(
                        get_logger(),
                        "Received scan: %zu ranges",
                        msg->ranges.size());
                });
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSub>());
    rclcpp::shutdown();
    return 0;
}
