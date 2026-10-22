
#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class LidarSim : public rclcpp::Node
{
public:
    LidarSim() : Node("lidar_sim"), count_(0)
    {
        publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS());

        timer_ = create_wall_timer(100ms, [this]() {
            sensor_msgs::msg::LaserScan msg;

            msg.header.stamp = now();
            msg.header.frame_id = "laser_frame";
            msg.angle_min = -1.57f;
            msg.angle_max = 1.57f;
            msg.angle_increment = 0.01f;
            msg.range_min = 0.1f;
            msg.range_max = 10.0f;
            msg.ranges.resize(315, 2.0f);

            publisher_->publish(msg);
            RCLCPP_INFO(get_logger(), "Published scan #%d", ++count_);
        });
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSim>());
    rclcpp::shutdown();
    return 0;
}
