#include <limits>

#include <gtest/gtest.h>
#include "robot_demo/obstacle_utils.hpp"

TEST(ObstacleUtils, FindsNearestValidDistance)
{
    sensor_msgs::msg::LaserScan scan;
    scan.range_min = 0.1f;
    scan.range_max = 10.0f;
    scan.ranges = {3.0f, 2.0f, 5.0f};

    float result = 0.0f;

    EXPECT_TRUE(robot_demo::find_min_valid_range(scan, result));
    EXPECT_FLOAT_EQ(result, 2.0f);
}

TEST(ObstacleUtils, IgnoresInvalidDistances)
{
    sensor_msgs::msg::LaserScan scan;
    scan.range_min = 0.1f;
    scan.range_max = 10.0f;
    scan.ranges = {
        std::numeric_limits<float>::quiet_NaN(),
        0.05f,
        12.0f,
        3.0f
    };

    float result = 0.0f;

    EXPECT_TRUE(robot_demo::find_min_valid_range(scan, result));
    EXPECT_FLOAT_EQ(result, 3.0f);
}

TEST(ObstacleUtils, ReturnsFalseWhenAllDistancesInvalid)
{
    sensor_msgs::msg::LaserScan scan;
    scan.range_min = 0.1f;
    scan.range_max = 10.0f;
    scan.ranges = {
        std::numeric_limits<float>::quiet_NaN(),
        0.05f,
        12.0f
    };

    float result = 0.0f;

    EXPECT_FALSE(robot_demo::find_min_valid_range(scan, result));
}