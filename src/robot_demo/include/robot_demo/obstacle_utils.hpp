#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot_demo
{

inline bool find_min_valid_range(
    const sensor_msgs::msg::LaserScan & scan,
    float & min_distance)
{
    min_distance = std::numeric_limits<float>::infinity();

    for (const float range : scan.ranges)
    {
        if (std::isfinite(range) &&
            range >= scan.range_min &&
            range <= scan.range_max)
        {
            min_distance = std::min(min_distance, range);
        }
    }

    return std::isfinite(min_distance);
}

}  // namespace robot_demo