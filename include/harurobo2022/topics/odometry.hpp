#pragma once

#include <std_msgs/Float32.h>

#include "../stringlike_types.hpp"
#include "../topic.hpp"
#include "../config.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace Topics
        {
            using odometry_x = CanRxTopic<StringlikeTypes::odometry_x, std_msgs::Float32, Config::CanId::Rx::odometry_x>;
            using odometry_y = CanRxTopic<StringlikeTypes::odometry_y, std_msgs::Float32, Config::CanId::Rx::odometry_y>;
            using odometry_yaw = CanRxTopic<StringlikeTypes::odometry_yaw, std_msgs::Float32, Config::CanId::Rx::odometry_yaw>;
        }
    }
}