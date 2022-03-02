#pragma once

#include <std_msgs/UInt8.h>
#include <harurobo2022/Twist.h>
#include <harurobo2022/Odometry.h>

#include "config.hpp"
#include "topic.hpp"
#include "stringlike_types.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace Topics
        {
            using body_twist = Topic<StringlikeTypes::body_twist, harurobo2022::Twist>;
            using odometry = CanRxTopic<StringlikeTypes::odometry, harurobo2022::Odometry, Config::CanId::Rx::odometry>;
            using state_topic = Topic<StringlikeTypes::state, std_msgs::UInt8>;
        }
    }
}