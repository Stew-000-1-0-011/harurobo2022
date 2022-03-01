#pragma once

#include <geometry_msgs/Twist.h>

#include "topic.hpp"
#include "stringlike_types.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace Topics
        {
            using body_twist = Topic<StringlikeTypes::body_twist, geometry_msgs::Twist>;
        }
    }
}