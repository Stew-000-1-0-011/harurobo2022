#pragma once

#include <std_msgs/UInt8.h>
#include "../stringlike_types.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace Topics
        {
            using state_topic = Topic<StringlikeTypes::state, std_msgs::UInt8>;
        }
    }
}