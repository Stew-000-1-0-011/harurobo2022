#pragma once

#include <std_msgs/Bool.h>
#include "../stringlike_types.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace Topics
        {
            using under_carriage_4wheel_active = Topic<StringlikeTypes::under_carriage_4wheel_active, std_msgs::Bool>;
        }
    }
}