#pragma once

#include <std_msgs/Bool.h>
#include "../stringlike_types.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace Topics
        {
            using auto_commander_active = Topic<StringlikeTypes::auto_commander_active, std_msgs::Bool>;
        }
    }
}