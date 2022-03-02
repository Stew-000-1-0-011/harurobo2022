#pragma once

#include <harurobo2022/Twist.h>

#include "../stringlike_types.hpp"
#include "../topic.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace Topics
        {
            using body_twist = Topic<StringlikeTypes::body_twist, harurobo2022::Twist>;
        }
    }
}