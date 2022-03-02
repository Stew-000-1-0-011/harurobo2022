#pragma once

#include <harurobo2022/Odometry.h>

#include "../stringlike_types.hpp"
#include "../topic.hpp"
#include "../config.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace Topics
        {
            using odometry = CanRxTopic<StringlikeTypes::odometry, harurobo2022::Odometry, Config::CanId::Rx::odometry>;
        }
    }
}