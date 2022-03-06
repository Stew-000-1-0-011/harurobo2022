#pragma once

#include <std_msgs/UInt8.h>

#include "../stringlike_types.hpp"
#include "../topic.hpp"
#include "../config.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace Topics
        {
            using stepping_motor = CanTxTopic<StringlikeTypes::stepping_motor, std_msgs::UInt8, Config::CanId::Tx::stepping_motor>;
        }

        namespace SteppingMotor
        {
            enum SteppingMotor : std::uint8_t
            {
                open = 0,
                close,
                disable,
                enable
            };
        }
    }
}