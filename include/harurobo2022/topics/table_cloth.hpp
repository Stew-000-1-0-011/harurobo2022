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
            using table_cloth = CanTxTopic<StringlikeTypes::table_cloth, std_msgs::UInt8, Config::CanId::Tx::table_cloth>;
        }

        namespace TableCloth
        {
            enum TableCloth : std::uint8_t
            {
                push = 0,
                pull
            };
        }
    }
}