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
            using table_cloth_active = CanTxTopic<StringlikeTypes::table_cloth_active, std_msgs::UInt8, Config::CanId::Tx::table_cloth_active>;
            using table_cloth_command = CanTxTopic<StringlikeTypes::table_cloth_command, std_msgs::UInt8, Config::CanId::Tx::table_cloth_command>;
        }

        namespace TableClothActive
        {
            inline constexpr std::uint8_t disable = 0;
            inline constexpr std::uint8_t enable = 1;
        }
        
        namespace TableClothCommand
        {
            inline constexpr std::uint8_t push = 0xFF;
            inline constexpr std::uint8_t pull = 0x0;
        }
    }
}