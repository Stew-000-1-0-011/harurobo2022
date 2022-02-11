#pragma once

#include <std_msgs/Empty.h>

#include "harurobo2022/raw_data/raw_data_template.hpp"

namespace Harurobo2022
{
    template<>
    struct RawData<std_msgs::Empty>
    {
        using Message = std_msgs::Empty;

        RawData() = default;
        RawData(const RawData&) = default;
        RawData(RawData&&) = default;
        RawData& operator=(const RawData&) = default;
        RawData& operator=(RawData&&) = default;
        ~RawData() = default;


        constexpr RawData(const Message& data) noexcept
        {}

        operator Message() const noexcept
        {
            return {};
        }
    };
}
