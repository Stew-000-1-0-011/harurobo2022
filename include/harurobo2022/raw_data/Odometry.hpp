#pragma once
#include "harurobo2022/raw_data/raw_data_template.hpp"
#include <harurobo2022/Odometry.h>

namespace Harurobo2022
{
    template<>
    struct RawData<harurobo2022::Odometry>
    {
        using Message = harurobo2022::Odometry;

        float pos_x{};
        float pos_y{};
        float rot_z{};

        RawData() = default;
        RawData(const RawData&) = default;
        RawData(RawData&&) = default;
        RawData& operator=(const RawData&) = default;
        RawData& operator=(RawData&&) = default;
        ~RawData() = default;

        constexpr RawData(const float pos_x, const float pos_y, const float rot_z) noexcept:
            pos_x(pos_x),
            pos_y(pos_y),
            rot_z(rot_z)
        {}

        constexpr RawData(const Message& data) noexcept:
            RawData(data.pos_x, data.pos_y, data.rot_z)
        {}

        operator Message() const noexcept
        {
            Message msg;
            msg.pos_x = pos_x;
            msg.pos_y = pos_y;
            msg.rot_z = rot_z;

            return msg;
        }
    };
}
