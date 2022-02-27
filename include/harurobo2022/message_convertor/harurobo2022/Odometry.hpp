#pragma once

#include "harurobo2022/message_convertor/message_convertor_template.hpp"
#include "harurobo2022/Odometry.h"

namespace Harurobo2022
{
    namespace
    {
        template<>
        struct MessageConvertor<harurobo2022::Odometry>
        {
            using Message = harurobo2022::Odometry;

            float pos_x{};
            float pos_y{};
            float rot_z{};

            MessageConvertor() = default;
            MessageConvertor(const MessageConvertor&) = default;
            MessageConvertor(MessageConvertor&&) = default;
            MessageConvertor& operator=(const MessageConvertor&) = default;
            MessageConvertor& operator=(MessageConvertor&&) = default;
            ~MessageConvertor() = default;

            constexpr MessageConvertor(const float pos_x, const float pos_y, const float rot_z) noexcept:
                pos_x(pos_x),
                pos_y(pos_y),
                rot_z(rot_z)
            {}

            constexpr MessageConvertor(const Message& data) noexcept:
                MessageConvertor(data.pos_x, data.pos_y, data.rot_z)
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
}