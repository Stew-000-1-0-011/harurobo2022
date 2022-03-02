#pragma once

#include "harurobo2022/Twist.h"

#include "../../lib/reverse_buffer.hpp"
#include "../template.hpp"


namespace Harurobo2022
{
    namespace
    {
        template<>
        struct MessageConvertor<harurobo2022::Twist>
        {
            using Message = harurobo2022::Twist;

            struct RawData final
            {
                float linear_x{};
                float linear_y{};
                float angular_z{};
            };

            struct CanData final
            {
                StewLib::ReverseBuffer<float> linear_x{};
                StewLib::ReverseBuffer<float> linear_y{};
                StewLib::ReverseBuffer<float> angular_z{};
                
                CanData() = default;

                void reverse() noexcept
                {
                    linear_x.reverse();
                    linear_y.reverse();
                    angular_z.reverse();
                }
            };

            RawData raw_data;

            MessageConvertor() = default;
            MessageConvertor(const MessageConvertor&) = default;
            MessageConvertor(MessageConvertor&&) = default;
            MessageConvertor& operator=(const MessageConvertor&) = default;
            MessageConvertor& operator=(MessageConvertor&&) = default;
            ~MessageConvertor() = default;

            constexpr MessageConvertor(const Message& msg) noexcept:
                MessageConvertor({msg.linear_x, msg.linear_y, msg.angular_z})
            {}

            constexpr MessageConvertor(const RawData& raw_data) noexcept:
                raw_data{raw_data}
            {}

            MessageConvertor(CanData can_data) noexcept
            {
                can_data.reverse();
            }

            operator Message() const noexcept
            {
                Message msg;
                msg.linear_x = raw_data.linear_x;
                msg.linear_y = raw_data.linear_y;
                msg.angular_z = raw_data.angular_z;

                return msg;
            }
        };
    }
}