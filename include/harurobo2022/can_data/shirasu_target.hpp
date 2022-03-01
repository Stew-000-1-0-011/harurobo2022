#pragma once

#include <cstdint>
#include <cstring>

#include <utility>

#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

namespace Harurobo2022
{
    namespace
    {
        struct ShirasuTarget final
        {
            using Message = std_msgs::Float32;

            struct Data final
            {
                std::uint8_t data[4]{};
            };
            Data data;

            float value;

            ShirasuTarget(const float value) noexcept:
                // data{std::bit_cast<std::uint32_t>(data)},
                value{value}
            {
                std::memcpy(&data.data, &value, 4);
                std::swap(data.data[0], data.data[3]);
                std::swap(data.data[1], data.data[2]);
            }

            operator Message() const noexcept
            {
                Message msg;
                msg.data = value;
                return msg;
            }
        };

        struct ShirasuCmd final
        {
            using Message = std_msgs::UInt8;

            std::uint8_t data{};

            ShirasuCmd(const std::uint8_t data) noexcept:
                data{data}
            {}

            operator Message() const noexcept
            {
                Message msg;
                msg.data = data;
                return msg;
            }
        };
    }

}