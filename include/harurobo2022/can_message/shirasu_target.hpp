#pragma once

#include <cstdint>
#include <cstring>

#include <utility>

#include <std_msgs/Float32.h>

namespace Harurobo2022
{
    namespace
    {


        struct ShirasuTarget final
        {
            using Message = std_msgs::Float32;

            std::uint8_t data[4]{};
            float value;

            ShirasuTarget(const float value) noexcept:
                // data{std::bit_cast<std::uint32_t>(data)},
                value{value}
            {
                std::memcpy(&data, &value, 4);
                std::swap(data[0], data[3]);
                std::swap(data[1], data[2]);
            }

            operator Message() const noexcept
            {
                Message msg;
                msg.data = value;
                return msg;
            }
        };
    }

}