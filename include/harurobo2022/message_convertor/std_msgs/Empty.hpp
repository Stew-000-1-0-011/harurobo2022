#pragma once

#include <cstdint>

#include <std_msgs/Empty.h>

#include "../template.hpp"



namespace Harurobo2022
{
    namespace
    {
        template<>
        struct MessageConvertor<std_msgs::Empty>
        {
            using Message = std_msgs::Empty;
            using RawData = void;
            using CanData = std::uint8_t;

            MessageConvertor() = default;
            MessageConvertor(const MessageConvertor&) = default;
            MessageConvertor(MessageConvertor&&) = default;
            MessageConvertor& operator=(const MessageConvertor&) = default;
            MessageConvertor& operator=(MessageConvertor&&) = default;
            ~MessageConvertor() = default;

            constexpr MessageConvertor(const Message&) noexcept
            {}

            constexpr MessageConvertor(const CanData&) noexcept
            {}

            operator Message() const noexcept
            {
                return {};
            }

            operator CanData() const noexcept
            {
                return {};
            }
        };
    }
}