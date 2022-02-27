#pragma once

#include <std_msgs/Empty.h>

#include "harurobo2022/message_convertor/message_convertor_template.hpp"



namespace Harurobo2022
{
    namespace
    {
        template<>
        struct MessageConvertor<std_msgs::Empty>
        {
            using Message = std_msgs::Empty;

            MessageConvertor() = default;
            MessageConvertor(const MessageConvertor&) = default;
            MessageConvertor(MessageConvertor&&) = default;
            MessageConvertor& operator=(const MessageConvertor&) = default;
            MessageConvertor& operator=(MessageConvertor&&) = default;
            ~MessageConvertor() = default;


            constexpr MessageConvertor([[maybe_unused]] const Message& data) noexcept
            {}

            operator Message() const noexcept
            {
                return {};
            }
        };
    }
}