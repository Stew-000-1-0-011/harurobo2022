#pragma once

#include <ros/ros.h>



namespace Harurobo2022
{
    namespace
    {
        template<class Message_>
        struct MessageConvertor
        {
            using Message = Message_;

            static_assert(ros::message_traits::IsMessage<Message_>::value, "1st argument must be message.");

            using RawData = Message_::_data_type;

            RawData data{};

            MessageConvertor() = default;
            MessageConvertor(const MessageConvertor&) = default;
            MessageConvertor(MessageConvertor&&) = default;
            MessageConvertor& operator=(const MessageConvertor&) = default;
            MessageConvertor& operator=(MessageConvertor&&) = default;
            ~MessageConvertor() = default;

            constexpr MessageConvertor(const RawData& data) noexcept:
                data(data)
            {}

            constexpr MessageConvertor(const Message& msg) noexcept:
                data(msg.data)
            {}

            operator Message() const noexcept
            {
                Message msg;
                msg.data = data;
                return msg;
            }

            operator RawData() const noexcept
            {
                return data;
            }
        };
    }
}