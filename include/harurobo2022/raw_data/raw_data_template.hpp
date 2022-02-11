#pragma once
#include <std_msgs/Float32.h>

namespace Harurobo2022
{
    template<class Message_>
    struct RawData
    {
        static_assert(ros::message_traits::IsMessage<Message_>::value);
        using Message = Message_;
        Message::_data_type data{};

        RawData() = default;
        RawData(const RawData&) = default;
        RawData(RawData&&) = default;
        RawData& operator=(const RawData&) = default;
        RawData& operator=(RawData&&) = default;
        ~RawData() = default;

        constexpr RawData(const float data) noexcept:
            data(data)
        {}

        constexpr RawData(const Message& msg) noexcept:
            RawData(msg.data)
        {}

        operator Message() const noexcept
        {
            Message msg;
            msg.data = data;
            return msg;
        }
    };
}