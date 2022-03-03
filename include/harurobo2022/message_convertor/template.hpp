#pragma once

#include <cstring>

#include <ros/ros.h>

#include "../lib/reverse_buffer.hpp"

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
            using CanData = StewLib::ReverseBuffer<RawData>;

            RawData raw_data{};

            MessageConvertor() = default;
            MessageConvertor(const MessageConvertor&) = default;
            MessageConvertor(MessageConvertor&&) = default;
            MessageConvertor& operator=(const MessageConvertor&) = default;
            MessageConvertor& operator=(MessageConvertor&&) = default;
            ~MessageConvertor() = default;

            constexpr MessageConvertor(const Message& msg) noexcept:
                raw_data(msg.data)
            {}

            constexpr MessageConvertor(const RawData& raw_data) noexcept:
                raw_data(raw_data)
            {}

            operator Message() const noexcept
            {
                Message msg;
                msg.data = raw_data;
                return msg;
            }

            operator RawData() const noexcept
            {
                return raw_data;
            }

            operator CanData() const noexcept
            {
                CanData ret = raw_data;
                ret.reverse();
                return ret;
            }
        };
    }
}