#pragma once

#include <cstdint>

#include <can_plugins/Frame.h>

#include "harurobo2022/lib/enumerate.hpp"

#include "harurobo2022/message_convertor/message_convertor_template.hpp"



namespace Harurobo2022
{
    namespace
    {
        template<>
        struct MessageConvertor<can_plugins::Frame>
        {
            using Message = can_plugins::Frame;

            std::uint32_t id{};
            bool is_rtr{false};
            bool is_extended{false};
            bool is_error{false};
            std::uint8_t dlc{};
            std::uint8_t data[8]{};

            MessageConvertor() = default;
            MessageConvertor(const MessageConvertor&) = default;
            MessageConvertor(MessageConvertor&&) = default;
            MessageConvertor& operator=(const MessageConvertor&) = default;
            MessageConvertor& operator=(MessageConvertor&&) = default;
            ~MessageConvertor() = default;

            constexpr MessageConvertor
                (
                    const std::uint32_t id, const std::uint8_t dlc, const std::uint8_t (&data)[8], 
                    const bool is_rtr = false, const bool is_extended = false, const bool is_error = false
                ) noexcept:
                MessageConvertor(id, is_rtr, is_extended, is_error, dlc, data, StewLib::EnumerateMake<8>::type())
            {}

            constexpr MessageConvertor(const Message& data) noexcept:
                MessageConvertor(data.id, data.is_rtr, data.is_extended, data.is_error, data.dlc, data.data.elems, StewLib::EnumerateMake<8>::type())
            {}

            operator Message() const noexcept
            {
                Message msg;

                msg.id = id;
                msg.is_rtr = is_rtr;
                msg.is_extended = is_extended;
                msg.is_error = is_error;
                msg.dlc = dlc;

                for(int i = 0; i < 8; ++i)
                {
                    msg.data[i] = data[i];
                }

                return msg;
            }

        private:
            template<std::size_t ... index8>
            constexpr MessageConvertor
                (
                    const std::uint32_t id,
                    const bool is_rtr, const bool is_extended, const bool is_error,
                    const std::uint8_t dlc, const std::uint8_t (&data)[8],
                    [[maybe_unused]] const StewLib::Enumerate<index8 ...> enumerate
                ) noexcept:
                id{id},
                is_rtr{is_rtr},
                is_extended{is_extended},
                is_error{is_error},
                dlc{dlc},
                data{data[index8] ...}
            {}
        };
    }
}