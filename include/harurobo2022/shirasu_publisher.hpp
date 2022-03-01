#pragma once

#include <cstdint>

#include "shirasu_util.hpp"

#include "can_publisher.hpp"
#include "can_data/shirasu_target.hpp"
#include "stringlike_types.hpp"


namespace Harurobo2022
{
    namespace
    {
        template<class MotorName_, std::uint16_t bid_>
        class ShirasuPublisher final
        {
        public:
            using MotorName = MotorName_;
            constexpr static std::uint16_t bid = bid_;
        
        private:
            using cmd = CanTxTopic<ShirasuTarget, bid, StewLib::Concat<MotorName, StringlikeTypes::_cmd>>;
            using target = CanTxTopic<ShirasuTarget, ShirasuUtil::target_id(bid), StewLib::Concat<MotorName, StringlikeTypes::_target>>;

            CanPublisher<cmd> cmd_canpub{10};
            CanPublisher<target> target_canpub{10};

        public:
            void send_cmd(const ShirasuUtil::Mode cmd) noexcept
            {
                cmd_canpub.can_publish(cmd);
            }

            void send_target(const float target) noexcept
            {
                target_canpub.can_publish(target);
            }

            void publish_cmd(const ShirasuUtil::Mode cmd) noexcept
            {
                cmd_canpub.publish(cmd);
            }

            void publish_target(const float target) noexcept
            {
                target_canpub.publish(target);
            }

            void activate() noexcept
            {
                cmd_canpub.activate();
                target_canpub.activate();
            }

            void deactivate() noexcept
            {
                cmd_canpub.deactivate();
                target_canpub.deactivate();
            }
        };
    }
}