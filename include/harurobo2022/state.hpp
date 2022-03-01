#pragma once

#include <std_msgs/UInt8.h>

#include "topic.hpp"
#include "stringlike_types.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"

namespace Harurobo2022
{
    namespace
    {
        enum class State: std::uint8_t
        {
            disable,
            reset,
            shutdown,
            automatic,
            manual,
            over_fence
        };

        class StateManager final
        {
            using state_topic = Topic<StringlikeTypes::state, std_msgs::UInt8>;
    
            Publisher<state_topic> pub{1};
            Subscriber<state_topic> sub{1, [&state](const typename state_topic::Message::ConstPtr& msg_p){ state = static_cast<State>(msg_p.data); }};

            State state{State::disable};

        public:
            StateManager(auto callback) noexcept
                sub{1, [&state](const typename state_topic::Message::ConstPtr& msg_p){ state = static_cast<State>(msg_p.data); callback(); }}
            {}

            StateManager() = default;

            State get_state() noexcept
            {
                return state;
            }

            inline void set_state(const State state) noexcept
            {
                this->state = state;
                pub.publish(static_cast<std::uint8_t>(state));
            }
        };
    }
}