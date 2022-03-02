#pragma once

#include <std_msgs/UInt8.h>

#include "topic.hpp"
#include "stringlike_types.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "topics/state_topic.hpp"

namespace Harurobo2022
{
    namespace
    {
        enum class State: std::uint8_t
        {
            disable,
            reset,
            automatic,
            manual,
            over_fence,
            game_over,
            game_clear
        };

        class StateManager final
        {
            using state_topic = Topics::state_topic;
    
            Publisher<state_topic> pub{1};

            State state{State::disable};
            Subscriber<state_topic> sub{1, [this](const typename state_topic::Message::ConstPtr& msg_p){ state = static_cast<State>(msg_p->data); }};

        public:
            template<class F>
            StateManager(F callback) noexcept:
                sub{1, [this, &callback](const typename state_topic::Message::ConstPtr& msg_p){ state = static_cast<State>(msg_p->data); callback(state); }}
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