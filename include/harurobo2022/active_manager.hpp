#pragma once

#include <type_traits>
#include <array>
#include <variant>

#include <std_msgs/Bool.h>

#include "topic.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"

#include "has_members.hpp"

namespace Harurobo2022
{
    namespace
    {
        template<class Name_, class ... PubSubP>
        class ActiveManager final
        {
        public:
            using Name = Name_;
            
            static_assert(((StewLib::has_activate_v<std::remove_cvref_t<PubSubP>> && StewLib::has_deactivate_v<std::remove_cvref_t<PubSubP>>) && ...), "Some of them dont have activate or deactivate.");
            static_assert(StewLib::is_stringlike_type_v<Name>, "argument must be StringlikeType.");

        private:
            using active_manager_topic = Topic<Name, std_msgs::Bool>;

            Publisher<active_manager_topic> pub{10};
            Subscriber<active_manager_topic> sub
            {
                10,
                [this](const typename active_manager_topic::Message::ConstPtr& msg_p)
                {
                    if(msg_p->data)
                    {
                        activate();
                    }
                    else
                    {
                        deactivate();
                    }
                }
            };
            
            std::array<std::variant<PubSubP * ...>, sizeof...(PubSubP)> pubsub_ptrs{};

        public:
            ActiveManager(PubSubP& ... pubsubs) noexcept:
                pubsub_ptrs{&pubsubs ...}
            {}

            void activate() noexcept
            {
                if constexpr(sizeof...(PubSubP))
                {
                    for(auto& pubsub_p : pubsub_ptrs)
                    {
                        std::visit([](auto pubsub_p) noexcept {pubsub_p->activate();}, pubsub_p);
                    }
                }
            }

            void deactivate() noexcept
            {
                if constexpr(sizeof...(PubSubP))
                {
                    for(auto& pubsub_p : pubsub_ptrs)
                    {
                        std::visit([](auto pubsub_p) noexcept {pubsub_p->deactivate();}, pubsub_p);
                    }
                }
            }
        };
    }
}