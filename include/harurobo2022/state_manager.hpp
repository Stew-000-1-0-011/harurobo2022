#pragma once

#include <cstdint>

#include <ros/ros.h>

#include "topics.hpp"


namespace Harurobo2022
{
    enum class State: std::uint8_t
    {
        reset = 0,
        automatic,
        manual,
        over_fence
    };

    class StateManager final
    {
        ros::Publisher state_pub;
        ros::Subscriber state_sub;
        State state;

    public:
        inline StateManager(ros::NodeHandle& nh) noexcept;

        inline State get_state() noexcept;
        inline void set_state(const State state) noexcept;

    private:
        inline void state_callback(const Topics::state::Message::ConstPtr& msg_p) noexcept;

    };

    inline StateManager::StateManager(ros::NodeHandle& nh) noexcept:
        state_pub{nh.advertise<Topics::state::Message>(Topics::state::topic, 1)},
        state_sub{nh.subscribe<Topics::state::Message>(Topics::state::topic, 1, &StateManager::state_callback, this)}
    {}

    inline State StateManager::get_state() noexcept
    {
        return state;
    }

    inline void StateManager::set_state(const State state) noexcept
    {
        this->state = state;
        Topics::state::Message msg;
        msg.data = static_cast<std::uint8_t>(state);
        state_pub.publish(msg);
    }

    inline void StateManager::state_callback(const Topics::state::Message::ConstPtr& msg_p) noexcept
    {
        this->state = static_cast<State>(msg_p->data);
    }

}