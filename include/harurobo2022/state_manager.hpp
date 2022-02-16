#pragma once

#include <cstdint>

#include <ros/ros.h>

#include "topics.hpp"


namespace Harurobo2022
{
    class StateManager final
    {
    public:
        enum class State: std::uint8_t
        {
            reset = 0,
            auto,
            manual,
            over_fence
        };

    private:
        ros::Publisher state_pub;
        ros::Subscriber state_sub;
        State state;

    public:
        inline StateManager(ros::NodeHandle nh&) noexcept;

        inline State get_state() noexcept;
        inline void set_state(const State state) noexcept;

    private:
        inline void state_callback(const Topics::state::Message& msg_p) noexcept;

    };

    inline StateManager::StateManager(ros::NodeHandle nh&) noexcept:
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
        state_pub.publish(std_msgs::UInt8(state));
    }

    inline void StateManager::state_callback(const Topics::state::Message& msg_p) noexcept
    {
        this->state = msg_p.data;
    }

}