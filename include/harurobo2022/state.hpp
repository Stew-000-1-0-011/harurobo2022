#pragma once

#include <cstdint>

#include <ros/ros.h>

#include <std_msgs/UInt8.h>

#include "harurobo2022/topics.hpp"


namespace Harurobo2022
{
    enum class State: std::uint8_t
    {
        shutdown,
        reset,
        automatic,
        manual,
        over_fence
    };

    class StateManager final
    {
        ros::Publisher pub;
        ros::Subscriber sub;
        State state;

    public:
        inline StateManager(ros::NodeHandle& nh) noexcept;

        inline State get_state() noexcept;
        inline void set_state(const State state) noexcept;

    private:
        inline void callback(const Topics::state_::Message::ConstPtr& msg_p) noexcept;

    };

    inline StateManager::StateManager(ros::NodeHandle& nh) noexcept:
        pub{nh.advertise<Topics::state_::Message>(Topics::state_::topic, 1)},
        sub{nh.subscribe<Topics::state_::Message>(Topics::state_::topic, 1, &StateManager::callback, this)}
    {}

    inline State StateManager::get_state() noexcept
    {
        return state;
    }

    inline void StateManager::set_state(const State state) noexcept
    {
        this->state = state;
        Topics::state_::Message msg;
        msg.data = static_cast<std::uint8_t>(state);
        pub.publish(msg);
    }

    inline void StateManager::callback(const Topics::state_::Message::ConstPtr& msg_p) noexcept
    {
        this->state = static_cast<State>(msg_p->data);
    }

}