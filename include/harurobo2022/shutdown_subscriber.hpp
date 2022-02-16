#pragma once

#include <ros/ros.h>

#include "harurobo2022/topics.hpp"

namespace Harurobo2022
{

    class ShutDownSubscriber final
    {
        const ros::Subscriber sub;

    public:
        inline ShutDownSubscriber(ros::NodeHandle& nh) noexcept;

    private:
        inline void callback(const Topics::shutdown_::Message msg) noexcept;
    };

    inline ShutDownSubscriber::ShutDownSubscriber(ros::NodeHandle& nh) noexcept:
        sub{nh.subscribe<Topics::shutdown_::Message>(Topics::shutdown_::topic, 1, &ShutDownSubscriber::callback, this)}
    {}

    inline void ShutDownSubscriber::callback(const Topics::shutdown_::Message msg) noexcept
    {
        ros::shutdown();
    }
}