#pragma once

#include <ros/ros.h>

#include "harurobo2022/topics.hpp"

namespace Harurobo2022
{
    class ShutDownSubscriber final
    {
        const ros::Subscriber shutdown_sub;

    public:
        inline ShutDownSubscriber(ros::NodeHandle& nh) noexcept;

    private:
        inline void shutdown_callback(const Topics::shutdown::Message msg) noexcept;
    };

    inline ShutDownSubscriber::ShutDownSubscriber(ros::NodeHandle& nh) noexcept:
        shutdown_sub{nh.subscribe<Topics::shutdown::Message>(Topics::shutdown::topic, 1, &ShutDownSubscriber::shutdown_callback, this)}
    {}

    inline void ShutDownSubscriber::shutdown_callback(const Topics::shutdown::Message msg) noexcept
    {
        ros::shutdown();
    }
}