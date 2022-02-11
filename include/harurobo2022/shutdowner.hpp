#pragma once

#include <ros/ros.h>

#include "harurobo2022/topics.hpp"

namespace Harurobo2022
{
    class ShutDowner final
    {
        ros::Subscriber shutdown_sub;

    public:
        inline ShutDowner(ros::NodeHandle& nh) noexcept;

    private:
        inline void shutdown_callback(const Topics::shutdown::Message msg) noexcept;
    };

    inline ShutDowner::ShutDowner(ros::NodeHandle& nh) noexcept:
        shutdown_sub{nh.subscribe<Topics::shutdown::Message>(Topics::shutdown::topic, 1, &ShutDowner::shutdown_callback, this)}
    {}

    inline void ShutDowner::shutdown_callback(const Topics::shutdown::Message msg) noexcept
    {
        ros::shutdown();
    }
}