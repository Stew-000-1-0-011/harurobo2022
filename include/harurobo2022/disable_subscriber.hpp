#pragma once

#include <cstdint>

#include <string>

#include <ros/ros.h>
#include <std_msgs/Empty.h>

namespace Harurobo2022
{
    class DisableSubscriber final
    {
        bool is_active_{false};
        ros::Subscriber disable_sub;

    public:
        inline DisableSubscriber(const std::string& node_name, ros::NodeHandle& other_nh, const bool is_active_ = false) noexcept;

        inline bool is_active() const noexcept;

    private:
        inline void disable_callback(const std_msgs::Empty::ConstPtr& msg_p) noexcept;
    };

    inline DisableSubscriber::DisableSubscriber(const std::string& node_name, ros::NodeHandle& other_nh, const bool is_active) noexcept:
        is_active_{is_active},
        disable_sub{other_nh.subscribe<std_msgs::Empty>(node_name + "/disable", 1, &DisableSubscriber::disable_callback, this)}
    {}

    inline bool DisableSubscriber::is_active() const noexcept
    {
        return is_active_;
    }

    inline void DisableSubscriber::disable_callback(const std_msgs::Empty::ConstPtr& msg_p) noexcept
    {
        is_active_ = !is_active_;
    }
}