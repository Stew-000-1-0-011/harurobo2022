#pragma once

#include <cstdint>

#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace Harurobo2022
{
    class ActivateSubscriber final
    {
        bool is_active_{false};
        ros::Subscriber sub;

    public:
        inline ActivateSubscriber(const std::string& node_name, ros::NodeHandle& nh, const bool is_active_ = false) noexcept;

        inline bool is_active() const noexcept;

    private:
        inline void callback(const std_msgs::Bool::ConstPtr& msg_p) noexcept;
    };

    inline ActivateSubscriber::ActivateSubscriber(const std::string& node_name, ros::NodeHandle& nh, const bool is_active_) noexcept:
        is_active_{is_active_},
        sub{nh.subscribe<std_msgs::Bool>(node_name + "/disable", 1, &ActivateSubscriber::callback, this)}
    {}

    inline bool ActivateSubscriber::is_active() const noexcept
    {
        return is_active_;
    }

    inline void ActivateSubscriber::callback(const std_msgs::Bool::ConstPtr& msg_p) noexcept
    {
        is_active_ = msg_p->data;
    }
}