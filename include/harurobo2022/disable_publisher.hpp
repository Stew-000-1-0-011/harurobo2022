#pragma once

#include <string>

#include <ros/ros.h>

#include <std_msgs/Empty.h>

namespace Harurobo2022
{
    class DisablePublisher final
    {
        ros::Publisher disable_pub;
        std_msgs::Empty msg{};

    public:
        DisablePublisher(const std::string& node_name, ros::NodeHandle& others_nh) noexcept:
            disable_pub{others_nh.advertise<std_msgs::Empty>(node_name + "/disable", 1)}
        {}

        void disable_publish() noexcept
        {
            disable_pub.publish(msg);
        }
    };
}