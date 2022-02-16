#pragma once

#include <string>

#include <ros/ros.h>

#include <std_msgs/Bool.h>

namespace Harurobo2022
{
    class ActivatePublisher final
    {
        ros::Publisher pub;

    public:
        ActivatePublisher(const std::string& node_name, ros::NodeHandle& nh) noexcept:
            pub{nh.advertise<std_msgs::Bool>(node_name + "/disable", 1)}
        {}

        void activate() noexcept
        {
            std_msgs::Bool msg;
            msg.data = true;
            pub.publish(msg);
        }

        void deactivate() noexcept
        {
            std_msgs::Bool msg;
            msg.data = false;
            pub.publish(msg);
        }
    };
}