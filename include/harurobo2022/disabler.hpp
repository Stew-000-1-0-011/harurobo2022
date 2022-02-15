#pragma once

#include <cstdint>

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include "harurobo2022/lib/cheap_string.hpp"

namespace Harurobo2022
{
    namespace Disabler::Implement
    {
        template<CheapString::String node_name>
        struct NodeName final
        {
            static constexpr char dummy;
            static std::string topic{reinterpret_cast<std::intptr_t>(&topic)};
        };

        template<class Node>
        class DisablePublisher final
        {
            ros::Publisher disable_pub;

        public:
            DisablePublisher(const ros::NodeHandle& others_nh) noexcept:
                disable_pub{others_nh.advertise<std_msgs::Empty>()}
            {}
        };
    }
}