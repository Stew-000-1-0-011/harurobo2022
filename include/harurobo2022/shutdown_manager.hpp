#pragma once

#include <ros/ros.h>

#include <std_msgs/Empty.h>

#include "topic.hpp"
#include "stringlike_types.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"

namespace Harurobo2022
{
    namespace
    {
        struct ShutDownManager final
        {
            using shutdown = Topic<StringlikeTypes::shutdown, std_msgs::Empty>;

            Publisher<shutdown> pub{1};
            Subscriber<shutdown> sub{1, [](const typename shutdown::Message::ConstPtr&){ ros::shutdown(); }};

            void shutdown() noexcept
            {
                pub.publish();
            }
        };
    }
}