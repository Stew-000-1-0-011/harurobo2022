#include <ros/ros.h>

#include "harurobo2022/topic_message_alias.hpp"

using namespace TopicMessageTypeAlias;

class CanSubscriber final
{
    ros::NodeHandle nh{};

    ros::Subscriber can_rx_sub{nh.subscribe<can_rx>(TOPIC(can_rx), 1000, &CanSubscriber::can_rx_callback, this)};

    ros::Publisher odometry_pub{/*TODO*/};

public:
    CanSubscriber() = default;
    ~CanSubscriber() = default;

    inline void can_rx_callback(const can_rx::ConstPtr& msg_p) const noexcept;
};