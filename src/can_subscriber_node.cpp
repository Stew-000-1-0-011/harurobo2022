#include <cstddef>
#include <cstdint>
#include <cstring>

#include <ros/ros.h>

#include "harurobo2022/topics.hpp"
#include "harurobo2022/raw_data/raw_data_all.hpp"
#include "harurobo2022/shutdown_subscriber.hpp"

using namespace Harurobo2022;

const char *const node_name = "can_subscriber";

template<class CanRxTopic>
struct CanRxBuffer final
{
    static_assert(is_can_rx_topic_v<CanRxTopic>);

    using RawData = Harurobo2022::RawData<typename CanRxTopic::Message>;

    std::uint8_t buffer[sizeof(RawData)];
    std::uint8_t * p{buffer};  // 一度書いてみたかったやつ

    const ros::Publisher data_pub;

    CanRxBuffer(ros::NodeHandle& nh) noexcept:
        data_pub(nh.advertise<typename CanRxTopic::Message>(CanRxTopic::topic, 1))
    {}

    inline void push(const Topics::can_rx::Message::ConstPtr& msg_p) noexcept
    {
        const auto dlc = msg_p->dlc;
        std::memcpy(p, msg_p->data.data(), dlc);

        p += dlc;

        if(p - buffer > (std::ptrdiff_t)sizeof(RawData))
        {
            p = buffer;
            RawData data;
            std::memcpy(&data, buffer, sizeof(RawData));
            typename CanRxTopic::Message msg = data;
            data_pub.publish(msg);
        }
    }
};

class CanSubscriberNode final
{

    ros::NodeHandle nh{};

    ros::Subscriber can_rx_sub{nh.subscribe<Topics::can_rx::Message>(Topics::can_rx::topic, 1000, &CanSubscriberNode::can_rx_callback, this)};

    CanRxBuffer<CanRxTopics::odometry> odometry_unpacker{nh};
    CanRxBuffer<CanRxTopics::stopped> stopped_unpacker{nh};

    ShutDownSubscriber shutdown_sub{nh};
    

public:
    CanSubscriberNode() = default;
    ~CanSubscriberNode() = default;

    inline void can_rx_callback(const Topics::can_rx::Message::ConstPtr& msg_p) noexcept;
};

inline void CanSubscriberNode::can_rx_callback(const Topics::can_rx::Message::ConstPtr& msg_p) noexcept
{
    const auto id = msg_p->id;

    switch(id)
    {
        case CanRxTopics::odometry::id:
        odometry_unpacker.push(msg_p);

        case CanRxTopics::stopped::id:
        stopped_unpacker.push(msg_p);

        default:
        ROS_ERROR("Unknown message arrived from usb_can_node.");
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, node_name);

    CanSubscriberNode can_subscriber_node;

    ROS_INFO("%s node has started.", node_name);

    ros::spin();

    ROS_INFO("%s node has terminated.", node_name);

}