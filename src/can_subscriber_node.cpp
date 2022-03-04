#include <cstddef>
#include <cstdint>
#include <cstring>

#include <ros/ros.h>

#include "harurobo2022/topic.hpp"
#include "harurobo2022/topics/odometry.hpp"
#include "harurobo2022/publisher.hpp"
#include "harurobo2022/subscriber.hpp"
#include "harurobo2022/static_init_deinit.hpp"

using namespace Harurobo2022;

namespace
{
    namespace CanSubscriberImplement
    {
        #include "harurobo2022/lib/macro/define_stringlike_type.hpp"
        Stew_StringlikeType(can_rx)
        Stew_StringlikeType(can_subscriber)
        #undef Stew_StringType
    }

    using can_rx = Topic<CanSubscriberImplement::can_rx, can_plugins::Frame>;

    template<class CanRxTopic_>
    struct CanRxBuffer final
    {
        using CanRxTopic = CanRxTopic_;
        static_assert(is_can_rx_topic_v<CanRxTopic>, "argument must be can_rx topic.");

        using MessageConvertor = CanRxTopic::MessageConvertor;
        using RawData = MessageConvertor::RawData;

        std::uint8_t buffer[sizeof(RawData)]{};
        std::uint8_t * p{buffer};

        Publisher<CanRxTopic,PublisherOption{.disable_can_rx_topic_assert = true}> data_pub;

        CanRxBuffer(const std::uint32_t pub_queue_size) noexcept:
            data_pub{pub_queue_size}
        {}

        inline void push(const typename can_rx::MessageConvertor::Message::ConstPtr& msg_p) noexcept
        {
            const std::uint8_t dlc = msg_p->dlc;
            std::memcpy(p, msg_p->data.data(), dlc);

            p += dlc;

            if(p - buffer > (std::ptrdiff_t)sizeof(RawData))
            {
                p = buffer;
                RawData raw_data;
                std::memcpy(&raw_data, buffer, sizeof(RawData));
                typename CanRxTopic::Message msg = MessageConvertor(raw_data);
                data_pub.publish(msg);
            }
        }
    };

    class CanSubscriberNode final
    {
        Subscriber<can_rx> can_rx_sub
        {
            1000,
            [this](const can_rx::Message::ConstPtr& msg_p) noexcept
            {
                can_rx_callback(msg_p);
            }
        };

        CanRxBuffer<Topics::odometry_x> odometry_x_unpacker{1};
        CanRxBuffer<Topics::odometry_y> odometry_y_unpacker{1};
        CanRxBuffer<Topics::odometry_yaw> odometry_yaw_unpacker{1};

        void can_rx_callback(const can_rx::Message::ConstPtr& msg_p) noexcept
        {
            const auto id = msg_p->id;

            switch(id)
            {
            case Topics::odometry_x::id:
                odometry_x_unpacker.push(msg_p);
                break;
            
            case Topics::odometry_y::id:
                odometry_y_unpacker.push(msg_p);
                break;
            
            case Topics::odometry_yaw::id:
                odometry_yaw_unpacker.push(msg_p);
                break;

            default:
                ROS_ERROR("Unknown message arrived from usb_can_node.");
            }
        }
    };
}

using namespace Harurobo2022;
using namespace CanSubscriberImplement;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, can_subscriber::str);
    StaticInitDeinit static_init_deinit;

    CanSubscriberNode can_subscriber_node;

    ROS_INFO("%s node has started.", can_subscriber::str);

    ros::spin();

    ROS_INFO("%s node has terminated.", can_subscriber::str);

}
