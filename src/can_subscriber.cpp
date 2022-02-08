#include <cstddef>
#include <cstdint>
#include <cstring>

#include <ros/ros.h>

#include "harurobo2022/topics.hpp"
#include "harurobo2022/can_publish.hpp"

using namespace Topics;

template<class Topic>
struct alignas(1) CanRxData;

template<class Topic>
inline void change_to_msg(const CanRxData<Topic>& data, typename Topic::Message& msg) noexcept;

// 送られてくるCanFrameのdlcの和がぴったりsizeof(T)であることが前提
// Tは代入可能、デフォルトコンストラクタを呼び出せる。
// Topic::Messageのdataの型はTである。
// ひとまずデータが欠落することはないものとする。
template<class Topic>
struct CanRxBuffer final
{
    using Data = CanRxData<Topic>;
    std::uint8_t buffer[sizeof(Data)];
    std::uint8_t * p{buffer};  // 一度書いてみたかったやつ

    ros::Publisher data_pub;

    inline CanRxBuffer(ros::NodeHandle nh) noexcept:
        data_pub(nh.advertise<typename Topic::Message>(Topic::topic, 1))
    {}

    inline void push(const can_rx::Message::ConstPtr& msg_p) noexcept
    {
        const auto dlc = msg_p->dlc;
        std::memcpy(p, msg_p->data.data(), dlc);
        
        p += dlc;

        if(p - buffer > sizeof(Data))
        {
            p = buffer;
            Data data;
            std::memcpy(&data, buffer, sizeof(Data));
            typename Topic::Message msg;
            CanRxData<Topic>::copy(msg, data);
            data_pub.publish(msg);
        }
    }
};

// 連続してるよね...?
template<>
struct alignas(1) CanRxData<odometry>
{
    float acc_x;
    float acc_y;
    float acc_z;

    float rot_x;
    float rot_y;
    float rot_z;

    float pos_x;
    float pos_y;

    // ものすごい間抜けなコードじゃないか...？
    static void copy(odometry::Message& msg, const CanRxData<odometry>& data) noexcept
    {
        msg.acc_x = data.acc_x;
        msg.acc_y = data.acc_y;
        msg.acc_z = data.acc_z;
        msg.rot_x = data.rot_x;
        msg.rot_y = data.rot_z;
        msg.rot_z = data.rot_z;
        msg.pos_x = data.pos_x;
        msg.pos_y = data.pos_y;
    }
};



class CanSubscriber final
{

    ros::NodeHandle nh{};

    ros::Subscriber can_rx_sub{nh.subscribe<can_rx::Message>(can_rx::topic, 1000, &CanSubscriber::can_rx_callback, this)};

    ros::Publisher odometry_pub{/*TODO*/};
    CanRxBuffer<odometry> odometry_unpacker{nh};


public:
    CanSubscriber() = default;
    ~CanSubscriber() = default;

    inline void can_rx_callback(const can_rx::Message::ConstPtr& msg_p) const noexcept;
};

inline void CanSubscriber::can_rx_callback(const can_rx::Message::ConstPtr& msg_p) const noexcept
{
    const auto id = msg_p->id;

    // switch(id)
    // {
    //     case /*TODO*/:
    //     odometry_unpacker.push(msg_p);

    //     default:
    //     ROS_ERROR("Unknown message arrived from usb_can_node.");
    // }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "can_subscriber");

    CanSubscriber can_subscriber;

    ROS_INFO("can_subscriber node has started.");

    ros::spin();

    ROS_INFO("can_subscriber node has terminated.");

}