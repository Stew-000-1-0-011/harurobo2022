#pragma once
#include <cstdint>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <harurobo2022/CanFrame.h>
#include <harurobo2022/Odometry.h>

#include "harurobo2022/lib/cheap_string.hpp"
#include "harurobo2022/config.hpp"

// さすがにどうかと思うので、もしかしたらトピックに固有の型からそのトピックの型とCanSubscriberから見たIDを返すような型を作るかも。
//
// ある一つのメッセージ型が複数のトピックで使われるのだから、メッセージはトピックじゃないのはわかる。
// でも、一つのトピックは一つのメッセージ型しか使えなさそうだ。そんなことはないのか？
// もし仮にそうなんだとしたら、なぜトピックは型にしないんだろうか。トピックからメッセージ型がコンパイル時に取れるようにしないんだろうか。
// なぜトピック型によって特殊化できるテンプレートを作ることができないんだろうか...
// #define TOPIC(message_type) #message_type

// #define PUBLISHER(node_handle, message_type, buffer_size) \
// node_handle.advertise<message_type>(TOPIC(message_type), buffer_size)

// #define SUBSCRIBER(node_handle, message_type, buffer_size, ...) \
// node_handle.subscribe<message_type>(TOPIC(message_type), buffer_size, __VA_ARGS__)

// #define TOPIC(topic_name, message_type, ...) \
// struct topic_name final \
// { \
//     using Message = message_type; \
//     static constexpr const char * topic{#topic_name}; \
//     static constexpr std::int16_t id{__VA_ARGS__}; \
// };

namespace Harurobo2022
{

    namespace Topics::Implement
    {
        // C++2aがいけるっぽいので使う。
        template<auto topic_, class Message_>
        struct Topic final
        {
            static constexpr const char * topic = topic_;
            using Message = Message_;
        };

        template<auto topic_, class Message_, std::uint16_t id_>
        struct CanTopic final
        {
            static constexpr const char * topic = topic_;
            using Message = Message_;
            static constexpr uint16_t id = id_;
            using Data = Data_;
        };
    }

    namespace Topics
    {
        using wheel_FR_vela = Implement::CanTopic<CheapString::String("wheel_FR_vela"), std_msgs::Float32, Config::CanId::Tx::DriveMotor::FR>;
        using wheel_FL_vela = Implement::CanTopic<CheapString::String("wheel_FL_vela"), std_msgs::Float32, Config::CanId::Tx::DriveMotor::FL>;
        using wheel_BL_vela = Implement::CanTopic<CheapString::String("wheel_BL_vela"), std_msgs::Float32, Config::CanId::Tx::DriveMotor::BL>;
        using wheel_BR_vela = Implement::CanTopic<CheapString::String("wheel_BR_vela"), std_msgs::Float32, Config::CanId::Tx::DriveMotor::BR>;
        // TOPIC(wheel_FR_vela, std_msgs::Float32, Config::CanId::Tx::DriveMotor::FR)
        // TOPIC(wheel_FL_vela, std_msgs::Float32, Config::CanId::Tx::DriveMotor::FL)
        // TOPIC(wheel_BL_vela, std_msgs::Float32, Config::CanId::Tx::DriveMotor::BL)
        // TOPIC(wheel_BR_vela, std_msgs::Float32, Config::CanId::Tx::DriveMotor::BR)

        using body_twist = Implement::Topic<CheapString::String("body_twist"), geometry_msgs::Twist>;
        // TOPIC(body_twist, geometry_msgs::Twist)

        using odometry = Implement::CanTopic<CheapString::String("odometry"), harurobo2022::Odometry, Config::CanId::Rx::odometry>;
        // TOPIC(odometry, harurobo2022::Odometry, Config::CanId::Rx::odometry)

        using can_rx = Implement::Topic<CheapString::String("can_rx"), harurobo2022::CanFrame>;
        // TOPIC(can_rx, harurobo2022::CanFrame)
        using can_tx = Implement::Topic<CheapString::String("can_tx"), harurobo2022::CanFrame>;
        // TOPIC(can_tx, harurobo2022::CanFrame)
    }

}
// #undef TOPIC