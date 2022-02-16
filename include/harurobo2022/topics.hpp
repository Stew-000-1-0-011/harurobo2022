#pragma once
#include <cstdint>
#include <type_traits>

#include <ros/ros.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#include <can_plugins/Frame.h>

#include <harurobo2022/Odometry.h>

#include "harurobo2022/lib/shirasu_util.hpp"
#include "harurobo2022/lib/cheap_string.hpp"
#include "harurobo2022/config.hpp"

// さすがにどうかと思うので、もしかしたらトピックに固有の型からそのトピックの型とCanSubscriberから見たIDを返すような型を作るかも。
//
// ある一つのメッセージ型が複数のトピックで使われるのだから、メッセージはトピックじゃないのはわかる。
// でも、一つのトピックは一つのメッセージ型しか使えなさそうだ。そんなことはないのか？
// もし仮にそうなんだとしたら、なぜトピックは型にしないんだろうか。トピックからメッセージ型がコンパイル時に取れるようにしないんだろうか。
// なぜトピック型によって特殊化できるテンプレートを作ることができないんだろうか...

// 実行時にトピック名が変わるからだろう。まあ今回は名前を実行時に変える必要がないのでこのままいく。しかし、結構重要な機能をドブに捨ててしまったようだ...
// ん？でもトピック名が実行時に変わるとして、それをプログラム書いてるときはどうやって知ればいいんだ...？

namespace Harurobo2022
{

    namespace Topics::Implement
    {
        // C++2aがいけるっぽいので使う。
        // 意識してなかったが、非型テンプレートパラメータオブジェクトは静的記憶域のオブジェクトらしい。だからこのコードは動いていたようだ。
        // あれ、静的記憶域を持つの？それって結構重くならない？
        template<auto topic_, class Message_>
        struct Topic final
        {
            static constexpr const char * topic = topic_;
            using Message = Message_;
        };
    }

    namespace CanTxTopics::Implement
    {
        template<auto topic_, class Message_, std::uint16_t id_>
        struct CanTxTopic final
        {
            static constexpr const char * topic = topic_;
            using Message = Message_;
            static constexpr uint16_t id = id_;
        };
    }

    namespace CanRxTopics::Implement
    {
        template<auto topic_, class Message_, std::uint16_t id_>
        struct CanRxTopic final
        {
            static constexpr const char * topic = topic_;
            using Message = Message_;
            static constexpr uint16_t id = id_;
        };
    }

    template<class T>
    struct is_topic: std::false_type
    {};

    template<auto topic_, class Message_>
    struct is_topic<Topics::Implement::Topic<topic_, Message_>>: std::true_type
    {};

    template<class T>
    inline constexpr bool is_topic_v = is_topic<T>::value;

    template<class T>
    struct is_can_tx_topic: std::false_type
    {};

    template<auto topic_, class Message_, std::uint16_t id_>
    struct is_can_tx_topic<CanTxTopics::Implement::CanTxTopic<topic_, Message_, id_>>: std::true_type
    {};

    template<class T>
    inline constexpr bool is_can_tx_topic_v = is_can_tx_topic<T>::value;

    template<class T>
    struct is_can_rx_topic: std::false_type
    {};

    template<auto topic_, class Message_, std::uint16_t id_>
    struct is_can_rx_topic<CanRxTopics::Implement::CanRxTopic<topic_, Message_, id_>>: std::true_type
    {};

    template<class T>
    inline constexpr bool is_can_rx_topic_v = is_can_rx_topic<T>::value;

    namespace Topics
    {
        using body_twist = Implement::Topic<CheapString::String("body_twist"), geometry_msgs::Twist>;

        using can_rx = Implement::Topic<CheapString::String("can_rx"), can_plugins::Frame>;
        using can_tx = Implement::Topic<CheapString::String("can_tx"), can_plugins::Frame>;

        using state_ = Implement::Topic<CheapString::String("state"), std_msgs::UInt8>;

        using shutdown_ = Implement::Topic<CheapString::String("shutdown"), std_msgs::Empty>;
    }

    namespace CanTxTopics
    {
        using wheel_FR_vela = Implement::CanTxTopic<CheapString::String("wheel_FR_vela"), std_msgs::Float32, ShirasuUtil::target_id(Config::CanId::Tx::DriveMotor::FR)>;
        using wheel_FL_vela = Implement::CanTxTopic<CheapString::String("wheel_FL_vela"), std_msgs::Float32, ShirasuUtil::target_id(Config::CanId::Tx::DriveMotor::FL)>;
        using wheel_BL_vela = Implement::CanTxTopic<CheapString::String("wheel_BL_vela"), std_msgs::Float32, ShirasuUtil::target_id(Config::CanId::Tx::DriveMotor::BL)>;
        using wheel_BR_vela = Implement::CanTxTopic<CheapString::String("wheel_BR_vela"), std_msgs::Float32, ShirasuUtil::target_id(Config::CanId::Tx::DriveMotor::BR)>;

        using emergency_stop = Implement::CanTxTopic<CheapString::String("emergency_stop"), std_msgs::Bool, Config::CanId::Tx::Emergency::power>;
    }
    
    namespace CanRxTopics
    {
        using odometry = Implement::CanRxTopic<CheapString::String("odometry"), harurobo2022::Odometry, Config::CanId::Rx::odometry>;
        using stopped = Implement::CanRxTopic<CheapString::String("stopped"), std_msgs::Empty, Config::CanId::Rx::stopped>;
    }
}