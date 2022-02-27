#pragma once

#include <type_traits>
#include <string>

#include <ros/ros.h>

#include "lib/stringlike_type.hpp"

#include "can_message.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace TopicsImplement
        {
            struct TopicBase{};
            struct CanTxTopicBase: TopicBase{};
            struct CanRxTopicBase: TopicBase{};
        }

        template<class T>
        using is_topic = std::is_base_of<TopicsImplement::TopicBase, T>;
        template<class T>
        inline constexpr bool is_topic_v = std::is_base_of_v<TopicsImplement::TopicBase, T>;

        template<class T>
        using is_can_tx_topic = std::is_base_of<TopicsImplement::CanTxTopicBase, T>;
        template<class T>
        inline constexpr bool is_can_tx_topic_v = std::is_base_of_v<TopicsImplement::CanTxTopicBase, T>;

        template<class T>
        using is_can_rx_topic = std::is_base_of<TopicsImplement::CanRxTopicBase, T>;
        template<class T>
        inline constexpr bool is_can_rx_topic_v = std::is_base_of_v<TopicsImplement::CanRxTopicBase, T>;

        template<class T>
        using is_can_topic = std::disjunction<std::is_base_of<TopicsImplement::CanTxTopicBase, T>, std::is_base_of<TopicsImplement::CanRxTopicBase, T>>;
        template<class T>
        inline constexpr bool is_can_topic_v = std::is_base_of_v<TopicsImplement::CanTxTopicBase, T> || std::is_base_of_v<TopicsImplement::CanRxTopicBase, T>;

        template<class Name_, class Message_>
        struct Topic: TopicsImplement::TopicBase
        {
            using Name = Name_;
            using Message = Message_;

            static_assert(StewLib::is_stringlike_type_v<Name_>, "1st argument must be StewLib::StringlikeType.");
            static_assert(ros::message_traits::IsMessage<Message_>::value, "2nd argument must be message.");
        };

        template<class CanData_, std::uint16_t id_, class Name_>
        struct CanTxTopic: TopicsImplement::CanTxTopicBase
        {
            using CanData = CanData_;
            constexpr static std::uint16_t id = id_;
            using Name = Name_;

            static_assert(CanMessageC<CanData>, "1st argument dont satisfy CanMessageC.");
            static_assert(StewLib::is_stringlike_type_v<Name_>, "3rd argument must be StewLib::StringlikeType.");

            using Message = CanData::Message;
        };

        template<class CanData_, std::uint16_t id_, class Name_>
        struct CanRxTopic: TopicsImplement::CanRxTopicBase
        {
            using CanData = CanData_;
            constexpr static std::uint16_t id = id_;
            using Name = Name_;

            static_assert(CanMessageC<CanData>, "1st argument dont satisfy CanMessageC.");
            static_assert(StewLib::is_stringlike_type_v<Name_>, "3rd argument must be StewLib::StringlikeType.");

            using Message = CanData::Message;
        };
    }
}