#pragma once

#include <type_traits>

#include <ros/ros.h>

#include "lib/stringlike_type.hpp"
#include "message_convertor/all.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace TopicImplement
        {
            struct TopicBase{};
            struct CanTxTopicBase: TopicBase{};
            struct CanRxTopicBase: TopicBase{};
        }

        template<class T>
        using is_topic = std::is_base_of<TopicImplement::TopicBase, T>;
        template<class T>
        inline constexpr bool is_topic_v = std::is_base_of_v<TopicImplement::TopicBase, T>;

        template<class T>
        using is_can_tx_topic = std::is_base_of<TopicImplement::CanTxTopicBase, T>;
        template<class T>
        inline constexpr bool is_can_tx_topic_v = std::is_base_of_v<TopicImplement::CanTxTopicBase, T>;

        template<class T>
        using is_can_rx_topic = std::is_base_of<TopicImplement::CanRxTopicBase, T>;
        template<class T>
        inline constexpr bool is_can_rx_topic_v = std::is_base_of_v<TopicImplement::CanRxTopicBase, T>;

        template<class T>
        using is_can_topic = std::disjunction<std::is_base_of<TopicImplement::CanTxTopicBase, T>, std::is_base_of<TopicImplement::CanRxTopicBase, T>>;
        template<class T>
        inline constexpr bool is_can_topic_v = std::is_base_of_v<TopicImplement::CanTxTopicBase, T> || std::is_base_of_v<TopicImplement::CanRxTopicBase, T>;

        template<class Name_, class Message_>
        struct Topic: TopicImplement::TopicBase
        {
            using Name = Name_;
            using Message = Message_;

            static_assert(StewLib::is_stringlike_type_v<Name_>, "1st argument must be StewLib::StringlikeType.");
            static_assert(ros::message_traits::IsMessage<Message_>::value, "2nd argument must be message.");
            
            using MessageConvertor = Harurobo2022::MessageConvertor<Message_>;
        };

        template<class Name_, class Message_, std::uint16_t id_>
        struct CanTxTopic: TopicImplement::CanTxTopicBase
        {
            using Name = Name_;
            using Message = Message_;
            constexpr static std::uint16_t id = id_;

            static_assert(StewLib::is_stringlike_type_v<Name_>, "1st argument must be StewLib::StringlikeType.");
            static_assert(ros::message_traits::IsMessage<Message_>::value, "2nd argument must be message.");

            using MessageConvertor = Harurobo2022::MessageConvertor<Message_>;
        };

        template<class Name_, class Message_, std::uint16_t id_>
        struct CanRxTopic: TopicImplement::CanRxTopicBase
        {
            using Name = Name_;
            using Message = Message_;
            constexpr static std::uint16_t id = id_;

            static_assert(StewLib::is_stringlike_type_v<Name_>, "1st argument must be StewLib::StringlikeType.");
            static_assert(ros::message_traits::IsMessage<Message_>::value, "2nd argument must be message.");

            using MessageConvertor = Harurobo2022::MessageConvertor<Message_>;
        };
    }
}