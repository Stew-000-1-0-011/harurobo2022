#pragma once

#include <type_traits>
#include <string>

#include <ros/ros.h>

#include "lib/stringlike_type.hpp"

#include "can_data_concept.hpp"

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
        };

        template<class CanData_, std::uint16_t id_, class Name_>
        struct CanTxTopic: TopicImplement::CanTxTopicBase
        {
            using CanData = CanData_;
            constexpr static std::uint16_t id = id_;
            using Name = Name_;

            static_assert(CanDataC<CanData>, "1st argument dont satisfy CanDataC.");
            static_assert(StewLib::is_stringlike_type_v<Name_>, "3rd argument must be StewLib::StringlikeType.");

            using Message = CanData::Message;
        };

        template<class CanData_, std::uint16_t id_, class Name_>
        struct CanRxTopic: TopicImplement::CanRxTopicBase
        {
            using CanData = CanData_;
            constexpr static std::uint16_t id = id_;
            using Name = Name_;

            static_assert(CanDataC<CanData>, "1st argument dont satisfy CanDataC.");
            static_assert(StewLib::is_stringlike_type_v<Name_>, "3rd argument must be StewLib::StringlikeType.");

            using Message = CanData::Message;
        };
    }
}