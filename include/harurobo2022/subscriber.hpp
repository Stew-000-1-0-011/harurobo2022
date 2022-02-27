/*

もうリファレンスというかコード読むの辛い。なんとなーく同一トピックに複数のSbuscriberがあるとまずそう。なんとなーく。

あ、コールバック関数を複数登録する方法って作れるのかな。それ面白そうだ。
コンパイル時にコールバック関数を追加する方法をなかなか思いつかなかったのと、そもそも使用例がなさそうなのと、
なにより春ロボまでの残り時間が少ないので今回は諦める。

*/

#pragma once

#include <cstdint>

#include <ros/ros.h>

#include "topic.hpp"


namespace Harurobo2022
{
    namespace
    {
        namespace SubscriberImplement
        {
            template<class TopicName>
            struct SubscriberBase
            {
                static_assert(StewLib::is_stringlike_type_v<TopicName>, "argumnt must be StewLib::Stringliketype");
            protected:
                inline static bool is_subscribed{false};
            };
        }

        struct SubscriberOption final
        {
            bool disable_can_tx_topic_assert{false};
        };

        // ここをもう少し綺麗に推論したかった。
        template<class Topic_, auto callback_p_, class ClassOfCallback = void, SubscriberOption opt = SubscriberOption()>
        class Subscriber final : SubscriberImplement::SubscriberBase<typename Topic_::Name>
        {
        public:
            using Topic = Topic_;
            constexpr static auto callback_p = callback_p_;
            using CallbackPType = decltype(callback_p_);
        
        private:
            static_assert(is_topic_v<Topic>, "1st argument must be topic.");
            static_assert(is_can_tx_topic_v<Topic> || opt.disable_can_tx_topic_assert , "1st argument must not be can_tx topic.");
            static_assert
                (
                    std::is_same_v<std::invoke_result_t<CallbackPType, const typename Topic::Message::ConstPtr&>, void>,
                    "2nd argument must be void(ClassOfCallback:: *)(const Message::Ptr&) opt_const opt_noexcept or void(*)(const Message::Ptr&) opt_const opt_noexcept."
                );

        public:
            using Message = Topic::Message;
            using TopicName = Topic::Name;

        private:
            // ros::NodeHandleがわからない...ってかROSわかんないよぉ...
            ros::NodeHandle nh{};
            std::uint32_t queue_size;
            ros::Subscriber sub;

        public:
            Subscriber(const std::uint32_t queue_size) noexcept:
                queue_size{queue_size},
                sub{nh.subscribe<Message>(TopicName::str, queue_size, callback_p, static_cast<CallbackPType *>(nullptr))}
            {
                if(Subscriber::is_subscribed)
                {
                    ROS_ERROR("Instance of Harurobo2022::Subscriber for %s has already constracted and not destructed.", TopicName::str);
                }

                Subscriber::is_subscribed = true;
            }

            ~Subscriber() noexcept
            {
                Subscriber::is_subscribed = false;
            }

            Subscriber(const Subscriber&) = delete;
            Subscriber& operator=(const Subscriber&) = delete;
            Subscriber(Subscriber&&) = delete;
            Subscriber& operator=(Subscriber&&) = delete;

            void change_buff_size(const std::uint32_t changed_queue_size) noexcept
            {
                sub = nh.subscribe<Message>(TopicName::str, changed_queue_size, callback_p, static_cast<CallbackPType *>(nullptr));
                queue_size = changed_queue_size;
            }

            void change_buff_size_if_larger(const std::uint32_t changed_queue_size) noexcept
            {
                if(changed_queue_size > queue_size)
                {
                    change_buff_size(changed_queue_size);
                }
            }

            ros::Subscriber get_sub() const noexcept
            {
                return sub;
            }

            void deactivate() noexcept
            {
                sub = ros::Subscriber();
            }

            void activate() noexcept
            {
                sub = nh.subscribe<Message>(TopicName::str, queue_size, callback_p, static_cast<ClassOfCallback *>(nullptr));
            }
        };

        template<class Topic_, auto callback_p_, SubscriberOption opt>
        class Subscriber<Topic_, callback_p_, void, opt> final : SubscriberImplement::SubscriberBase<typename Topic_::Name>
        {
        public:
            using Topic = Topic_;
            constexpr static auto callback_p = callback_p_;
            using CallbackPType = decltype(callback_p_);
        
        private:
            static_assert(is_topic_v<Topic>, "1st argument must be topic.");
            static_assert(is_can_tx_topic_v<Topic> || opt.disable_can_tx_topic_assert , "1st argument must not be can_tx topic.");
            static_assert
                (
                    std::is_same_v<std::invoke_result_t<CallbackPType, const typename Topic::Message::ConstPtr&>, void>,
                    "2nd argument must be void(ClassOfCallback:: *)(const Message::Ptr&) opt_const opt_noexcept or void(*)(const Message::Ptr&) opt_const opt_noexcept."
                );

        public:
            using Message = Topic::Message;
            using TopicName = Topic::Name;

        private:
            // ros::NodeHandleがわからない...ってかROSわかんないよぉ...
            ros::NodeHandle nh{};
            std::uint32_t queue_size;
            ros::Subscriber sub;

        public:
            Subscriber(const std::uint32_t queue_size) noexcept:
                queue_size{queue_size},
                sub{nh.subscribe<Message>(TopicName::str, queue_size, callback_p)}
            {
                if(Subscriber::is_subscribed)
                {
                    ROS_ERROR("Instance of Harurobo2022::Subscriber for %s has already constracted and not destructed.", TopicName::str);
                }

                Subscriber::is_subscribed = true;
            }

            ~Subscriber() noexcept
            {
                Subscriber::is_subscribed = false;
            }

            Subscriber(const Subscriber&) = delete;
            Subscriber& operator=(const Subscriber&) = delete;
            Subscriber(Subscriber&&) = delete;
            Subscriber& operator=(Subscriber&&) = delete;

            void change_buff_size(const std::uint32_t changed_queue_size) noexcept
            {
                sub = nh.subscribe<Message>(TopicName::str, changed_queue_size, callback_p);
                queue_size = changed_queue_size;
            }

            void change_buff_size_if_larger(const std::uint32_t changed_queue_size) noexcept
            {
                if(changed_queue_size > queue_size)
                {
                    change_buff_size(changed_queue_size);
                }
            }

            ros::Subscriber get_sub() const noexcept
            {
                return sub;
            }

            void deactivate() noexcept
            {
                sub = ros::Subscriber();
            }

            void activate() noexcept
            {
                sub = nh.subscribe<Message>(TopicName::str, queue_size, callback_p);
            }
        };
    }
}