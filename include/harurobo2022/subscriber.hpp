/*

もうリファレンスというかコード読むの辛い。なんとなーく同一トピックに複数のSbuscriberがあるとまずそう。なんとなーく。

あ、コールバック関数を複数登録する方法って作れるのかな。それ面白そうだ。
コンパイル時にコールバック関数を追加する方法をなかなか思いつかなかったのと、そもそも使用例がなさそうなのと、
なにより春ロボまでの残り時間が少ないので今回は諦める。

*/

#pragma once

#include <cstdint>
#include <functional>

#include <ros/ros.h>

#include "topic.hpp"


namespace Harurobo2022
{
    namespace
    {
        namespace SubscriberImplement
        {
            struct SubscriberBase
            {
            protected:
                // static_assert(StewLib::is_stringlike_type_v<TopicName>, "argumnt must be StewLib::Stringliketype");
                template<class TopicName> // コンセプト使う
                inline static bool is_subscribed{false};
            };
        }

        struct SubscriberOption final
        {
            bool disable_can_tx_topic_assert{false};
        };

        // ここをもう少し綺麗に推論したかった。
        template<class Topic_, SubscriberOption opt = SubscriberOption()>
        class Subscriber final : SubscriberImplement::SubscriberBase
        {
        public:
            using Topic = Topic_;
        
        private:
            static_assert(is_topic_v<Topic>, "1st argument must be topic.");
            static_assert(!is_can_tx_topic_v<Topic> || opt.disable_can_tx_topic_assert , "1st argument must not be can_tx topic.");

        public:
            using Message = Topic::Message;
            using TopicName = Topic::Name;

        private:
            using CallbackSignature = void(const typename Message::ConstPtr&);
            // ros::NodeHandleがわからない...ってかROSわかんないよぉ...
            ros::NodeHandle nh{};
            std::uint32_t queue_size;
            std::function<CallbackSignature> callback;
            ros::Subscriber sub;

        public:
            template<class F>
            Subscriber(const std::uint32_t queue_size,const F& callback) noexcept:
                queue_size{queue_size},
                callback{callback},
                sub{nh.subscribe<Message>(TopicName::str, queue_size, callback)}
            {
                if(is_subscribed<TopicName>)
                {
                    ROS_ERROR("Instance of Harurobo2022::Subscriber for %s has already constracted and not destructed.", TopicName::str);
                }

                is_subscribed<TopicName> = true;
            }

            ~Subscriber() noexcept
            {
                is_subscribed<TopicName> = false;
            }

            Subscriber(const Subscriber&) = delete;
            Subscriber& operator=(const Subscriber&) = delete;
            Subscriber(Subscriber&&) = delete;
            Subscriber& operator=(Subscriber&&) = delete;

            void change_buff_size(const std::uint32_t changed_queue_size) noexcept
            {
                sub = nh.subscribe<Message>(TopicName::str, changed_queue_size, callback);
                queue_size = changed_queue_size;
            }

            void change_buff_size_if_larger(const std::uint32_t changed_queue_size) noexcept
            {
                if(changed_queue_size > queue_size)
                {
                    change_buff_size(changed_queue_size);
                }
            }

            template<class F>
            void change_callback(const F& changed_callback) noexcept
            {
                sub = nh.subscribe<Message>(TopicName::str, queue_size, changed_callback);
                callback = changed_callback;
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
                sub = nh.subscribe<Message>(TopicName::str, queue_size, callback);
            }
        };
    }
}