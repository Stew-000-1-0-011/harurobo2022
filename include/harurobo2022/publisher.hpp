/*

ros::Publisherを包んだクラス。劣化版。もともとある機能のうち色々使えない。

トピックから型であるメッセージなどにアクセスしたかったのでトピックを表すコンパイル時定数ができ、
そいつにパブリッシャーからアクセスしたかったのでこうなった。

ros::AdveriseOptionsを見るといかに貧弱になってるかがわかる。気が向いたらなんとかしたいが、とても実装力が足りない気がする。

しかし、キューのサイズを変えるのは難しい(同じトピックのPublisherが生きていると実は変わらない？
(ros::TopicManager::advertised_topics_の中に既にあると変わらない？))し、
トピックの名前を変えるというのも、ましてトピックに対応するメッセージ型を変えるというのも使う機会が思いつかない。
どちらかというと間違いの元な気もする。

そこで、複数インスタンスを作るのは禁止とし、トピック名やメッセージ型は固定とした。

*/

#pragma once

#include <string>

#include <ros/ros.h>

#include "lib/stringlike_type.hpp"
#include "topic.hpp"
#include "message_convertor/all.hpp"

#include "harurobo2022/lib/macro/static_warn.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace PublisherImplement
        {
            // template<class TopicName>  // できるようになったらコンセプト付ける
            // inline bool is_published{false};

            // internal compiler error で segmentation fault. 継承しちゃいけんのか？
            // おそらく単なるメモリ不足。やはりStewLib::Stringは重い...。
            // StriglikeTypeで対応した。
            
            struct PublisherBase
            {
            protected:
                // static_assert(StewLib::is_stringlike_type_v<TopicName>, "argument must be StewLib::StringlikeType.");
                template<class TopicName>  // コンセプト
                inline static bool is_published{false};
            };
        }

        struct PublisherOption final
        {
            bool disable_can_rx_topic_assert{false};
            bool disable_can_tx_topic_warn{false};
        };

        // レイアウトは標準でないとはいえ同じはずなのにそのままコンテナに詰められないのはなんでなんだ。
        // 詰めるときに無理やりポインタをキャストすれば(逆参照した時点で？)未定義動作だし、共用体も未定義動作になるはず。
        template<class Topic_, PublisherOption opt = PublisherOption()>
        class Publisher final : PublisherImplement::PublisherBase
        {
        public:
            using Topic = Topic_;

        private:
            static_assert(is_topic_v<Topic>, "argument must be topic.");
            static_assert(!is_can_rx_topic_v<Topic> || opt.disable_can_rx_topic_assert, "argument must not be can_rx topic.");
            Stew_static_warn(is_can_tx_topic_v<Topic> || opt.disable_can_tx_topic_warn, "argument is can tx topic. you should use Harurobo2022::CanPublisher.");

        public:
            using MessageConvertor = Topic::MessageConvertor;
            using Message = Topic::Message;
            using TopicName = Topic::Name;

        private:
            // NodeHandleを複数個作ってもいいのかわからなかった。
            ros::NodeHandle nh{};
            std::uint32_t queue_size;
            ros::Publisher pub;

        public:
            Publisher(const std::uint32_t queue_size) noexcept:
                queue_size{queue_size},
                pub{nh.advertise<Message>(TopicName::str, queue_size)}
            {
                if(is_published<TopicName>)
                {
                    ROS_ERROR("Instance of Harurobo2022::Publisher for %s has already constracted and not destructed.", TopicName::str);
                }

                is_published<TopicName> = true;
            }

            ~Publisher() noexcept
            {
                is_published<TopicName> = false;
            }

            Publisher(const Publisher&) = delete;
            Publisher& operator=(const Publisher&) = delete;
            Publisher(Publisher&&) = delete;
            Publisher& operator=(Publisher&&) = delete;

            void publish(const MessageConvertor& conv) const noexcept
            {
                if(pub) pub.publish(static_cast<Message>(conv));
            }

            void change_buff_size(const std::uint32_t changed_queue_size) noexcept
            {
                pub = nh.advertise<Message>(TopicName::str, changed_queue_size);
                queue_size = changed_queue_size;
            }

            void change_buff_size_if_larger(const std::uint32_t changed_queue_size) noexcept
            {
                if(changed_queue_size > queue_size)
                {
                    change_buff_size(changed_queue_size);
                }
            }

            ros::Publisher get_pub() const noexcept
            {
                return pub;
            }

            void deactivate() noexcept
            {
                pub = ros::Publisher();
            }

            void activate() noexcept
            {
                pub = nh.advertise<Message>(TopicName::str, queue_size);
            }
        };
    }
}