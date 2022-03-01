#pragma once

#include <array>

#include <boost/array.hpp>

#include <can_plugins/Frame.h>

#include "lib/serialize.hpp"

#include "message_convertor/message_convertor_all.hpp"
#include "topic.hpp"
#include "publisher.hpp"

#include "stringlike_types.hpp"


namespace Harurobo2022
{
    namespace
    {
        namespace CanPublisherImplement
        {
            struct CanPublisherBase
            {
            private:
                using can_tx = Topic<StringlikeTypes::can_tx, can_plugins::Frame>;
            protected:
                inline static Publisher<can_tx> canpub{1};
            };
        }

        template<class CanTxTopic_>
        class CanPublisher final : CanPublisherImplement::CanPublisherBase
        {
        public:
            using CanTxTopic = CanTxTopic_;

            static_assert(is_can_tx_topic_v<CanTxTopic>, "argument must be can tx topic.");

            using CanData = CanTxTopic::CanData;
            using Message = CanTxTopic::CanData::Message;
        
        private:
            Publisher<CanTxTopic, PublisherOption{.disable_can_tx_topic_warn{true}}> pub;

        public:
            CanPublisher(const std::uint32_t can_queue_size, const std::uint32_t nomal_queue_size = 0) noexcept:
                pub{(nomal_queue_size)? nomal_queue_size : can_queue_size}
            {
                canpub.change_buff_size_if_larger(can_queue_size);
            }

            void can_publish(const CanData& can_data) noexcept
            {
                StewLib::Serialize serialize{can_data.data};

                if constexpr(serialize.chunks_size)
                {
                    for(std::size_t i = 0; i < serialize.chunks_size; ++i)
                    {
                        canpub.publish({CanTxTopic::id, 8, serialize.chunks[i]});
                    }
                }

                canpub.publish({CanTxTopic::id, serialize.last_size, serialize.last_chunk});
            }

            void publish(const CanData& can_data) noexcept
            {
                pub.publish(can_data);
            }

            ros::Publisher get_pub() const noexcept
            {
                return pub.get_pub();
            }

            ros::Publisher get_canpub() const noexcept
            {
                return canpub.get_pub();
            }

            void deactivate() noexcept
            {
                canpub.deactivate();
                pub.deactivate();
            }

            void activate() noexcept
            {
                canpub.activate();
                pub.activate();
            }
        };
    }
}