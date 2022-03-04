#pragma once

#include <array>

#include <can_plugins/Frame.h>

#include "lib/serialize.hpp"
#include "message_convertor/all.hpp"
#include "topic.hpp"
#include "publisher.hpp"
#include "stringlike_types.hpp"
#include "static_init_deinit.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace CanPublisherImplement
        {
            struct CanPublisherBase
            {
                using can_tx = Topic<StringlikeTypes::can_tx, can_plugins::Frame>;
                inline static Publisher<can_tx> * canpub_p{nullptr};
            };
        }

        template<class CanTxTopic_>
        class CanPublisher final : protected CanPublisherImplement::CanPublisherBase
        {
        public:
            using CanTxTopic = CanTxTopic_;

            static_assert(is_can_tx_topic_v<CanTxTopic>, "argument must be can tx topic.");

            using MessageConvertor = CanTxTopic::MessageConvertor;
            using Message = CanTxTopic::Message;
        
        private:
            Publisher<CanTxTopic, PublisherOption{.disable_can_tx_topic_warn{true}}> pub;

        public:
            CanPublisher(const std::uint32_t can_queue_size, const std::uint32_t nomal_queue_size = 0) noexcept:
                pub{(nomal_queue_size)? nomal_queue_size : can_queue_size}
            {
                canpub_p->change_buff_size_if_larger(can_queue_size);
            }

            void can_publish(const MessageConvertor& conv) noexcept
            {
                StewLib::Serialize<8, typename MessageConvertor::CanData> serialize{conv};

                for(std::size_t i = 0; i < serialize.chunks_size - 1; ++i)
                {
                    canpub_p->publish({CanTxTopic::id, 8, serialize.chunks[i]});
                }

                canpub_p->publish({CanTxTopic::id, serialize.last_size, serialize.chunks[serialize.chunks_size - 1]});
                pub.publish(conv);
            }

            void publish(const MessageConvertor& conv) noexcept
            {
                pub.publish(conv);
            }

            ros::Publisher get_pub() const noexcept
            {
                return pub.get_pub();
            }

            ros::Publisher get_canpub() const noexcept
            {
                return canpub_p->get_pub();
            }

            void deactivate() noexcept
            {
                pub.deactivate();
            }

            void activate() noexcept
            {
                pub.activate();
            }
        };

        namespace CanPublisherImplement
        {
            inline const auto init =
            []() noexcept
            {
                CanPublisherImplement::CanPublisherBase::canpub_p = new std::remove_pointer_t<decltype(CanPublisherImplement::CanPublisherBase::canpub_p)>{20};
            };

            inline const auto deinit = []() noexcept
            {
                delete CanPublisherImplement::CanPublisherBase::canpub_p;
            };

            inline static const char dummy = 
            []() noexcept
            {
                StaticInitDeinit::initialize_list.push_back(init);
                StaticInitDeinit::deinitialize_list.push_back(deinit);

                return 0;
            }();
        }
    }
}