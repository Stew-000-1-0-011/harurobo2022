
#include <std_msgs/UInt32.h>

#include "harurobo2022/can_publisher.hpp"
#include "harurobo2022/subscriber.hpp"

#include "harurobo2022/lib/macro/define_stringlike_type.hpp"

Stew_stringlikeType(can_hoge)

using namespace Harurobo2022;

struct can_hoge_data
{
    using Message = std_msgs::UInt32;
    operator Message() const noexcept
    {
        return Message();
    }
};

using can_hoge = CanTxTopic<can_hoge_data, 666, StewLib::StringlikeTypes::can_hoge>;
void func(const typename can_hoge::Message::ConstPtr&){}

int main()
{
    CanPublisher<can_hoge> canpub{1};
    Subscriber<can_hoge, &func, void, SubscriberOption{.disable_can_tx_topic_assert = true}> sub{1};

}