
#include "harurobo2022/topics.hpp"
#include "harurobo2022/active_manager.hpp"
#include "harurobo2022/motors.hpp"
#include "harurobo2022/can_publisher.hpp"

using namespace Harurobo2022;

struct HogeNode final
{
    Publisher<Topics::body_twist> pub1{1};
    Subscriber<Topics::state_topic> sub2{1, [](const Topics::state_topic::Message::ConstPtr&){}};
    DriveMotors drive_motors{};
    LiftMotors lift_motors{};
    
    struct can_hoge_str final : StewLib::StringlikeTypeImplement::StringlikeTypeBase
    {
        constexpr static std::size_t size = sizeof("can_hoge_str");
        constexpr static const char * str = "can_hoge_str";
    };

    using can_hoge = CanTxTopic<can_hoge_str, std_msgs::Float32, 0>;
    CanPublisher<can_hoge> hoge_canpub{1};
    
    ActiveManager<StringlikeTypes::odometry, decltype(pub1), decltype(sub2), decltype(hoge_canpub)> active_manager{pub1, sub2, hoge_canpub};
};

int main()
{
    HogeNode hoge;
    hoge.active_manager.activate();
    ros::spin();
}