#include <ros/ros.h>

#include "harurobo2022/topics.hpp"
#include "harurobo2022/can_publisher.hpp"

using namespace Harurobo2022;

class EmergencyController final
{
    ros::NodeHandle nh{};

    ros::Publisher can_tx_pub{nh.advertise<Topics::can_tx>(Topics::can_tx::topic, 10)};
    ros::Publisher shutdown_pub{nh.advertise<Topics::shutdown>(Topics::shutdown::topic, 1)};

    CanPublisher<CanTxTopics::emergency_stop> emergency_stop_canpub{can_tx_pub};

    ros::Subscriber stopped_sub{nh.subscribe<CanRxTopics::stopped::Message>(CanRxTopics::stopped::topic, 1, &EmergencyController::stopped_callback, this)};

    ros::Timer publish_timer{nh.createTimer(ros::Duration(1.0 / Config::ExecutionInterval::not_emergency_signal_freq), &EmergencyController::publish_timer_callback, this)};

public:
    EmergencyController() = default;

private:
    inline void publish_timer_callback(const ros::TimerEvent& event) const noexcept;
    inline void stopped_callback(const CanRxTopics::stopped::Message::ConstPtr& msg_p) noexcept;
};

inline void EmergencyController::publish_timer_callback(const ros::TimerEvent& event) const noexcept
{
    CanTxTopics::emergency_stop::Message msg;
    msg.data = false;
    emergency_stop_canpub.publish(msg);
}

inline void EmergencyController::stopped_callback(const CanRxTopics::stopped::Message::ConstPtr& msg_p) noexcept
{
    shutdown_pub.publish(Topics::shutdown::Message());
    ros::shutdown();
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "emergency_controller");

    EmergencyController emergency_controller;

    ROS_INFO("emergency_controller node has started.");

    ros::spin();

    ROS_INFO("emergency_controller node has terminated.");

}