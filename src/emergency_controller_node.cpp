#include <ros/ros.h>

#include "harurobo2022/topics.hpp"
#include "harurobo2022/can_publisher.hpp"
#include "harurobo2022/state.hpp"
#include "harurobo2022/shutdown_subscriber.hpp"

using namespace Harurobo2022;

const char *const node_name = "emergency_controller";

class EmergencyControllerNode final
{
    ros::NodeHandle nh{};

    ros::Publisher can_tx_pub{nh.advertise<Topics::can_tx::Message>(Topics::can_tx::topic, 10)};
    ros::Publisher shutdown_pub{nh.advertise<Topics::shutdown_::Message>(Topics::shutdown_::topic, 1)};

    CanPublisher<CanTxTopics::emergency_stop> emergency_stop_canpub{can_tx_pub};

    StateManager state_manager{nh};
    ShutDownSubscriber shutdown_sub{nh};


    ros::Subscriber stopped_sub{nh.subscribe<CanRxTopics::stopped::Message>(CanRxTopics::stopped::topic, 1, &EmergencyControllerNode::stopped_callback, this)};

    ros::Timer publish_timer{nh.createTimer(ros::Duration(1.0 / Config::ExecutionInterval::not_emergency_signal_freq), &EmergencyControllerNode::publish_timer_callback, this)};

public:
    EmergencyControllerNode() = default;

private:
    inline void publish_timer_callback(const ros::TimerEvent& event) const noexcept;
    inline void stopped_callback(const CanRxTopics::stopped::Message::ConstPtr& msg_p) noexcept;
};

inline void EmergencyControllerNode::publish_timer_callback(const ros::TimerEvent& event) const noexcept
{
    CanTxTopics::emergency_stop::Message msg;
    msg.data = false;
    emergency_stop_canpub.publish(msg);
}

inline void EmergencyControllerNode::stopped_callback(const CanRxTopics::stopped::Message::ConstPtr& msg_p) noexcept
{
    state_manager.set_state(State::shutdown);
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, node_name);

    EmergencyControllerNode emergency_controller_node;

    ROS_INFO("%s node has started.", node_name);

    ros::spin();

    ROS_INFO("%s node has terminated.", node_name);

}