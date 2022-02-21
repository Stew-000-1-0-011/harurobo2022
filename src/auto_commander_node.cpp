/*

*/

#include <ros/ros.h>

#include "harurobo2022/lib/vec2d.hpp"
#include "harurobo2022/lib/circle.hpp"

#include "harurobo2022/topics.hpp"
#include "harurobo2022/shutdown_subscriber.hpp"
#include "harurobo2022/activate_subscriber.hpp"
#include "harurobo2022/can_publisher.hpp"
#include "harurobo2022/state.hpp"
#include "harurobo2022/chart.hpp"


using namespace StewMath;
using namespace Harurobo2022;

const char *const node_name = "auto_commander";

class AutoCommanderNode final
{
    ros::NodeHandle nh{};

    ros::Publisher can_tx_pub{nh.advertise<Topics::can_tx::Message>(Topics::can_tx::topic, /*TODO*/20)};

    CanPublisher<CanTxTopics::> collector_lift{};

    ros::Subscriber odometry_sub{nh.subscribe<CanRxTopics::odometry::Message>(CanRxTopics::odometry::topic, 1, &AutoCommanderNode::odometry_callback, this)};
    
    ros::Timer check_position{nh.createTimer(ros::Duration(1.0 / Config::ExecutionInterval::auto_commander_freq), &AutoCommanderNode::check_position_callback, this)};

    ShutDownSubscriber shutdown_sub{nh};
    StateManager state_manager{nh};

    // ひとまずは位置と姿勢を速度上限つきP制御で追う。目標位置姿勢と現在位置姿勢の差を定数倍して並進速度角速度にする。
    Vec2D<float> now_pos{};
    float now_rot_z{};
    
    std::list<Command>::const_iterator target_iter{trajectory.cbegin()};
    std::list<std::list<Command>>::iterator missions_iter{steps.begin()};

public:
    ActivateSubscriber activate_sub{node_name, nh};

public:
    AutoCommanderNode() = default;

private:
    inline void odometry_callback(const CanRxTopics::odometry::Message::ConstPtr& msg_p) noexcept;
    inline void check_position_callback(const ros::TimerEvent& event) noexcept;
    inline void do_work(const Work work) noexcept;
    inline void case_collector_down() noexcept;
    inline void case_collector_up() noexcept;
    inline void case_collector_shovel() noexcept;
    inline void case_collector_tablecloth() noexcept;
};

inline void AutoCommanderNode::odometry_callback(const CanRxTopics::odometry::Message::ConstPtr& msg_p) noexcept
{
    if(!activate_sub.is_active()) return;

    now_pos = {msg_p->pos_x, msg_p->pos_y};
    now_rot_z = msg_p->rot_z;
}

inline void AutoCommanderNode::check_position_callback(const ros::TimerEvent& event) noexcept
{
    if(!activate_sub.is_active()) return;

    if(missions_iter->empty()) return;

    for(auto mission_iter = missions_iter->begin(); mission_iter != missions_iter->end();)
    {
        if(mission_iter->pass_near_circle.is_in(now_pos))
        {
            do_work(mission_iter->work);
            mission_iter = missions_iter->erase(mission_iter);
        }
        else
        {
            ++mission_iter;
        }
    }
}

/* TODO: 実装 */
inline void AutoCommanderNode::do_work(const Work work) noexcept
{
    switch(work)
    {
    case Work::transit:
        break;
    
    case Work::collector_down:
        case_collector_down();
        break;
    
    case Work::collector_up:
        case_collector_up();
        break;

    case Work::collector_shovel:
        case_collector_shovel();
        break;
    
    case Work::collector_tablecloth:
        case_collector_tablecloth();
        break;

    case Work::change_to_over_fence:
        break;
    }
}

inline void AutoCommanderNode::case_collector_down() noexcept
{
    can_publish(can_tx_pub, )
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, node_name);

    AutoCommanderNode auto_commander_node;

    ROS_INFO("%s node has started.", node_name);

    ros::spin();

    ROS_INFO("%s node has terminated.", node_name);
}