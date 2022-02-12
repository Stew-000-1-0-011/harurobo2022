/*

*/

#include <ros/ros.h>

#include "harurobo2022/lib/vec2d.hpp"
#include "harurobo2022/lib/circle.hpp"

#include "harurobo2022/topics.hpp"
#include "harurobo2022/shutdowner.hpp"


using namespace StewMath;
using namespace Harurobo2022;

class AutoCommander final
{
    ros::NodeHandle nh{};
    
    ros::Subscriber odometry_sub{nh.subscribe<CanRxTopics::odometry::Message>(CanRxTopics::odometry::topic, 1, &AutoCommander::odometry_callback, this)};
    
    ros::Timer check_position{nh.createTimer(ros::Duration(1.0 / Config::ExecutionInterval::auto_commander_freq), &AutoCommander::check_position_callback)};

    ShutDowner ShutDowner{nh};

    // ひとまずは位置と姿勢をP制御で追う。目標位置姿勢と現在位置姿勢の差を定数倍して並進速度角速度にする。
    Vec2D<float> now_pos{};
    float now_rot_z{};
    Vec2D<float> target_pos{};
    float target_rot_z{};


public:
    AutoCommander() = default;

private:
    inline void odometry_callback(const CanRxTopics::odometry::Message::ConstPtr& msg_p) noexcept;
    inline void check_position_callback(const ros::TimerEvent& event) noexcept;
};

inline void AutoCommander::odometry_callback(const CanRxTopics::odometry::Message::ConstPtr& msg_p) noexcept
{
    now_pos = {msg_p->pos_x, msg_p->pos_y};
    now_rot_z = msg_p->rot_z;
}

inline void check_position_callback(const ros::TimerEvent& event) noexcept
{

}