/*
// 1/25
// 通信するデータはそんな大きくもないので、プロセスを別にしたほうが良さげ。ということでNodeletはつかわないつもり(何もしらないので今後変えるかも)
// 1/27
// basecotroller4wheelを参考にした
*/

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include "harurobo2022/lib/vec2d.hpp"
#include "harurobo2022/config.hpp"
#include "harurobo2022/topics.hpp"
#include "harurobo2022/can_publisher.hpp"

using namespace StewMath;
using namespace Harurobo2022;

class UnderCarriage4Wheel final
{
    ros::NodeHandle nh{};
    ros::Timer publish_timer{nh.createTimer(ros::Duration(1.0 / Config::ExecutionInterval::under_carriage_freq), &UnderCarriage4Wheel::publish_timer_callback, this)};

#define WheelVelaPublisher(topic_name) Harurobo2022::CanPublisher<Topics::topic_name> topic_name##_pub{nh.advertise<Topics::topic_name::Message>(Topics::topic_name::topic, 1)};
    WheelVelaPublisher(wheel_FR_vela)
    WheelVelaPublisher(wheel_FL_vela)
    WheelVelaPublisher(wheel_BL_vela)
    WheelVelaPublisher(wheel_BR_vela)
#undef WheelVelaPublisher

    ros::Subscriber body_twist_sub{nh.subscribe<Topics::body_twist::Message>(Topics::body_twist::topic, 1, &UnderCarriage4Wheel::body_twist_callback, this)};

    Vec2D<double> body_vell{};
    double body_vela{};

    double wheels_vela[4]{};
    double pre_wheels_vela[4]{};

public:
    UnderCarriage4Wheel() = default;
    ~UnderCarriage4Wheel() = default;

private:
    inline void body_twist_callback(const Topics::body_twist::Message::ConstPtr& msg_p) noexcept;
    inline void publish_timer_callback(const ros::TimerEvent& event) noexcept;
    inline void calc_wheels_vela() noexcept;
};

inline void UnderCarriage4Wheel::body_twist_callback(const Topics::body_twist::Message::ConstPtr& msg_p) noexcept
{
    body_vell = {msg_p->linear.x, msg_p->linear.y};
    body_vela = msg_p->angular.z;
}

inline void UnderCarriage4Wheel::publish_timer_callback(const ros::TimerEvent& event) noexcept
{
    calc_wheels_vela();

    wheel_FR_vela_pub.publish(wheels_vela[0]);
    wheel_FL_vela_pub.publish(wheels_vela[1]);
    wheel_BL_vela_pub.publish(wheels_vela[2]);
    wheel_BR_vela_pub.publish(wheels_vela[3]);
}

inline void UnderCarriage4Wheel::calc_wheels_vela() noexcept
{
    using namespace Config::Wheel;

    const auto body_vell = this->body_vell;
    const auto body_vela = this->body_vela;
    double wheels_vela[4];

    const double tmp_vela = body_vela * (Config::body_radius * rot(~Pos::FR,Constant::PI_2) * ~Direction::FR);
    for(int i = 0; i < 4; ++i)
    {
        wheels_vela[i] = (~Direction::all[i] * body_vell + tmp_vela) / Config::wheel_radius;
    }

    if constexpr (Config::Limitation::wheel_acca)
    {
        double diffs_vela[4];
        for(int i = 0; i < 4; ++i)
        {
            diffs_vela[i] = pre_wheels_vela[i] - wheels_vela[i];
        }

        auto max = diffs_vela[0];
        for(int i = 1; i < 4; ++i)
        {
            if(max < diffs_vela[i]) max = diffs_vela[i];
        }
        
        if(max > Config::Limitation::wheel_acca)
        {
            ROS_WARN("under_carriage_4wheel: warning: The accelaretion of the wheels is too high. Speed is limited.");
            auto limit_factor = Config::Limitation::wheel_acca / max;
            for(int i = 0; i < 4; ++i)
            {
                wheels_vela[i] += pre_wheels_vela[i] + (limit_factor * diffs_vela[i]);
            }
        }

        for(int i = 0; i < 4; ++i)
        {
            this->wheels_vela[i] = wheels_vela[i];
        }
    }

    if constexpr (Config::Limitation::wheel_vela)
    {
        auto max = wheels_vela[0];
        for(int i = 1; i < 4; ++i)
        {
            if(max < wheels_vela[i]) max = wheels_vela[i];
        }
        
        if(max > Config::Limitation::wheel_vela)
        {
            ROS_WARN("under_carriage_4wheel: warning: The speed of the wheels is too high. Speed is limited.");
            auto limit_factor = Config::Limitation::wheel_vela / max;
            for(int i = 0; i < 4; ++i)
            {
                wheels_vela[i] *= limit_factor;
            }
        }
    }

    if constexpr(Config::Limitation::wheel_acca)
    {
        for(int i = 0; i < 4; ++i)
        {
            pre_wheels_vela[i] = wheels_vela[i];
        }
    }
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "under_carriage_4wheel");

    UnderCarriage4Wheel under_carriage_4wheel;

    ROS_INFO("under_carriage_4wheel node has started.");

    ros::spin();

    ROS_INFO("under_carriage_4wheel node has terminated.");

}
