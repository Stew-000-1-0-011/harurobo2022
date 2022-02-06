/*
// 1/25
// 通信するデータはそんな大きくもないので、プロセスを別にしたほうが良さげ。ということでNodeletはつかわないつもり(何もしらないので今後変えるかも)
// 1/27
// basecotroller4wheelを参考にした
*/

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include "harurobo2022/vec2d.hpp"
#include "harurobo2022/config.hpp"
#include "harurobo2022/literals_config.hpp"
#include "harurobo2022/topic_message_alias.hpp"
#include "harurobo2022/can_publish.hpp"

using namespace StewMath;
using namespace QuantityUnit::Literals;
using namespace TopicMessageTypeAlias;

class UnderCarriage4Wheel final
{
    ros::NodeHandle nh{};
    ros::Timer publish_timer{nh.createTimer(ros::Duration(1.0 / Config::under_carriage_freq.value), &UnderCarriage4Wheel::publish_timer_callback, this)};

    // ros::Publisher wheel_FR_vela_pub{nh.advertise<wheel_FR_vela>(TOPIC(wheel_FR_vela), 1)};
    // ros::Publisher wheel_FL_vela_pub{nh.advertise<wheel_FL_vela>(TOPIC(wheel_FL_vela), 1)};
    // ros::Publisher wheel_BL_vela_pub{nh.advertise<wheel_BL_vela>(TOPIC(wheel_BL_vela), 1)};
    // ros::Publisher wheel_BR_vela_pub{nh.advertise<wheel_BR_vela>(TOPIC(wheel_BR_vela), 1)};

    ros::Publisher can_tx_pub{nh.advertise<can_tx>(TOPIC(can_tx), 1)};

    ros::Subscriber body_twist_sub{nh.subscribe<body_twist>(TOPIC(body_twist), 1, &UnderCarriage4Wheel::body_twist_callback, this)};

    Vec2D<VelL<double>> body_vell{};
    VelA<double> body_vela{};

    std_msgs::Float32 wheels_vela_msg[4]{};
    VelA<double> pre_wheels_vela[4]{};

public:
    UnderCarriage4Wheel() = default;
    ~UnderCarriage4Wheel() = default;

private:
    inline void body_twist_callback(const body_twist::ConstPtr& msg_p) noexcept;
    inline void publish_timer_callback(const ros::TimerEvent& event) noexcept;
    inline void calc_wheels_vela() noexcept;
};

inline void UnderCarriage4Wheel::body_twist_callback(const body_twist::ConstPtr& msg_p) noexcept
{
    body_vell = {msg_p->linear.x, msg_p->linear.y};
    body_vela = msg_p->angular.z;
}

inline void UnderCarriage4Wheel::publish_timer_callback(const ros::TimerEvent& event) noexcept
{
    calc_wheels_vela();

    // wheel_FR_vela_pub.publish(wheels_vela_msg[0]);
    // wheel_FL_vela_pub.publish(wheels_vela_msg[1]);
    // wheel_BL_vela_pub.publish(wheels_vela_msg[2]);
    // wheel_BR_vela_pub.publish(wheels_vela_msg[3]);
    
    for(int i = 0; i < 4; ++i)
    {
        CanPublish::can_publish(can_tx_pub, Config::CanId::DriveMotor::all[i], wheels_vela_msg[i]);
    }
}

inline void UnderCarriage4Wheel::calc_wheels_vela() noexcept
{
    using namespace Config::Wheel;

    const auto body_vell = this->body_vell;
    const auto body_vela = this->body_vela;
    VelA<double> wheels_vela[4];

    const VelL<double> tmp_vela = body_vela * (Config::body_radius * rot(~Pos::FR,Constant::PI_2) * ~Direction::FR);
    for(int i = 0; i < 4; ++i)
    {
        wheels_vela[i] = (~Direction::all[i] * body_vell + tmp_vela) / Config::wheel_radius;
    }

    if constexpr (Config::wheel_acca_limit)
    {
        AccA<double> diffs_vela[4];
        for(int i = 0; i < 4; ++i)
        {
            diffs_vela[i] = (AccA<double>)pre_wheels_vela[i] - (AccA<double>)wheels_vela[i];
        }

        auto max = diffs_vela[0];
        for(int i = 1; i < 4; ++i)
        {
            if(max < diffs_vela[i]) max = diffs_vela[i];
        }
        
        if(max > Config::wheel_acca_limit)
        {
            ROS_WARN("under_carriage_4wheel: warning: The accelaretion of the wheels is too high. Speed is limited.");
            auto limit_factor = Config::wheel_acca_limit / max;
            for(int i = 0; i < 4; ++i)
            {
                wheels_vela[i] += pre_wheels_vela[i] + (VelA<double>)(limit_factor * diffs_vela[i]);
            }
        }

        for(int i = 0; i < 4; ++i)
        {
            wheels_vela_msg[i].data = wheels_vela[i].value;
        }
    }

    if constexpr (Config::wheel_vela_limit)
    {
        auto max = wheels_vela[0];
        for(int i = 1; i < 4; ++i)
        {
            if(max < wheels_vela[i]) max = wheels_vela[i];
        }
        
        if(max > Config::wheel_vela_limit)
        {
            ROS_WARN("under_carriage_4wheel: warning: The speed of the wheels is too high. Speed is limited.");
            auto limit_factor = Config::wheel_vela_limit / max;
            for(int i = 0; i < 4; ++i)
            {
                wheels_vela[i] *= limit_factor;
            }
        }
    }

    if constexpr(Config::wheel_acca_limit)
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
