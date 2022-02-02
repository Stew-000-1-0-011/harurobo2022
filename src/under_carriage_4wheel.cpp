/*
// 1/25
// 通信するデータはそんな大きくもないので、プロセスを別にしたほうが良さげ。ということでNodeletはつかわないつもり(何もしらないので今後変えるかも)
// 1/27
// basecotroller4wheelをパクることに。
// 1/28
// 使ってる関数の例外送出の可能性をいちいち調べるのは面倒なので、コンテナに詰めるオブジェクトのmove_ctorでもなきゃ気にしないことにした。
*/

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include "Harurobo2022/vec2d.hpp"
#include "Harurobo2022/config.hpp"
#include "Harurobo2022/literals_config.hpp"

using namespace StewMath;
using namespace QuantityUnit::Literals;

class UnderCarriage4Wheel final
{
    ros::NodeHandle nh;
    ros::Timer publish_timer;

    ros::Publisher wheel_FR_vela_pub;
    ros::Publisher wheel_FL_vela_pub;
    ros::Publisher wheel_BL_vela_pub;
    ros::Publisher wheel_BR_vela_pub;

    ros::Subscriber body_twist_sub;

    Vec2D<VelL<double>> body_vell;
    VelA<double> body_vela;

public:
    inline UnderCarriage4Wheel();  // 一応明示しておく。
    UnderCarriage4Wheel(const UnderCarriage4Wheel&) = delete;
    UnderCarriage4Wheel(UnderCarriage4Wheel&&) = delete;
    ~UnderCarriage4Wheel() = default;

private:
    inline void body_twist_callback(const geometry_msgs::Twist::ConstPtr& msg_p);
    inline void publish_timer_callback(const ros::TimerEvent& event);
};

inline UnderCarriage4Wheel::UnderCarriage4Wheel():
    nh{},
    publish_timer{nh.createTimer(ros::Duration(1.0 / Config::under_carriage_freq.get_primitive()), &UnderCarriage4Wheel::publish_timer_callback, this)},

    wheel_FR_vela_pub{nh.advertise<std_msgs::Float32>("wheel_FR_vela", 1)},
    wheel_FL_vela_pub{nh.advertise<std_msgs::Float32>("wheel_FL_vela", 1)},
    wheel_BL_vela_pub{nh.advertise<std_msgs::Float32>("wheel_BL_vela", 1)},
    wheel_BR_vela_pub{nh.advertise<std_msgs::Float32>("wheel_BR_vela", 1)},

    body_twist_sub{nh.subscribe<geometry_msgs::Twist>("body_twist", 1, &UnderCarriage4Wheel::body_twist_callback, this)}
{}

inline void UnderCarriage4Wheel::body_twist_callback(const geometry_msgs::Twist::ConstPtr& msg_p)
{
    body_vell = {VelL<double>(msg_p->linear.x), VelL<double>(msg_p->linear.y)};
    body_vela = VelA<double>(msg_p->angular.z);
}

inline void UnderCarriage4Wheel::publish_timer_callback(const ros::TimerEvent& event)
{
    using namespace Config::Wheel;

    const auto body_vell = this->body_vell;
    const auto body_vela = this->body_vela;

    const Vec2D<Dim0<double>> tmp3 = rot(~Pos::FR,Constant::PI_2);

    const Vec2D<M<double>> tmp2 = Config::body_radius * tmp3;

    const Vec2D<M<double>> tmp = tmp2 * ~Direction::FR;

    const Vec2D<VelL<double>> tmp_vela = body_vela * tmp;//(Config::body_radius * rot(~Pos::FR,Constant::PI_2) * ~Direction::FR);

    VelA<double> FR = (~Direction::FR * body_vell + tmp_vela) / Config::wheel_radius;
    VelA<double> FL = (~Direction::FL * body_vell + tmp_vela) / Config::wheel_radius;
    VelA<double> BL = (~Direction::BL * body_vell + tmp_vela) / Config::wheel_radius;
    VelA<double> BR = (~Direction::BR * body_vell + tmp_vela) / Config::wheel_radius;
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "under_carriage_4wheel");

    auto under_carriage_4wheel = UnderCarriage4Wheel();

    ROS_INFO("under_carriage_4wheel node has started.");

    ros::spin();

    ROS_INFO("under_carriage_4wheel node has terminated.");

}
