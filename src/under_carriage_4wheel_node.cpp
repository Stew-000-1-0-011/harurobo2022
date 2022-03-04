/*
base_controllerを参考にした。
機体に固定された座標でのgeometry::Twistを受け取り、各モーターへの速度をslcan_bridgeに向けてpublishしている。
*/

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include "harurobo2022/lib/vec2d.hpp"
#include "harurobo2022/config.hpp"
#include "harurobo2022/topics/body_twist.hpp"
#include "harurobo2022/topics/under_carriage_4wheel_active.hpp"
#include "harurobo2022/subscriber.hpp"
#include "harurobo2022/static_init_deinit.hpp"
#include "harurobo2022/motors.hpp"
#include "harurobo2022/timer.hpp"

using namespace StewLib;
using namespace Harurobo2022;
namespace CanId = Harurobo2022::Config::CanId;

namespace
{
    class UnderCarriage4WheelNode final
    {


        Timer publish_timer{1.0 / Config::ExecutionInterval::under_carriage_freq, [this](const ros::TimerEvent& event) noexcept { publish_timer_callback(event); }};

        DriveMotors drive_motors{};

        bool is_active{false};
        Subscriber<Topics::under_carriage_4wheel_active> active_sub{1, [this](const typename std_msgs::Bool::ConstPtr& msg_p){ is_active = msg_p->data; }};

        Subscriber<Topics::body_twist> body_twist_sub
        {
            1,
            [this](const typename Topics::body_twist::Message::ConstPtr& msg_p)
            {
                body_vell = {msg_p->linear_x, msg_p->linear_y};
                body_vela = msg_p->angular_z;
            }
        };

        Vec2D<double> body_vell{};
        double body_vela{};

        double wheels_vela[4]{};
        double pre_wheels_vela[4]{};

    public:
        UnderCarriage4WheelNode() noexcept
        {}

    private:
        void publish_timer_callback(const ros::TimerEvent&) noexcept
        {
            if(!is_active) return;
            
            calc_wheels_vela();

            drive_motors.FR_pub.send_target(wheels_vela[0]);
            drive_motors.FL_pub.send_target(wheels_vela[1]);
            drive_motors.BL_pub.send_target(wheels_vela[2]);
            drive_motors.BR_pub.send_target(wheels_vela[3]);
        }
        
        inline void calc_wheels_vela() noexcept
        {
            using namespace Config::Wheel;

            constexpr double rot_factors[4] =
            {
                Config::body_radius * rot(~Pos::FR,Constant::PI_2) * ~Direction::FR,
                Config::body_radius * rot(~Pos::FL,Constant::PI_2) * ~Direction::FL,
                Config::body_radius * rot(~Pos::BL,Constant::PI_2) * ~Direction::BL,
                Config::body_radius * rot(~Pos::BR,Constant::PI_2) * ~Direction::BR
            };

            const auto body_vell = this->body_vell;
            const auto body_vela = this->body_vela;
            double wheels_vela[4];

            for(int i = 0; i < 4; ++i)
            {
                wheels_vela[i] = (~Direction::all[i] * body_vell + rot_factors[i] * body_vela) / Config::wheel_radius;
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
                    ROS_WARN("%s: warning: The accelaretion of the wheels is too high. Speed is limited.", StringlikeTypes::under_carriage_4wheel::str);
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
                    ROS_WARN("%s: warning: The speed of the wheels is too high. Speed is limited.", StringlikeTypes::under_carriage_4wheel::str);
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
    };
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, StringlikeTypes::under_carriage_4wheel::str);
    StaticInitDeinit static_init_deinit;

    UnderCarriage4WheelNode under_carriage_4wheel_node;

    ROS_INFO("%s node has started.", StringlikeTypes::under_carriage_4wheel::str);
    ROS_INFO("sizeof(MessageConvertor<std_msgs::Float32>::CanData) %ld", sizeof(MessageConvertor<std_msgs::Float32>::CanData));

    ros::spin();

    ROS_INFO("%s node has terminated.", StringlikeTypes::under_carriage_4wheel::str);

}

