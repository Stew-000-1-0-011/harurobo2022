/*
base_controllerを参考にした。
機体に固定された座標でのgeometry::Twistを受け取り、各モーターへの速度をslcan_bridgeに向けてpublishしている。
*/

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include "harurobo2022/lib/vec2d.hpp"
#include "harurobo2022/config.hpp"
#include "harurobo2022/topics.hpp"
#include "harurobo2022/subscriber.hpp"
#include "harurobo2022/active_manager.hpp"
#include "harurobo2022/shirasu_publisher.hpp"
#include "harurobo2022/static_init_deinit.hpp"

using namespace StewLib;
using namespace Harurobo2022;
namespace CanId = Harurobo2022::Config::CanId;

namespace
{
    #include "harurobo2022/lib/macro/define_stringlike_type.hpp"
    Stew_StringlikeType(under_carriage_4wheel)
    Stew_StringlikeType(FRdrive)
    Stew_StringlikeType(FLdrive)
    Stew_StringlikeType(BLdrive)
    Stew_StringlikeType(BRdrive)
    #undef Stew_StringlikeType

    class UnderCarriage4WheelNode final
    {
        ros::NodeHandle nh{};

        ros::Timer publish_timer
        {
            nh.createTimer
            (
                ros::Duration(1.0 / Config::ExecutionInterval::under_carriage_freq),
                &UnderCarriage4WheelNode::publish_timer_callback, this
            )
        };

        ShirasuPublisher<FRdrive,CanId::Tx::DriveMotor::FR> FR_pub;
        ShirasuPublisher<FLdrive,CanId::Tx::DriveMotor::FL> FL_pub;
        ShirasuPublisher<BLdrive,CanId::Tx::DriveMotor::BL> BL_pub;
        ShirasuPublisher<BRdrive,CanId::Tx::DriveMotor::BR> BR_pub;

        Subscriber<Topics::body_twist> body_twist_sub
        {
            1,
            [this](const typename Topics::body_twist::Message::ConstPtr& msg_p)
            {
                body_vell = {msg_p->linear.x, msg_p->linear.y};
                body_vela = msg_p->angular.z;
            }
        };

        ActiveManager
        <
            under_carriage_4wheel, decltype(FR_pub), decltype(FL_pub), decltype(BL_pub), decltype(BR_pub), decltype(body_twist_sub)
        > active_manager
        {
            FR_pub, FL_pub, BL_pub, BR_pub, body_twist_sub
        };

        Vec2D<double> body_vell{};
        double body_vela{};

        double wheels_vela[4]{};
        double pre_wheels_vela[4]{};

    public:
        UnderCarriage4WheelNode()
        {
            active_manager.deactivate();
        }

    private:
        void publish_timer_callback([[maybe_unused]] const ros::TimerEvent& event) noexcept
        {
            calc_wheels_vela();

            FR_pub.send_target(wheels_vela[0]);
            FL_pub.send_target(wheels_vela[1]);
            BL_pub.send_target(wheels_vela[2]);
            BR_pub.send_target(wheels_vela[3]);
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
                    ROS_WARN("%s: warning: The accelaretion of the wheels is too high. Speed is limited.", under_carriage_4wheel::str);
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
                    ROS_WARN("%s: warning: The speed of the wheels is too high. Speed is limited.", under_carriage_4wheel::str);
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
    ros::init(argc, argv, under_carriage_4wheel::str);
    StaticInitDeinit static_init_deinit;

    UnderCarriage4WheelNode under_carriage_4wheel_node;

    ROS_INFO("%s node has started.", under_carriage_4wheel::str);

    ros::spin();

    ROS_INFO("%s node has terminated.", under_carriage_4wheel::str);

}

