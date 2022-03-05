/*

*/

#include <std_msgs/UInt8.h>
#include <harurobo2022/Twist.h>

#include "harurobo2022/lib/pid_functor.hpp"
#include "harurobo2022/timer.hpp"
#include "harurobo2022/state.hpp"
#include "harurobo2022/can_publisher.hpp"
#include "harurobo2022/publisher.hpp"
#include "harurobo2022/motors.hpp"
#include "harurobo2022/topics/stepping_motor.hpp"
#include "harurobo2022/topics/body_twist.hpp"
#include "harurobo2022/topics/table_cloth.hpp"
#include "harurobo2022/topics/odometry.hpp"
#include "harurobo2022/chart.hpp"

using namespace StewLib;
using namespace Harurobo2022;

namespace
{
    class AutoCommanderNode final
    {
        ChartManager chart_manager;

        LiftMotors lift_motors{};

        Publisher<Topics::body_twist> twist_pub{1};

        CanPublisher<Topics::stepping_motor> stepping_motor_pub{1};

        CanPublisher<Topics::table_cloth> table_cloth_pub{1};

        Subscriber<Topics::odometry_x> odometry_x_sub{1, [this](const typename Topics::odometry_x::Message::ConstPtr& msg_p) noexcept { odometry_x_callback(msg_p); }};
        Subscriber<Topics::odometry_y> odometry_y_sub{1, [this](const typename Topics::odometry_y::Message::ConstPtr& msg_p) noexcept { odometry_y_callback(msg_p); }};
        Subscriber<Topics::odometry_yaw> odometry_yaw_sub{1, [this](const typename Topics::odometry_yaw::Message::ConstPtr& msg_p) noexcept { odometry_yaw_callback(msg_p); }};

        Timer timer{1.0 / Config::ExecutionInterval::auto_commander_freq, [this](const auto&){ timer_callback(); }};

        StateManager state_manager
        {
            [this](const State& state) noexcept
            {
                if(state == State::reset)
                {
                    chart_manager.reset_chart();
                }
            }
        };

        // ひとまずは位置と姿勢を速度上限つきP制御で追う。目標位置姿勢と現在位置姿勢の差を定数倍して並進速度角速度にする。
        Vec2D<float> now_pos{};
        float now_rot_z{};

        StewLib::Pid<StewLib::Vec2D<float>> position_pid{Config::Pid::position_k_p, Config::Pid::position_k_i, Config::Pid::position_k_d};
        StewLib::Pid<float> rot_z_pid{Config::Pid::rot_z_k_p, Config::Pid::rot_z_k_i, Config::Pid::rot_z_k_d};

    public:
        AutoCommanderNode() = default;

    private:
        void odometry_x_callback(const Topics::odometry_x::Message::ConstPtr& msg_p) noexcept
        {
            now_pos.x = msg_p->data + Config::InitialState::position.x;
        }

        void odometry_y_callback(const Topics::odometry_y::Message::ConstPtr& msg_p) noexcept
        {
            now_pos.y = msg_p->data + Config::InitialState::position.y;
        }

        void odometry_yaw_callback(const Topics::odometry_yaw::Message::ConstPtr& msg_p) noexcept
        {
            now_rot_z = msg_p->data + Config::InitialState::rot_z;
        }

        void timer_callback() noexcept
        {
            if(chart_manager.current_work->pass_near_circle.is_in(now_pos))
            {
                do_work(chart_manager.current_work->work);
                chart_manager.current_work_update();
            }

            if(chart_manager.target_position->pass_near_circle.is_in(now_pos))
            {
                chart_manager.target_position_update();
            }

            auto twist = calc_twist();

            twist_pub.publish(twist);
        }

        /* TODO over_fenceの実装 */
        void do_work(const Work work) noexcept
        {
            switch(work)
            {
            case Work::collector_bottom:
                case_collector_bottom();
                break;
            
            case Work::collector_step1:
                case_collector_step1();
                break;

            case Work::collector_step2:
                case_collector_step2();
                break;
            
            case Work::collector_step3:
                case_collector_step3();
                break;

            case Work::collector_shovel_open:
                case_collector_shovel_open();
                break;

            case Work::collector_shovel_close:
                case_collector_shovel_close();
                break;
            
            case Work::collector_tablecloth_push:
                case_collector_tablecloth_push();
                break;

            case Work::collector_tablecloth_pull:
                case_collector_tablecloth_pull();
                break;

            case Work::change_to_over_fence:
                break;
            
            case Work::transit:
            case Work::game_clear:
            default:
                break;
            }
        }

        void case_collector_bottom() noexcept
        {
            lift_motors.collector_pub.send_target(Config::collector_bottom_position);
        }

        void case_collector_step1() noexcept
        {
            lift_motors.collector_pub.send_target(Config::collector_step1_position);
        }

        void case_collector_step2() noexcept
        {
            lift_motors.collector_pub.send_target(Config::collector_step2_position);
        }

        void case_collector_step3() noexcept
        {
            lift_motors.collector_pub.send_target(Config::collector_step3_position);
        }

        void case_collector_shovel_open() noexcept
        {
            stepping_motor_pub.can_publish(SteppingMotor::open);
        }

        void case_collector_shovel_close() noexcept
        {
            stepping_motor_pub.can_publish(SteppingMotor::close);
        }

        void case_collector_tablecloth_push() noexcept
        {
            table_cloth_pub.can_publish(TableCloth::push);
        }

        void case_collector_tablecloth_pull() noexcept
        {
            table_cloth_pub.can_publish(TableCloth::pull);
        }

        Topics::body_twist::MessageConvertor::RawData calc_twist() noexcept
        {
            const auto target_pos = chart_manager.target_position->pass_near_circle.center;
            const auto target_rot_z = chart_manager.current_work->target_rot_z;

            const auto linear_global = position_pid(target_pos - now_pos);
            const auto angular = rot_z_pid(target_rot_z - now_rot_z);

            const auto linear_onbody = rot(linear_global, -now_rot_z);

            return {static_cast<float>(linear_onbody.x), static_cast<float>(linear_onbody.y), angular};
        }
    };
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, StringlikeTypes::auto_commander::str);
    StaticInitDeinit satic_init_deinit;

    AutoCommanderNode auto_commander_node;

    ROS_INFO("%s node has started.", StringlikeTypes::auto_commander::str);

    ros::spin();

    ROS_INFO("%s node has terminated.", StringlikeTypes::auto_commander::str);
}