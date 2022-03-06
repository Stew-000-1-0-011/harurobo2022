
#include "harurobo2022/state.hpp"
#include "harurobo2022/shirasu_publisher.hpp"
#include "harurobo2022/subscriber.hpp"
#include "harurobo2022/config.hpp"
#include "harurobo2022/motors.hpp"
#include "harurobo2022/topics/auto_commander_active.hpp"
#include "harurobo2022/topics/under_carriage_4wheel_active.hpp"
#include "harurobo2022/topics/table_cloth.hpp"

using namespace Harurobo2022;

namespace
{
    class StateManagerNode final
    {
        using state_topic = Topics::state_topic;
        StateManager state_manager
        {
            [this](const State& state) noexcept
            {
                state_callback(state);
            }
        };

        Publisher<Topics::under_carriage_4wheel_active> under_carriage_4wheel_active_pub{1};
        Publisher<Topics::auto_commander_active> auto_commander_active_pub{1};

        DriveMotors drive_motors{};
        LiftMotors lift_motors{};

        CanPublisher<Topics::table_cloth_active> table_cloth_active_pub{1};

        void state_callback(const State& state) noexcept
        {
            switch(state)
            {
            case State::disable:
                case_disable();
                break;
            
            case State::manual:
                case_manual();
                break;

            case State::reset:
                case_reset();
                break;

            case State::automatic:
                case_automatic();
                break;
            
            case State::game_over:
                case_game_over();
                break;

            case State::game_clear:
                case_game_clear();
                break;
            
            default:
                ROS_INFO("state_manager: Change to unimplemented state.");
                break;
            }
        }
        
        void case_manual() noexcept
        {
            ROS_INFO("State changed to Manual.");

            under_carriage_4wheel_active_pub.publish(true);
            auto_commander_active_pub.publish(false);

            drive_motors.send_cmd_all(ShirasuUtil::velocity_mode);
            lift_motors.send_cmd_all(ShirasuUtil::position_mode); // ここで零点が初期化されたりとかしないよね...？
            lift_motors.send_target0_all();

            table_cloth_active_pub.can_publish(TableClothActive::enable);
        }

        void case_reset() noexcept
        {
            ROS_INFO("State changed to Reset.");

            under_carriage_4wheel_active_pub.publish(false);
            auto_commander_active_pub.publish(false);

            drive_motors.send_cmd_all(ShirasuUtil::disable_mode);
            lift_motors.send_cmd_all(ShirasuUtil::homing_mode);

            table_cloth_active_pub.can_publish(TableClothActive::disable);
        }

        void case_automatic() noexcept
        {
            ROS_INFO("State changed to Automatic.");

            under_carriage_4wheel_active_pub.publish(true);
            auto_commander_active_pub.publish(true);

            drive_motors.send_cmd_all(ShirasuUtil::velocity_mode);
            lift_motors.send_cmd_all(ShirasuUtil::position_mode);
            lift_motors.send_target0_all();

            table_cloth_active_pub.can_publish(TableClothActive::enable);
        }

        void case_disable() noexcept
        {
            ROS_INFO("State changed to Disable.");

            under_carriage_4wheel_active_pub.publish(false);
            auto_commander_active_pub.publish(false);

            drive_motors.send_cmd_all(ShirasuUtil::disable_mode);
            lift_motors.send_cmd_all(ShirasuUtil::disable_mode);

            table_cloth_active_pub.can_publish(TableClothActive::disable);
        }

        void case_game_over() noexcept
        {
            ROS_INFO("State changed to GameOver.");

            play_kokode_shinubeki_sadame_deha_nai_to();
        }

        void case_game_clear() noexcept
        {
            ROS_INFO("State changed to GameClear.");

            play_umapyoi();
        }

        // 余裕があれば。
        void play_kokode_shinubeki_sadame_deha_nai_to() noexcept {}
        void play_umapyoi() noexcept {}
    };
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, StringlikeTypes::state_manager::str);
    StaticInitDeinit static_init_deinit;

    StateManagerNode state_manager_node;

    ROS_INFO("%s node has started.", StringlikeTypes::state_manager::str);

    ros::spin();

    ROS_INFO("%s node has terminated.", StringlikeTypes::state_manager::str);
}