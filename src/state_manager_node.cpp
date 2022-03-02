
#include "harurobo2022/state.hpp"
#include "harurobo2022/active_manager.hpp"
#include "harurobo2022/shirasu_publisher.hpp"
#include "harurobo2022/subscriber.hpp"
#include "harurobo2022/config.hpp"
#include "harurobo2022/motors.hpp"

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

        ActiveManager<StringlikeTypes::under_carriage_4wheel> under_carriage_4wheel_active_manager{};
        ActiveManager<StringlikeTypes::auto_commander> auto_commander_active_manager{};

        DriveMotors drive_motors{};
        LiftMotors lift_motors{};

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

            under_carriage_4wheel_active_manager.activate();
            auto_commander_active_manager.deactivate();

            drive_motors.send_cmd_all(ShirasuUtil::velocity_mode);
            lift_motors.send_cmd_all(ShirasuUtil::position_mode); // ここで零点が初期化されたりとかしないよね...？
        }

        void case_reset() noexcept
        {
            ROS_INFO("State changed to Reset.");

            under_carriage_4wheel_active_manager.deactivate();
            auto_commander_active_manager.deactivate();

            drive_motors.send_cmd_all(ShirasuUtil::disable_mode);
            lift_motors.send_cmd_all(ShirasuUtil::homing_mode);
        }

        void case_automatic() noexcept
        {
            ROS_INFO("State changed to Automatic.");

            under_carriage_4wheel_active_manager.activate();
            auto_commander_active_manager.activate();

            drive_motors.send_cmd_all(ShirasuUtil::velocity_mode);
            lift_motors.send_cmd_all(ShirasuUtil::position_mode);
        }

        void case_disable() noexcept
        {
            ROS_INFO("State changed to Desable.");

            under_carriage_4wheel_active_manager.deactivate();
            auto_commander_active_manager.deactivate();

            drive_motors.send_cmd_all(ShirasuUtil::disable_mode);
            lift_motors.send_cmd_all(ShirasuUtil::disable_mode);
        }

        void case_game_over() noexcept
        {
            ROS_INFO("State changed to GameOver.");

            play_mada_kokode_shinubeki_sadame_deha_nai_to();
        }

        void case_game_clear() noexcept
        {
            ROS_INFO("State changed to GameClear.");

            play_umapyoi();
        }

        // 余裕があれば。
        void play_mada_kokode_shinubeki_sadame_deha_nai_to() noexcept {}
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