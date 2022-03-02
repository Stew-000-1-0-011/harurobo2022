
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
            [this](const state_topic::Message::ConstPtr& msg_p) noexcept
            {
                state_callback(msg_p);
            }
        };

        // ActiveManager<StringlikeTypes::under_carriage_4wheel> under_carriage_4wheel_active_manager{};
        // ActiveManager<StringlikeTypes::auto_commander> auto_commander_active_manager{};

        DriveMotors drive_motors{};
        LiftMotors lift_motors{};

        void state_callback(const state_topic::Message::ConstPtr& msg_p) noexcept
        {
            switch(static_cast<State>(msg_p->data))
            {
            case State::disable:
                case_desable();
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
            
            default:
                break;
            }
        }
        
        inline void case_manual() noexcept
        {
            ROS_INFO("State changed to Manual.");

            //under_carriage_4wheel_active_manager.activate();
            //auto_commander_active_manager.deactivate();

            drive_motors.send_cmd_all(ShirasuUtil::velocity_mode);
            lift_motors.send_cmd_all(ShirasuUtil::position_mode); // ここで零点が初期化されたりとかしないよね...？
        }

        inline void case_reset() noexcept
        {
            ROS_INFO("State changed to Reset.");

            //under_carriage_4wheel_active_manager.deactivate();
            //auto_commander_active_manager.deactivate();

            drive_motors.send_cmd_all(ShirasuUtil::disable_mode);
            lift_motors.send_cmd_all(ShirasuUtil::homing_mode);
        }

        inline void case_automatic() noexcept
        {
            ROS_INFO("State changed to Automatic.");

            //under_carriage_4wheel_active_manager.activate();
            //auto_commander_active_manager.activate();

            drive_motors.send_cmd_all(ShirasuUtil::velocity_mode);
            lift_motors.send_cmd_all(ShirasuUtil::position_mode);
        }

        inline void case_desable() noexcept
        {
            ROS_INFO("State changed to Desable.");

            //under_carriage_4wheel_active_manager.deactivate();
            //auto_commander_active_manager.deactivate();

            drive_motors.send_cmd_all(ShirasuUtil::disable_mode);
            lift_motors.send_cmd_all(ShirasuUtil::disable_mode);
        }
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