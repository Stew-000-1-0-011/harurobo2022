/*
書いた人: 春ロボ2022 制御 本吉
*/

/*
編集した人: 春ロボ2022 制御 田巻

加えた変更：
・config.hppから変更しやすいようにした。
・使われている変数のうちこのファイル以外に関係する部分の名前を変えた。
・波括弧は改行後に入れるよう変更した(ごめん)。
・アクセス指定子を整えた。伴って変数をクラスの始めで定義するようにした(ごめん)。
・インデントを整えた(ごめん)。
・if(0 < latest_joy.axes.size())を消した。
・移動以外を書いている。
・その他色々変更した。
*/


/*
// quitaに載ってるros講座07のコードを参考にした
// https://qiita.com/srs/items/9114bb3c27a148e0b855
*/


#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <harurobo2022/Twist.h>

#include "harurobo2022/lib/vec2d.hpp"

#include "harurobo2022/config.hpp"
#include "harurobo2022/topics.hpp"
#include "harurobo2022/publisher.hpp"
#include "harurobo2022/can_publisher.hpp"
#include "harurobo2022/subscriber.hpp"
#include "harurobo2022/state.hpp"
#include "harurobo2022/timer.hpp"

using namespace StewLib;
using namespace Harurobo2022;

namespace
{
    // XInputにのみ対応
    namespace Axes
    {
        enum Axes : std::uint8_t
        {
            l_stick_LR = 0,
            l_stick_UD,
            l_trigger,
            r_stick_LR,
            r_stick_UD,
            r_trigger,
            cross_LR,
            cross_UD,

            N
        };
    }

    namespace Buttons
    {
        enum Buttons : std::uint8_t
        {
            a = 0,
            b,
            x,
            y,
            lb,
            rb,
            back,  // 緊急停止
            start,
            l_push,
            r_push,

            N
        };
    }

    struct JoyInput final
    {
        sensor_msgs::Joy latest_joy{[]{sensor_msgs::Joy msg; msg.axes = std::vector<float>(Axes::N, 0); msg.buttons = std::vector<std::int32_t>(Buttons::N, 0); return msg;}()};
        sensor_msgs::Joy old_joy{[]{sensor_msgs::Joy msg; msg.axes = std::vector<float>(Axes::N, 0); msg.buttons = std::vector<std::int32_t>(Buttons::N, 0); return msg;}()};
        bool once_pushed[Buttons::N]{};

        JoyInput() = default;

        bool is_being_pushed(const Buttons::Buttons button) const noexcept
        {
            return latest_joy.buttons[button];
        }

        bool is_pushed_once(const Buttons::Buttons button) noexcept
        {
            if(old_joy.buttons[button] && !latest_joy.buttons[button] && !once_pushed[button])
            {
                once_pushed[button] = true;
                return true;
            }
            return false;
        }

        void update(const sensor_msgs::Joy& joy) noexcept
        {
            old_joy = latest_joy;
            latest_joy = joy;
            for(int i = 0; i < Buttons::N; ++i)
            {
                once_pushed[i] = false;
            }
        }
    };

    class ManualCommanderNode
    {
        // friend JoyInput;
        using joy_topic = Topic<StringlikeTypes::joy, sensor_msgs::Joy>;

        ros::NodeHandle nh{};

        Publisher<Topics::body_twist> body_twist_pub{1};

        Subscriber<joy_topic> joy_sub{1, [this](const typename joy_topic::Message::ConstPtr& msg_p) noexcept { joyCallback(*msg_p); }};

        StateManager state_manager{};

        Timer timer{1.0 / Config::ExecutionInterval::manual_commander_freq, [this](const auto& event) noexcept { timerCallback(event); }};

        JoyInput joy_input{};


    public:
        ManualCommanderNode() = default;

    private:
        void joyCallback(const sensor_msgs::Joy& joy_msg)
        {
            joy_input.update(joy_msg);
        }

        void timerCallback(const ros::TimerEvent&)
        {
            switch(state_manager.get_state())
            {
            case State::disable:
                case_disable();
                break;

            case State::manual:
                case_manual();
                break;
            
            case State::automatic:
                case_automatic();
                break;

            case State::reset:
                case_reset();
                break;

            default:
                break;
            }
        }

        void case_disable() noexcept
        {
            if(joy_input.is_pushed_once(Buttons::start))
            {
                state_manager.set_state(State::reset);
            }
        }

        void case_manual() noexcept
        {
            harurobo2022::Twist cmd_vel;

            const Vec2D<float> input_vec = ~Vec2D<float>{joy_input.latest_joy.axes[Axes::l_stick_UD], joy_input.latest_joy.axes[Axes::l_stick_LR]};

            cmd_vel.linear_x = Config::Limitation::body_vell * input_vec.x;
            cmd_vel.linear_y = Config::Limitation::body_vell * input_vec.y;
            cmd_vel.angular_z = Config::Limitation::body_vela * joy_input.latest_joy.axes[Axes::r_stick_LR];

            body_twist_pub.publish(cmd_vel);

            /* TODO: ちりとりや足上げの制御 */
        }

        void case_reset() noexcept
        {
            if(joy_input.is_pushed_once(Buttons::start))
            {
                state_manager.set_state(State::automatic);
            }
        }

        void case_automatic() noexcept
        {
            if(joy_input.is_pushed_once(Buttons::start))
            {
                state_manager.set_state(State::manual);
            }
        }
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, StringlikeTypes::manual_commander::str);
    StaticInitDeinit static_init_deinit;

    ManualCommanderNode manual_commander_node;
    
    ROS_INFO("%s node has started.", StringlikeTypes::manual_commander::str);
    
    ros::spin();
    
    ROS_INFO("%s node has terminated.", StringlikeTypes::manual_commander::str);
    
    return 0;
}