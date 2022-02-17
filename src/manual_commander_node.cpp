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
・if(0 < last_joy_.axes.size())を消した。
・移動以外を書いている。
*/


/*
// quitaに載ってるros講座07のコードを参考にした
// https://qiita.com/srs/items/9114bb3c27a148e0b855
*/


#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include "harurobo2022/config.hpp"
#include "harurobo2022/topics.hpp"
#include "harurobo2022/can_publisher.hpp"
#include "harurobo2022/shutdown_subscriber.hpp"
#include "harurobo2022/state.hpp"


using namespace Harurobo2022;

const char *const node_name = "manual_commander";

namespace Keys
{
    namespace Axes
    {
        enum Axes : std::uint8_t
        {
            l_stick_LR = 0,
            l_stick_UD,
            r_stick_LR,
            r_stick_UD,

            N
        };
    }

    namespace Buttons
    {
        enum Buttons : std::uint8_t
        {
            select = 0,
            l3,
            r3,
            start,

            N
        };
    }
}

class ManualCommanderNode
{
    ros::NodeHandle nh_{};

    ros::Publisher can_tx_pub_{nh_.advertise<Topics::can_tx::Message>(Topics::can_tx::topic, 1)};
    ros::Publisher body_twist_pub_{nh_.advertise<Topics::body_twist::Message>(Topics::body_twist::topic, 1)};

    CanPublisher<CanTxTopics::emergency_stop> emergency_stop_canpub_{can_tx_pub_};

    ros::Subscriber joy_sub_{nh_.subscribe("joy", 1, &ManualCommanderNode::joyCallback, this)};

    ShutDownSubscriber shutdown_sub{nh_};
    StateManager state_manager{nh_};
    
    ros::Timer timer_{nh_.createTimer(ros::Duration(1.0 / Config::ExecutionInterval::manual_commander_freq), &ManualCommanderNode::timerCallback, this)};

    sensor_msgs::Joy last_joy_{[]{sensor_msgs::Joy msg; msg.axes = std::vector<float>(Keys::Axes::N, 0); msg.buttons = std::vector<std::int32_t>(Keys::Buttons::N, 0); return msg;}()};


public:
    ManualCommanderNode() = default;

private:
    void joyCallback(const sensor_msgs::Joy& joy_msg)
    {
        last_joy_ = joy_msg;
    }

    void timerCallback(const ros::TimerEvent& e)
    {
        switch(state_manager.get_state())
        {

        case State::manual:
            case_manual();
            break;
        
        case State::automatic:
            /* TODO: 手動操作への切り替え */
            break;

        case State::reset:
            case_reset();
            break;

        default:
            break;
        }

        /* TODO: 緊急停止などいつでも操作せねばならない部分 */
        if(last_joy_.buttons[Keys::Buttons::select])
        {
            CanTxTopics::emergency_stop::Message msg;
            msg.data = true;
            emergency_stop_canpub_.publish(msg);
        }
    }

    void case_manual() noexcept
    {
        geometry_msgs::Twist cmd_vel;

        cmd_vel.linear.x = Config::Limitation::body_vell / 2 * last_joy_.axes[Keys::Axes::l_stick_LR];  //[0]はコントローラーの左スティック左右の割り当て
        cmd_vel.linear.y = Config::Limitation::body_vell / 2 * last_joy_.axes[Keys::Axes::l_stick_UD];  //[1]はコントローラーの左スティック上下の割り当て
        cmd_vel.angular.z = Config::Limitation::body_vela * last_joy_.axes[Keys::Axes::r_stick_LR];  //[2]はコントローラーの右スティック左右の割り当て

        body_twist_pub_.publish(cmd_vel);

        /* TODO: ちりとりや足上げの制御 */
    }

    void case_reset() noexcept
    {
        if(last_joy_.buttons[Keys::Buttons::start])
        {
            state_manager.set_state(State::manual);
            // state_manager.set_state(State::automatic);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, node_name);

    ManualCommanderNode manual_commander_node;
    
    ROS_INFO("%s node has started.", node_name);
    
    ros::spin();
    
    ROS_INFO("%s node has terminated.", node_name);
    
    return 0;
}