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

using namespace Harurobo2022;

class ManualCommand
{
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber joy_sub_;
    ros::Timer timer_;
    sensor_msgs::Joy last_joy_;

public:
    ManualCommand() noexcept
    {
        cmd_pub_ = nh_.advertise<Topics::body_twist>(Topics::body_twist::topic, 1);
        joy_sub_ = nh_.subscribe("joy", 1, &ManualCommand::joyCallback, this);
        timer_ = nh_.createTimer(ros::Duration(1.0 / Config::ExecutionInterval::manual_commander_freq), &ManualCommand::timerCallback, this);
    }

private:
    void joyCallback(const sensor_msgs::Joy& joy_msg)
    {
        last_joy_ = joy_msg;
    }

    void timerCallback(const ros::TimerEvent& e)
    {

        geometry_msgs::Twist cmd_vel;

        cmd_vel.linear.x = Config::Limitation::body_vell / 2 * last_joy_.axes[0];     //[0]はコントローラーの左スティック左右の割り当て
        cmd_vel.linear.y = Config::Limitation::body_vell / 2 * last_joy_.axes[1];     //[1]はコントローラーの左スティック上下の割り当て
        cmd_vel.angular.z = Config::Limitation::body_vela * last_joy_.axes[2];     //[2]はコントローラーの右スティック左右の割り当て
        
        cmd_pub_.publish(cmd_vel);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manual_command");

    ManualCommand manual_command;
    
    ROS_INFO("manual_command node has started.");
    
    ros::spin();
    
    ROS_INFO("manual_command node has terminated.");
    
    return 0;
}