#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <harurobo2022/CanFrame.h>

#define TOPIC(message_type) #message_type

// #define PUBLISHER(node_handle, message_type, buffer_size) \
// node_handle.advertise<message_type>(TOPIC(message_type), buffer_size)

// #define SUBSCRIBER(node_handle, message_type, buffer_size, ...) \
// node_handle.subscribe<message_type>(TOPIC(message_type), buffer_size, __VA_ARGS__)

namespace TopicMessageTypeAlias
{
    using wheel_FR_vela = std_msgs::Float32;
    using wheel_FL_vela = std_msgs::Float32;
    using wheel_BL_vela = std_msgs::Float32;
    using wheel_BR_vela = std_msgs::Float32;

    using body_twist = geometry_msgs::Twist;

    using can_rx = harurobo2022::CanFrame;
    using can_tx = harurobo2022::CanFrame;
}