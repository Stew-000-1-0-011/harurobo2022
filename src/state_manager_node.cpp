
#include <ros/ros.h>

const char *const node_name = "state_manager";

int main(int argc, char ** argv)
{
    ROS_INFO("%s node has started.", node_name);
    
    ros::spin();
    
    ROS_INFO("%s node has terminated.", node_name);
}