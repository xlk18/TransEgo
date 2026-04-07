#include <ros/ros.h>
#include "../include/quadrotor_nmpc_advanced.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "advanced_mpc_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 实例化已经完全封装好的独立节点类
    AdvancedMPCNode node(nh, pnh);

    // 进入 ROS 主循环
    ros::spin();
    return 0;
}
