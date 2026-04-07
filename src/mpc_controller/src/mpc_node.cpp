#include "../include/quadrotor_nmpc.h"

// 整个C++节点的程序入口
/**
 * @brief MPC控制器独立进程入口
 * 负责初始化ROS节点，创建NMPC实例并挂起线程等待回调数据驱动
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_node"); // 系统节点注册命名
    ros::NodeHandle nh;                // 根公开ROS句柄
    ros::NodeHandle pnh("~");          // 私有带名空间ROS句柄 (方便读取私有参数)

    // 创建实例化MPC控制器主对象
    QuadrotorNMPC nmpc_controller(nh, pnh);

    // 阻塞并在主线程无限监听回调事件流，避免退出执行
    ros::spin();

    return 0; // 结束执行
}
