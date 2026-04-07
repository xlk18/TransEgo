#ifndef QUADROTOR_NMPC_H
#define QUADROTOR_NMPC_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>
#include <chrono>

// 控制器状态枚举
/**
 * @brief 无人机控制器状态机枚举
 * 管理从初始化、等待里程计、悬停、轨迹跟踪到紧急降落的完整生命周期
 */
enum class ControllerState
{
    INIT,             // 初始化阶段
    WAITING_FOR_ODOM, // 等待里程计数据
    HOVER,            // 悬停模式
    TRACKING,         // 轨迹跟踪模式
    EMERGENCY_LAND    // 紧急降落模式
};

// NMPC控制器参数配置
/**
 * @brief NMPC算法运行所需的物理与优化参数结构体
 * 对应YAML配置项，决定了预测步长、重量、推力饱和及误差惩罚权重
 */
struct MPCParameters
{
    int N;                           // 预测步数
    double dt;                       // 离散时间步长
    double mass;                     // 无人机质量
    double gravity;                  // 重力加速度
    double max_thrust;               // 推力上限
    double min_thrust;               // 推力下限
    double max_bodyrate_x;           // x 轴角速度上限
    double max_bodyrate_y;           // y 轴角速度上限
    double max_bodyrate_z;           // z 轴角速度上限
    double drag_x;                   // x 方向阻力系数
    double drag_y;                   // y 方向阻力系数
    double drag_z;                   // z 方向阻力系数
    double tau_thrust;               // 推力延迟时间常数
    double tau_rate;                 // 角速度延迟时间常数
    bool use_aero_drag_and_delay;    // 阻力与延迟开关
    double terminal_cost_multiplier; // 终端代价放大系数

    std::vector<double> Q_diag; // 状态误差权重对角项
    std::vector<double> R_diag; // 控制输入权重对角项
};

/**
 * @brief 四旋翼NMPC非线性模型预测控制核心类
 * 封装CasADi前向动力学构建、IPOPT非线性规划求解器、以及ROS/PX4底层指令的发布
 */
class QuadrotorNMPC
{
private:
    ros::NodeHandle nh_;  // 公有句柄：话题通信
    ros::NodeHandle pnh_; // 私有句柄：参数读取

    ros::Subscriber odom_sub_;         // 里程计订阅器
    ros::Subscriber traj_sub_;         // 轨迹订阅器
    ros::Subscriber mavros_state_sub_; // 飞控状态订阅器（预留）
    ros::Publisher ctrl_pub_;          // 控制指令发布器
    ros::Publisher pred_path_pub_;     // 预测轨迹可视化发布器
    ros::Timer mpc_timer_;             // 控制循环定时器

    ControllerState current_state_;   // 当前状态机状态
    mavros_msgs::State mavros_state_; // 当前飞控状态
    Eigen::VectorXd current_x_;       // 当前系统状态向量
    Eigen::Vector4d last_cmd_;        // 上一拍控制指令
    Eigen::MatrixXd ref_traj_;        // 当前参考轨迹

    std::mutex odom_mutex_; // 里程计数据互斥锁
    std::mutex traj_mutex_; // 轨迹数据互斥锁
    bool has_odom_;         // 是否已收到里程计
    bool has_traj_;         // 是否已收到轨迹

    MPCParameters params_; // 控制器参数集合

    casadi::Opti opti_;     // CasADi 优化器对象
    casadi::MX X_var_;      // 状态决策变量
    casadi::MX U_var_;      // 控制决策变量
    casadi::MX X0_param_;   // 初始状态参数
    casadi::MX Xref_param_; // 参考状态参数
    casadi::MX Q_param_;    // 状态权重参数矩阵
    casadi::MX R_param_;    // 控制权重参数矩阵

    std::vector<double> u_warm_start_; // 控制热启动缓存
    std::vector<double> x_warm_start_; // 状态热启动缓存

    ros::Time last_optimization_time_; // 上次优化时间戳

    /**
     * @brief 构建四旋翼的连续推力-姿态动力学模型
     * @return CasADi函数，表示状态导数 f(x,u) = dx/dt
     */
    casadi::Function buildDynamicsFunction();
    /**
     * @brief 初始化并配置非线性规划问题 (NLP)
     * 设定目标代价函数(Cost Function)、动力学多重打靶约束及变量限幅约束
     */
    void setupNLP();
    // 里程计回调：更新当前状态
    void odomCb(const nav_msgs::Odometry::ConstPtr &msg);
    // 轨迹回调：更新参考轨迹
    void trajCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg);
    // 读取并校验参数
    void loadParameters();
    // 主控制循环：求解并发布控制
    void controlLoop(const ros::TimerEvent &event);

public:
    // 构造函数：初始化节点与优化器
    QuadrotorNMPC(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    // 析构函数：回收对象资源
    ~QuadrotorNMPC();
};

#endif // QUADROTOR_NMPC_H
