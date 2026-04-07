#ifndef QUADROTOR_NMPC_ADVANCED_H
#define QUADROTOR_NMPC_ADVANCED_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <memory>
#include <mutex>

class QuadrotorNMPCAdvanced
{
public:
    // 构造函数：初始化模型参数与预测步长
    QuadrotorNMPCAdvanced(double mass, double gravity, double dt, int N,
                          double drag_x, double drag_y, double drag_z,
                          double tau_thrust, double tau_rate,
                          bool use_aero_drag_and_delay);
    // 析构函数：释放对象资源
    ~QuadrotorNMPCAdvanced();

    // 构建连续动力学函数 f(x,u)
    casadi::Function buildDynamicsFunction();
    // 计算终端 LQR 代价矩阵
    Eigen::MatrixXd computeLQRTerminalCost(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R);
    // 求解一次 NMPC；可选输出整段预测状态
    bool solve(const Eigen::VectorXd &x_curr, const Eigen::MatrixXd &X_ref, const Eigen::MatrixXd &U_ref,
               Eigen::Vector4d &u_opt, Eigen::MatrixXd *x_pred = nullptr);

    // 搭建并初始化 NLP 优化问题
    void setupNLP(const std::vector<double> &Q_diag, const std::vector<double> &R_diag,
                  double thrust_min, double thrust_max,
                  double rate_x, double rate_y, double rate_z,
                  double terminal_cost_multiplier);

private:
    // 四元数乘法：组合两个姿态
    casadi::MX quatMultiply(const casadi::MX &q1, const casadi::MX &q2);
    // 四元数求逆：用于姿态误差计算
    casadi::MX quatInverse(const casadi::MX &q);

    double mass_;                  // 机体质量
    double gravity_;               // 重力加速度
    double dt_;                    // 离散时间步长
    int N_;                        // 预测步数
    double drag_x_;                // x 方向线性阻力系数
    double drag_y_;                // y 方向线性阻力系数
    double drag_z_;                // z 方向线性阻力系数
    double tau_thrust_;            // 推力一阶延迟时间常数
    double tau_rate_;              // 角速度一阶延迟时间常数
    bool use_aero_drag_and_delay_; // 是否启用阻力与延迟模型

    casadi::Opti opti_;     // CasADi 优化器对象
    casadi::MX X_var_;      // 状态决策变量
    casadi::MX U_var_;      // 控制决策变量
    casadi::MX X0_param_;   // 初始状态参数
    casadi::MX Xref_param_; // 状态参考轨迹参数
    casadi::MX Uref_param_; // 控制参考轨迹参数

    std::vector<double> x_warm_start_; // 状态热启动缓存
    std::vector<double> u_warm_start_; // 控制热启动缓存
};

class AdvancedMPCNode
{
public:
    // 构造函数：接入 ROS 通信并初始化控制器
    AdvancedMPCNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);

private:
    ros::NodeHandle nh_;                  // 公有句柄：话题通信
    ros::NodeHandle pnh_;                 // 私有句柄：参数读取
    ros::Subscriber odom_sub_, traj_sub_; // 里程计与轨迹订阅器
    ros::Publisher ctrl_pub_;             // 控制指令发布器
    ros::Publisher pred_path_pub_;        // 预测轨迹可视化发布器
    ros::Timer timer_;                    // 控制循环定时器

    std::unique_ptr<QuadrotorNMPCAdvanced> nmpc_; // Advanced NMPC 核心对象
    Eigen::VectorXd current_x_;                   // 当前 14 维状态
    Eigen::Vector4d last_cmd_;                    // 上一拍控制指令
    Eigen::MatrixXd ref_traj_;                    // 状态参考轨迹
    Eigen::MatrixXd u_ref_traj_;                  // 控制参考轨迹

    bool has_odom_, has_traj_;     // 数据就绪标志
    int N_;                        // 预测步数
    double dt_;                    // 控制周期
    double mass_;                  // 质量参数
    double gravity_;               // 重力参数
    double max_thrust_;            // 最大推力限制
    double min_thrust_;            // 最小推力限制
    double max_rate_x_;            // x 轴角速度上限
    double max_rate_y_;            // y 轴角速度上限
    double max_rate_z_;            // z 轴角速度上限
    bool use_aero_drag_and_delay_; // 阻力与延迟开关

    std::mutex odom_mutex_, traj_mutex_; // 里程计与轨迹互斥锁

    // 里程计回调：更新当前状态
    void odomCb(const nav_msgs::Odometry::ConstPtr &msg);
    // 轨迹回调：更新参考轨迹
    void trajCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg);
    // 主循环：求解并发布控制与可视化
    void loop(const ros::TimerEvent &e);
};

#endif // QUADROTOR_NMPC_ADVANCED_H
