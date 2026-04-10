#include <nav_msgs/Path.h>
#include "../include/quadrotor_nmpc.h"

using namespace casadi;

// 构建无人机连续动力学方程 f(x, u) = dx/dt
casadi::Function QuadrotorNMPC::buildDynamicsFunction()
{
    MX x = MX::sym("x", 14); // 状态变量(位置3, 速度3, 四元数4, 推力1, xyz机体角速度3)
    MX u = MX::sym("u", 4);  // 控制变量(推力指令, xyz角速度指令)

    MX p = x(Slice(0, 3));       // 提取位置 [px, py, pz]
    MX v = x(Slice(3, 6));       // 提取速度 [vx, vy, vz]
    MX q = x(Slice(6, 10));      // 提取姿态四元数 [qw, qx, qy, qz]
    MX T_act = x(10);            // 实际推力
    MX w_act = x(Slice(11, 14)); // 实际机体角速度 [wx, wy, wz]

    MX T_cmd = u(0);  // 推力指令
    MX wx_cmd = u(1); // x轴机体角速度指令
    MX wy_cmd = u(2); // y轴机体角速度指令
    MX wz_cmd = u(3); // z轴机体角速度指令

    MX wx = w_act(0); // 实际x轴机体角速度
    MX wy = w_act(1); // 实际y轴机体角速度
    MX wz = w_act(2); // 实际z轴机体角速度

    MX qw = q(0);
    MX qx = q(1);
    MX qy = q(2);
    MX qz = q(3);

    // 计算从机体系到世界系的旋转矩阵 (3x3)
    MX R11 = 1 - 2 * (qy * qy + qz * qz);
    MX R12 = 2 * (qx * qy - qw * qz);
    MX R13 = 2 * (qx * qz + qw * qy);

    MX R21 = 2 * (qx * qy + qw * qz);
    MX R22 = 1 - 2 * (qx * qx + qz * qz);
    MX R23 = 2 * (qy * qz - qw * qx);

    MX R31 = 2 * (qx * qz - qw * qy);
    MX R32 = 2 * (qy * qz + qw * qx);
    MX R33 = 1 - 2 * (qx * qx + qy * qy);

    // 拼接旋转矩阵
    MX R = MX::vertcat({MX::horzcat({R11, R12, R13}),
                        MX::horzcat({R21, R22, R23}),
                        MX::horzcat({R31, R32, R33})});

    MX dp = v; // 位置的导数为速度

    MX actual_T = params_.use_aero_drag_and_delay ? T_act : T_cmd;
    MX actual_wx = params_.use_aero_drag_and_delay ? wx : wx_cmd;
    MX actual_wy = params_.use_aero_drag_and_delay ? wy : wy_cmd;
    MX actual_wz = params_.use_aero_drag_and_delay ? wz : wz_cmd;

    // 计算实际推力引起的加速度向量(机体系下只有z轴推力)
    MX thrust_vec = MX::vertcat({0.0, 0.0, actual_T / params_.mass});
    // 世界系下的重力加速度向量
    MX gravity_vec = MX::vertcat({0.0, 0.0, params_.gravity});

    // 空气阻力：与速度相关的线性阻力减速项（阻力系数 * 速度）
    MX drag_force = params_.use_aero_drag_and_delay ? MX::vertcat({params_.drag_x * v(0), params_.drag_y * v(1), params_.drag_z * v(2)})
                                                    : MX::vertcat({0.0, 0.0, 0.0});

    // 速度的导数等于推力在世界系下的加速度减去重力加速度，再减去空气阻力
    MX dv = mtimes(R, thrust_vec) - gravity_vec - drag_force;

    // 根据实际角速度计算四元数的变化率
    MX dqw = 0.5 * (-qx * actual_wx - qy * actual_wy - qz * actual_wz);
    MX dqx = 0.5 * (qw * actual_wx - qz * actual_wy + qy * actual_wz);
    MX dqy = 0.5 * (qz * actual_wx + qw * actual_wy - qx * actual_wz);
    MX dqz = 0.5 * (-qy * actual_wx + qx * actual_wy + qw * actual_wz);
    MX dq = MX::vertcat({dqw, dqx, dqy, dqz});

    // 动力系统底层响应延迟产生的一阶惯性环节模型：dx/dt = (x_cmd - x) / tau
    MX dT = params_.use_aero_drag_and_delay ? (T_cmd - T_act) / params_.tau_thrust : (T_cmd - T_act);
    MX dwx = params_.use_aero_drag_and_delay ? (wx_cmd - wx) / params_.tau_rate : (wx_cmd - wx);
    MX dwy = params_.use_aero_drag_and_delay ? (wy_cmd - wy) / params_.tau_rate : (wy_cmd - wy);
    MX dwz = params_.use_aero_drag_and_delay ? (wz_cmd - wz) / params_.tau_rate : (wz_cmd - wz);
    MX dw = MX::vertcat({dwx, dwy, dwz});

    // 合并所有状态的导数（共14维：位置3、速度3、四元数4、推力1、角速度3）
    MX dx = MX::vertcat({dp, dv, dq, dT, dw});

    // 返回包含动力学模型的CasADi函数
    return Function("f", {x, u}, {dx}, {"x", "u"}, {"dx"});
}

// 初始化NLP（非线性规划）优化问题
void QuadrotorNMPC::setupNLP()
{
    ROS_INFO("Setting up CasADi NMPC Optimization Problem..."); // 打印配置日志

    opti_ = Opti(); // 实例化CasADi优化环境对象

    int N = params_.dt > 0 ? params_.N : 10; // 获取预测步数参数，默认为10步
    double dt = params_.dt;                  // 获取时间步长参数

    X_var_ = opti_.variable(14, N + 1); // 声明系统在N+1个时间节点的状态变量 (10维)
    U_var_ = opti_.variable(4, N);      // 声明系统在N个时间节点的控制输入变量 (4维)

    X0_param_ = opti_.parameter(14);          // 声明当前初始状态参数 (由里程计更新)
    Xref_param_ = opti_.parameter(14, N + 1); // 声明参考轨迹参数 (N+1步的10维状态)
    Q_param_ = opti_.parameter(14, 14);       // 声明状态误差代价权重矩阵
    R_param_ = opti_.parameter(4, 4);         // 声明控制输入代价权重矩阵

    Function f = buildDynamicsFunction(); // 调用已配置好的系统动力学模型映射对象

    MX cost = 0.0; // 初始化代价函数变量为0

    // 强约束条件：规划起始点的状态必须严格等于给定的初始观测状态
    opti_.subject_to(X_var_(Slice(), 0) == X0_param_);

    // 使用四阶龙格库塔法(Runge-Kutta 4)对方程进行多步离散化积分
    for (int k = 0; k < N; ++k)
    {
        MX x_k = X_var_(Slice(), k); // 第k步的状态
        MX u_k = U_var_(Slice(), k); // 第k步的控制

        std::vector<MX> f1_res = f(std::vector<MX>{x_k, u_k}); // 第一斜率 k1
        MX k1 = f1_res[0];

        std::vector<MX> f2_res = f(std::vector<MX>{x_k + 0.5 * dt * k1, u_k}); // 第二斜率 k2
        MX k2 = f2_res[0];

        std::vector<MX> f3_res = f(std::vector<MX>{x_k + 0.5 * dt * k2, u_k}); // 第三斜率 k3
        MX k3 = f3_res[0];

        std::vector<MX> f4_res = f(std::vector<MX>{x_k + dt * k3, u_k}); // 第四斜率 k4
        MX k4 = f4_res[0];

        // 通过RK4公式预测出第k+1步的离散状态期望值
        MX x_next = x_k + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

        // 加入多重射击约束(Multiple Shooting)：保证决策变量在前后步符合动力递推
        opti_.subject_to(X_var_(Slice(), k + 1) == x_next);

        // 提取预测得到的下一状态中的四元数分量
        MX q_next = X_var_(Slice(6, 10), k + 1);
        // 对四元数范数的平方进行软约束（容许轻微松弛，范围0.99~1.01）防止优化问题无解
        opti_.subject_to(sumsqr(q_next) >= 0.99);
        opti_.subject_to(sumsqr(q_next) <= 1.01);

        // 控制限幅硬约束：推力不能低于最小维持推力门限
        opti_.subject_to(u_k(0) >= params_.min_thrust);
        // 控制限幅硬约束：推力不能高于电机满载上限
        opti_.subject_to(u_k(0) <= params_.max_thrust);

        // 角速度约束：对x轴机体角速度(滚转)的上下限作规范
        opti_.subject_to(u_k(1) >= -params_.max_bodyrate_x);
        opti_.subject_to(u_k(1) <= params_.max_bodyrate_x);
        // 角速度约束：对y轴机体角速度(俯仰)的上下限作规范
        opti_.subject_to(u_k(2) >= -params_.max_bodyrate_y);
        opti_.subject_to(u_k(2) <= params_.max_bodyrate_y);
        // 角速度约束：对z轴机体角速度(偏航)的上下限作规范
        opti_.subject_to(u_k(3) >= -params_.max_bodyrate_z);
        opti_.subject_to(u_k(3) <= params_.max_bodyrate_z);

        // 计算第k步系统的预测状态和目标预设参考轨迹的状态偏差量
        MX error = x_k - Xref_param_(Slice(), k);

        // 代价叠加规则之一：基于给定的权重矩阵Q对跟踪表现的偏量施加代价惩罚
        cost += mtimes(mtimes(error.T(), Q_param_), error);
        // 代价叠加规则之二：基于给定的权重矩阵R对控制输出大小施加代价惩罚，防止突变振荡
        cost += mtimes(mtimes(u_k.T(), R_param_), u_k);
    }

    // 构建预测视界最后一步的终端误差状态差值
    MX error_term = X_var_(Slice(), N) - Xref_param_(Slice(), N);

    // 终端代价惩罚：针对最后达到的节点采用通常10倍以上强效的Q矩阵乘数，拉住末端收敛
    cost += mtimes(mtimes(error_term.T(), Q_param_), error_term) * params_.terminal_cost_multiplier;

    // 设置目标函数到优化器，要求问题最终趋于求解此项代价函数极小化的变量分布
    opti_.minimize(cost);

    // 给定求解器（Ipopt）相关的内部调校参数
    Dict solver_opts;
    solver_opts["expand"] = true;               // 将SX计算树全部展开提升求解评估速度
    solver_opts["ipopt.print_level"] = 0;       // 关闭Ipopt迭代日志的刷屏输出
    solver_opts["print_time"] = 0;              // 关闭各部求解用时通报
    solver_opts["ipopt.max_iter"] = 100;        // 避免卡死，强制截断最大迭代步数为100步
    solver_opts["ipopt.tol"] = 1e-3;            // 指定Ipopt完成求解的一般公差下限
    solver_opts["ipopt.acceptable_tol"] = 1e-2; // 放宽求解器验收标准以在算力有限时保持帧率稳定
    solver_opts["ipopt.sb"] = "yes";            // 干掉头部商标打印输出

    // 挂载内点法非线性求解器Ipopt并装填字典参
    opti_.solver("ipopt", solver_opts);

    // 重设将要送去给优化器作为试探冷启动依据的状态初值数组形状
    u_warm_start_.resize(4 * N, 0.0);
    x_warm_start_.resize(14 * (N + 1), 0.0);

    // 利用先验物理特性合理化冷启动猜测分布增强收敛可能
    for (int k = 0; k < N; ++k)
    {
        u_warm_start_[k * 4] = params_.mass * params_.gravity; // 初猜推力为重力抵消状态量
        x_warm_start_[k * 14 + 6] = 1.0;                       // 初猜四元数的w部实部为1（正向悬停偏置）
    }
    x_warm_start_[N * 14 + 6] = 1.0; // 视界最后一个多状态位同样赋予w为1

    ROS_INFO("CasADi NLP Formulation completed successfully."); // NLP构建完毕回打确认消息
}

// 里程计状态数据接收与私有参数同步的回调函数
void QuadrotorNMPC::odomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_); // 抢占多线程锁互斥写操作确保一致

    // 将ROS导航消息里的3维空间绝对坐标值同步给当前系统10维状态量开端
    current_x_(0) = msg->pose.pose.position.x;
    current_x_(1) = msg->pose.pose.position.y;
    current_x_(2) = msg->pose.pose.position.z;

    // 同步消息结构体里的世界坐标系向下的XYZ向线速度
    current_x_(3) = msg->twist.twist.linear.x;
    current_x_(4) = msg->twist.twist.linear.y;
    current_x_(5) = msg->twist.twist.linear.z;

    // 解解剖结构抓出四元素并赋零时名预检
    double qw = msg->pose.pose.orientation.w;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;

    // 获取并检查传包而来的四元数自身模长合法性
    double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm < 1e-6) // 遇极端故障出现纯0则规整复位
    {
        // 直接将10维向量中指代位姿状态的成员强掰回物理单位直立元
        current_x_(6) = 1.0;
        current_x_(7) = 0.0;
        current_x_(8) = 0.0;
        current_x_(9) = 0.0;
    }
    else
    {
        // 通过严格归除保证送去做矩阵运算的基数值纯净度（等于或略微接近1均将被强制打平到严格单位1）
        current_x_(6) = qw / norm;
        current_x_(7) = qx / norm;
        current_x_(8) = qy / norm;
        current_x_(9) = qz / norm;
    }

    // 更新扩展状态层：将上一时刻下发的指令作为当前时刻的内部实际状态观测
    current_x_(10) = last_cmd_(0) > 0 ? last_cmd_(0) : params_.mass * params_.gravity;
    current_x_(11) = last_cmd_(1);
    current_x_(12) = last_cmd_(2);
    current_x_(13) = last_cmd_(3);

    has_odom_ = true; // 在成功读包校验通过后拨开许可全局工作流的标志位
}

// 目标轨迹回调函数，接收规划器生成的全状态参考轨迹
void QuadrotorNMPC::trajCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(traj_mutex_); // 加锁，保证轨迹更新的线程安全

    if (msg->points.empty()) // 检查接收到的轨迹是否为空
    {
        ROS_WARN_THROTTLE(1.0, "[NMPC] Received empty trajectory!"); // 限制输出频率，发出警告
        return;
    }

    int traj_size = msg->points.size(); // 获取轨迹点数量

    Eigen::MatrixXd new_ref = Eigen::MatrixXd::Zero(14, params_.N + 1); // 构建新的参考轨迹矩阵 (状态维度10 x 预测步数N+1)

    for (int k = 0; k <= params_.N; ++k) // 遍历预测时域，填充参考状态
    {
        // 限制索引，若参考轨迹较短，则末端保持最后一个点
        int idx = std::min(k, traj_size - 1);
        auto &point = msg->points[idx];

        // 填充位置参考值
        new_ref(0, k) = point.transforms[0].translation.x;
        new_ref(1, k) = point.transforms[0].translation.y;
        new_ref(2, k) = point.transforms[0].translation.z;

        // 填充速度参考值
        new_ref(3, k) = point.velocities[0].linear.x;
        new_ref(4, k) = point.velocities[0].linear.y;
        new_ref(5, k) = point.velocities[0].linear.z;

        // 提取加速度（正常MPC使用简单四元数）这里简单使用平坦性映射一下姿态或者直接用速度方向，或者用传过来的偏航角算四元数
        // 保持和旧版本类似的悬停四元数处理
        double qw = point.transforms[0].rotation.w;
        double qx = point.transforms[0].rotation.x;
        double qy = point.transforms[0].rotation.y;
        double qz = point.transforms[0].rotation.z;

        if (std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz) < 1e-4)
        {
            new_ref(6, k) = 1.0;
            new_ref(7, k) = 0.0;
            new_ref(8, k) = 0.0;
            new_ref(9, k) = 0.0;
        }
        else
        {
            new_ref(6, k) = qw;
            new_ref(7, k) = qx;
            new_ref(8, k) = qy;
            new_ref(9, k) = qz;
        }

        // 参考状态的推力设为悬停推力，角速度参考保持为0
        new_ref(10, k) = params_.mass * params_.gravity;
        new_ref(11, k) = 0.0;
        new_ref(12, k) = 0.0;
        new_ref(13, k) = 0.0;
    }

    ref_traj_ = new_ref; // 更新内部参考轨迹

    has_traj_ = true; // 标记已接收到轨迹

    current_state_ = ControllerState::TRACKING; // 切换状态机至轨迹跟踪模式
}

// 从ROS参数服务器加载MPC配置参数
void QuadrotorNMPC::loadParameters()
{
    pnh_.param("N", params_.N, 20);
    pnh_.param("dt", params_.dt, 0.05);
    pnh_.param("mass", params_.mass, 1.0);
    pnh_.param("gravity", params_.gravity, 9.81);
    pnh_.param("max_thrust", params_.max_thrust, 20.0);
    pnh_.param("min_thrust", params_.min_thrust, 1.0);
    pnh_.param("max_bodyrate_x", params_.max_bodyrate_x, 3.0);
    pnh_.param("max_bodyrate_y", params_.max_bodyrate_y, 3.0);
    pnh_.param("max_bodyrate_z", params_.max_bodyrate_z, 1.5);

    pnh_.param("drag_x", params_.drag_x, 0.1);
    pnh_.param("drag_y", params_.drag_y, 0.1);
    pnh_.param("drag_z", params_.drag_z, 0.1);
    pnh_.param("tau_thrust", params_.tau_thrust, 0.05);
    pnh_.param("tau_rate", params_.tau_rate, 0.02);
    pnh_.param("use_aero_drag_and_delay", params_.use_aero_drag_and_delay, true);
    pnh_.param("terminal_cost_multiplier", params_.terminal_cost_multiplier, 10.0);

    if (!pnh_.getParam("Q_diag", params_.Q_diag))
    {
        ROS_WARN("[NMPC] Missing 'Q_diag', using default 14D weights.");
        params_.Q_diag = {50.0, 50.0, 100.0, 10.0, 10.0, 10.0, 1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1, 0.1};
    }
    if (!pnh_.getParam("R_diag", params_.R_diag))
    {
        ROS_WARN("[NMPC] Missing 'R_diag', using default 4D weights.");
        params_.R_diag = {0.1, 10.0, 10.0, 5.0};
    }

    if (params_.Q_diag.size() != 14)
    {
        ROS_ERROR("[NMPC] Q_diag must be 14D, got %zu. Reset to defaults.", params_.Q_diag.size());
        params_.Q_diag = {50.0, 50.0, 100.0, 10.0, 10.0, 10.0, 1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1, 0.1};
    }
    if (params_.R_diag.size() != 4)
    {
        ROS_ERROR("[NMPC] R_diag must be 4D, got %zu. Reset to defaults.", params_.R_diag.size());
        params_.R_diag = {0.1, 10.0, 10.0, 5.0};
    }
}

// 控制器主循环定时器回调 (运行频率100Hz)
void QuadrotorNMPC::controlLoop(const ros::TimerEvent &event)
{
    // 检查里程计数据，无状态估计时拒绝进入控制
    if (!has_odom_)
    {
        ROS_WARN_THROTTLE(2.0, "[NMPC] Waiting for Odometry...");
        return; // 退出当前循环
    }

    Eigen::VectorXd x0_curr;   // 用于存储当前时刻起点状态
    Eigen::MatrixXd xref_curr; // 用于存放本周期的参考轨迹

    {
        std::lock(odom_mutex_, traj_mutex_); // 同时加锁防止死锁发生
        std::lock_guard<std::mutex> lock1(odom_mutex_, std::adopt_lock); // 锁定里程计状态
        std::lock_guard<std::mutex> lock2(traj_mutex_, std::adopt_lock); // 锁定轨迹状态
        x0_curr = current_x_;                           // 提取当前机体状态

        // 判断是否具备跟踪条件
        if (current_state_ == ControllerState::TRACKING && has_traj_)
        {
            xref_curr = ref_traj_; // 使用目标参考轨迹
        }
        else
        {
            // 如果没有跟踪任务，则生成一条以当前位置为驻点的悬停参考轨迹
            xref_curr = Eigen::MatrixXd::Zero(14, params_.N + 1);

            for (int k = 0; k <= params_.N; ++k) // 所有预测步保持目标点不变
            {
                xref_curr(0, k) = x0_curr(0);
                xref_curr(1, k) = x0_curr(1);
                xref_curr(2, k) = x0_curr(2);
                xref_curr(6, k) = 1.0;
                xref_curr(10, k) = params_.mass * params_.gravity;
            }
        }
    } // 释放互斥锁，允许回调函数继续更新状态

    // 将参数转换为矩阵形式（对角阵）供CasADi推断使用
    Eigen::MatrixXd Q_mat = Eigen::VectorXd::Map(params_.Q_diag.data(), 14).asDiagonal();
    Eigen::MatrixXd R_mat = Eigen::VectorXd::Map(params_.R_diag.data(), 4).asDiagonal();

    // 转换起点状态到CasADi DM格式
    std::vector<double> x0_vec(x0_curr.data(), x0_curr.data() + x0_curr.size());
    casadi::DM P_x0 = casadi::DM(x0_vec); // 起点状态DM格式

    casadi::DM P_xref = casadi::DM::zeros(14, params_.N + 1); // 初始化CasADi参考轨迹DM
    for (int i = 0; i < 14; i++)                              // 填充参考信息
    {
        for (int j = 0; j <= params_.N; j++)
        {
            P_xref(i, j) = xref_curr(i, j); // 写入每个步长的参考量
        }
    }

    casadi::DM P_Q = casadi::DM::zeros(14, 14); // 打包Q方阵到DM
    casadi::DM P_R = casadi::DM::zeros(4, 4);   // 打包R方阵到DM
    for (int i = 0; i < 14; i++)
        P_Q(i, i) = params_.Q_diag[i];
    for (int i = 0; i < 4; i++)
        P_R(i, i) = params_.R_diag[i];

    // 向构建好的优化问题参数对象传递本轮数值计算的变量值
    opti_.set_value(X0_param_, P_x0);
    opti_.set_value(Xref_param_, P_xref);
    opti_.set_value(Q_param_, P_Q);
    opti_.set_value(R_param_, P_R);

    // 设置优化器的初始猜想 (热启动 Warm Start)，提高收敛速度
    casadi::DM X_guess = casadi::DM::zeros(14, params_.N + 1);
    casadi::DM U_guess = casadi::DM::zeros(4, params_.N);
    std::copy(x_warm_start_.begin(), x_warm_start_.end(), X_guess.ptr());
    std::copy(u_warm_start_.begin(), u_warm_start_.end(), U_guess.ptr());
    opti_.set_initial(X_var_, X_guess); // 状态的初始猜测
    opti_.set_initial(U_var_, U_guess); // 控制量的初始猜测

    auto start_opt = std::chrono::high_resolution_clock::now(); // 记录优化起算时间
    std::vector<double> opt_u0(4, 0.0);                         // 存放优化的最优第一拍控制量
    bool success = false;                                       // 优化成功标志位

    try
    {
        // 调用底层的Ipopt求解器进行非线性规划计算
        casadi::OptiSol sol = opti_.solve();

        // 若成功则提取完整控制序列与状态轨迹序列
        casadi::DM u_opt = sol.value(U_var_);
        casadi::DM x_opt = sol.value(X_var_);

        // 将解答矩阵表第一列（即当前控制步需要施加的指令）存入变量
        for (int i = 0; i < 4; ++i)
            opt_u0[i] = static_cast<double>(u_opt(i, 0));

        // 利用滑窗法则构造下一优化周期的热启动初值数组
        for (int i = 0; i < params_.N - 1; i++)
        {
            for (int j = 0; j < 4; j++)
                u_warm_start_[i * 4 + j] = static_cast<double>(u_opt(j, i + 1));
            for (int j = 0; j < 14; j++)
                x_warm_start_[i * 14 + j] = static_cast<double>(x_opt(j, i + 1));
        }

        // 对于空缺的最末端状态，采用前一时刻状态平移填补
        for (int j = 0; j < 4; j++)
            u_warm_start_[(params_.N - 1) * 4 + j] = static_cast<double>(u_opt(j, params_.N - 1));
        for (int j = 0; j < 14; j++)
        {
            x_warm_start_[(params_.N - 1) * 14 + j] = static_cast<double>(x_opt(j, params_.N));
            x_warm_start_[params_.N * 14 + j] = static_cast<double>(x_opt(j, params_.N));
        }

        // --- 发布NMPC预测的未来状态轨迹用于RViz可视化 ---
        nav_msgs::Path predicted_path;
        predicted_path.header.stamp = ros::Time::now();
        predicted_path.header.frame_id = "world"; // 全局固定坐标系
        for (int i = 0; i <= params_.N; i++)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = predicted_path.header;
            pose.pose.position.x = static_cast<double>(x_opt(0, i));
            pose.pose.position.y = static_cast<double>(x_opt(1, i));
            pose.pose.position.z = static_cast<double>(x_opt(2, i));
            pose.pose.orientation.w = static_cast<double>(x_opt(6, i));
            pose.pose.orientation.x = static_cast<double>(x_opt(7, i));
            pose.pose.orientation.y = static_cast<double>(x_opt(8, i));
            pose.pose.orientation.z = static_cast<double>(x_opt(9, i));
            predicted_path.poses.push_back(pose);
        }
        pred_path_pub_.publish(predicted_path);

        success = true; // 求解成功标志
    }
    // 如果求解失败或不收敛，进行异常捕获
    catch (const casadi::CasadiException &e)
    {
        // 终端输出错误提醒，并触发原地悬停的降级保底保护逻辑
        ROS_ERROR_STREAM_THROTTLE(1.0, "[NMPC] CasADi solver failed. Hovering fallback. Error: " << e.what());
        // 推力控制回归悬停定推力，其它角速度令为0保持抗坠平衡
        opt_u0[0] = params_.mass * params_.gravity;
        opt_u0[1] = 0.0; // 滚转率清零
        opt_u0[2] = 0.0; // 俯仰率清零
        opt_u0[3] = 0.0; // 偏航率清零
    }

    auto finish_opt = std::chrono::high_resolution_clock::now();    // 记录优化结束时间
    std::chrono::duration<double> elapsed = finish_opt - start_opt; // 计算消耗时间(秒)

    // 倘若求解运算开销大于 0.01秒 (10ms)，发频次限制性警告防止飞控掉帧卡顿
    if (elapsed.count() > 0.01)
    {
        ROS_WARN_THROTTLE(1.0, "[NMPC] Solver took %f ms", elapsed.count() * 1000.0);
    }

    // 构造发送给PX4的控制量格式（通过MAVROS消息接口）
    mavros_msgs::AttitudeTarget att_msg;
    att_msg.header.stamp = ros::Time::now(); // 当前时间戳
    att_msg.header.frame_id = "base_link";   // 基于机体系

    // 设定掩码，忽略绝对姿态，仅下发期望的三轴角速度及推力
    att_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

    // 装载求解出来的三轴角速度指令
    att_msg.body_rate.x = opt_u0[1];
    att_msg.body_rate.y = opt_u0[2];
    att_msg.body_rate.z = opt_u0[3];

    // 下发的油门是基于0到1范围归一化的（推比标定），在此作比例映射限制化
    double T_raw = opt_u0[0];
    
    double T_normalized = 0.0;
    if (params_.max_thrust - params_.min_thrust > 1e-5) {
        T_normalized = (T_raw - params_.min_thrust) / (params_.max_thrust - params_.min_thrust);
    }
    
    T_normalized = std::max(0.0, std::min(1.0, T_normalized)); // 夹紧油门至[0, 1]阈值防超限

    att_msg.thrust = T_normalized;
    last_cmd_ = Eigen::Vector4d(T_raw, opt_u0[1], opt_u0[2], opt_u0[3]); // 备份真实物理推力指令及角速度用于下一次延迟积分 // 赋值归一化推力

    ctrl_pub_.publish(att_msg); // 发布最终执行指令以驱动机身
}
// QuadrotorNMPC类的构造函数
QuadrotorNMPC::QuadrotorNMPC(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh), current_state_(ControllerState::INIT), current_x_(Eigen::VectorXd::Zero(14)), has_odom_(false), has_traj_(false)
{
    loadParameters(); // 调用参数加载方法

    setupNLP(); // 初始化并构建底层CasADi NLP优化模型环境

    // 定义ROS通信回调收发主题
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/Odometry", 1, &QuadrotorNMPC::odomCb, this);
    traj_sub_ = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/trajectory", 1, &QuadrotorNMPC::trajCb, this);
    // 主控制出口声明
    ctrl_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    pred_path_pub_ = nh_.advertise<nav_msgs::Path>("/nmpc_predicted_path", 1);
    // 绑定心跳定时主循环计时器并在100Hz执行
    mpc_timer_ = nh_.createTimer(ros::Duration(0.01), &QuadrotorNMPC::controlLoop, this);
    ROS_INFO("NMPC Node Initialized at 100Hz.");
}

QuadrotorNMPC::~QuadrotorNMPC() {} // 析构函数，对象生命周期由ROS句柄自动回收
