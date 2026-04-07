#include "../include/quadrotor_nmpc_advanced.h"
#include <iostream>
#include <chrono>

using namespace casadi;

QuadrotorNMPCAdvanced::QuadrotorNMPCAdvanced(double mass, double gravity, double dt, int N,
                                             double drag_x, double drag_y, double drag_z,
                                             double tau_thrust, double tau_rate,
                                             bool use_aero_drag_and_delay)
    // 使用初始化列表写入基础模型参数，避免构造后再赋值
    : mass_(mass), gravity_(gravity), dt_(dt), N_(N),
      drag_x_(drag_x), drag_y_(drag_y), drag_z_(drag_z),
      tau_thrust_(tau_thrust), tau_rate_(tau_rate),
      use_aero_drag_and_delay_(use_aero_drag_and_delay)
{
    // 启动日志：提示 Advanced MPC 内核已初始化
    ROS_INFO("Initializing Advanced SO(3) Error-State NMPC Engine w/ Drag & First-Order Delay...");
    // 预分配热启动缓存：状态为14*(N+1)，控制为4*N
    x_warm_start_.resize(14 * (N + 1), 0.0);
    u_warm_start_.resize(4 * N, 0.0);
}

// 析构函数：当前无需手动释放资源
QuadrotorNMPCAdvanced::~QuadrotorNMPCAdvanced() {}

// CasADi 四元数乘法：返回 q = q1 ⊗ q2
casadi::MX QuadrotorNMPCAdvanced::quatMultiply(const casadi::MX &q1, const casadi::MX &q2)
{
    // Hamilton 乘法公式（标量在前）
    MX qw = q1(0) * q2(0) - q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3);
    MX qx = q1(0) * q2(1) + q1(1) * q2(0) + q1(2) * q2(3) - q1(3) * q2(2);
    MX qy = q1(0) * q2(2) - q1(1) * q2(3) + q1(2) * q2(0) + q1(3) * q2(1);
    MX qz = q1(0) * q2(3) + q1(1) * q2(2) - q1(2) * q2(1) + q1(3) * q2(0);
    // 拼接为 4x1 四元数向量
    return MX::vertcat({qw, qx, qy, qz});
}

// CasADi 四元数求逆：单位四元数下等于共轭
casadi::MX QuadrotorNMPCAdvanced::quatInverse(const casadi::MX &q)
{
    // q^{-1} = [qw, -qx, -qy, -qz]
    return MX::vertcat({q(0), -q(1), -q(2), -q(3)});
}

// 14维状态：[p(3), v(3), q(4), T(1), w(3)]
casadi::Function QuadrotorNMPCAdvanced::buildDynamicsFunction()
{
    // 定义符号状态与控制输入
    MX x = MX::sym("x", 14);
    MX u = MX::sym("u", 4);

    // 状态拆分：位置、速度、四元数、实际推力、实际角速度
    MX p = x(Slice(0, 3));
    MX v = x(Slice(3, 6));
    MX q = x(Slice(6, 10));
    MX T_act = x(10);
    MX w_act = x(Slice(11, 14));

    // 控制拆分：推力指令与三轴角速度指令
    MX T_cmd = u(0);
    MX wx_cmd = u(1);
    MX wy_cmd = u(2);
    MX wz_cmd = u(3);

    // 实际角速度分量
    MX wx = w_act(0);
    MX wy = w_act(1);
    MX wz = w_act(2);

    // 四元数分量（w, x, y, z）
    MX qw = q(0), qx = q(1), qy = q(2), qz = q(3);

    // 由四元数构造旋转矩阵 R（机体系 -> 世界系）
    MX R11 = 1 - 2 * (qy * qy + qz * qz);
    MX R12 = 2 * (qx * qy - qw * qz);
    MX R13 = 2 * (qx * qz + qw * qy);

    MX R21 = 2 * (qx * qy + qw * qz);
    MX R22 = 1 - 2 * (qx * qx + qz * qz);
    MX R23 = 2 * (qy * qz - qw * qx);

    MX R31 = 2 * (qx * qz - qw * qy);
    MX R32 = 2 * (qy * qz + qw * qx);
    MX R33 = 1 - 2 * (qx * qx + qy * qy);

    // 拼接为 3x3 旋转矩阵
    MX R = MX::vertcat({MX::horzcat({R11, R12, R13}),
                        MX::horzcat({R21, R22, R23}),
                        MX::horzcat({R31, R32, R33})});

    // 位置导数等于速度
    MX dp = v;

    // 开关逻辑：启用时用“实际值”，关闭时直接用“指令值”
    MX actual_T = use_aero_drag_and_delay_ ? T_act : T_cmd;
    MX actual_wx = use_aero_drag_and_delay_ ? wx : wx_cmd;
    MX actual_wy = use_aero_drag_and_delay_ ? wy : wy_cmd;
    MX actual_wz = use_aero_drag_and_delay_ ? wz : wz_cmd;

    // 推力、重力、阻力项
    MX thrust_vec = MX::vertcat({0.0, 0.0, actual_T / mass_});
    MX gravity_vec = MX::vertcat({0.0, 0.0, gravity_});
    MX drag_force = use_aero_drag_and_delay_ ? MX::vertcat({drag_x_ * v(0), drag_y_ * v(1), drag_z_ * v(2)})
                                             : MX::vertcat({0.0, 0.0, 0.0});

    // 速度导数：推力投影 - 重力 - 阻力
    MX dv = mtimes(R, thrust_vec) - gravity_vec - drag_force;

    // 四元数导数（由机体系角速度驱动）
    MX dqw = 0.5 * (-qx * actual_wx - qy * actual_wy - qz * actual_wz);
    MX dqx = 0.5 * (qw * actual_wx - qz * actual_wy + qy * actual_wz);
    MX dqy = 0.5 * (qz * actual_wx + qw * actual_wy - qx * actual_wz);
    MX dqz = 0.5 * (-qy * actual_wx + qx * actual_wy + qw * actual_wz);
    MX dq = MX::vertcat({dqw, dqx, dqy, dqz});

    // 一阶延迟模型：启用时按 tau 滤波，关闭时直接跟随
    MX dT = use_aero_drag_and_delay_ ? (T_cmd - T_act) / tau_thrust_ : (T_cmd - T_act);
    MX dwx = use_aero_drag_and_delay_ ? (wx_cmd - wx) / tau_rate_ : (wx_cmd - wx);
    MX dwy = use_aero_drag_and_delay_ ? (wy_cmd - wy) / tau_rate_ : (wy_cmd - wy);
    MX dwz = use_aero_drag_and_delay_ ? (wz_cmd - wz) / tau_rate_ : (wz_cmd - wz);
    MX dw = MX::vertcat({dwx, dwy, dwz});

    // 合并得到完整状态导数
    MX dx = MX::vertcat({dp, dv, dq, dT, dw});
    // 返回 CasADi 动力学函数对象
    return Function("f", {x, u}, {dx}, {"x", "u"}, {"dx"});
}

// 采用9维状态误差的终端LQR增益分配，用于求解无限时域终端惩罚矩阵 P
// 9维简化误差状态为: [位置(3), 速度(3), 姿态差向角(3)]
Eigen::MatrixXd QuadrotorNMPCAdvanced::computeLQRTerminalCost(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R)
{
    // 构造连续时间系统的雅可比矩阵 Ac (9x9) 和 Bc (9x4)，假设工作点为悬停平衡点
    Eigen::MatrixXd Ac = Eigen::MatrixXd::Zero(9, 9);
    Eigen::MatrixXd Bc = Eigen::MatrixXd::Zero(9, 4);

    // dp = v (位置的微分为速度)
    Ac.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3);
    // 考虑小角度悬停假设：dv_x = g * pitch, dv_y = -g * roll (重力在水平方向上的投影)
    Ac(3, 7) = gravity_;
    Ac(4, 6) = -gravity_;
    // 控制输入对状态的影响：推力产生Z轴加速度 dv_z = T / m
    Bc(5, 0) = 1.0 / mass_;
    // 控制输入角速度直接作为系统姿态变化的驱动: d(theta) = omega
    Bc.block<3, 3>(6, 1) = Eigen::MatrixXd::Identity(3, 3);

    // 采用前向欧拉法离散化连续系统矩阵: Ad = I + Ac * dt, Bd = Bc * dt
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(9, 9) + Ac * dt_;
    Eigen::MatrixXd Bd = Bc * dt_;

    // 迭代求解离散代数黎卡提方程 (Discrete Algebraic Riccati Equation, DARE)
    Eigen::MatrixXd P = Q; // 初始化黎卡提方程的解 P，初值为代价矩阵 Q
    Eigen::MatrixXd P_next = Eigen::MatrixXd::Zero(9, 9);
    double tolerance = 1e-6; // 求解收敛容差

    for (int i = 0; i < 1000; ++i)
    {
        // 1. 计算反馈增益 K_term = (R + B^T * P * B)^-1 * (B^T * P * A)
        Eigen::MatrixXd BPB_R = R + Bd.transpose() * P * Bd;
        Eigen::MatrixXd K_term = BPB_R.llt().solve(Bd.transpose() * P * Ad);

        // 2. 更新 P_next = A^T * P * A - A^T * P * B * K_term + Q
        P_next = Ad.transpose() * P * Ad - Ad.transpose() * P * Bd * K_term + Q;

        // 3. 检查矩阵 P 的变化量(F-norm)是否已达到收敛要求
        if ((P_next - P).norm() < tolerance)
            break;
        P = P_next;
    }
    return P; // 返回稳态最优终端代价矩阵 P
}

// 核心NLP构造函数：定义优化变量、参数、代价函数和系统动力学约束
void QuadrotorNMPCAdvanced::setupNLP(const std::vector<double> &Q_diag, const std::vector<double> &R_diag,
                                     double thrust_min, double thrust_max,
                                     double rate_x, double rate_y, double rate_z,
                                     double terminal_cost_multiplier)
{
    opti_ = casadi::Opti(); // 实例化 CasADi 的非线性优化接口 Opti

    // --- 声明优化变量 (Variables) ---
    // 沿所有周期计算真实状态而非纯误差动态，X_var_ 维度：14*(N+1)
    X_var_ = opti_.variable(14, N_ + 1);
    // 控制输入优化变量（要由求解器算出的控制量），U_var_ 维度：4*N
    U_var_ = opti_.variable(4, N_);

    // --- 声明外部传入参数 (Parameters) ---
    X0_param_ = opti_.parameter(14, 1);        // NMPC起点状态（当前无人机实际状态反馈）
    Xref_param_ = opti_.parameter(14, N_ + 1); // 预测期内的全状态参考轨迹 (如位置、速度等)
    Uref_param_ = opti_.parameter(4, N_);      // 预测期内的控制量参考轨迹 (一般用于前馈重力补偿推力)

    // 强约束条件：起始状态 X_var_[0] 必须等同于系统的当前观测状态 X0_param_
    opti_.subject_to(X_var_(Slice(), 0) == X0_param_);

    // 构造对角代价权重矩阵
    casadi::MX Q_matrix = casadi::MX::diag(casadi::MX(Q_diag));
    casadi::MX R_matrix = casadi::MX::diag(casadi::MX(R_diag));

    // 先通过Eigen调用前面实现的LQR方法，解算出终端惩罚矩阵 P_eig
    Eigen::Map<const Eigen::VectorXd> eig_Q(Q_diag.data(), Q_diag.size());
    Eigen::Map<const Eigen::VectorXd> eig_R(R_diag.data(), R_diag.size());
    Eigen::MatrixXd P_eig = computeLQRTerminalCost(eig_Q.asDiagonal(), eig_R.asDiagonal());

    // 将 Eigen 格式的矩阵 P_eig 逐步赋值转移给 CasADi 符号矩阵 P_matrix
    casadi::MX P_matrix = casadi::MX::zeros(9, 9);
    for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j)
            P_matrix(i, j) = P_eig(i, j);

    // 取出预先构造好的基于符号计算的动力学方程 f(x, u)
    casadi::Function f = buildDynamicsFunction();
    casadi::MX obj = 0; // 目标代价函数清零

    // 遍历预测时域内的每个步长 k (0 到 N-1)
    for (int k = 0; k < N_; ++k)
    {
        // 提出当前步 k 下的状态和控制量，以及它们对应的设定参考值
        casadi::MX x_k = X_var_(Slice(), k);
        casadi::MX u_k = U_var_(Slice(), k);
        casadi::MX x_ref = Xref_param_(Slice(), k);
        casadi::MX u_ref = Uref_param_(Slice(), k);

        // --- 计算代价惩罚所需的系统误差 ---
        // 为了方便求解LQR和减小位姿误差奇异性，仅仅提取切流形上的 9个 误差项（位置 p, 速度 v, 差向角 dtheta）
        casadi::MX dp = x_k(Slice(0, 3)) - x_ref(Slice(0, 3)); // 平移偏差
        casadi::MX dv = x_k(Slice(3, 6)) - x_ref(Slice(3, 6)); // 速度偏差

        // 获取并计算SO(3)姿态流形上的距离
        casadi::MX q_k = x_k(Slice(6, 10));          // 当前四元数
        casadi::MX q_ref = x_ref(Slice(6, 10));      // 期望四元数
        casadi::MX q_inv = quatInverse(q_ref);       // 期望姿态的逆/共轭
        casadi::MX q_err = quatMultiply(q_inv, q_k); // 求出两姿态间的相对偏差四元数 q_e = q_d^{-1} * q
        // CasADi下的简化四元数夹角映射 -> 由误差四元数的虚部提取 SO(3) 差向角
        casadi::MX dtheta = 2.0 * q_err(Slice(1, 4));

        // 拼接成完整的处于切空间的 9维 状态误差（位置，速度，局部姿态偏差）
        casadi::MX err_9d = MX::vertcat({dp, dv, dtheta});
        // 控制量的偏差度量（与前馈期望的差异）
        casadi::MX du = u_k - u_ref;

        // 累加本次积分区间的二次型运行代价： x^T.Q.x + u^T.R.u
        obj += casadi::MX::mtimes({err_9d.T(), Q_matrix, err_9d});
        obj += casadi::MX::mtimes({du.T(), R_matrix, du});

        // --- 施加多元数值积分动力学约束 (Multiple Shooting) ---
        // 采用经典的四阶龙格-库塔(Runge-Kutta 4)积分方法进行高精度状态递推
        // k1 = f(x_k, u_k)
        std::vector<casadi::MX> f1_res = f(std::vector<casadi::MX>{x_k, u_k});
        casadi::MX k1 = f1_res[0];
        // k2 = f(x_k + 0.5*dt*k1, u_k)
        std::vector<casadi::MX> f2_res = f(std::vector<casadi::MX>{x_k + 0.5 * dt_ * k1, u_k});
        casadi::MX k2 = f2_res[0];
        // k3 = f(x_k + 0.5*dt*k2, u_k)
        std::vector<casadi::MX> f3_res = f(std::vector<casadi::MX>{x_k + 0.5 * dt_ * k2, u_k});
        casadi::MX k3 = f3_res[0];
        // k4 = f(x_k + dt*k3, u_k)
        std::vector<casadi::MX> f4_res = f(std::vector<casadi::MX>{x_k + dt_ * k3, u_k});
        casadi::MX k4 = f4_res[0];

        // 经RK4推算出理论上步的下一时刻状态 x_{k+1}, 这里 u_k 采用前向欧拉的一阶保持(ZOH)
        casadi::MX x_next = x_k + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

        // 引入等式约束：告诉求解器，轨迹链条必须符合物理递推关系
        opti_.subject_to(X_var_(Slice(), k + 1) == x_next);

        // --- 引入不等式控制范围限幅约束 ---
        opti_.subject_to(opti_.bounded(thrust_min, u_k(0), thrust_max)); // 推力上下界限制
        opti_.subject_to(opti_.bounded(-rate_x, u_k(1), rate_x));        // 机体系滚转角速度限幅
        opti_.subject_to(opti_.bounded(-rate_y, u_k(2), rate_y));        // 机体系俯仰角速度限幅
        opti_.subject_to(opti_.bounded(-rate_z, u_k(3), rate_z));        // 机体系偏航角速度限幅
    }

    // 后续LQR终端接驳
    casadi::MX x_N = X_var_(Slice(), N_);                         // 取终端状态
    casadi::MX x_ref_N = Xref_param_(Slice(), N_);                // 取终端参考状态
    casadi::MX dp_N = x_N(Slice(0, 3)) - x_ref_N(Slice(0, 3));    // 终端位置误差
    casadi::MX dv_N = x_N(Slice(3, 6)) - x_ref_N(Slice(3, 6));    // 终端速度误差
    casadi::MX q_N = x_N(Slice(6, 10));                           // 终端四元数
    casadi::MX q_ref_N = x_ref_N(Slice(6, 10));                   // 终端参考四元数
    casadi::MX q_err_N = quatMultiply(quatInverse(q_ref_N), q_N); // 终端姿态误差四元数
    casadi::MX dtheta_N = 2.0 * q_err_N(Slice(1, 4));             // 终端姿态差向角
    casadi::MX err_9d_N = MX::vertcat({dp_N, dv_N, dtheta_N});    // 拼接终端9维误差

    // 加上终端代价项并乘以权重放大系数
    obj += casadi::MX::mtimes({err_9d_N.T(), P_matrix, err_9d_N}) * terminal_cost_multiplier;
    opti_.minimize(obj); // 设置最小化目标函数

    // 配置Ipopt求解器参数
    casadi::Dict solver_opts;
    solver_opts["expand"] = true;         // 展开计算图，加速求导
    solver_opts["ipopt.print_level"] = 0; // 关闭Ipopt的打印输出
    solver_opts["print_time"] = 0;        // 关闭求解时间打印
    solver_opts["ipopt.tol"] = 1e-3;      // 求解器收敛容差
    solver_opts["ipopt.sb"] = "yes";      // 隐藏Ipopt的启动横幅
    opti_.solver("ipopt", solver_opts);   // 绑定Ipopt求解器

    ROS_INFO("Advanced NLP Engine Ready with Absolute State and SO(3) Metrics.");
}

// 求解器主函数：给定当前状态和参考轨迹，求解最优控制量和预测轨迹
bool QuadrotorNMPCAdvanced::solve(const Eigen::VectorXd &x_curr, const Eigen::MatrixXd &X_ref, const Eigen::MatrixXd &U_ref,
                                  Eigen::Vector4d &u_opt, Eigen::MatrixXd *x_pred)
{
    // 将当前状态转换为std::vector并设定为NLP的初始参数
    std::vector<double> x0_vec(x_curr.data(), x_curr.data() + x_curr.size());
    opti_.set_value(X0_param_, casadi::DM(x0_vec));

    // 构建并赋值参考状态矩阵参数
    casadi::DM Xref_dm = casadi::DM::zeros(14, N_ + 1);
    for (int i = 0; i < 14; ++i)
    {
        for (int j = 0; j <= N_; ++j)
        {
            Xref_dm(i, j) = X_ref(i, j);
        }
    }
    opti_.set_value(Xref_param_, Xref_dm);

    // 构建并赋值参考控制量矩阵参数
    casadi::DM Uref_dm = casadi::DM::zeros(4, N_);
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < N_; ++j)
        {
            Uref_dm(i, j) = U_ref(i, j);
        }
    }
    opti_.set_value(Uref_param_, Uref_dm);

    // 使用上一次的求解结果作为热启动初始值，加速收敛
    opti_.set_initial(X_var_, casadi::DM(x_warm_start_));
    opti_.set_initial(U_var_, casadi::DM(u_warm_start_));

    try
    {
        // 调用CasADi求解器求解
        casadi::OptiSol sol = opti_.solve();
        casadi::Matrix<double> U_res = sol.value(U_var_); // 提取控制输出
        casadi::Matrix<double> X_res = sol.value(X_var_); // 提取状态输出

        // 提取第0步的最优控制量作为当前实际控制指令
        u_opt(0) = static_cast<double>(U_res(0, 0));
        u_opt(1) = static_cast<double>(U_res(1, 0));
        u_opt(2) = static_cast<double>(U_res(2, 0));
        u_opt(3) = static_cast<double>(U_res(3, 0));

        // 如果需要，保存完整的预测状态轨迹
        if (x_pred != nullptr)
        {
            x_pred->resize(14, N_ + 1);
            for (int i = 0; i < 14; ++i)
            {
                for (int j = 0; j <= N_; ++j)
                {
                    (*x_pred)(i, j) = static_cast<double>(X_res(i, j));
                }
            }
        }

        // 更新热启动缓存向量
        for (int i = 0; i < 14 * (N_ + 1); ++i)
            x_warm_start_[i] = X_res.ptr()[i];
        for (int i = 0; i < 4 * N_; ++i)
            u_warm_start_[i] = U_res.ptr()[i];

        return true; // 求解成功
    }
    catch (std::exception &e)
    {
        // 捕获异常，求解失败
        ROS_ERROR("NMPC Advanced Solve Failed!");
        return false;
    }
}

// 构造函数：初始化ROS句柄、加载参数、初始化NLP求解器、设置通讯接口
AdvancedMPCNode::AdvancedMPCNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh), has_odom_(false), has_traj_(false)
{
    // 加载基础物理参数和预测时域参数
    pnh_.param("mass", mass_, 1.0);
    pnh_.param("gravity", gravity_, 9.81);
    pnh_.param("dt", dt_, 0.05);
    pnh_.param("N", N_, 20);

    // 加载控制限幅参数
    pnh_.param("max_thrust", max_thrust_, 20.0);
    pnh_.param("min_thrust", min_thrust_, 1.0);
    pnh_.param("max_bodyrate_x", max_rate_x_, 3.0);
    pnh_.param("max_bodyrate_y", max_rate_y_, 3.0);
    pnh_.param("max_bodyrate_z", max_rate_z_, 1.5);
    // 加载空气阻力及延迟补偿开关
    pnh_.param("use_aero_drag_and_delay", use_aero_drag_and_delay_, true);

    // 提取状态目标权重Q和输入代价权重R
    std::vector<double> Q_diag, R_diag;
    if (!pnh_.getParam("Q_diag_advanced", Q_diag))
    {
        ROS_WARN("[AdvancedMPC] Missing 'Q_diag_advanced', using default 9D weights.");
        Q_diag = {80.0, 80.0, 120.0, 10.0, 10.0, 15.0, 5.0, 5.0, 3.0};
    }
    if (!pnh_.getParam("R_diag", R_diag))
    {
        ROS_WARN("[AdvancedMPC] Missing 'R_diag', using default 4D weights.");
        R_diag = {0.1, 0.5, 0.5, 0.5};
    }

    // 权重维度检查与保护
    if (Q_diag.size() != 9)
    {
        ROS_ERROR("[AdvancedMPC] Q_diag_advanced must be 9D, got %zu. Reset to defaults.", Q_diag.size());
        Q_diag = {80.0, 80.0, 120.0, 10.0, 10.0, 15.0, 5.0, 5.0, 3.0};
    }
    if (R_diag.size() != 4)
    {
        ROS_ERROR("[AdvancedMPC] R_diag must be 4D, got %zu. Reset to defaults.", R_diag.size());
        R_diag = {0.1, 0.5, 0.5, 0.5};
    }

    // 终端代价放大系数
    double terminal_cost_multiplier;
    pnh_.param("terminal_cost_multiplier", terminal_cost_multiplier, 10.0);

    // 阻力系数和一阶延迟时间常数
    double drag_x, drag_y, drag_z, tau_thrust, tau_rate;
    pnh_.param("drag_x", drag_x, 0.1);
    pnh_.param("drag_y", drag_y, 0.1);
    pnh_.param("drag_z", drag_z, 0.1);
    pnh_.param("tau_thrust", tau_thrust, 0.05);
    pnh_.param("tau_rate", tau_rate, 0.02);

    // 实例化高级NMPC核心类
    nmpc_.reset(new QuadrotorNMPCAdvanced(mass_, gravity_, dt_, N_, drag_x, drag_y, drag_z, tau_thrust, tau_rate, use_aero_drag_and_delay_));
    // 配置并初始化NLP优化问题
    nmpc_->setupNLP(Q_diag, R_diag, min_thrust_, max_thrust_, max_rate_x_, max_rate_y_, max_rate_z_, terminal_cost_multiplier);

    // 配置ROS订阅者和发布者
    odom_sub_ = nh_.subscribe("/Odometry", 1, &AdvancedMPCNode::odomCb, this);
    traj_sub_ = nh_.subscribe("/trajectory", 1, &AdvancedMPCNode::trajCb, this);
    ctrl_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    pred_path_pub_ = nh_.advertise<nav_msgs::Path>("/nmpc_predicted_path", 1);

    // 以参数dt创建周期性定时器，执行MPC主循环
    timer_ = nh_.createTimer(ros::Duration(dt_), &AdvancedMPCNode::loop, this);

    // 初始化系统当前状态向量
    current_x_ = Eigen::VectorXd::Zero(14);
    current_x_(6) = 1.0;               // 初始化姿态四元数 qw
    current_x_(10) = mass_ * gravity_; // 初始化实际推力为悬停状态
    // 初始化上一次下发的控制指令
    last_cmd_ = Eigen::Vector4d(mass_ * gravity_, 0.0, 0.0, 0.0);
}

// 里程计回调函数：更新当前无人机状态
void AdvancedMPCNode::odomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_); // 线程锁，防止数据竞争

    // 更新三维位置
    current_x_[0] = msg->pose.pose.position.x;
    current_x_[1] = msg->pose.pose.position.y;
    current_x_[2] = msg->pose.pose.position.z;

    // 更新三维线速度
    current_x_[3] = msg->twist.twist.linear.x;
    current_x_[4] = msg->twist.twist.linear.y;
    current_x_[5] = msg->twist.twist.linear.z;

    // 更新姿态四元数
    current_x_[6] = msg->pose.pose.orientation.w;
    current_x_[7] = msg->pose.pose.orientation.x;
    current_x_[8] = msg->pose.pose.orientation.y;
    current_x_[9] = msg->pose.pose.orientation.z;

    // 状态机中附加“上一时刻下发的指令”用于延迟模型更新计算
    current_x_[10] = last_cmd_(0); // 推力
    current_x_[11] = last_cmd_(1); // 滚转角速度
    current_x_[12] = last_cmd_(2); // 俯仰角速度
    current_x_[13] = last_cmd_(3); // 偏航角速度

    has_odom_ = true; // 标记里程计信息就绪
}

// ============================================
// 【解析全状态轨迹】: 接收规划器下发的轨迹包含位置速度和加速度
// ============================================
void AdvancedMPCNode::trajCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(traj_mutex_); // 线程锁

    if (msg->points.empty()) // 如果轨迹空则忽略
        return;

    // 初始化参考状态矩阵和控制矩阵
    ref_traj_ = Eigen::MatrixXd::Zero(14, N_ + 1);
    u_ref_traj_ = Eigen::MatrixXd::Zero(4, N_);

    int traj_size = msg->points.size();

    // 1. 提取所有参考状态
    for (int i = 0; i <= N_; ++i)
    {
        // 用末端点填补超出部分的轨迹预测点
        int idx = std::min(i, traj_size - 1);
        auto &point = msg->points[idx];

        // 解析参考位置
        ref_traj_(0, i) = point.transforms[0].translation.x;
        ref_traj_(1, i) = point.transforms[0].translation.y;
        ref_traj_(2, i) = point.transforms[0].translation.z;

        // 解析参考速度
        ref_traj_(3, i) = point.velocities[0].linear.x;
        ref_traj_(4, i) = point.velocities[0].linear.y;
        ref_traj_(5, i) = point.velocities[0].linear.z;

        // 解析参考加速度
        Eigen::Vector3d a(point.accelerations[0].linear.x, point.accelerations[0].linear.y, point.accelerations[0].linear.z);

        // 解析姿态四元数 (由规划器给定Yaw + 加速度微分平坦生成)
        ref_traj_(6, i) = point.transforms[0].rotation.w;
        ref_traj_(7, i) = point.transforms[0].rotation.x;
        ref_traj_(8, i) = point.transforms[0].rotation.y;
        ref_traj_(9, i) = point.transforms[0].rotation.z;

        // 根据牛顿动力学计算参考推力：推力向量 = 期望加速度方向 + 重力反抗方向系数
        Eigen::Vector3d a_total = a + Eigen::Vector3d(0, 0, gravity_);
        double thrust = mass_ * a_total.norm();

        // 填充控制参考值（假设前馈角速度为0）
        if (i < N_)
        {
            u_ref_traj_(0, i) = thrust; // 期望推力
            u_ref_traj_(1, i) = 0.0;    // 期望滚转角速度
            u_ref_traj_(2, i) = 0.0;    // 期望俯仰角速度
            u_ref_traj_(3, i) = 0.0;    // 期望偏航角速度
        }
    }

    has_traj_ = true; // 标记轨迹信息就绪
}

// 控制器主循环：定期求解NMPC问题并发布控制指令
void AdvancedMPCNode::loop(const ros::TimerEvent &e)
{
    Eigen::VectorXd x_curr;
    Eigen::MatrixXd X_ref, U_ref;

    {
        // 加锁提取当前状态和参考轨迹避免与回调发生脏读
        std::lock(odom_mutex_, traj_mutex_); // 同时加锁防止死锁发生
        std::lock_guard<std::mutex> o_lock(odom_mutex_, std::adopt_lock);
        std::lock_guard<std::mutex> t_lock(traj_mutex_, std::adopt_lock);
        // 如果里程计或轨迹中有一方未就绪，则跳过本次运算
        if (!has_odom_ || !has_traj_)
            return;
        x_curr = current_x_;
        X_ref = ref_traj_;
        U_ref = u_ref_traj_;
    }

    Eigen::Vector4d u_opt;  // 求解输出：最优控制量
    Eigen::MatrixXd x_pred; // 求解输出：预测状态轨迹

    // 调用高级NMPC进行在线优化求解
    if (nmpc_->solve(x_curr, X_ref, U_ref, u_opt, &x_pred))
    {
        // === 预测轨迹可视化发布 ===
        nav_msgs::Path predicted_path;
        predicted_path.header.stamp = ros::Time::now();
        predicted_path.header.frame_id = "world";
        for (int i = 0; i < x_pred.cols(); ++i)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = predicted_path.header;
            pose.pose.position.x = x_pred(0, i);
            pose.pose.position.y = x_pred(1, i);
            pose.pose.position.z = x_pred(2, i);
            pose.pose.orientation.w = x_pred(6, i);
            pose.pose.orientation.x = x_pred(7, i);
            pose.pose.orientation.y = x_pred(8, i);
            pose.pose.orientation.z = x_pred(9, i);
            predicted_path.poses.push_back(pose);
        }
        pred_path_pub_.publish(predicted_path); // 将预测轨迹发布至RViz

        // === 构造控制指令 ===
        mavros_msgs::AttitudeTarget ctrl_msg;
        ctrl_msg.header.stamp = ros::Time::now();
        ctrl_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE; // 忽略绝对姿态，仅使用角速度控制模式

        // 推力按PX4归一化限幅，而不是强硬写死 / 20.0
        double cur_thrust = std::max(min_thrust_, std::min(max_thrust_, u_opt(0))); // 推力死区死角限幅

        // 防除零保护
        double norm_thrust = 0.0;
        if (max_thrust_ - min_thrust_ > 1e-5)
        {
            norm_thrust = (cur_thrust - min_thrust_) / (max_thrust_ - min_thrust_); // 推力标幺化处理映射至 0~1
        }

        norm_thrust = std::max(0.0, std::min(1.0, norm_thrust)); // 截断保障安全不产生溢出

        ctrl_msg.thrust = norm_thrust;   // 下发归一化推力
        ctrl_msg.body_rate.x = u_opt(1); // 期望横滚角速度
        ctrl_msg.body_rate.y = u_opt(2); // 期望俯仰角速度
        ctrl_msg.body_rate.z = u_opt(3); // 期望偏航角速度
        ctrl_pub_.publish(ctrl_msg);     // 通过MAVROS下发控制指令

        // 缓存本次控制指令作为下次实时优化的延迟输入模型推算
        last_cmd_ = Eigen::Vector4d(cur_thrust, u_opt(1), u_opt(2), u_opt(3));
    }
    else
    {
        ROS_WARN_THROTTLE(1.0, "[AdvancedMPC] NMPC Solve Failed! Publishing hover fallback command.");
        mavros_msgs::AttitudeTarget ctrl_msg;
        ctrl_msg.header.stamp = ros::Time::now();
        ctrl_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE; // 忽略绝对姿态，仅使用角速度控制模式

        // 悬停推力兜底保护
        double hover_thrust = mass_ * gravity_;
        double cur_thrust = std::max(min_thrust_, std::min(max_thrust_, hover_thrust));
        double norm_thrust = 0.0;
        if (max_thrust_ - min_thrust_ > 1e-5)
        {
            norm_thrust = (cur_thrust - min_thrust_) / (max_thrust_ - min_thrust_);
        }

        ctrl_msg.thrust = std::max(0.0, std::min(1.0, norm_thrust));
        ctrl_msg.body_rate.x = 0.0;
        ctrl_msg.body_rate.y = 0.0;
        ctrl_msg.body_rate.z = 0.0;
        ctrl_pub_.publish(ctrl_msg);

        last_cmd_ = Eigen::Vector4d(cur_thrust, 0.0, 0.0, 0.0);
    }
}
