#include "../include/kalman_filter.h"
#include <iostream>
using namespace Eigen;
using namespace std;
// 构造函数：初始化ID并设置目标的初始状态向量
KalmanFilter::KalmanFilter(int id, const Eigen::VectorXd &init_state, double p_factor, double q_factor, double r_factor) : id_(id)
{
    state_dim_ = 9; // 状态向量维度为9：位置(x,y,z)，速度(vx,vy,vz)，尺寸(w,h,d)
    meas_dim_ = 6;  // 观测向量维度为6：位置(x,y,z)，尺寸(w,h,d)

    x_ = init_state; // 初始化状态向量

    P_ = MatrixXd::Identity(state_dim_, state_dim_) * p_factor; // 初始化协方差矩阵，赋予较大的初始不确定性

    F_ = MatrixXd::Identity(state_dim_, state_dim_); // 初始化状态转移矩阵

    H_ = MatrixXd::Zero(meas_dim_, state_dim_); // 初始化观测矩阵
    H_(0, 0) = 1.0;                             // 观测位置 X
    H_(1, 1) = 1.0;                             // 观测位置 Y
    H_(2, 2) = 1.0;                             // 观测位置 Z
    H_(3, 6) = 1.0;                             // 观测尺寸 W
    H_(4, 7) = 1.0;                             // 观测尺寸 H
    H_(5, 8) = 1.0;                             // 观测尺寸 D

    Q_ = MatrixXd::Identity(state_dim_, state_dim_) * q_factor; // 过程噪声协方差矩阵

    R_ = MatrixXd::Identity(meas_dim_, meas_dim_) * r_factor; // 测量噪声协方差矩阵

    age_ = 1;                 // 目标存在时长 (周期数)
    hits_ = 1;                // 目标成功关联的次数
    time_since_update_ = 0;   // 自上次更新以来的周期数
    track_state_ = TENTATIVE; // 新目标的初始状态为试探状态
}

// 根据时间步长 dt 进行状态预测
void KalmanFilter::predict(double dt)
{
    // 匀速运动模型的位置预测 (p = p + v * dt)
    F_(0, 3) = dt;
    F_(1, 4) = dt;
    F_(2, 5) = dt;

    x_ = F_ * x_;                       // 预测下一时刻状态
    P_ = F_ * P_ * F_.transpose() + Q_; // 更新协方差矩阵

    time_since_update_++; // 未更新时间递增
    age_++;               // 目标生命周期递增

    // 如果目标处于确认状态且在当前周期未被观测到，则将其状态设为丢失
    if (time_since_update_ > 0 && track_state_ == CONFIRMED)
    {
        track_state_ = LOST;
    }
}

// 根据新的测量值更新目标状态
void KalmanFilter::update(const Eigen::VectorXd &z, int min_hits)
{
    time_since_update_ = 0; // 成功匹配，重置未更新时间
    hits_++;                // 匹配次数递增

    // 状态机转换
    if (track_state_ == TENTATIVE && hits_ >= min_hits)
    {
        track_state_ = CONFIRMED; // 匹配达到阈值，转为确认状态
    }
    else if (track_state_ == LOST)
    {
        track_state_ = CONFIRMED; // 丢失目标重新出现，恢复确认状态
    }

    VectorXd y = z - H_ * x_;                       // 计算测量残差 (Innovation)
    MatrixXd S = H_ * P_ * H_.transpose() + R_;     // 计算残差协方差
    MatrixXd K = P_ * H_.transpose() * S.inverse(); // 计算卡尔曼增益

    x_ = x_ + K * y; // 更新系统状态
    MatrixXd I = MatrixXd::Identity(state_dim_, state_dim_);
    P_ = (I - K * H_) * P_; // 更新误差协方差矩阵
}

Eigen::VectorXd KalmanFilter::getState() const { return x_; } // 获取当前状态向量

int KalmanFilter::getId() const { return id_; } // 获取目标 ID

int KalmanFilter::getAge() const { return age_; } // 获取目标生命周期

int KalmanFilter::getHits() const { return hits_; } // 获取目标匹配次数

int KalmanFilter::getTimeSinceUpdate() const { return time_since_update_; } // 获取目标未更新时间

TrackState KalmanFilter::getTrackState() const { return track_state_; } // 获取目标当前状态

// 追踪状态

// --- 深度学习特征与时序波形维护接口 ---

void KalmanFilter::recordHistory(const Eigen::VectorXd &state)
{
    history_states_.push_back(state);
    if (history_states_.size() > 50)
    { // 限制最大缓存数量以防内存泄漏
        history_states_.pop_front();
    }
}

std::vector<float> KalmanFilter::getHistoricalFeatures(int hist_len, int feat_dim) const
{
    std::vector<float> features(hist_len * feat_dim, 0.0f);
    int start_idx = 0;
    int available_history = history_states_.size();

    // 若历史不足以填满要求长度，则复用最早的一帧 (Zero-Padding的一种)
    for (int i = 0; i < hist_len; ++i)
    {
        int hist_idx = available_history - hist_len + i;
        Eigen::VectorXd state_to_use;
        if (hist_idx < 0)
        {
            state_to_use = available_history > 0 ? history_states_.front() : x_;
        }
        else
        {
            state_to_use = history_states_[hist_idx];
        }

        // 假设 feature 依次填充 x, y, z, vx, vy, vz
        for (int j = 0; j < std::min(feat_dim, (int)state_to_use.size()); ++j)
        {
            features[i * feat_dim + j] = state_to_use(j);
        }
    }
    return features;
}

void KalmanFilter::setPredictedCurve(const std::vector<Eigen::Matrix<double, 6, 1>> &curve)
{
    predicted_curve_ = curve;
}

std::vector<Eigen::Matrix<double, 6, 1>> KalmanFilter::getPredictedCurve() const
{
    return predicted_curve_;
}

void KalmanFilter::overridePositionByDeepModel(const Eigen::Vector3d &pos)
{
    x_(0) = pos(0);
    x_(1) = pos(1);
    x_(2) = pos(2);
    // 可选：利用连续2次override的时间差强行算速度。这里仅做直接坐标覆盖，保持卡尔曼误差依然生效
}
