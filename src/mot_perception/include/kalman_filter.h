#pragma once
#include <Eigen/Dense>
#include <deque>
#include <vector>

/**
 * @brief 障碍物追踪状态枚举
 * TENTATIVE: 试探态(刚发现) | CONFIRMED: 确认态(稳定追踪) | LOST: 丢失态(被遮挡或离开视野)
 */
enum TrackState
{
    TENTATIVE,
    CONFIRMED,
    LOST
};

/**
 * @brief 线性卡尔曼滤波器类
 * 利用恒速模型(CV)对动态障碍物的3D位置与速度进行预测(Predict)和量测更新(Update)
 */
class KalmanFilter
{
public:
    KalmanFilter(int id, const Eigen::VectorXd &init_state, double p_factor = 10.0, double q_factor = 0.1, double r_factor = 0.1);
    void predict(double dt);
    void update(const Eigen::VectorXd &z, int min_hits);

    Eigen::VectorXd getState() const;
    int getId() const;
    int getAge() const;
    int getHits() const;
    int getTimeSinceUpdate() const;
    TrackState getTrackState() const;

    // --- 方案二架构新增接口：支持深度时序分析与多模态波形存储 ---
    void recordHistory(const Eigen::VectorXd &state);
    std::vector<float> getHistoricalFeatures(int hist_len, int feat_dim) const;
    void setPredictedCurve(const std::vector<Eigen::Matrix<double, 6, 1>> &curve);
    std::vector<Eigen::Matrix<double, 6, 1>> getPredictedCurve() const;
    void overridePositionByDeepModel(const Eigen::Vector3d &pos);

private:
    int id_;
    int state_dim_;
    int meas_dim_;

    // 历史特征库与未来推演库，辅助 Transformer 输入输出
    std::deque<Eigen::VectorXd> history_states_;
    std::vector<Eigen::Matrix<double, 6, 1>> predicted_curve_;

    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;

    int age_;
    int hits_;
    int time_since_update_;
    TrackState track_state_;
};
