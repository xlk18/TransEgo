#pragma once
#include <vector>

/**
 * @brief 匈牙利二分图最大匹配算法
 * 用于将当前帧检测到的BoundingBox与上一帧保存的KalmanFilter轨迹库进行最优代价匹配
 */
class HungarianAlgorithm
{
public:
    HungarianAlgorithm();
    double Solve(std::vector<std::vector<double>> &DistMatrix, std::vector<int> &Assignment);

private:
    void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);
};
