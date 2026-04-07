#pragma once
#include <Eigen/Dense>

/**
 * @brief 三维轴向包围盒(AABB)特征结构体
 * 存储经过聚类算法提取出来的障碍物体积中心点和长宽高尺寸
 */
struct BoundingBox
{
    Eigen::Vector3f position;
    Eigen::Vector3f dimensions;
    Eigen::VectorXd toMeasurement() const;
};
