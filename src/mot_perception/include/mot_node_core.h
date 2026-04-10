#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>

#include "kalman_filter.h"
#include "hungarian_algorithm.h"
#include "bounding_box.h"
#include <NvInfer.h>
#include <cuda_runtime_api.h>

typedef pcl::PointXYZI PointType;

class MOTLogger : public nvinfer1::ILogger
{
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kWARNING)
            ROS_WARN("[TRT_MOT] %s", msg);
    }
};

/**
 * @brief 多目标追踪(MOT)雷达感知节点核心类
 * 完成激光雷达点云的地面过滤、欧式聚类识别、并利用卡尔曼滤波与匈牙利算法维护生命周期
 */
class MOTNode
{
public:
    MOTNode(ros::NodeHandle &nh);
    ~MOTNode();

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_lidar_;
    ros::Publisher pub_markers_;
    ros::Publisher pub_static_cloud_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string lidar_topic_;
    std::string world_frame_;
    double leaf_size_;
    double distance_threshold_;
    double eps_angle_;
    double tf_timeout_;
    double z_min_;
    double z_max_;
    double q_factor_;
    double r_factor_;
    double p_factor_;
    double dt_config_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double max_distance_;
    int max_age_;
    int min_hits_;
    int lost_threshold_;

    // --- 新增参数化逻辑配置 ---
    bool use_transformer_;         // 是否启用 Transformer 模型进行预测
    double dynamic_bbx_expansion_; // 判断动态点云包围盒向外膨胀余量
    double size_penalty_weight_;   // 匈牙利匹配时长宽高差异对代价的惩罚权重
    double angle_penalty_weight_;  // 匈牙利匹配时速度方向不一致造成的附加惩罚权重
    double vel_heading_thresh_;    // 启用追踪器速度朝向角度惩罚的最低速度阈值

    double last_time_;
    int current_id_;
    std::vector<KalmanFilter> trackers_;

    // --- TensorRT 深度学习追踪预测融合组件 ---
    MOTLogger gLogger_;
    nvinfer1::IRuntime *runtime_{nullptr};
    nvinfer1::ICudaEngine *engine_{nullptr};
    nvinfer1::IExecutionContext *context_{nullptr};
    void *buffers_[2] = {nullptr, nullptr};
    int input_index_{-1};
    int output_index_{-1};

    int HIST_LEN;
    int FUT_LEN;
    int FEAT_DIM;
    int OUT_DIM;
    std::string engine_path_;
    ros::Publisher pub_pred_trajs_;

    pcl::PointCloud<PointType>::Ptr preprocessCloud(const pcl::PointCloud<PointType>::Ptr &cloud);
    std::vector<BoundingBox> getDetections(const pcl::PointCloud<PointType>::Ptr &cloud);
    void trackObjects(const std::vector<BoundingBox> &detections, double dt, const ros::Time &stamp);
    void publishMarkers(const ros::Time &stamp);
};
