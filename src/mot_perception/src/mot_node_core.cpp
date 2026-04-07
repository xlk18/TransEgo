#include "../include/mot_node_core.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <iostream>
using namespace Eigen;
using namespace std;
// 构造函数：初始化 ROS 节点、加载参数、配置发布者和订阅者
MOTNode::MOTNode(ros::NodeHandle &nh) : input_index_(-1), output_index_(-1), nh_(nh), current_id_(0), last_time_(0.0), runtime_(nullptr), engine_(nullptr), context_(nullptr)
{
    buffers_[0] = nullptr;
    buffers_[1] = nullptr;
    ros::NodeHandle pnh("~"); // 创建私有节点句柄，用于读取当前节点的配置参数

    // --- 第一部分：读取 ROS 参数服务器配置 ---
    // 强制依赖参数检索，若未找到则抛出 FATAL 异常并终止节点运行
    if (!pnh.getParam("lidar_topic", lidar_topic_))
    {
        ROS_FATAL("Failed to load parameter 'lidar_topic'");
        ros::shutdown();
        return;
    }
    if (!pnh.getParam("world_frame", world_frame_))
    {
        ROS_FATAL("Failed to load parameter 'world_frame'");
        ros::shutdown();
        return;
    }

    pnh.param("voxel_size", leaf_size_, 0.1);

    // 新增从 mot_params.yaml 读取的参数
    pnh.param("z_min", z_min_, -1.0);
    pnh.param("z_max", z_max_, 2.0);
    pnh.param("distance_threshold", distance_threshold_, 0.1);
    pnh.param("eps_angle", eps_angle_, 0.261799);
    pnh.param("tf_timeout", tf_timeout_, 0.1);
    pnh.param("dt", dt_config_, 0.1);
    pnh.param("p_factor", p_factor_, 10.0);
    pnh.param("q_factor", q_factor_, 0.1);
    pnh.param("r_factor", r_factor_, 0.1);

    pnh.param("cluster_tolerance", cluster_tolerance_, 0.5);
    pnh.param("min_cluster_size", min_cluster_size_, 10);
    pnh.param("max_cluster_size", max_cluster_size_, 50000);
    pnh.param("max_distance", max_distance_, 1.5);
    pnh.param("max_age", max_age_, 5);
    pnh.param("min_hits", min_hits_, 3);

    // 神经网络有关的特征维度与模型路径参数加载（带有默认值保障）
    pnh.param<bool>("use_transformer", use_transformer_, true);
    pnh.param<int>("hist_len", HIST_LEN, 10);
    pnh.param<int>("fut_len", FUT_LEN, 20);
    pnh.param<int>("feat_dim", FEAT_DIM, 6);
    pnh.param<int>("out_dim", OUT_DIM, 3);
    pnh.param<std::string>("engine_path", engine_path_, "/home/yyf/TransEgo/traj_prediction/models/transformer_model.engine");

    // 加载数据关联代价权重以及动态障碍物判定相关的超参数估值
    pnh.param<double>("dynamic_bbx_expansion", dynamic_bbx_expansion_, 0.3);
    pnh.param<double>("size_penalty_weight", size_penalty_weight_, 1.5);
    pnh.param<double>("angle_penalty_weight", angle_penalty_weight_, 2.0);
    pnh.param<double>("vel_heading_thresh", vel_heading_thresh_, 0.1);

    // --- 第二部分：初始化 TensorRT 推理引擎 ---
    if (use_transformer_)
    {
        std::ifstream file(engine_path_, std::ios::binary); // 以二进制方式尝试读取模型文件
        if (file.good())
        {
            // 游标定位至文件末尾，获取当前网络引擎文件的体积大小
            file.seekg(0, std::ios::end);
            size_t size = file.tellg();
            file.seekg(0, std::ios::beg); // 将游标回拨至起始位置

            // 分配空间并将模型数据流读入内存
            std::vector<char> trtModelStream(size);
            file.read(trtModelStream.data(), size);
            file.close();

            // 依次构建运行时环境、反序列化引擎并创建实际推理执行的上下文
            runtime_ = nvinfer1::createInferRuntime(gLogger_);
            engine_ = runtime_->deserializeCudaEngine(trtModelStream.data(), size);
            context_ = engine_->createExecutionContext();

            // 提取模型对应输入层、输出层张量的绑定索引引用号
            input_index_ = engine_->getBindingIndex("input_tensor");
            output_index_ = engine_->getBindingIndex("output_tensor");

            // 确保索引皆有效且未超出预设 buffers 数量的边界后，直接在 GPU 显存端开辟对应大小的缓冲内存以供读写
            if (input_index_ >= 0 && input_index_ < 2 && output_index_ >= 0 && output_index_ < 2)
            {
                cudaMalloc(&buffers_[input_index_], 1 * HIST_LEN * FEAT_DIM * sizeof(float));
                cudaMalloc(&buffers_[output_index_], 1 * FUT_LEN * OUT_DIM * sizeof(float));
                ROS_INFO("TensorRT Engine Loaded from %s", engine_path_.c_str());
            }
            else
            {
                ROS_ERROR("Failed to get binding index for tensorrt engine.");
            }
        }
        else
        {
            ROS_WARN("TensorRT engine file not found at %s. Transformer module disabled.", engine_path_.c_str());
            use_transformer_ = false; // 降级为单纯卡尔曼模式
        }
    }
    else
    {
        ROS_INFO("Transformer prediction module is DISABLED via config. Using pure Kalman Filter prediction.");
    }

    // --- 第三部分：配置 ROS 节点通信接口（订阅、发布、坐标变换树） ---
    // 订阅前置雷达输入的原始点云话题，触发 MOTNode 的主回调周期
    sub_lidar_ = nh_.subscribe(lidar_topic_, 1, &MOTNode::cloudCallback, this);

    // 注册发布可视化包，用于在 RViz 环境中渲染代表该追踪实体的 3D 边界框与文本编号
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("mot_markers", 1);

    // 注册发布深度学习推算的轨迹线，提供给下游模块（如局部运动规划 ego-planning 作为态势参考）
    pub_pred_trajs_ = pnh.advertise<visualization_msgs::MarkerArray>("/traj_prediction/predicted_trajectories", 10);

    // 注册静态发布器，过滤掉带有速度/活跃确认的动态物体后的清洗地图，确保建图不受拖影影响
    pub_static_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("static_cloud", 1);

    // 初始化 TF 坐标系变换查询监听器，以将雷达坐标系动态换算融入全局世界（map）系
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    ROS_INFO("MOTNode initialized. Listening to: %s", lidar_topic_.c_str());
}

// 析构函数：负责彻底释放类结束时的显存驻留及清退 TensorRT 引擎内存资源
MOTNode::~MOTNode()
{
    // 如果显存曾被成功分配，则归还释放输入/输出的纯粹 GPU 空间
    if (input_index_ >= 0 && input_index_ < 2 && output_index_ >= 0 && output_index_ < 2)
    {
        if (buffers_[input_index_])
            cudaFree(buffers_[input_index_]);
        if (buffers_[output_index_])
            cudaFree(buffers_[output_index_]);
    }
    // 按照被依赖的生成层级从上至下依次执行销毁，防止发生资源悬空与内存泄漏
    if (context_)
        context_->destroy();
    if (engine_)
        engine_->destroy();
    if (runtime_)
        runtime_->destroy();
}

// 接收雷达点云的回调函数：执行多目标追踪的主链路
void MOTNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // 获取当前时间戳
    double current_time = msg->header.stamp.toSec();
    if (last_time_ == 0.0) // 初始化：记录首帧时间
    {
        last_time_ = current_time;
        return;
    }

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>); // 声明 PCL 点云对象指针
    try
    {
        // 获取雷达坐标系到世界坐标系(map)的 TF 变换矩阵
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(world_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(tf_timeout_));

        // 将原始点云从雷达坐标系转换至世界坐标系
        pcl::PointCloud<PointType> cloud_raw;
        pcl::fromROSMsg(*msg, cloud_raw);
        Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform);
        pcl::transformPointCloud(cloud_raw, *cloud, transform_eigen);
    }
    catch (tf2::TransformException &ex) // 若 TF 变换查寻失败，打印警告并跳过当前帧处理
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    double dt = current_time - last_time_; // 计算两帧之间的时间步长，用于卡尔曼滤波器的时间更新
    last_time_ = current_time;             // 更新上一帧时间戳记录

    // 步骤一：点云预处理（体素滤波降采样、Z 轴高度裁剪、RANSAC 剔除地面点）
    pcl::PointCloud<PointType>::Ptr filtered_cloud = preprocessCloud(cloud);

    // 步骤二：目标聚类（利用欧氏聚类算法提取独立的 3D 目标边界框）
    std::vector<BoundingBox> detections = getDetections(filtered_cloud);

    // 步骤三：目标追踪（结合 Transformer 预测模型与卡尔曼滤波完成数据关联与状态更新）
    trackObjects(detections, dt, msg->header.stamp);

    // 步骤四：结果可视化（打包封装追踪物体的相关信息并发往 RViz 显示）
    publishMarkers(msg->header.stamp);

    pcl::PointCloud<PointType>::Ptr static_cloud(new pcl::PointCloud<PointType>); // 步骤五：动态分离与静态地图发布
    for (const auto &pt : cloud->points)                                          // 遍历每一束雷达点，判断其是否属于运动物体
    {
        bool is_dynamic = false;              // 默认为静态点标记
        for (const auto &tracker : trackers_) // 遍历当前活动的追踪器列表
        {
            if (tracker.getTrackState() != CONFIRMED) // 只从已稳定建立 (CONFIRMED) 的目标区域中分离动态点
                continue;

            VectorXd state = tracker.getState(); // 获取目标的跟踪状态矩阵
            double dx = pt.x - state(0);         // 沿 X 轴到目标中心的距离
            double dy = pt.y - state(1);         // 沿 Y 轴到目标中心的距离
            double dz = pt.z - state(2);         // 沿 Z 轴到目标中心的距离

            // 判断该点是否坠入目标的 3D 包围点内部 (添加 dynamic_bbx_expansion_ 余量以补偿模型边缘)
            if (std::abs(dx) < state(6) / 2.0 + dynamic_bbx_expansion_ &&
                std::abs(dy) < state(7) / 2.0 + dynamic_bbx_expansion_ &&
                std::abs(dz) < state(8) / 2.0 + dynamic_bbx_expansion_)
            {
                is_dynamic = true; // 该点属于移动物体范围
                break;
            }
        }

        if (!is_dynamic) // 若点位未被动态边界框所涵盖，判定为静态背景点并予以保留
        {
            static_cloud->points.push_back(pt);
        }
    }

    // 设置静态点云结构的维度等属性
    static_cloud->width = static_cloud->points.size();
    static_cloud->height = 1;

    sensor_msgs::PointCloud2 static_cloud_msg;         // 声明用于发布的点云消息
    pcl::toROSMsg(*static_cloud, static_cloud_msg);    // PCL 格式转回 ROS 消息格式
    static_cloud_msg.header.frame_id = world_frame_;   // 配置相应的坐标系
    static_cloud_msg.header.stamp = msg->header.stamp; // 配置发包时间戳
    pub_static_cloud_.publish(static_cloud_msg);       // 推送剥离出动态对象的静态点云，用于后续建图模块
}

// 执行点云降采样、高度截取与路面消杂的流程工具
pcl::PointCloud<PointType>::Ptr MOTNode::preprocessCloud(const pcl::PointCloud<PointType>::Ptr &cloud)
{
    pcl::PointCloud<PointType>::Ptr filtered1(new pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> vg; // 声明体素网格滤波器，用作高频处理下的数据量压缩
    vg.setInputCloud(cloud);
    vg.setLeafSize(static_cast<float>(leaf_size_), static_cast<float>(leaf_size_), static_cast<float>(leaf_size_)); // 配置单体素晶格大小
    vg.filter(*filtered1);

    pcl::PointCloud<PointType>::Ptr filtered2(new pcl::PointCloud<PointType>);
    pcl::PassThrough<PointType> pass; // 声明直通滤波器，实现空间裁剪
    pass.setInputCloud(filtered1);
    pass.setFilterFieldName("z");         // 调整过滤器的操作轴为 Z (垂直高度)
    pass.setFilterLimits(z_min_, z_max_); // 去除太低或过高 (-1.0m 至 5.0m 区间外) 的冗余噪声点云
    pass.filter(*filtered2);

    // 采用 RANSAC 算法执行地平面拟合并将其从主点云移除
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // 如果过滤后点云数量不足以拟合平面(至少需要3个点)，则直接返回原点云防崩溃
    if (filtered2->points.size() < 3)
    {
        return filtered2;
    }

    pcl::SACSegmentation<PointType> seg; // 声明分割器对象
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // 拟合带有垂直法线约束的水平平面，防止误吞垂直墙面或卡车侧面
    seg.setMethodType(pcl::SAC_RANSAC);                  // 选择的拟合优化方法即为 RANSAC
    seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));         // 设定法向量平行于 Z 轴朝上
    seg.setEpsAngle(eps_angle_);                         // 容忍与地平面的倾斜偏差角度 (即上下坡 15 度内也当路面处理)
    seg.setDistanceThreshold(distance_threshold_);       // 定义判断误差的距离阈值：即到估计平面距离小于该值的都归为地面点
    seg.setInputCloud(filtered2);
    seg.segment(*inliers, *coefficients); // 对输入点集执行平面的内点识别并生成索引组

    pcl::PointCloud<PointType>::Ptr no_ground(new pcl::PointCloud<PointType>); // 声明不带有地面的处理后点云缓存
    pcl::ExtractIndices<PointType> extract;                                    // 实例化基于序号切片的索引提取工具
    extract.setInputCloud(filtered2);                                          // 装载已经过下采样及轴向裁剪的点云
    extract.setIndices(inliers);                                               // 输入被分割器标定为地面群的序号
    extract.setNegative(true);                                                 // 指令模式翻转，即剔除地面索引序号对应的点阵
    extract.filter(*no_ground);                                                // 析出分离后的场景物块点云堆

    return no_ground; // 回传无地平面的高可用场景点云供追踪级使用
}

// 解析无底面杂音的点云，开展欧式区域聚类，回传所有成型物理目标的 3D 包围盒集合
std::vector<BoundingBox> MOTNode::getDetections(const pcl::PointCloud<PointType>::Ptr &cloud)
{
    std::vector<BoundingBox> detections; // 结构序列：用来归纳此帧的识别检测框
    if (cloud->empty())                  // 安全保护：拦截空点云造成的处理崩溃
        return detections;

    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>); // 实例化 Kd-Tree，提供点间的就近快检支撑
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices; // 用于按组归置聚落点的总群编号清单
    pcl::EuclideanClusterExtraction<PointType> ec;  // 构造 PCL 的欧氏聚类分析对象
    ec.setClusterTolerance(cluster_tolerance_);     // 分隔参数 - 点与点之间最长的容差间距(超过此数算拆离为二)
    ec.setMinClusterSize(min_cluster_size_);        // 对象体积参数 - 极小点群舍弃 (过滤飘散孤立的噪点)
    ec.setMaxClusterSize(max_cluster_size_);        // 对象体积参数 - 极大点群阻断 (防止全图连作一片墙)
    ec.setSearchMethod(tree);                       // 指定寻近核心为 Kd-Tree
    ec.setInputCloud(cloud);                        // 输入欲分类的点集
    ec.extract(cluster_indices);                    // 全面执行分隔分析，将各自点串索引抛入对应族类数组

    // 并行计算每簇独立群点的几何中心点乃至立体的 AABB 轴向包围盒结构
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cluster->points.push_back(cloud->points[*pit]); // 从原大散点库依照分类号集齐构建该一实体的实存积点
        }
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        PointType min_pt, max_pt;                   // 容纳 3 维极致边界信息的最小/最大对角端点
        pcl::getMinMax3D(*cluster, min_pt, max_pt); // PCL 函数快捷量出所及区域的轴向范围边界

        BoundingBox box;                                                                                                   // 创建一个目标检测包围框实例
        box.position = Vector3f((min_pt.x + max_pt.x) / 2.0f, (min_pt.y + max_pt.y) / 2.0f, (min_pt.z + max_pt.z) / 2.0f); // 定心：将对角距离的各道中值作为箱体原心
        box.dimensions = Vector3f(max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z);                          // 定径：求作各面差值以取得长、宽、高的尺度度量
        detections.push_back(box);                                                                                         // 完成此单体的构造并插入当前周期的归整队标中
    }

    return detections; // 将整合出的所有独立检测箱块集体反馈给后续节点
}

// 获取观测数据与已有追踪目标的关联，并执行追踪器预测与更新
void MOTNode::trackObjects(const std::vector<BoundingBox> &detections, double dt, const ros::Time &stamp)
{
    // =========================================================================
    // --- 第一阶段：追踪目标状态的前向预测（卡尔曼 + 深度学习大模型方案） ---
    // =========================================================================
    visualization_msgs::MarkerArray pred_markers;

    for (auto &tracker : trackers_)
    {
        // 1. 卡尔曼滤波器执行线性状态预测
        tracker.predict(dt);

        // 2. Transformer 轨迹序列预测 (当开关启用时)
        if (use_transformer_ && (tracker.getTrackState() == CONFIRMED || tracker.getTrackState() == LOST))
        {
            // 记录当前状态，为模型提供历史序列信息
            tracker.recordHistory(tracker.getState());

            // --- 执行 TensorRT 推理任务 ---
            std::vector<float> input_data = tracker.getHistoricalFeatures(HIST_LEN, FEAT_DIM);
            std::vector<Eigen::Matrix<double, 6, 1>> predicted_curve;

            if (context_ && input_index_ >= 0 && input_index_ < 2 && output_index_ >= 0 && output_index_ < 2)
            {
                // 将输入特征数据从 CPU 拷贝至 GPU 显存 (HostToDevice)
                cudaMemcpy(buffers_[input_index_], input_data.data(), input_data.size() * sizeof(float), cudaMemcpyHostToDevice);
                // 调用推理引擎，生成未来预测序列
                context_->executeV2(buffers_);

                std::vector<float> output_data(FUT_LEN * OUT_DIM, 0.0f);
                // 将预测结果从 GPU 取回至 CPU 内存 (DeviceToHost)
                cudaMemcpy(output_data.data(), buffers_[output_index_], output_data.size() * sizeof(float), cudaMemcpyDeviceToHost);

                // 解析模型输出的未来预测点云轨迹与方差
                for (int f = 0; f < FUT_LEN; ++f)
                {
                    Eigen::Matrix<double, 6, 1> pt;
                    pt << output_data[f * OUT_DIM], output_data[f * OUT_DIM + 1], output_data[f * OUT_DIM + 2],
                        output_data[f * OUT_DIM + 3], output_data[f * OUT_DIM + 4], output_data[f * OUT_DIM + 5];
                    predicted_curve.push_back(pt);
                }
            }

            // 将预测轨迹保存在追踪器中，供目标遮挡丢失情况下的数据关联使用
            tracker.setPredictedCurve(predicted_curve);

            // 丢失目标状态兜底机制：使用模型预测的首个未来点修正卡尔曼滤波器的推测坐标
            if (tracker.getTrackState() == LOST && !predicted_curve.empty())
            {
                tracker.overridePositionByDeepModel(predicted_curve[0].head<3>());
            }

            // 组装用于轨迹可视化的 Marker 信息
            if (!predicted_curve.empty())
            {
                visualization_msgs::Marker marker;
                marker.header.frame_id = world_frame_;
                marker.header.stamp = stamp; // 修复时间戳脱节问题
                marker.ns = "prediction";
                marker.id = tracker.getId();
                marker.type = visualization_msgs::Marker::SPHERE_LIST;
                marker.action = visualization_msgs::Marker::ADD;
                marker.scale.x = 0.2;
                marker.scale.y = 0.2;
                marker.scale.z = 0.2;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.5;
                for (const auto &pt : predicted_curve)
                {
                    geometry_msgs::Point p;
                    std_msgs::ColorRGBA c;

                    p.x = pt[0];
                    p.y = pt[1];
                    p.z = pt[2];
                    marker.points.push_back(p);

                    // 将预估的方差/标准差借用颜色通道传递给局导模块作为避障安全半径膨胀
                    c.r = pt[3];
                    c.g = pt[4];
                    c.b = pt[5];
                    c.a = 1.0;
                    marker.colors.push_back(c);
                }
                pred_markers.markers.push_back(marker);
            }
        }
    }
    pub_pred_trajs_.publish(pred_markers); // 发布所有轨迹预测结果

    // --- 第二阶段：冷启动/无历史目标的初始化处理 ---
    if (trackers_.empty())
    {
        for (const auto &det : detections)
        {
            VectorXd init_state = VectorXd::Zero(9); // 初始化状态向量
            init_state.head(3) = det.position.cast<double>();
            init_state.tail(3) = det.dimensions.cast<double>();
            trackers_.push_back(KalmanFilter(current_id_++, init_state, p_factor_, q_factor_, r_factor_)); // 追加到追踪序列库
        }
        return;
    }

    std::vector<std::vector<double>> cost_matrix; // 声明代价矩阵：[追踪器数量 x 当前检测框数量]
    if (!trackers_.empty() && !detections.empty())
    {
        // 初始化代价矩阵
        cost_matrix.resize(trackers_.size(), std::vector<double>(detections.size(), 0.0));
        for (size_t t = 0; t < trackers_.size(); ++t) // 遍历所有当前正在追踪的目标
        {
            for (size_t d = 0; d < detections.size(); ++d) // 遍历本帧捕获的所有检测框
            {
                VectorXd state = trackers_[t].getState(); // 提取追踪器的当前预测状态
                Vector3d pos_tracker(state(0), state(1), state(2));
                Vector3d vel_tracker(state(3), state(4), state(5));
                Vector3d dim_tracker(state(6), state(7), state(8));

                Vector3d pos_det = detections[d].position.cast<double>();
                Vector3d dim_det = detections[d].dimensions.cast<double>();

                double dist = (pos_tracker - pos_det).norm(); // 基础代价 1：质心空间几何距离

                // 基础代价 2：包围盒外形尺寸差异惩罚
                double size_diff = (dim_tracker - dim_det).norm();
                double cost = dist + size_penalty_weight_ * size_diff;

                // 基础代价 3：运动方向差异惩罚（只有在目标具备一定速度时才进行惩罚）
                if (vel_tracker.norm() > vel_heading_thresh_)
                {
                    Vector3d displacement = pos_det - pos_tracker;
                    if (displacement.norm() > vel_heading_thresh_)
                    {
                        double cos_angle = vel_tracker.dot(displacement) / (vel_tracker.norm() * displacement.norm());
                        double angle_penalty = (1.0 - cos_angle); // 方向重合度：0 (平行同向) 到 2 (反向)
                        cost += angle_penalty_weight_ * angle_penalty;
                    }
                }

                // --- 遮挡恢复期专用：基于 Transformer 轨迹波形的匹配重激活 ---
                // 当目标因短时遮挡处于丢失状态 (LOST) 时，依赖预测轨迹曲线而非单一点去匹配
                if (trackers_[t].getTrackState() == LOST)
                {
                    // 计算新的检测点到 Transformer 生成的预测曲线上的最短投影距离
                    double min_dist_to_curve = dist;
                    std::vector<Eigen::Matrix<double, 6, 1>> predicted_curve = trackers_[t].getPredictedCurve();
                    if (!predicted_curve.empty())
                    {
                        for (const auto &pred_pt : predicted_curve)
                        {
                            double pt_dist = (pred_pt.head<3>() - pos_det).norm();
                            if (pt_dist < min_dist_to_curve)
                                min_dist_to_curve = pt_dist;
                        }
                        // 遮挡恢复时，使用“对预测轨迹的最短投射距离”代替常规模糊的卡尔曼点推测
                        cost = min_dist_to_curve + size_penalty_weight_ * size_diff;
                    }
                }

                cost_matrix[t][d] = cost;
            }
        }

        HungarianAlgorithm hungarian;              // --- 第三单元：使用匈牙利算法求解二分图最优匹配 ---
        std::vector<int> assignments;              // 存储最优匹配结果对应的索引清单
        hungarian.Solve(cost_matrix, assignments); // 解算代价矩阵的最优连接方案，求解最小系统总代价

        std::vector<bool> matched_detections(detections.size(), false); // 记录每个检测框是否已被成功分配
        std::vector<int> matched_trackers;                              // 记录成功匹配上观测值的追踪器索引

        // --- 第四单元：合法匹配鉴权验证与状态确认 ---
        for (size_t i = 0; i < assignments.size(); ++i)
        {
            int det_idx = assignments[i]; // 取出追踪器[i]所匹配的检测框索引

            // 规则验证：分配索引合法且组合代价必须小于预设的最大容忍距离阈值
            if (det_idx >= 0 && cost_matrix[i][det_idx] < max_distance_)
            {
                trackers_[i].update(detections[det_idx].toMeasurement(), min_hits_); // 匹配成功：使用新捕获检测框的客观观测特征更新此追踪器模型
                matched_detections[det_idx] = true;                                  // 标记对应的检测框已被合法认领
                matched_trackers.push_back(i);                                       // 登记有效匹配历史
            }
        }

        // 处理未匹配的检测框：为其创建新的目标追踪器
        for (size_t d = 0; d < matched_detections.size(); ++d)
        {
            if (!matched_detections[d]) // 如果该检测框未与任何现有追踪器匹配
            {
                VectorXd init_state = VectorXd::Zero(9);
                init_state.head(3) = detections[d].position.cast<double>();
                init_state.tail(3) = detections[d].dimensions.cast<double>();
                trackers_.push_back(KalmanFilter(current_id_++, init_state, p_factor_, q_factor_, r_factor_)); // 实例化新的卡尔曼滤波器并将其加入追踪列表
            }
        }
    }

    // --- 第五单元：清理长时间未更新的过期追踪器 ---
    auto it = trackers_.begin();
    while (it != trackers_.end())
    {
        if (it->getTimeSinceUpdate() > max_age_) // 如果追踪器丢失（无观测）状态超过了允许的最大寿命
        {
            it = trackers_.erase(it); // 彻底移除该过期目标
        }
        else
        {
            ++it; // 保留活跃或刚丢失不久的目标，指针后移
        }
    }
}

// RViz 视图渲染接口：针对感知信息封装标准化外发内容包
void MOTNode::publishMarkers(const ros::Time &stamp)
{
    visualization_msgs::MarkerArray marker_array; // ROS Marker 数据载体包

    visualization_msgs::Marker delete_all;
    delete_all.action = visualization_msgs::Marker::DELETEALL; // 写入清理上一级操作（删除所有的历史图块）
    marker_array.markers.push_back(delete_all);

    for (const auto &tracker : trackers_) // 查询现存的可用信息
    {
        // 保护性要求：非合法 CONFIRMED 或丢失 LOST 下的目标均不配画图（未达 min_hits 观察要求）
        if (tracker.getTrackState() == CONFIRMED || tracker.getTrackState() == LOST)
        {
            VectorXd state = tracker.getState();

            visualization_msgs::Marker marker;
            marker.header.frame_id = world_frame_;
            marker.header.stamp = stamp;
            marker.ns = "mot_trackers";                      // 指定该画图流名称与 ID
            marker.id = tracker.getId();                     // 关联源体与标识码
            marker.type = visualization_msgs::Marker::CUBE;  // 图框配置样式
            marker.action = visualization_msgs::Marker::ADD; // 命令属性配载

            // 给定图心
            marker.pose.position.x = state(0);
            marker.pose.position.y = state(1);
            marker.pose.position.z = state(2);
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // 取出尺寸应用并延画拉伸
            marker.scale.x = state(6);
            marker.scale.y = state(7);
            marker.scale.z = state(8);

            // 主色彩与半通透设置
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;

            marker.lifetime = ros::Duration(0.5);   // 保留寿命限制 0.5s（超时即无刷新自动消解）
            marker_array.markers.push_back(marker); // 装配单元素

            visualization_msgs::Marker text_marker; // 声明该物体专属的ID文本标记
            text_marker.header = marker.header;
            text_marker.ns = "mot_ids";
            text_marker.id = tracker.getId();                                // 与实体保持相同的ID
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING; // 将字体设置为始终朝向相机的视角
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose.position.x = state(0);
            text_marker.pose.position.y = state(1);
            text_marker.pose.position.z = state(2) + state(8) / 2.0 + 0.5; // 将文本位置上浮至包围盒顶部上方0.5米处，避免被物体自身遮挡
            text_marker.pose.orientation.x = 0.0;
            text_marker.pose.orientation.y = 0.0;
            text_marker.pose.orientation.z = 0.0;
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = 0.5; // 设置文字尺寸高度
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;                                   // 设置不透明的纯白字体
            text_marker.text = "ID: " + std::to_string(tracker.getId()); // 渲染文本
            text_marker.lifetime = ros::Duration(0.5);
            marker_array.markers.push_back(text_marker); // 将文本标记加入发布队列
        }
    }

    pub_markers_.publish(marker_array); // 统一发布所有可视化标记至RViz
}
