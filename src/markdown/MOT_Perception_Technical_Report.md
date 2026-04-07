# 多目标追踪与深度学习轨迹预测系统 (MOT Perception) 技术审查与使用报告

## 1. 概述与核心功能

`mot_perception` 是一个基于 ROS 和 C++14 开发且高度并行的 3D 多目标雷达追踪感知核心组件。该系统旨在为无人机及移动机器人提供周围动态与静态环境的高保真态势感知，同时前瞻性地利用 NVDIA TensorRT 引擎挂载基于 Transformer 的时序网络（序列到序列）对所有动态运动者的未来轨迹进行推理与预判。

**主要功能模块：**
1. **输入与预处理**：从原始 `/livox/lidar` 收取大规模点云，执行硬件级 Voxel 降采样、通过 PassThrough 滤除安全高度外的非活动区，并使用 SAC-RANSAC 平面拟合算法剔除地平面干扰。
2. **聚类检测**：利用基于 Kd-Tree 驱动的 Euclidean Cluster Extraction 提供高鲁棒性的 3D Bounding Box （目标包围盒）实例化检测。
3. **数据关联(Data Association)**：结合匈牙利算法求解由尺寸距离误差、速度指向夹角所组合的二分图多元关联代价矩阵（Cost Matrix）。
4. **混合预测追踪 (Kalman + DL Transformer)**：对于合法追踪（CONFIRMED）目标，使用标准卡尔曼(Kalman Velocity Model)与重载的 Transformer 引擎双线预测，赋予系统在目标受到环境遮挡(LOST)时强大的“寻回(Cold-start/Recovery)能力”。
5. **动态背景分离**：在输出建图点云前执行逆遍历，将存在跟踪的移动人员/车辆抠除，输出一张纯净静态的世界地图（`static_cloud`）供给下游碰撞规划器（Ego-Planner）。

---

## 2. Pipeline 数据链路详解

本节点的生命周期在 `MOTNode::cloudCallback` 被激光雷达的高频（如 10Hz, 100ms周期）中断触发后，依次流式开展：

### 阶段一：点云过滤 (`preprocessCloud`)
*   **输入**：雷达原始 `sensor_msgs::PointCloud2` (通常拥有数十万个三维点)
*   **流程**：`VoxelGrid` 体素素化压缩 $\rightarrow$ `PassThrough` 垂直切分截取 $\rightarrow$ `SACSegmentation` (RANSAC算法) 拟合剥出底面并生成负索引
*   **输出**：干练的 `pcl::PointCloud<PointType>::Ptr` (即“干净空中浮块”点云)

### 阶段二：实例检测 (`getDetections`)
*   **输入**：阶段一出具的纯净高密度点云
*   **流程**：输入给 `KdTree`，后用 `EuclideanClusterExtraction` 基于设定的 `cluster_tolerance` 区隔容差将点云撕裂为一个个独立的离散点簇。每个点簇经对角求极值（`getMinMax3D`）后绑定形成物理 Bounding Box
*   **输出**：当前帧的所有未加编号的 `std::vector<BoundingBox>`。

### 阶段三：混合预测与追踪更新 (`trackObjects`)
*   **预测层**：追踪器内部优先使用 `tracker.predict(dt)` 卡尔曼线性延展；若 `use_transformer` 开启，则抽提追踪器内部的历史滑窗数据（10 个特征点）推入 GPU TensorRT；
*   **关联层**：根据“空间欧式距离 + Box边界差集惩罚 + 速度/夹角反向惩罚(防包交叉)” 形成二维代价矩阵。再将代价矩阵委托给匈牙利矩阵 (`HungarianAlgorithm`) 解算出本帧的最优框匹配配对。
*   **更新层**：若未超过 `max_distance` 的阈值容错，则喂入卡尔曼观测值。更新存活时长(age)，消除过时过期对象。
*   **输出**：维护稳定的全局追踪列表 `std::vector<KalmanFilter> trackers_`。

### 阶段四：渲染与剔除 (`publishMarkers` & `Static Map Generator`)
*   **流程**：组装 MarkerArray 输出 RViz。同步扫描原点云内的所有散点是否陷入任一 Confirmed Bounding Box，未陷入的点予以保留发送。

---

## 3. ROS Topic 输入/输出接口规范

| 流向 | Topic 名称 | 消息类型 (Ros-Msg) | 说明 |
| :--- | :--- | :--- | :--- |
| **IN** | `/livox/lidar` | `sensor_msgs::PointCloud2` | 硬件下发的原始大规模散乱激点云流 |
| **IN** | TF Tree | `tf2_ros::Buffer` | 提供 `lidar_link` 射向 `world_frame(map)` 的运动刚体坐标转换关系 |
| **OUT** | `/mot_markers` | `visualization_msgs::MarkerArray` | 用于 RViz 显示的带有颜色、文字 ID（浮空呈现）、边框比例的追踪实例立体框群体 |
| **OUT** | `/traj_prediction/predicted_trajectories` | `visualization_msgs::MarkerArray` | 将 Transformer 生成的未来散点运动线路曲线予以渲染发包，并将预测的标准差(std_xyz)压缩至 ColorRGBA 颜色通道中传递，供下游 Ego-Planner 规划器作膨胀避障 |
| **OUT** | `/static_cloud` | `sensor_msgs::PointCloud2` | 提供给 **ego_planner** / **路径搜索** 模块用的干净环境雷达场（抠除了动态物体防串扰拖影） |

---

## 4. TensorRT 模型深度学习层通信格式

模型由 `mot_node_core.cpp` 与 GPU 原生 API `cudaMemcpy` 无缝直接对接，无需跨进程通信：

*   **引擎要求格式**：`.engine` 序列化二进制文件，只能由本机的 `trtexec` 从 ONNX 或 Torch 导出转换而来。
*   **绑定输入**：张量名必为 `"input_tensor"`，特征形态必为 `[1, 10, 6]` (对应 Batch=1, `hist_len` 时间步, `feat_dim` 个物理特征)，且只认浮点数 FP32 矩阵。
*   **绑定输出**：张量名必为 `"output_tensor"`，输出形态为 `[1, 20, 6]` (对应 Batch=1, `fut_len` 的点位时间步, `out_dim` 的 `(x, y, z)` 三维地理坐标预测以及 `(std_x, std_y, std_z)` 高斯标准差，用作动态避障安全半径)。

---

## 5. 参数字典 (Configuration Parameters)
**配置文件位置**：`mot_perception/config/mot_params.yaml`

| 字段名称 | 推荐值 | 物理/算法现实意义解释 |
|:---|:---|:---|
| **lidar_topic** | `"/livox/lidar"` | 对接前段设备的 Topic 点串口 |
| **world_frame** | `"map"` | 发送追踪方框/点云参照的绝对世界地图坐标系帧头 |
| **cluster_tolerance** | `0.5` | 欧式提取时“两点可以被算作统一个物体”的最大分离距离 $(m)$ |
| **max_age** / **min_hits** | `5` / `3` | 若丢失5帧(0.5秒)则注销ID放弃追踪；被检测到至少连续3帧才升级为 Confirmed 稳态 |
| **dynamic_bbx_expansion** | `0.3` | **[膨胀比]** 在剔除非静态的点时，额外将BoundingBox体积向外放缩0.3米，防物体四肢、衣服散点未被切净 |
| **size_penalty_weight** | `1.5` | 匈牙利匹配时，对目标长宽高前后两帧发生突然形变这一异常事实施加的惩罚代价影响因数 |
| **angle_penalty_weight** | `2.0` | 匹配时如果新抓获框导致算得的临时朝向与其历史预估朝向截然相反(如迎面撞向而过)，施加强烈互斥避碰惩罚 |
| **use_transformer** | `true/false`| TensorRT 加固启动指令。切断时模型层关闭显存调度，全量隐退至卡尔曼线性代数法。|
| **hist_len** / **fut_len** | `10` / `20`| [Rolling-Window 滑动自持窗口参数] 过去保留 10 个周期帧距，推发未来 20 个周期的散连曲线。 |

---

## 6. 使用与拓展指南

*   **部署与启动**：
    所有修改和 C++ 标准化已全闭环完成。
    通过执行下述指令刷新构建编译链接：
    ```bash
    cd /home/yyf/TransEgo
    catkin_make --pkg mot_perception
    source devel/setup.bash
    roslaunch mot_perception mot.launch
    ```
*   **未来调休支持**：本代码采用模块化隔离(Kalman/Detection/Dl Model)，当您需要新增预测手段（例如多模态概率预测），可以直接在 `MOTNode::trackObjects` 层扩充 `cudaMemcpy` 获取后张量的解码途径即可。
## 7. 匈牙利算法 (Hungarian Algorithm) 数据关联内核原理

在 `mot_perception` 节点进行多目标追踪时，需要将"上一帧的轨迹预测"（Trackers）与"当前雷达斩获的实体框"（Detections）进行最优的匹配，这是典型的**二分图最大权/最小权匹配问题**。本代码在 `hungarian_algorithm.cpp` 中通过高效的 C++ 数组展平运算进行了实现，具体流程解析如下：

### 7.1 二维代价矩阵展平 (`Solve` 接口)
为了适应底层的寻优计算，`Solve` 函数首先截取当前空间内所有的 Trackers 数量作行 (`nRows`)，新检测的 Detections 数量作列 (`nCols`)。
它将传入的 `std::vector<std::vector<double>> DistMatrix`（二层容器，包含了坐标相差距离、尺寸惩罚、速度角度惩罚三项综合 Cost）使用经典的 `i + nRows * j` 列优先方式铺展为连续的 `double* distMatrixIn` 内存条，极大提升了指针寻址命中率与算法的速度。

### 7.2 矩阵掩码与最优化求解 (`assignmentoptimal`底层实现)
在将追踪框和检测实体分配时，可能面临 `nRows != nCols` 的非对称情况（目标消失，或新目标乱入）。算法在内部做了精细的行列填充补齐：
1. **行/列缩减步 (Matrix Reduction)**：扫描补足后代价矩阵的每一行并减去行最小值，再扫描每一列并减去列最小值，从而在矩阵中刻意制造大量的 `0` 值代价。
2. **打点涂色 (Covering Zeros)**：用尽可能少的水平线和垂直线（`coveredRows`, `coveredColumns`）覆盖住矩阵中出现的所有 `0`。
3. **探路分配 (Star/Prime 标记)**：基于没有被覆盖或冲突的 `0` 元素记录星号 (`starMatrix`)，表示潜在的匹配选择；若覆盖线数量不能达到总维度 `N`，则代表需要进一步对剩下的未覆盖非零元素进行代数迭代消减计算 (`step-6`，追加或减少最小值补偿)，直到覆盖线数量等于矩阵维度为止。
4. **结果回拨**：最终 `starMatrix` 中形成合法唯一占位的元素的行列号，即对应生成最优且无冲突的配对 ID。

### 7.3 最优无向解出参 (`Assignment`数组提取)
函数不仅会将所有合法匹配结果（如：Tracker[0] 匹配 Detention[2]）写入 `std::vector<int> &Assignment` 传递回 MOTNode 给卡尔曼滤波器更新，还会回传 `double cost` 作为综合验证依据，帮助感知链判定这是否是个合法或过于牵强的分配逻辑（如距离大于 `max_distance_`），拒绝跨越物理法则的“瞬移关联”。

这种利用原生数组指针偏移结合星号路径图遍历的匈牙利求解器，免除了高频 ROS 调用的冗杂计算负担，能在微秒级别解算上百个密集无人机群集的追踪指派图，保证了 `MOTNode` 实时性的下限性能。
