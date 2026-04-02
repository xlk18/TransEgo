基于 Transformer 与 Ego-Planner 的全向动态避障系统工作流

Module 1: 前端全向感知与多目标跟踪 (C++/ROS)
目标：处理 Mid-360 原始点云，剔除静态背景，提取并持续跟踪动态障碍物，为下游提供带 ID 的历史轨迹序列。

1.1 具体实现步骤
里程计与建图集成：接入 FAST-LIO2，订阅 Mid-360 的 /livox/lidar 和 /livox/imu，获取高精度里程计 /Odometry 和局部增量地图（ikd-Tree）。

动态点云提取 (Background Subtraction)：编写 ROS 节点，将当前帧点云通过里程计变换到全局坐标系。遍历当前帧点云，在 ikd-Tree 中搜索最近邻。若距离阈值 $> 0.2m$，则标记为动态点。

几何聚类 (Clustering)：对动态点云应用 VoxelGrid 降采样（例如 leaf size 0.1m）。使用 PCL 的 EuclideanClusterExtraction 提取聚类，计算每个簇的形心 $(x,y,z)$ 和 3D Bounding Box $(l,w,h)$。

多目标跟踪 (MOT)：为每个簇初始化状态变量为 $[x, y, z, v_x, v_y, v_z, l, w, h]$ 的卡尔曼滤波器 (KF)。实现匈牙利算法 (Hungarian Algorithm)：结合欧氏距离、速度方向夹角和 Bounding Box 尺寸差异，构建代价矩阵，进行前后帧 ID 关联。

航迹管理：实现 Tentative、Confirmed、Lost 三种状态的逻辑流转。当目标被遮挡进入 Lost 状态时，暂时使用 KF 的预测步维持 ID。

1.2 模块验证方法 (Validation)
可视化验证：在 RViz 中，静态背景（如墙壁、树木）应清晰稳定，走动的人或移动物体周围应出现 3D Bounding Box。
ROSbag 回放验证：使用录制好的带有动态障碍物的 Mid-360 点云 bag 包进行离线测试，确保在不启动仿真的情况下，KF 追踪器仍能稳定输出轨迹且不发生 TF 树断裂。

ID 稳定性测试：让两个行人交叉走过，RViz 中显示的 ID 标号不发生对换（无 ID Switch）。

性能指标：整个感知与 MOT 节点的运行频率必须 $\ge 10\text{Hz}$（处理延迟 $< 100\text{ms}$）。

Module 2: 时空轨迹预测模型 (Python -> TensorRT)
目标：接收 MOT 提供的历史轨迹序列，推断动态障碍物的非线性意图，输出未来一段时间内的轨迹及膨胀不确定性。

2.1 具体实现步骤
数据流构建：定义模型输入张量形状：$(Batch, H, Feature)$，其中 $H$ 为历史帧数（如 10 帧），$Feature$ 为 $[x, y, z, v_x, v_y, v_z]$。定义输出张量形状：$(Batch, F, 3)$，其中 $F$ 为预测未来帧数（如 20 帧），输出未来坐标。

网络结构设计 (PyTorch)：实现 Positional Encoding，注入时间步序列信息。实现轻量级 Transformer Encoder-Decoder。利用 Self-Attention 提取单体运动特征。

模型训练：使用 ETH/UCY 等公开数据集（或 Gazebo 提取的带噪仿真数据）进行训练。损失函数使用 MSE Loss。

工程部署加速：将 PyTorch 训练好的 .pth 模型导出为 .onnx 格式。编写 C++ 推理类，利用 TensorRT API 加载 ONNX 模型，以实现极低延迟的前向推理，并将其封装入 ROS 节点。

2.2 模块验证方法 (Validation)
指标验证：在验证集上计算 ADE (Average Displacement Error) 和 FDE (Final Displacement Error)，需优于传统的匀速预测模型（CV Baseline）。

延迟验证：TensorRT C++ 节点的单次前向推理时间必须严格 $< 10\text{ms}$，以满足高频控制需求。

遮挡恢复测试：将 Transformer 预测结果反哺给 Module 1 的 Lost 状态目标，验证在物体绕过树木等遮挡物后，重识别成功率的提升。

Module 3: 预测引导的动态 Ego-Planner-MINCO版本(C++/ROS)
目标：修改底层轨迹优化器，使其能够理解预测出的动态膨胀势场，生成主动避让的安全MINCO轨迹。

3.1 具体实现步骤
数据接收与同步：Ego-Planner 节点除了订阅静态地图（Grid Map/ESDF），还需订阅 Module 2 发布的 Predicted_Trajectories 话题。

动态有效半径计算 (Dynamic Expansion)：实现不确定性膨胀方程：$R_{eff}(t_i) = \max(l,w,h)/2 + r_{uav} + k \cdot (t_i - t_{current})$，其中 $k$ 为不确定性膨胀系数。

首先阅读MINCO轨迹论文，MINCO论文可以在本地/home/yyf/Zotero/storage/TYHS7KRD找到。
目标函数重构：深入 Ego-Planner 的Ego_v2/planner/traj_opt/src/poly_traj_optimizer.cppp。在原有的平滑惩罚（Smoothness）、动力学惩罚（Feasibility）和静态碰撞惩罚（Static Collision）基础上，新增 J_dynamic_collision。遍历MINCOG轨迹控制点 $Q_i$，计算其与同时刻预测障碍物位置 $X_{future}(t_i)$ 的距离。若距离 $< R_{eff}(t_i)$，则产生梯度惩罚：$(-d_{i,m})^3$。严格按照 MINCO 的多项式求导法则推导 $J_{dynamic\_collision}$ 的梯度，不要混淆为 B 样条的凸包特性求导。

梯度推导与求解：在代码中正确推导并实装 J_dynamic_collision 对控制点 $Q_i$ 的偏导数（Jacobian），供 L-BFGS 求解器使用。

3.2 模块验证方法 (Validation)
单点测试：在 RViz 中发布一个虚拟的迎面飞来的动态障碍物及预测轨迹。观察 Ego-Planner 生成的绿色轨迹是否会提前发生形变（向侧上方或侧下方避让）。

平滑度验证：避让轨迹的曲率连续性必须满足无人机的动力学约束（加速度与 Jerk 不超限）。

Moudle 4: MPC控制器底层控制
新写一个 MPC 控制器来替换当前的 PID 前馈控制器，`/home/yyf/casadi`,` /home/yyf/Ipopt`,` /home/yyf/ThirdParty-ASL`,` /home/yyf/ThirdParty-HSL`,` /home/yyf/ThirdParty-Mumps` 这些是我目前已经安装好的求解器，你可以直接检查并使用。

4.1 接收fastlio的里程计信息以及Ego-Planner发布的轨迹，输出具体的控制信号发布给飞控，参考quadrotor_PID_controller，这是我现在有的PID前馈控制器，里面的接口已经是可以使用的。`quadrotor_PID_controller/launch/PX4_sim_single_uav.launch`是仿真启动文件，实物实验是`real_exp_single_uav.launch`,使用激光雷达里程计信息，忽略其中动捕的内容。

4.2 验证：MPC控制器可以接受上层的轨迹信息，并输出推力角速度信息给飞控，实现稳定追踪上层轨迹。

Module 5: 系统级联调与仿真闭环 (Gazebo/ROS)
目标：在高度逼真的物理引擎中，验证整个闭环系统的可靠性和避障性能。

5.1 具体实现步骤
仿真场景搭建：在 Gazebo 中构建包含树木、走廊等静态障碍物的测试场。添加随机游走插件（Actor Plugin）的行人模型或其他飞行器，模拟非线性运动的动态障碍物。
gazebo仿真参考 `quadrotor_PID_controller/launch/PX4_sim_single_uav.launch`文件.

全链路串联：编写统一的 launch 文件：拉起 Gazebo -> 启动 FAST-LIO2 -> 启动感知与追踪节点 -> 启动 TensorRT 预测节点 -> 启动改进版 Ego-Planner -> 启动底层控制器。

性能对比与消融实验 (Ablation Study)：设计固定航线，让无人机穿越动态人群。分别记录：(A) 原始 Ego-Planner, (B) KF 线性预测 + Ego-Planner, (C) Transformer 预测 + Ego-Planner (本文方法) 的飞行表现。

5.2 模块验证方法 (Validation)
量化数据提取：统计并对比三组实验的成功率 (Success Rate)、平均飞行时间 (Average Flight Time)、重规划触发次数 (Re-planning Count) 以及 Jerk 积分。

定性表现：无人机不应出现“走走停停”或“陷入死锁”的现象，应表现出如同老司机般“预判并丝滑绕行”的飞行姿态。
