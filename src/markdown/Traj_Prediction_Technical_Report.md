# 深度学习轨迹预测系统 (Traj Prediction) 技术审查与使用报告

## 1. 概述与核心定位

`traj_prediction` 是 Ego-Planner 无人机系统中的纯神经网络预测子包。核心目标是接收前端感知模块上报的历史行人/目标运动轨迹数据（MOT），输出其在未来时间区间（如未来 2 秒，20 个步长）的具体位置点序列以及伴随的**路线不确定度（Gaussian Variance)**，供后端航迹规划使用。

本次重大重构后，本系统不仅从输出纯坐标的三维模型（3D）飞跃至输出包含高斯概率分布的六维模型（6D），还通过算法内置的方式完成了相对坐标的平滑适配，斩断了各种边界计算带来的隐患。

---

## 2. 核心模块与技术方法

### 2.1 架构模型 (Transformer Predictor)
模型定义在 `scripts/transformer_model.py` 中。架构沿用了标准的 `Encoder-Decoder Transformer`：
*   **Positional Encoding**: 利用正余弦位置编码 (Sin/Cos PE)，维持不同时间步输入的序列关系不乱序。
*   **Encoder**: 结合历史信息的 `coord_emb` 特征词向量和自注意力机制。
*   **Decoder**: 使用 `Query` 解码未来步长，并在初始状态融合来自于历史片段末端的信息。

### 2.2 内部相对坐标与平移不变性 (Translation Invariance)
**核心改动一**：这是提升泛化能力的最关键改动！直接将全局绝对坐标放入大模型训练很难收敛且缺乏物理环境无关性。
*   **现存逻辑**：外部发送任何绝对空间原点，模型内部在其 `forward` 阶段自动抽取历史坐标系最后一步作为原点基标 `last_pos`。
*   **自动处理**：将输入转化为相距原点的偏差矢量 (`src_rel = src - last_pos`)；最后输出阶段再自动将解码的相对坐标系加回原点形成绝对预测 (`pred_abs_pos = pred_rel_pos + last_pos`)。
*   **收益**：消除了外部 ROS 节点进行繁琐坐标变换易出错的 Bug (双重抵消问题已修复)；对任意环境起始点的泛化能力指数级提升。

### 2.3 高斯方差联合预测 (Gaussian Variance Output)
**核心改动二**：下游规划器如果仅获得了一条固定轨迹，则极易擦挂动态物体。
*   **方差预估**：将输出维度 `out_dim` 由 `3` 提升为 `6` (涵盖 $x, y, z$ 与 $\sigma_x, \sigma_y, \sigma_z$)。模型后三维输出预测对数方差 `pred_log_var`。
*   **方差限幅**：经过 `torch.clamp(-5.0, 2.0)` 将 Log-Variance 圈定在安全范围以解决指数计算溢出问题。
*   **损失函数**：训练时使用了负对数似然损失 `nn.GaussianNLLLoss()`，强制神经网络不仅要准，而且要自知“这部分预测大概有多不可靠”，并输出高方差补偿损失。

---

## 3. ROS 输入 / 输出与接口协议

不管是底层高并发 C++ 的 `trt_predictor_node` 抑或是 Python 端验证的原型 `transformer_predict.py`，他们均共享如下通信格式与封装准则。

| 模块 / 属性 | 名称 | 数据类型 | 说明 |
| :--- | :--- | :--- | :--- |
| **订阅 (Sub)** | `/mot_markers` | `visualization_msgs::MarkerArray` | 获取多目标感知传发过来的带 ID、包含历史连续点的 Bounding Box 流 |
| **发布 (Pub)** | `/traj_prediction/predicted_trajectories` | `visualization_msgs::MarkerArray` | 发布给 Planner 的预测阵列（通过 SPHERE_LIST / LINE_STRIP形式组织）|
| **时空同步** | Stamp & Frame | ROS Time / string | 强绑定 Frame 到 `world` 坐标系，时间戳硬性拉齐到最新检测时刻，防止坐标轴飘飞。 |
| **通道重载** | `Marker.color(R,G,B)` | `std_msgs::ColorRGBA` | **[关键约定]** 将 Tensor 下发的 $(\sigma_x, \sigma_y, \sigma_z)$ 标准差硬塞入 ROS Marker 的 R，G，B 三通道浮点数中发送，用颜色位借道供 Ego Planner 解析！|

---

## 4. 模块操作指南 (Usage)

本预测模块完全解耦，可在本地独立完成所有集训与验证，只在最终上机时介入 ROS 工作区。

### (1) 模型训练体系 (Train)
依赖于 `transformer_dataset.py` 内部的 `DummyTrajectoryDataset`，它虚拟构建行人匀速直线外加正弦波干扰及卡尔曼噪音残留。
```bash
# 执行训练并生成 weights.pth 指令
cd ~/TransEgo/src/traj_prediction/scripts
python train_transformer.py
```

### (2) 模型验证测试 (Evaluate)
不保存梯度，测试该组模型预估的均方误差(MSELoss)基准精度。
```bash
python evaluate_transformer.py
```

### (3) ROS C++ 高性能部署节点 (TRT Inference)
`trt_predictor_node`
*   需先使用 NVIDIA 的 `trtexec` 将刚刚产出的 PTH $\rightarrow$ ONNX $\rightarrow$ 固化成 `.engine`。
*   程序内部直接借助 `cudaMemcpyDeviceToHost` 打通显存到内存的 6 维预测（包含 3 维位置与 3 维方差），实时性最强，无 Python 解释器开销，是实车部署终极形态。
```bash
roslaunch traj_prediction trt_predictor.launch
```

### (4) ROS Python 原型调试节点 (Python ROS INfe)
`transformer_predict.py`
*   开发期间、环境未装 TensorRT 环境时的 Pytorch 原生过渡方案（模型加载 `weights.pth`）。
*   可直接作为 ROS 节点挂载，包含完整的 MarkerArray 封装发射及方差转颜色通道上报能力。

---

## 5. Major Check & Fixes (本次联调整改汇总)

为杜绝底层崩溃，本版本进行了一次系统级深度校准联调：
1. **彻底解耦重影去重**：之前 Python `transformer_predict.py` 内含有残留的去原点逻辑，现已被根除，模型全权包办该逻辑。
2. **打通方差通道传输**：补全 C++ 和 Python 两边的数据发布协议，如果 `OUT_DIM == 6` 从输出取值填充 `marker.colors.push_back(c)` 发送方差。
3. **修复底层数据灾难**：修补了 `mot_node_core.cpp` 返回取值类型断崖报错 (`Eigen::Vector3d` -> `Eigen::Matrix<double, 6, 1>`)。
4. **训练与评测打通**：更新由于增加 GaussianVariance 方差维度导致的验证推理不兼容问题。全线使用 `preds[:, :, 0:3]` 切割评测均方验证误差。