# 调研汇总（Research Summary）

## 目标

汇总 20 个调研方向，形成后续开发与论文写作的可追溯输入。

## 校验规则（强制）

每篇文献必须满足：

1. 来源于真实数据库（arXiv / IEEE / ACM / CVPR / ICRA / IROS / NeurIPS / ICML / ICLR / RAL / RSS / IJRR / TRO 等）
2. 含 DOI 或 arXiv ID
3. 可由两个来源检索验证
4. 可下载 PDF

不满足项统一标记：`[INVALID — Missing DOI/arXiv]`

## 20 个方向汇总

| Agent | 方向 | 输出文件 | 状态 |
| --- | --- | --- | --- |
| 1 | SLAM与多传感器融合 | SLAM与多传感器融合.md | In Progress |
| 2 | 基于点云的动态目标检测 | 基于点云的动态目标检测.md | In Progress |
| 3 | 3D多目标跟踪(MOT) | 3D多目标跟踪(MOT).md | In Progress |
| 4 | 卡尔曼滤波在无人机中的应用 | 卡尔曼滤波在无人机中的应用.md | In Progress |
| 5 | Transformer在轨迹预测中的应用 | Transformer在轨迹预测中的应用.md | In Progress |
| 6 | 基于深度学习的端到端轨迹预测 | 基于深度学习的端到端轨迹预测.md | In Progress |
| 7 | TensorRT模型部署与加速 | TensorRT模型部署与加速.md | In Progress |
| 8 | MINCO轨迹参数化方法 | MINCO轨迹参数化方法.md | In Progress |
| 9 | 无人机避障的势场法与扩展 | 无人机避障的势场法与扩展.md | In Progress |
| 10 | 非线性模型预测控制(NMPC) | 非线性模型预测控制(NMPC).md | In Progress |
| 11 | CasADi与Ipopt在MPC中的应用 | CasADi与Ipopt在MPC中的应用.md | In Progress |
| 12 | 时间最优轨迹规划 | 时间最优轨迹规划.md | In Progress |
| 13 | 无人机编队控制 | 无人机编队控制.md | In Progress |
| 14 | 动态环境下的航迹重规划 | 动态环境下的航迹重规划.md | In Progress |
| 15 | 基于采样的轨迹规划(A*, RRT*) | 基于采样的轨迹规划(A*, RRT*).md | In Progress |
| 16 | 基于优化的轨迹生成 | 基于优化的轨迹生成.md | In Progress |
| 17 | 多智能体RL避障 | 多智能体RL避障.md | In Progress |
| 18 | ROS2与Gazebo仿真环境 | ROS2与Gazebo仿真环境.md | In Progress |
| 19 | PX4飞控接口与底层控制 | PX4飞控接口与底层控制.md | In Progress |
| 20 | 激光雷达数据预处理与去畸变 | 激光雷达数据预处理与去畸变.md | In Progress |

## 对开发阶段的直接输入

- 感知模块：点云动态分割阈值与聚类参数范围
- 预测模块：历史窗口长度与预测步长
- 规划模块：动态障碍物安全半径模型
- 控制模块：MPC 状态量与约束集合
