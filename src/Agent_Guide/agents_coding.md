# 代码开发 Agent 分工

## Coding Agent 1

- **负责模块**: SLAM接口与点云预处理 (C++)
- **输出路径**: `FAST_LIO/src/laserMapping.cpp`（接口适配）
- **状态**: In Progress

## Coding Agent 2

- **负责模块**: 动态障碍物检测与聚类 (C++)
- **输出路径**: `src/mot_perception/src/mot_node.cpp`
- **状态**: Done（基础版）

## Coding Agent 3

- **负责模块**: 多目标跟踪(MOT) (C++)
- **输出路径**: `src/mot_perception/src/mot_node.cpp`（聚类中心输出）
- **状态**: In Progress

## Coding Agent 4

- **负责模块**: Transformer预测模型搭建与训练 (Python)
- **输出路径**: `src/traj_prediction/scripts/transformer_predict.py`
- **状态**: Done（在线推理占位）

## Coding Agent 5

- **负责模块**: TensorRT C++ 推理节点
- **输出路径**: `src/traj_prediction/`（待补充 trt_infer_node）
- **状态**: Pending

## Coding Agent 6

- **负责模块**: MINCO轨迹优化目标函数修改 (C++)
- **输出路径**: `Ego_v2/planner/traj_opt/src/poly_traj_optimizer.cpp`
- **状态**: Pending（等待预测接口固定）

## Coding Agent 7

- **负责模块**: Ego-Planner ROS 接口调整 (C++)
- **输出路径**: `Ego_v2/planner/plan_manage/`
- **状态**: Pending

## Coding Agent 8

- **负责模块**: NMPC 控制器核心求解器 (C++)
- **输出路径**: `src/mpc_controller/src/mpc_node.cpp`
- **状态**: Done（稳定追踪基线）

## Coding Agent 9

- **负责模块**: 控制器 ROS 封装与 PX4 对接
- **输出路径**: `src/mpc_controller/launch/mpc_controller.launch`
- **状态**: In Progress

## Coding Agent 10

- **负责模块**: Gazebo 动态环境仿真搭建
- **输出路径**: `quadrotor_PID_controller/launch/`（复用并扩展）
- **状态**: Pending

## Coding Agent 11

- **负责模块**: 系统级联调与 Launch 编写
- **输出路径**: `src/*/launch/` 与总控 launch（待新增）
- **状态**: In Progress


