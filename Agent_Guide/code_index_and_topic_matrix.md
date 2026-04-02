# 代码索引结构图与 ROS Topic 接口矩阵

## 1. 核心代码索引

- Ego-Planner 核心优化：`Ego_v2/planner/traj_opt/src/poly_traj_optimizer.cpp`
- Fast-LIO 主节点：`FAST_LIO/src/laserMapping.cpp`
- 现有控制器：`quadrotor_PID_controller/src/OffboardWrapper.cpp`
- 新增感知模块：`src/mot_perception/src/mot_node.cpp`
- 新增预测模块：`src/traj_prediction/scripts/transformer_predict.py`
- 新增控制模块：`src/mpc_controller/src/mpc_node.cpp`

## 2. 关键 Topic 矩阵（已落地）

| 模块 | 订阅 | 发布 |
| --- | --- | --- |
| FAST-LIO | `/livox/lidar`, `/livox/imu` | `/Odometry` 等 |
| mot_perception | `/livox/lidar` | `/dynamic_cloud`, `/mot_tracks` |
| traj_prediction | `/mot_tracks` | `/predicted_trajectories` |
| Ego-Planner | `odom_world`, 地图与目标 | `/position_cmd` |
| mpc_controller | `/Odometry`, `/position_cmd` | `/mavros/setpoint_raw/attitude` |

## 3. 待完成接口

- `predicted_trajectories` → Ego-Planner 动态代价项输入
- Ego-Planner 新代价梯度与 L-BFGS 一致性验证
- Gazebo 闭环指标自动采集
