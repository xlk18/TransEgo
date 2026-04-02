# workflow_implement（执行版）

## 1. 执行范围

本文件定义从代码开发到系统联调的实际执行顺序，确保与现有工程兼容：

- Ego-Planner（路径规划）
- FAST-LIO（里程计）
- quadrotor_PID_controller（现有控制基线）
- 新模块：`mot_perception`、`traj_prediction`、`mpc_controller`

## 2. 统一话题约定

### 输入

- `/livox/lidar`：点云输入
- `/Odometry`：里程计输入
- `/position_cmd`：上层期望轨迹/控制命令

### 中间输出

- `/dynamic_cloud`：动态点云
- `/mot_tracks`：跟踪目标序列
- `/predicted_trajectories`：未来轨迹

### 控制输出

- `/mavros/setpoint_raw/attitude`：姿态/推力控制

## 3. Agent 执行序列

1. **Research Stage**
   - 维护 `research/*.md`
   - 汇总到 `research/Summary.md`

2. **Paper Stage**
   - 维护 `papers/*.md`
   - 汇总到 `papers/Summary.md`

3. **Code Stage**
   - 感知跟踪：`src/mot_perception`
   - 轨迹预测：`src/traj_prediction`
   - 控制器：`src/mpc_controller`

4. **Integration Stage**
   - 统一 launch 串联
   - 检查话题连通和频率

5. **Validation Stage**
   - rosbag 回放
   - Gazebo 仿真
   - 指标统计（成功率、重规划次数、轨迹平滑性）

6. **Writing Stage**
   - 写作 Agent 同步更新论文草稿

## 4. 启动顺序建议

1. 启动 FAST-LIO（里程计）
2. 启动 `mot_perception.launch`
3. 启动 `traj_prediction.launch`
4. 启动 Ego-Planner 相关节点
5. 启动 `mpc_controller.launch`
6. 最后启动仿真与可视化

## 5. 验收清单

- [ ] `mot_perception` 输出稳定（>10Hz）
- [ ] `traj_prediction` 有有效未来轨迹输出
- [ ] `mpc_controller` 持续输出控制指令
- [ ] 规划与控制全链路无话题中断
- [ ] 关键实验可复现并记录在论文草稿

## 6. 版本归档规范

- 每个 Agent 修改后写入 `logs/agent_dev_logs/`
- 每次功能合入附带：
  - 变更摘要
  - 影响模块
  - 验证方式
