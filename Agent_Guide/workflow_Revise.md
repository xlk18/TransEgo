# workflow_Revise（可执行修订版）

## 0. 修订目标

将原始工作流升级为“可落地、可验证、可追溯”的研发链路，覆盖：

- 感知 → 跟踪 → 预测 → 规划 → 控制 → 仿真评估
- 文档沉淀与代码联动
- Agent 产出可审计（日志、验收标准、版本记录）

---

## 1. 系统主链路（数据流）

1. FAST-LIO 输出里程计与点云
2. `mot_perception` 进行点云降采样与聚类，输出动态目标中心点
3. `traj_prediction` 输出未来轨迹序列
4. Ego-Planner（现有）消费地图与动态预测，生成规划结果
5. `mpc_controller` 订阅 `/position_cmd` 与 `/Odometry`，发布飞控控制量

---

## 2. Agent 分工修订

### 2.1 Research Agents（20）

- 严格执行真实文献校验：DOI/arXiv + 双数据库可检索
- 统一输出字段：Title / Authors / Venue / Year / DOI(arXiv) / PDF / Verified From

### 2.2 Paper Reading Agents（20）

- 仅可使用已调研文献
- 不得改标题，不得新增未验证条目
- 输出公式、实验设置、可复现条件、开源仓库地址

### 2.3 Coding Agents（11）

- 每个 Agent 只维护一个清晰边界模块
- 提交前必须提供：接口说明、单元验证、运行频率

### 2.4 Writing Agent（1）

- 全程跟随工程进度
- 仅使用已验证引用与真实实验结果
- 论文版本与代码版本同步标签

---

## 3. 质量门禁（QA Gates）

### Gate A：接口正确性

- ROS Topic 名称、消息类型、坐标系定义一致

### Gate B：实时性

- 感知+跟踪频率 ≥ 10Hz
- 预测节点延迟满足在线要求

### Gate C：稳定性

- 连续运行无崩溃
- 关键节点可重启恢复

### Gate D：可追溯

- 每个 Agent 有日志文件
- 每项实验有参数、bag、结果汇总

---

## 4. 代码与文档并行机制

1. 代码 Agent 提交模块实现
2. Review Agent 给出问题列表
3. Integration Agent 联调后更新 Launch
4. Writing Agent 同步更新论文章节草稿

---

## 5. 当前修订落地状态（本轮）

- 已创建新功能包骨架：`mot_perception` / `traj_prediction` / `mpc_controller`
- 已补充对应 launch、package、CMake
- 已输出可执行工作流文档与分工文档

后续重点：

- 将动态障碍物代价项正式接入 Ego-Planner 优化器（非注释占位）
- 完成 Gazebo 闭环实验与消融评估
