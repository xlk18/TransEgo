# AI 多-Agent 系统研发总指南（Master Guide）

## 🎯 目标（Project Goal）

本项目旨在构建一个 **可扩展、多 Agent 协作、具备自主调研、代码生成、代码审查、系统集成能力的 AI 研发体系**。  
最终目标包括：

- 自动化调研（20 个 Agent）
- 自动化论文阅读（150+ 篇）
- 自动化知识整理（Markdown 文档）
- 自动化工作流优化（更新 workflow.md）
- 自动化代码开发（11个 Agent至少）
- 自动化代码审查与集成
- 自动化 Git 提交与版本管理
- 最终与 Ego-Planner / Fast-LIO 等现有工程深度融合
- 根据Research Writing Agent.md，将工程实现同时写成一篇学术论文

---

# 1. 项目输入（Project Inputs）

## 1.1 Idea 构思过程
来源：  
- https://gemini.google.com/share/a09ef42bd8d3  
内容包括：  
- idea构思过程  
- 任务分解方法  
- 研发流程构思  
- 技术细节探讨  

## 1.2 当前工程代码
- Ego-Planner（轨迹规划模块）
- Fast-LIO（激光雷达里程计）
- quadrotor_PID_controller（PID前馈控制器）

## 1.3 论文库
- 至少 150 篇相关论文（无人机轨迹规划、动态环境无人机自主导航、轨迹预测、模型预测控制、点云分类等）

---

# 2. 总体流程（Master Workflow）

整个系统的研发流程分为 5 个阶段：

---

## Phase 1：调研 Agent 构建（20 Agents）

### 任务
- 阅读Idea构思过程与，workflow.md文档
- 参考agents_reasearch.md,创建 20 个调研 Agent  根据前一步确定20个最相关的方向（如 SLAM、轨迹预测、避障、控制、Agent 框架等）
- 每个 Agent 负责一个子方向
- 每个 Agent 阅读 5–10 篇论文  
- 输出结构化调研报告（Markdown）
- 所有论文必须真实存在，必须满足以下条件：
  1. 必须来自 arXiv / IEEE / ACM / CVPR / ICRA / IROS / NeurIPS / ICML / ICLR / RAL / RSS / IJRR / TRO / Science Robotics等真实数据库。
  2. 必须包含至少一个可验证标识：DOI、arXiv ID、会议名称、期刊卷号。
  3. 必须通过两个不同数据库检索到（如 arXiv + Scholar）。
  4. 禁止生成论文标题，所有标题必须来自数据库检索。
  5. 必须能下载 PDF。
  6. 必须包含作者、年份、机构等元信息。
  7. 输出必须包含：Title, Authors, Venue, Year, DOI/arXiv, PDF Link, Verified From。
  任何不满足以上条件的论文一律视为不存在，不得纳入调研结果。

### 输出
- 每个agent按照自己的调研方向将结果保存在`/reasearch/xx方向.md` 下
- 每个agent负责的方向保存在`agents_reasearch.md`
- `/research/Summary.md`（最后将所有结果自动汇总在此）

---

## Phase 2：论文阅读与知识整理（150+ Papers）

### 任务
- 自动化阅读论文  
- 提取：方法、公式、实验、优缺点、可复现性  
- 如果论文中有开源代码地址的，自动去阅读开源代码，将有价值的仓库地质保存在`/reasearch/repo.md`文件中，后续开发时可以直接参考使用
- 生成统一格式的论文卡片（Paper Card）
- 阅读 Agent 有以下要求：
  1. 只能阅读调研agent已经调研的论文
  2. 不允许新增论文
  3. 不允许修改论文标题

### 输出
- `/papers/xx(论文英文标题).md` （不同论文独自建立）
- `/papers/Summary.md`

---

## Phase 3：workflow.md文档优化

### 任务
- 阅读调研结果  
- 对原 workflow优化升级（针对不合理地方改进，具体每一步技术细节，验证方式等）
- 引入新的 workflow、模块划分、Agent 角色  
- 形成新版本

### 输出
- `/Agent_Guide/workflow_Revise.md`

---

## Phase 4：新 Workflow 设计（workflow_implement.md）

### 任务
- 基于调研与workflow_Revise.md
- 设计最后具体执行的端到端工作流 
- 包含：  
  - Agent 协作方式，工作划分
  - 代码生成  
  - 代码审查  
  - 系统集成  
  - UAV 工程链路整合（Ego-Planner + Fast-LIO）

### 输出
- `/Agent_Guide/workflow_implement.md`

---

## Phase 5：代码开发与审查（11 Agents）

### 任务
- 创建至少 11 个开发 Agent，分工参考agents_coding.md，具体自己分析。
- 如果是新的功能包新建一个文件夹，与其他包并列，如果是在已有包上直接修改即可
- 每个 Agent 负责一个模块（如 SLAM 接口、轨迹预测、控制器、仿真等）  
- 自动生成代码  
- 自动审查代码  
- 自动集成  
- 自动 Git commit

### 输出
- 每个agent负责的方向保存在`agents_coding.md`
- `/src/...`  
- `/logs/agent_dev_logs/`  
- Git 提交记录

---

## Phase 6：学术论文书写

### 任务
- 参考Research Writing Agent.md文档，建立一个学术写作agent，在工程实现的同时写出一篇学术论文

### 输出
- 一篇符合学术规范的学术论文

---

# 3. Agent 体系结构（Agent Architecture）

## 3.1 Agent 类型

| Agent 类型 | 数量 | 作用 |
|-----------|------|------|
| 调研 Agent | 20 | 论文调研、技术分析 |
| 阅读 Agent | 20 | 论文阅读、知识提取 |
| 开发 Agent | 11 | 代码生成、调试、审查 |
| 审查 Agent | 若干 | 代码审查、风格检查 |
| 集成 Agent | 若干 | 模块集成、构建、测试 |
| Git Agent | 1 | 自动 commit、生成注释 |

---

# 4. Git 工作流（Git Workflow）

## 4.1 自动化 Git 提交策略

- 每个 Agent 完成任务后自动 commit  
- Commit message 模板：
- [Agent-XX] 完成任务：<任务名称>
    修改内容：
    影响模块：
    验证方式：
- 先保存到本地仓库（如果没有建立自己建立）最后提交到远程仓库git@github.com:xlk18/TransEgo.git
- 重大变更使用 PR（Pull Request）  
- 审查 Agent 自动审查 PR

---

# 5. 文档结构（Documentation Structure）

/Agent_Guide
workflow_implement.md
...
/research
xx方向.md
Summary.md
...
/papers
xx(论文英文标题).md
Summary.md
...
/Ego_v2（已有）
/FAST_LIO（已有）
/新功能包名
<代码模块>
...
/agents
<Agent 配置与日志>

---

# 6. 最终交付物（Deliverables）

- 20 个调研 Agent 的报告  
- 150 篇论文卡片  
- 新的workflow文档
- 11个开发 Agent 的代码  
- 完整 Git 仓库  
- UAV 系统集成（Ego-Planner + Fast-LIO + 新的功能模块）

---

# 7. 严格执行步骤（Execution Guide）

* **Step 1**: 吸收 Idea 构思与总体目标，建立项目上下文。
* **Step 2**: 构建现有工程（Ego_v2, Fast-LIO, PID_Controller）的代码索引结构图与 ROS Topic 接口矩阵，不求通读全文，但求掌握依赖关系。（重点看../Ego_v2/planner/traj_opt/src/poly_traj_optimizer.cpp，../FAST_LIO/src/laserMapping.cpp）
* **Step 3**: 唤醒 20 个 Research Agents，按领域并行检索并输出技术基线。同时参考`Research Writing Agent.md`创建一个学术写作agent，在调研，阅读论文全程参与，最后工程实现的同时产出一篇学术论文
* **Step 4**: 唤醒 Paper Reading Agents 提取 150+ 论文的数学公式、SOTA 指标与开源代码仓库。
* **Step 5**: 综合研究结果，自我审查并升版 `workflow.md`，修复潜在的数学矛盾与架构缺陷。
* **Step 6**: 唤醒 11 个核心 Coding Agents（精锐小队），认领专属功能包模块。
* **Step 7**: 模块化开发与 ROSbag 单元测试（强制执行）。代码必须通过 QA Agent 的实时性（>10Hz）和内存泄漏审查。所有的rosbag新建一个文件保存。
* **Step 8**: 集成联调 Agent 接管，编写完整的系统级 Launch 文件。
* **Step 9**: 启动 Gazebo 闭环仿真，执行消融实验并自动提取量化指标（Jerk, 成功率）。
* **Step 10**: Git 自动规范化提交与项目归档。
---

# 🎯 总结

本指南作为整个多 Agent 研发体系的顶层设计文档，定义了：

- 项目目标  
- 输入  
- 多阶段工作流  
- Agent 架构  
- Git 工作流  
- 文档结构  
- 最终交付物  
- 执行步骤  
