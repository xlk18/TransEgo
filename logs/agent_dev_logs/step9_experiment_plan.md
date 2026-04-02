# Step9 闭环仿真与消融实验计划

## 对比方法

- baseline_ego
- kf_plus_ego
- trans_ego

## 指标

- Success Rate
- Average Flight Time
- Re-planning Count
- Jerk Integral

## 执行输出

- CSV: `logs/agent_dev_logs/step9_ablation_template.csv`
- 汇总: 由 `src/trans_ego_integration/scripts/ablation_metrics.py` 生成
