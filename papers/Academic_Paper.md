# TransEgo: An Omnidirectional Dynamic Obstacle Avoidance System

## Abstract

This draft presents the current engineering integration status of TransEgo, a UAV navigation pipeline combining LiDAR odometry, dynamic obstacle perception, trajectory prediction, planning, and low-level control. The implementation is under active development, and this manuscript only reports verifiable module interfaces and reproducible procedures. Quantitative conclusions are intentionally deferred until closed-loop simulation and real-world tests are completed.

## 1. Introduction

UAV navigation in dynamic environments remains challenging due to moving obstacles, uncertainty growth over prediction horizon, and tight real-time constraints. This work integrates existing components in the current repository, including FAST-LIO, Ego-Planner, and a new controller/prediction interface layer, to form an end-to-end engineering baseline.

## 2. Methodology

### 2.1 Omnidirectional Perception

The perception pipeline consumes LiDAR point clouds and performs voxel downsampling followed by Euclidean clustering. Cluster centroids are published as track seeds for prediction.

### 2.2 Transformer-based Prediction

The current implementation provides an online prediction node interface and message protocol. A learnable Transformer model can be substituted without changing downstream ROS topics.

### 2.3 MINCO-based Active Avoidance

The planner-side dynamic collision cost design follows a penalty form, e.g. $J_{dynamic}$ based on predicted obstacle distance. The exact implementation and gradient verification are pending completion inside Ego-Planner optimizer source.

### 2.4 NMPC Controller

The controller module currently provides a stable tracking baseline with ROS-compatible attitude/thrust output, serving as a bridge toward full NMPC integration.

## 3. Experiments

Planned evaluation includes:

- Simulation: Gazebo + PX4 SITL
- Metrics: success rate, re-planning count, jerk-related smoothness indicators, inference latency
- Ablation: baseline planner vs prediction-guided planner

No fabricated quantitative results are reported in this draft.

## 4. Conclusion

This draft documents the reproducible system interface and incremental implementation status. Full experimental evidence and validated citations will be added after QA completion.

## References

- [INVALID — Missing DOI/arXiv] To be populated from validated research cards only.

