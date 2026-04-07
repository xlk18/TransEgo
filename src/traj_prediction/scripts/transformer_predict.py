#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    该节点它负责：
    1. 接收前端(Module 1)卡尔曼滤波(KF)输出的多目标跟踪(MOT)历史轨迹序列。
    2. 维护每个 ID 的历史轨迹滑动窗口 (长度 H=10)。
    3. 利用 PyTorch 加速进行 Batch 并行推理，预测未来轨迹 (长度 F=20)。
    4. 将预测结果打包成 ROS 消息发给 Ego-Planner (Module 3) 进行主动避障计算。

    【训练数据说明 (关于您问的"用什么数据去训练网络使其适应 KF 输出的序列")】：
    - 理论数据来源：ETH/UCY 等开源行人轨迹数据集、Stanford Drone Dataset，或者在 Gazebo 中使用 Actor 插件随机游走录制的数据。
    - 适应 KF 输出的噪声增强策略 (Data Augmentation)：由于真实场景下前端 MOT 的卡尔曼滤波输出带有收敛噪声和滞后性，
      在训练时不能只使用绝对平滑的 Ground Truth (GT) 轨迹。必须在 Ground Truth 的历史序列上，注入符合 KF 后验协方差分布的
      高斯噪声（Gaussian Noise, e.g., N(0, P_kf)），甚至模拟丢帧 (Drop-out) 再用线性插值补全的操作，放入 Transformer 的 Encoder 端，
      Decoder 端仍然采用无噪的 GT 计算 MSE Loss。这样训练出的网络才能对真实的、带噪的序列具有极强的鲁棒性。

    【输出数据说明与 Ego-Planner 集成 (关于"发给规划模块并在其中如何避障")】：
    - 输出：输出为当前时间戳 `stamp` 起点后的 $F=20$ 个时刻的绝对预测坐标序列，通过 MarkerArray (兼容可视化) 或具体序列话题发送。
    - Ego-Planner 集成：在 Ego-Planner 的 `poly_traj_optimizer.cpp` 中，订阅该话题并保存到一个全局 Map 中。
    - 避障成本项计算 (MINCO 曲线惩罚)：
      在生成 MINCO 控制点 $Q_i$ 进行非线性优化计算 $J$（Cost）和 $\nabla J$（Gradient）时，遍历轨迹对应时刻 $t_k$，
      计算无人机在 $t_k$ 的位置 $p_{uav}(t_k)$ 与同时刻障碍物预测位置 $p_{obs}(t_k)$ 之间的距离 $d = ||p_{uav} - p_{obs}||_2$。
      若 $d < R_{safe} + k \cdot t_k$ (随时间扩大的膨胀不确定性)，则向代价注入惩罚 $J_{dyn} += (R_{safe} - d)^3$，
      并利用链式法则通过 Jacobian 矩阵将使得这段轨向反方向推开（产生侧向推力梯度），进而实现了预判主动避障。
"""

import os
import sys
import copy
import math
import time
import threading
from collections import defaultdict, deque

import numpy as np

# ROS 基础库
import rospy
import tf2_ros
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Vector3, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

# PyTorch 核心推理库
import torch
import torch.nn as nn
from torch.autograd import Variable

# 导入已抽离的模型架构
from transformer_model import TransformerPredictor


# ==============================================================================
# 卡尔曼滤波 (KF) 输出的状态缓存管理器
# ==============================================================================

class TrackState:
    """
    维护单个目标的历史轨迹队列缓存。
    用于将不规律的感知数据，对齐为高频、等间距的张量，供模型推理使用。
    """
    def __init__(self, track_id: int, max_history_len: int = 10):
        self.track_id = track_id                    # 目标的唯一追踪ID
        self.max_history_len = max_history_len      # 保存历史轨迹的最大长度
        
        # 使用双端队列 (deque) 保存历史状态，每个元素为: (时间戳, X, Y, Z)
        self.history = deque(maxlen=max_history_len)
        self.last_update_time = rospy.Time(0)       # 记录上一次接收到该目标的时间
        
    def update(self, stamp: rospy.Time, position: np.ndarray):
        """ 接收到 MOT 新的一帧时，更新该目标的轨迹 """
        # 数据合法性检查：拒绝时间戳倒退的异常数据
        if stamp <= self.last_update_time:
            rospy.logwarn(f"接收到过时的 MOT 时间数据 for track {self.track_id}.")
            return
            
        # 将时间戳转换为秒，并放入队列 (t, x, y, z)
        self.history.append((stamp.to_sec(), position[0], position[1], position[2]))
        self.last_update_time = stamp               # 刷新最后更新时间
        
    def get_history_matrix(self, target_dt: float = 0.1, target_len: int = 10):
        """
        重采样函数：将不规则的历史观测点，通过线性插值转换为等时间间隔的数据序列。
        提供给 Transformer 的固定形状输入 (target_len, 3)。
        """
        # 如果缓存为空，则直接返回无数据
        if len(self.history) == 0:
            return None
            
        # 提取时间戳数组与对应的空间坐标(x,y,z)数组
        times = np.array([t[0] for t in self.history])
        coords = np.array([[t[1], t[2], t[3]] for t in self.history])
        
        # 根据最后一帧的时间往前倒推，生成我们理想的等间距时间点数组
        t_last = times[-1]
        t_targets = np.linspace(t_last - (target_len - 1) * target_dt, t_last, target_len)
        
        # 创建空数组用于存放置插值后的坐标矩阵
        resampled_coords = np.zeros((target_len, 3))
        for i in range(3):
            # 对 X/Y/Z 三个维度分别进行一维线性插值计算
            resampled_coords[:, i] = np.interp(t_targets, times, coords[:, i])
            
        # 针对队列初期长度不足（甚至还没收集满 target_len 帧）的情况
        t_first = times[0]
        mask_missing = (t_targets < t_first)
        # 对于比最老观测还早的时间点，将坐标固定为最老的观测位置，即原位复制 (Padding)
        if np.any(mask_missing):
            resampled_coords[mask_missing] = coords[0]
            
        return resampled_coords

    def is_stale(self, current_time: rospy.Time, timeout: float = 0.5) -> bool:
        """ 判断目标是否已经丢失超过 timeout 时间（僵尸轨道）"""
        return (current_time - self.last_update_time).to_sec() > timeout


# ==============================================================================
# ROS 节点核心抽象逻辑
# ==============================================================================

class TrajectoryPredictorNode:
    """
    ROS 节点核心：用于桥接 PyTorch 网络推理，负责订阅追踪信息、运行模型，并把预测轨迹通过话题给规划器。
    """
    def __init__(self):
        # 注册 ROS 专属节点，保证进程唯一性
        rospy.init_node('transformer_trajectory_predictor_node', anonymous=False)
        
        # 加载 ROS 参数服务器中的设置 (从 launch 文件中获取核心参数)
        self.hist_len = rospy.get_param('~hist_len', 10)       # 历史序列输入长度（即输入 Transformer 的点数）
        self.fut_len = rospy.get_param('~fut_len', 20)       # 预测未来轨迹的长度（即输出多少个点）
        self.dt = rospy.get_param('~dt', 0.1)                  # 轨迹采样时间间隔 (秒，如 0.1 秒代表 10Hz)
        self.mot_topic = rospy.get_param('~mot_topic_name', '/mot_markers')  # 订阅：多目标跟踪(MOT)的话题
        self.pred_topic = rospy.get_param('~pred_topic_name', '/traj_prediction/predicted_trajectories') # 发布：预测轨迹的话题
        self.model_path = rospy.get_param('~model_path', '')   # PyTorch (.pth) 模型权重的绝对路径
        self.track_timeout = rospy.get_param('~track_timeout', 0.5) # 判定某个 ID 丢失的超时容忍时间
        self.use_gpu = rospy.get_param('~use_gpu', True)       # 是否启用 CUDA GPU 加速
        
        # 智能设备选择：如果请求使用 GPU 且可用，则在 GPU 执行计算，否则退回 CPU
        self.device = torch.device("cuda" if self.use_gpu and torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"[{rospy.get_name()}] 加速预测计算核心设备已被映射为: {self.device}")
        
        # 初始化并载入 Transformer 模型结构和权重文件
        self.model = self._load_model()
        
        # 维护一个字典，保存当前所有正在进行跟踪的物体（通过 ID 映射到 TrackState）
        self.tracks = {}                            
        # 并发线程锁：防止 ROS 订阅回调线程(写入)和 ROS 定时推理线程(读取)发生数据竞争
        self.tracks_lock = threading.Lock()
        
        # 初始化 ROS 订阅器，接收 MOT 模块发布的目标位置聚类及状态
        self.sub_mot = rospy.Subscriber(self.mot_topic, MarkerArray, self.mot_callback, queue_size=1)
        # 初始化 ROS 发布器，供后端 Ego_Planner 获取未来的多智能体轨迹进行碰撞检测
        self.pub_pred = rospy.Publisher(self.pred_topic, MarkerArray, queue_size=1)
        
        # 设置按照物理 dt 作为周期（如0.1s），循环触发预测的定时中断系统
        timer_period = self.dt
        self.timer = rospy.Timer(rospy.Duration(timer_period), self.prediction_loop)
        
        rospy.loginfo(f"[{rospy.get_name()}] Transformer 预测节点已经就绪运行。")

    def _load_model(self) -> nn.Module:
        """
        负责创建 Transformer 模型、并且把指定路径的预训练权重加载进入显存。
        """
        # 构建定义了层数、维度和注意力的预测模型
        model = TransformerPredictor(
            feat_dim=6, out_dim=6,
            d_model=64,
            nhead=4,
            num_encoder_layers=3,
            num_decoder_layers=3,
            dim_feedforward=256,
            dropout=0.1,
            fut_len=self.fut_len
        ).to(self.device)
        
        # 确保指定路径下有训练好的 .pth 文件模型可用
        if self.model_path and os.path.exists(self.model_path):
            try:
                # 动态加载 PyTorch 权重字典至对应的 GPU/CPU
                state_dict = torch.load(self.model_path, map_location=self.device)
                model.load_state_dict(state_dict)
                rospy.loginfo(f"[{rospy.get_name()}] 成功拉取并灌入网络权重由文件: {self.model_path}")
            except Exception as e:
                rospy.logerr(f"[{rospy.get_name()}] 加载预训练模型失败: {e}")
        else:
            # 文件不存在时，仅留空跑状态，发出强烈提醒
            rospy.logwarn(f"[{rospy.get_name()}] 未找到模型，正在注入无意义权重作为快速原型跑通调试占位。")
            
        # 设置模型为评估态，关闭梯度跟踪及随机 Dropout（对于真实推理，这极大幅降低时间和内存花费）
        model.eval() 
        return model

    def mot_callback(self, msg: MarkerArray):
        """
        ROS 订阅回调函数：接收 MOT (跟踪算法) 解析发布的 Marker。
        将提取目标三维位置，存入各目标所属的时序管理器。
        """
        current_time = rospy.Time.now()
        
        # 进入线程安全临界区，避免和周期推送线程发生读写数据崩塌
        with self.tracks_lock:
            active_ids = set() # 本帧有效的目标 ID 集合，后面用于踢掉失效/丢失的物体
            
            # 使用提取每一个 Marker 包含的目标状态
            for marker in msg.markers:
                track_id = marker.id
                active_ids.add(track_id) # 记录有效活跃物体的心跳
                
                # 读取该物体本帧三维坐标
                pos = np.array([
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z
                ])
                
                # 如果这个 ID 从未见过或者新产生，则新建该 ID 的缓存队列
                if track_id not in self.tracks:
                    # 分配两倍的目标数据缓存空间，应对某些历史插值补偿使用
                    self.tracks[track_id] = TrackState(track_id, max_history_len=self.hist_len * 2)
                    
                # 对这个轨迹更新刚刚抽取的位置及时间信息
                self.tracks[track_id].update(current_time, pos)
                
            # 执行垃圾回收逻辑：清除长期没有收到的轨迹缓存资源
            inactive_ids = list(self.tracks.keys())
            for tid in inactive_ids:
                # 即使本帧它没出现，只有超过预定的 timeout（超时期限）才宣告遮挡或者永久离开
                if tid not in active_ids: 
                    if self.tracks[tid].is_stale(current_time, self.track_timeout):
                        del self.tracks[tid] # 彻底释放对应的内存管理

    def prediction_loop(self, event=None):
        """
        主循环：每隔 `self.dt` 执行计算全局轨迹的并行网络推理。
        """
        now = rospy.Time.now()
        
        with self.tracks_lock:
            track_ids = []                          # 准备参与运算的目标ID列表
            histories = []                          # 准备批量打包输入模型的矩阵数据
            
            # 阶段 1：组装多批次的历史序列作为 PyTorch 输入矩阵
            for tid, state in self.tracks.items():
                if state.is_stale(now, self.track_timeout):
                    continue # 放弃超时轨道
                    
                # 每条轨迹提取历史标准矩阵：形如 [hist_len, 3] 
                hist_matrix = state.get_history_matrix(target_dt=self.dt, target_len=self.hist_len)
                if hist_matrix is not None:
                    track_ids.append(tid)
                    histories.append(hist_matrix)
                    
        # 阶段 2：如果场景内一个有效跟踪器都没有，则停止执行消耗资源的网络
        if not histories:
            return
            
        # 阶段 3：启动 GPU 模型的推断
        batch_size = len(histories)
        
        with torch.no_grad(): # 推断阶段不必且不可求梯度
            # CPU 端把所有的输入数据合并成为批量张量: Shape 是 [batch_size, hist_len, 3]
            x_tensor = torch.tensor(np.stack(histories), dtype=torch.float32).to(self.device)
            
            # 执行前向推衍计算：
            start_t = time.time()
            y_pred = self.model(x_tensor)
            inf_time = time.time() - start_t # 测试这步耗费时间
            
            # 推理完毕，切回 CPU 获得 Numpy，脱离 PyTorch 环境回归可执行原生的 ROS 处理
            y_pred_np = y_pred.cpu().numpy()
            
        # 阶段 4：将网络得到的未来预测轨迹序列包装，发给需要避障的系统（和可视化引擎）
        self.publish_predictions(track_ids, histories, y_pred_np, now)
        
        # 用于节点监控耗时情况
        rospy.logdebug(f"[{rospy.get_name()}] 成功并行推算了 {batch_size} 条轨道. 网络延时性能耗费: {inf_time*1000:.2f} ms")


    def publish_predictions(self, track_ids, histories_base, predictions, stamp):
        """
        转换并发送预测数据供 Rviz 可视化以及 Ego Planner 接收使用。
        它承载了未来每一步位置的预估及该点协方差(高斯方差)信息作为避障重要膨胀参考。
        """
        out_msg = MarkerArray()                     # 要发出去的话题消息主体
        
        for idx, tid in enumerate(track_ids):
            marker = Marker()
            marker.header.frame_id = "world" # 将基准坐标系锁定到了世界系供规划计算
            marker.header.stamp = stamp      # 标定最新一次刻录的时钟，方便倒算插值
            marker.ns = "predicted_trajectories"
            marker.id = tid                  # 承袭之前的相同运动 ID
            marker.type = Marker.LINE_STRIP  # Marker类型设置为线性断点相连曲线
            marker.action = Marker.ADD
            
            marker.pose.orientation.w = 1.0  # 基本位姿还原四元数单位基
            
            # 使用刻度限定 Rviz 中的粗细可视效果
            marker.scale.x = 0.1
            
            # 生成该目标特有标识性颜色
            color = ColorRGBA()
            color.r = max(0.2, (tid * 123 % 255) / 255.0)
            color.g = max(0.2, (tid * 321 % 255) / 255.0)
            color.b = max(0.2, (tid * 213 % 255) / 255.0)
            color.a = 0.8
            marker.color = color
            
            # 该显示线条超过该时间未更新则被 RViz 删除，防止重影
            marker.lifetime = rospy.Duration(self.dt * 2)
            
            # 将最后一帧感知点无缝当做预测首坐标相连，产生完整动画平滑闭环
            current_pos = histories_base[idx][-1]
            p_start = Point(x=current_pos[0], y=current_pos[1], z=current_pos[2])
            marker.points.append(p_start)
            
            c_start = ColorRGBA() # 给首点一个颜色
            c_start.r = 0.0
            c_start.g = 0.0
            c_start.b = 0.0
            c_start.a = 1.0
            marker.colors.append(c_start)
            
            # 将神经网络未来推理的每一步的坐标串联放入路径
            pred_pts = predictions[idx] # 网络产出的这个目标的数列
            for i in range(self.fut_len):
                p = Point()
                p.x = float(pred_pts[i, 0])
                p.y = float(pred_pts[i, 1])
                p.z = float(pred_pts[i, 2])
                marker.points.append(p)
                
                # Ego Planner 约定的利用 Color 各通道临时存放方差的暗号策略
                c = ColorRGBA()
                if pred_pts.shape[1] >= 6:
                    c.r = float(pred_pts[i, 3])
                    c.g = float(pred_pts[i, 4])
                    c.b = float(pred_pts[i, 5])
                else:
                    c.r = 0.0
                    c.g = 0.0
                    c.b = 0.0
                c.a = 1.0
                marker.colors.append(c)
                
            out_msg.markers.append(marker)
            
        # 一次性全部通过 Topic 发出
        self.pub_pred.publish(out_msg)

if __name__ == '__main__':
    try:
        # ROS 纯 Python 自启动环境包裹捕捉异常（包含 Ctrl+C ）
        node = TrajectoryPredictorNode()            # 申请节点运行并持锁
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("预测系统收到退出信令，终止一切系统时钟并结束销毁。")
        sys.exit(0)
