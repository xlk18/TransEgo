#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file evaluate_transformer.py
@brief 评估基于Transformer的轨迹预测模型
"""

import os
import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from transformer_dataset import DummyTrajectoryDataset
from transformer_model import TransformerPredictor

def evaluate():
    # 自动检测当前是否有可用的 GPU 硬件，否则回退到 CPU 运算
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"[Evaluation Info] Using computational device: {device}")
    
    # 定义测试时的批量大小
    batch_size = 64
    # 获取当前 py 脚本所在目录，并拼接出权重文件 weights.pth 的绝对路径
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "weights.pth")

    # 检查权重文件是否存在，避免在未训练的情况下直接运行报错
    if not os.path.exists(save_path):
        print(f"[Error] Weight file not found at {save_path}. Please train the model first.")
        return

    # 实例化测试用的虚拟轨迹数据集 (设 1000 条样本用于抽样评估)
    dataset = DummyTrajectoryDataset(num_samples=1000, hist_len=10, fut_len=20, feat_dim=6)
    # 用 DataLoader 封装以便按批次并行加载数据，测试阶段无需打乱数据 (shuffle=False)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=False, num_workers=2)

    # 按照与训练时完全相同的拓扑结构参数来初始化 Transformer 模型
    model = TransformerPredictor(
        feat_dim=6, out_dim=6, d_model=64, nhead=4,
        num_encoder_layers=3, num_decoder_layers=3,
        dim_feedforward=256, dropout=0.1, fut_len=20
    ).to(device)

    # 从磁盘中反序列化加载训练好的权重字典，并适配至当前的计算设备 (防止跨设备卡顿)
    model.load_state_dict(torch.load(save_path, map_location=device))
    # 将模型切换至评估验证模式 (关闭 Dropout 和 BatchNorm 的随机性，确保输出确定稳定)
    model.eval()

    # 验证阶段主要看的是实际预测点位准不准，所以采用传统的均方误差 (MSELoss) 作为评估标准
    criterion = nn.MSELoss()
    
    print("[Evaluation Info] Start evaluating transformer predictor...")

    total_loss = 0.0
    # torch.no_grad() 上下文管理器：在此区域内不计算并且不存储梯度，大幅节约算力和显存
    with torch.no_grad():
        # 遍历数据加载器中的每一个批次数据
        for batch_idx, (src_batch, tgt_batch) in enumerate(dataloader):
            # 把历史轨迹输入张量移动至运算设备上
            src_batch = src_batch.to(device)
            # 抽离未来真实轨迹(GT)：抛弃末尾的速度或其他无关项，只取真正的 [x, y, z] 前三维用于比对
            tgt_batch = tgt_batch.to(device)[:, :, :3]

            # 执行模型推理计算，下发预测张量
            preds = model(src_batch) 
            # 模型的输出是六维 [x, y, z, std_x, std_y, std_z]，仅截取前三维物理坐标去测算绝对精度
            mean_preds = preds[:, :, 0:3]
            
            # 使用 MSELoss 计算本批次的预测平均位置偏差
            loss = criterion(mean_preds, tgt_batch)
            # 将误差数值抽出并逐步累加
            total_loss += loss.item()

    # 计算整个数据集运行下来的宏观平均损失
    avg_loss = total_loss / len(dataloader)
    print(f"====> Evaluation Complete. Average MSE Loss = {avg_loss:.4f} <====")

if __name__ == "__main__":
    # 进入该脚本的入口触发点
    evaluate()
