#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file train_transformer.py
@brief 训练基于Transformer的轨迹预测模型
"""

import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader

# 导入分离后的数据集合模型
from transformer_dataset import DummyTrajectoryDataset
from transformer_model import TransformerPredictor

def train():
    # 自动获取当前可用的计算设备（如果安装了 GPU 及 CUDA 则优先使用 GPU 加速，否则退回 CPU 计算）
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"[Training Info] Using computational device: {device}")
    
    # 设定超参数：总训练轮数（Epoch）、每批次样本体积（Batch Size）、优化学习率（Learning Rate）
    epochs = 15
    batch_size = 64
    learning_rate = 1e-3

    # 实例化测试级别的虚拟物理训练集（包含一万条携带卡尔曼环境漂移噪声的行人预测模型轨迹，历史抽取10帧，未来预测20帧）
    dataset = DummyTrajectoryDataset(num_samples=10000, hist_len=10, fut_len=20, feat_dim=6)
    # 用 DataLoader 封装管理数据，开启随机打乱(shuffle=True)增强模型抗噪泛化，启用4条后勤线程并发读取
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=4)

    # 实例化 Transformer 前瞻预测网络模型，向其阐述：输入特征为 6 维，输出亦为 6 维（3D运动坐标 + 3D不确定方差）
    model = TransformerPredictor(
        feat_dim=6, out_dim=6, d_model=64, nhead=4,
        num_encoder_layers=3, num_decoder_layers=3,
        dim_feedforward=256, dropout=0.1, fut_len=20
    ).to(device) # 将搭建完毕的所有神经元结构抛送进显卡/内存算区

    # 选择高斯负对数似然损失函数，用以同时优化预测坐标体系(Mean)并评判自回退补偿的不确定度预测(Variance)
    criterion = nn.GaussianNLLLoss()
    # 选用业界主流的 Adam 梯度优化器承接负责神经网络参数集的实时推挤更新
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)

    print("[Training Info] Start training transformer predictor...")

    # 最外围生命核心循环，统筹整体网络要将所有数据集咀嚼多少轮次才结束
    for epoch in range(epochs):
        model.train() # 指挥预埋模块使模型退回到训练模式（会激起并启用 Dropout 层和 BatchNorm 特征）
        epoch_loss = 0.0

        # 从加载循环管道中遍历抓取每一个批次里的历史散点源组(source) 与 未来对齐靶向组(target)
        for batch_idx, (src_batch, tgt_batch) in enumerate(dataloader):
            src_batch = src_batch.to(device) # 把这片输入内存运移投递给计算池
            
            # 手动切片缩减真理目标的参考值维度（我们只需核对未来预估得准不准位置，所以只须对照前三维物理坐标点）
            tgt_batch = tgt_batch.to(device)[:, :, :3]

            # 在执行任何反向梯度传播算术前，切记向优化机发起一波梯度清空以切断遗留梯度的叠加污染串烧
            optimizer.zero_grad()
            # 大手一推，令网络模型消化源数据并当堂给出一次正向推理计算，下发多步推演集结果 preds
            preds = model(src_batch) 
            
            # 由于模型现在返回维数为 6 维，我们在这里当即把它拆作 Mean(前3位的预想xyz)与 Std(后3位的标准差方差基)
            mean = preds[:, :, 0:3]
            std = preds[:, :, 3:6]
            
            # Pytorch 底层的 GaussianNLLLoss 要求最后一个投喂入的测参变量是纯 Variance(平方方差而非标准差)，予以平方翻倍转换！
            var = std ** 2
            
            # 以算入的损失函数求该批次的均值大错漏！这套函数判定其走位越跟真实重合并其方差分布评估越稳健，扣损就越轻。
            loss = criterion(mean, tgt_batch, var)
            
            # 反其道而行！开始调用梯度的微积分自动链式反向求导法则 (算出每个神经节究竟在这个错漏里推波助澜了哪些锅)
            loss.backward()
            # 鞭策优化器借用上行刚解出的这帮反导系数步进一下去实质更新修改神经元里的核心实化权值
            optimizer.step()

            # 将本片段的 Loss 余值抽出，用作整个回合运转耗完后的全局宏观均值评价
            epoch_loss += loss.item()

            # 设置个简单的屏显监控触发器：每成功滚过转 50 个大批次就上报刷新一次目前所处进程与损失跳频
            if (batch_idx + 1) % 50 == 0:
                print(f"[Epoch {epoch+1}/{epochs}] | Batch {batch_idx+1}/{len(dataloader)} | NLL Loss: {loss.item():.4f}")

        # 遍历完整轮(Epoch) 盘点结束，把整体累积平均后输出本圈综合退化表现报告
        avg_loss = epoch_loss / len(dataloader)
        print(f"====> Epoch {epoch+1} Complete. Average NLL Loss = {avg_loss:.4f} <====")

    # 寻获该脚原本尊目前所处绝对根系路径，将打算盘出最后生成的网络“重量存储包”名妥妥安加在它近旁
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "weights.pth")
    # 令 Pytorch 去下发封榜固化！正式刻录进磁盘形成网络权值字典保存档。
    torch.save(model.state_dict(), save_path)
    print(f"\n[Success] Model state strictly saved to: {save_path}")

if __name__ == "__main__":
    # 进入程序的最高触发激活命令函数
    train()
