import numpy as np
import torch
from torch.utils.data import Dataset

class DummyTrajectoryDataset(Dataset):
    """
    模拟ETH/UCY行人轨迹数据集的PyTorch Dataset
    目标:
        1. 随机生成行人行走的真实轨迹 (以直线或正弦曲线类比真实运动规律)
        2. 给输入(历史观测H=10)加入卡尔曼滤波残差级别的多变量高斯噪声 N(0, P_kf)
        3. 让输出(未来预测F=20)保持相对平滑无噪声，以此让模型学习抗噪推断
    """
    def __init__(self, num_samples=5000, hist_len=10, fut_len=20, feat_dim=6):
        super().__init__()
        self.num_samples = num_samples # 数据集的样本总数
        self.H = hist_len              # H=10
        self.F = fut_len              # F=20
        self.feat_dim = feat_dim

        P_kf_diag = [0.05, 0.05, 0.01]
        self.P_kf = np.diag(P_kf_diag)

    def __len__(self):
        return self.num_samples

    def __getitem__(self, idx):
        seq_len = self.H + self.F
        t = np.arange(seq_len)
        
        vx = np.random.uniform(-1.5, 1.5)
        vy = np.random.uniform(-1.5, 1.5)
        start_x = np.random.uniform(-10.0, 10.0)
        start_y = np.random.uniform(-10.0, 10.0)
        z_height = np.random.uniform(0.0, 2.0)
        
        gt_x = start_x + vx * t + np.sin(t * 0.2) * np.random.uniform(0, 0.5)
        gt_y = start_y + vy * t + np.cos(t * 0.2) * np.random.uniform(0, 0.5)
        gt_z = np.full_like(t, z_height, dtype=float) 
        
        # Generate [x, y, z] trajectory
        gt_traj = np.stack([gt_x, gt_y, gt_z], axis=-1)

        # Generate [vx, vy, vz] velocities (simple numerical derivative)
        gt_vel = np.zeros_like(gt_traj)
        gt_vel[1:] = gt_traj[1:] - gt_traj[:-1]
        gt_vel[0] = gt_vel[1]
        
        # Combine pos and vel into 6-dim states
        gt_state = np.concatenate([gt_traj, gt_vel], axis=-1)

        hist_clean = gt_state[:self.H, :] 
        noise_pos = np.random.multivariate_normal(mean=[0, 0, 0], cov=self.P_kf, size=self.H)
        noise_vel = np.random.multivariate_normal(mean=[0, 0, 0], cov=self.P_kf, size=self.H)
        noise = np.concatenate([noise_pos, noise_vel], axis=-1)
        hist_noisy = hist_clean + noise
        
        # Future ground truth only needs [x, y, z] (3-dim output)
        future_gt = gt_traj[self.H:, :] 
        
        src_tensor = torch.tensor(hist_noisy, dtype=torch.float32)
        tgt_tensor = torch.tensor(future_gt, dtype=torch.float32)
        
        return src_tensor, tgt_tensor
