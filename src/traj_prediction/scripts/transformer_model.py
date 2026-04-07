import math
import torch
import torch.nn as nn

class PositionalEncoding(nn.Module):
    """
    位置编码模块 (Positional Encoding)
    Transformer 无递归结构，需要此模块向输入序列中显式注入绝对时间位置信息
    """
    def __init__(self, d_model: int, dropout: float = 0.1, max_len: int = 5000):
        super(PositionalEncoding, self).__init__()
        # 随机丢弃部分神经元，防止记忆过拟合并增加模型的泛化鲁棒性
        self.dropout = nn.Dropout(p=dropout)
        
        # 初始化一个全零矩阵，用于存放按照正余弦公式计算好的位置编码 (共 max_len 个点，每个点 d_model 维)
        pe = torch.zeros(max_len, d_model)
        # 生成形如 [[0], [1], ..., [max_len-1]] 的位置索引列向量
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        # 根据 Transformer 论文中的缩放公式求出不同维度的分母除数张量
        div_term = torch.exp(torch.arange(0, d_model, 2).float() * (-math.log(10000.0) / d_model))
        
        # 偶数维度（0,2,4...）填入正弦 (Sin) 编码
        pe[:, 0::2] = torch.sin(position * div_term)
        # 奇数维度（1,3,5...）填入余弦 (Cos) 编码
        pe[:, 1::2] = torch.cos(position * div_term)
        
        # 增加一个 Batch 维度并转置，使形状符合 Pytorch 特化的 [max_len, 1, d_model]
        pe = pe.unsqueeze(0).transpose(0, 1)
        # 注册为 buffer：这样它会被一并保存在模型的 weights.pth 字典中，但不会被梯度的回传优化干预修改
        self.register_buffer('pe', pe)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # 截取与本轮输入序列 x 一样长的时间步编码，硬叠加到 x 上进行时序特征融合
        x = x + self.pe[:x.size(0), :]
        # 经过 dropout 操作后返回融合特征
        return self.dropout(x)

class TransformerPredictor(nn.Module):
    """
    基于 Transformer 的编解码预测器核心网络 (Encoder-Decoder)
    """
    def __init__(self, feat_dim=6, out_dim=6, d_model=64, nhead=4, num_encoder_layers=3, 
                 num_decoder_layers=3, dim_feedforward=256, dropout=0.1, fut_len=20):
        super(TransformerPredictor, self).__init__()
        self.feat_dim = feat_dim
        self.d_model = d_model
        self.fut_len = fut_len

        # 特征升维映射层：将 6 维底层物理状态（坐标/速度等）扩充映射到高维 (d_model=64) 隐藏语义空间
        self.coord_emb = nn.Linear(feat_dim, d_model)
        # 实例化先前定义好的位置编码器
        self.pos_encoder = PositionalEncoding(d_model, dropout)
        
        # 挂载 Pytorch 原生的标准 Transformer 网络主体架构 (自带多头 Self-Attention 和前馈网络层)
        self.transformer = nn.Transformer(
            d_model=d_model, nhead=nhead,
            num_encoder_layers=num_encoder_layers,
            num_decoder_layers=num_decoder_layers,
            dim_feedforward=dim_feedforward,
            dropout=dropout
        )
        
        # 解码器自学习查询锚点(Query)：负责像钥匙一样去历史记忆(Memory)库里“打捞”挖掘出未来 fut_len 个时间步的信息
        self.decoder_query = nn.Parameter(torch.randn(fut_len, 1, d_model))
        
        # 尾部输出预测器 (MLP)：将获取到的高维隐藏特征重新降维，暴露出推测的坐标与方差
        self.fc_out = nn.Sequential(
            nn.Linear(d_model, d_model // 2),  # 倒数第二层稍微压缩降维平滑化
            nn.GELU(),                         # 使用具有自限流且更加平滑的高斯误差激活函数 (GELU)
            # 注意此处：下发逻辑利用到了前 6 维截取当作 xyz与std_xyz。预留足够空间即可
            nn.Linear(d_model // 2, out_dim * 2) # 输出意图：空间位置均值（3维）+ Log-Variance（3维）
        )

    def forward(self, src: torch.Tensor) -> torch.Tensor:
        # src 的常见形状为：[batch_size, hist_len, feat_dim]
        batch_size = src.size(0)
        
        # 1. 提取出历史轨迹的最后一帧点作为参考原点 Anchor (x,y,z)，奠定后续推算的基石
        last_pos = src[:, -1, 0:3].unsqueeze(1) # 截取后得到形状保留层级 [batch, 1, 3]

        # 2. 将输入从绝对场地坐标系，强制剥离转化为相对 Anchor 的偏差距离，以此斩获“相对平移不变性”并大幅提升模型泛化力
        src_rel = src.clone()
        src_rel[:, :, 0:3] = src[:, :, 0:3] - last_pos

        # 【Encoder 区域：时空压缩】
        # 调配转换出 Pytorch Transformer 必须强行遵循的特定序列张量形状 (SeqLen, Batch, Dim)
        src_rel = src_rel.permute(1, 0, 2)
        # 输入序列经历坐标词向量投射 (Embedding) 并叠加时间节奏位置编码 (PosEncoding)
        src_emb = self.coord_emb(src_rel)
        src_emb = self.pos_encoder(src_emb)
        
        # 编码器执行自注意力密集运算，提炼出包罗全局所有历史意图的深层特征压缩矩阵 (Memory)
        memory = self.transformer.encoder(src_emb)

        # 【Decoder 区域：未来发生推理】
        # 3. 让 Decoder 端初始化使用的 Query 加上当前最后一帧的动作状态，这有助于机器从最新的运动趋势开始顺延外推
        last_obs_emb = memory[-1, :, :].unsqueeze(0) # 取出历史尽头的压缩状态 [1, batch, d_model]
        # 将随机初始化的 Query 向整个批次展开扩充，随后叠加尾挂状态产生清晰连贯的解码指示靶 (Target)
        tgt = self.decoder_query.repeat(1, batch_size, 1) + last_obs_emb
        
        # 将 Query 组 (tgt) 和 历史记忆库 (memory) 全权交由解码器执行交叉注意力推测未来发展
        out_emb = self.transformer.decoder(tgt, memory)
        
        # 将脑内深沉的隐藏思绪通过最后一层全连接网，翻译成物理学层面的真实偏移数字
        out = self.fc_out(out_emb) # shape 目前在 [fut_len, batch, out_dim*2]
        out = out.permute(1, 0, 2) # 手动调回我们惯用的舒适区排列 [batch, fut_len, out_dim*2]
        
        # 4. 从生成的通道数据带里解码：前 3 维剥出是未来步长相对锚点的 xyz 偏移位预测
        pred_rel_pos = out[:, :, 0:3]
        # 后 3 维剥出正是为避障所用的对应三轴的高斯分布自然对数方差 (Log-Variance)
        pred_log_var = out[:, :, 3:6]
        
        # 对底层的 Log-Variance 实施斩断强力限幅，遏制在 [-5, 2] 内缓冲，严守指数还原计算中可能遇难的极端数值溢出爆炸！
        pred_log_var = torch.clamp(pred_log_var, min=-5.0, max=2.0)
        # 对方差利用求 exp，然后再乘0.5(即等同化于开二次方根) 的操作，彻底解算出能给业务用的标尺数据——标准差 (Std.Dev)
        pred_std = torch.exp(0.5 * pred_log_var)

        # 5. 把刚刚进网络前减掉的历史基标点给“欲擒故纵”地补偿加回去，重新拼合出了在真实天地中的未来绝对预测直观路线
        pred_abs_pos = pred_rel_pos + last_pos
        
        # 最终阶段：将复原的未来三维绝对坐标和各自的三维标准差给并排串联拼装起来下发
        # 终极输出形状严格贴合 [batch, fut_len, 6]：涵盖 (x, y, z, std_x, std_y, std_z)
        return torch.cat([pred_abs_pos, pred_std], dim=-1)
