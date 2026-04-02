#!/usr/bin/env python3

import rospy
import torch
import torch.nn as nn
from std_msgs.msg import Float32MultiArray

class TrajectoryTransformer(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder = nn.TransformerEncoderLayer(d_model=6, nhead=2)
        self.fc = nn.Linear(6, 3)
    def forward(self, x):
        x = self.encoder(x)
        return self.fc(x)

class PredictorNode:
    def __init__(self):
        self.model = TrajectoryTransformer()
        self.model.eval()
        self.horizon = rospy.get_param("~horizon", 10)
        self.dt = rospy.get_param("~dt", 0.1)

        self.sub = rospy.Subscriber('/mot_tracks', Float32MultiArray, self.callback, queue_size=10)
        self.pub = rospy.Publisher('/predicted_trajectories', Float32MultiArray, queue_size=10)

    def callback(self, msg):
        # 输入格式: [id, x, y, z, size, ...]
        if len(msg.data) < 5:
            return

        out = Float32MultiArray()
        for i in range(0, len(msg.data), 5):
            if i + 4 >= len(msg.data):
                break
            track_id = msg.data[i]
            x = msg.data[i + 1]
            y = msg.data[i + 2]
            z = msg.data[i + 3]

            # 轻量占位预测：保持匀速先验(当前无速度输入，默认零速度)
            # 输出格式: [id, t, x, y, z, id, t, x, y, z, ...]
            for k in range(self.horizon):
                t = (k + 1) * self.dt
                out.data.extend([track_id, t, x, y, z])

        self.pub.publish(out)
        rospy.loginfo_throttle(1.0, "traj_prediction: input_tracks=%d, output_points=%d",
                               len(msg.data) // 5, len(out.data) // 5)

if __name__ == '__main__':
    rospy.init_node('traj_prediction_node')
    PredictorNode()
    rospy.spin()
