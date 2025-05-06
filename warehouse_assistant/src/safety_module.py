import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import torch
import torch.nn as nn

class RNNPredictor(nn.Module):
    def __init__(self):
        super(RNNPredictor, self).__init__()
        self.rnn = nn.LSTM(2, 16, 1)
        self.fc = nn.Linear(16, 2)

    def forward(self, x):
        _, (h, _) = self.rnn(x)
        return self.fc(h.squeeze(0))

class SafetyModuleNode(Node):
    def __init__(self):
        super().__init__('safety_module_node')
        # Subscriptions
        self.obstacle_sub = self.create_subscription(Detection3DArray, '/obstacles', self.obstacle_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/robot/pose', self.pose_callback, 10)
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # RNN predictor
        self.predictor = RNNPredictor().eval()
        self.costmap = np.zeros((100, 100))  # 2D grid
        self.robot_pose = np.array([0.0, 0.0])

    def obstacle_callback(self, msg):
        humans = [(det.bbox.center.position.x, det.bbox.center.position.y)
                  for det in msg.detections if det.results[0].hypothesis.class_id == 'human']
        self.update_costmap(humans)
        self.adjust_velocity()

    def pose_callback(self, msg):
        self.robot_pose = np.array([msg.pose.position.x, msg.pose.position.y])

    def update_costmap(self, humans):
        self.costmap.fill(0)
        for human in humans:
            # Dummy Gaussian cost
            x, y = int(human[0] / 0.1), int(human[1] / 0.1)
            if 0 <= x < 100 and 0 <= y < 100:
                self.costmap[x, y] = 100
                # Predict trajectory (simplified)
                trajectory = torch.tensor([[human[0], human[1]]], dtype=torch.float32)
                pred = self.predictor(trajectory)
                px, py = int(pred[0] / 0.1), int(pred[1] / 0.1)
                if 0 <= px < 100 and 0 <= py < 100:
                    self.costmap[px, py] = 50

    def adjust_velocity(self):
        # Simplified velocity adjustment based on costmap
        rx, ry = int(self.robot_pose[0] / 0.1), int(self.robot_pose[1] / 0.1)
        if 0 <= rx < 100 and 0 <= ry < 100 and self.costmap[rx, ry] > 0:
            speed = 0.5 / (1 + self.costmap[rx, ry] / 100)  # Reduce speed
        else:
            speed = 1.0
        cmd = Twist()
        cmd.linear.x = speed
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = SafetyModuleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()