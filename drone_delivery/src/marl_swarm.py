import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import torch
import torch.nn as nn

class PPOAgent:
    def __init__(self):
        self.policy = nn.Sequential(nn.Linear(15, 64), nn.ReLU(), nn.Linear(64, 3))  # Simplified
        self.value = nn.Sequential(nn.Linear(15, 64), nn.ReLU(), nn.Linear(64, 1))

    def act(self, state):
        return self.policy(torch.from_numpy(state).float()).detach().numpy()

class SwarmCoordinationNode(Node):
    def __init__(self):
        super().__init__('swarm_coordination_node')
        # Subscriptions
        self.neighbor_sub = self.create_subscription(PoseStamped, '/v2v/neighbors', self.neighbor_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/drone/pose', self.pose_callback, 10)
        # Publisher
        self.action_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # PPO agent
        self.agent = PPOAgent()
        self.state = np.zeros(15)  # [self_pose, self_vel, neighbor_pose, neighbor_vel]
        self.neighbors = []

    def neighbor_callback(self, msg):
        self.neighbors.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def pose_callback(self, msg):
        self.state[:3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.state[3:6] = np.random.rand(3)  # Dummy velocity
        if self.neighbors:
            self.state[6:9] = self.neighbors[0]  # Closest neighbor
            self.state[9:12] = np.random.rand(3)  # Neighbor velocity
        action = self.agent.act(self.state)
        self.publish_action(action)

    def publish_action(self, action):
        twist = Twist()
        twist.linear.x = action[0]
        twist.linear.y = action[1]
        twist.linear.z = action[2]
        self.action_pub.publish(twist)

def main():
    rclpy.init()
    node = SwarmCoordinationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()