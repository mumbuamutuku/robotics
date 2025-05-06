import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import torch
import torch.nn as nn

class DDPGAgent:
    def __init__(self):
        self.actor = nn.Sequential(nn.Linear(9, 64), nn.ReLU(), nn.Linear(64, 3))  # Simplified
        self.critic = nn.Sequential(nn.Linear(12, 64), nn.ReLU(), nn.Linear(64, 1))

    def act(self, state):
        return self.actor(torch.from_numpy(state).float()).detach().numpy()

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        # Subscriptions
        self.obstacle_sub = self.create_subscription(Detection3DArray, '/obstacles', self.obstacle_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/drone/pose', self.pose_callback, 10)
        # Publisher
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        # DDPG agent
        self.agent = DDPGAgent()
        self.state = np.zeros(9)  # [pose, velocity, goal]
        self.obstacles = []

    def obstacle_callback(self, msg):
        self.obstacles = [(det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.center.position.z)
                          for det in msg.detections]

    def pose_callback(self, msg):
        self.state[:3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # Simulate velocity and goal
        self.state[3:6] = np.random.rand(3)
        self.state[6:9] = [10.0, 10.0, 5.0]  # Dummy goal
        action = self.agent.act(self.state)
        self.publish_path(action)

    def publish_path(self, action):
        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = self.state[0] + action[0]
        pose.pose.position.y = self.state[1] + action[1]
        pose.pose.position.z = self.state[2] + action[2]
        path.poses.append(pose)
        self.path_pub.publish(path)

def main():
    rclpy.init()
    node = PathPlanningNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()