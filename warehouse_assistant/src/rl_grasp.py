import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
import numpy as np
import torch
import torch.nn as nn

class DDPGAgent:
    def __init__(self):
        self.actor = nn.Sequential(nn.Linear(7, 64), nn.ReLU(), nn.Linear(64, 6))  # [pose, depth] -> [grasp_pose]
        self.critic = nn.Sequential(nn.Linear(13, 64), nn.ReLU(), nn.Linear(64, 1))

    def act(self, state):
        return self.actor(torch.from_numpy(state).float()).detach().numpy()

class GraspPlannerNode(Node):
    def __init__(self):
        super().__init__('grasp_planner_node')
        # Subscriptions
        self.item_sub = self.create_subscription(Pose, '/item_pose', self.item_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth/image_raw', self.depth_callback, 10)
        # Publisher
        self.grasp_pub = self.create_publisher(Pose, '/grasp_pose', 10)
        # DDPG agent
        self.agent = DDPGAgent()
        self.state = np.zeros(7)  # [item_pose, depth]

    def item_callback(self, msg):
        self.state[:3] = [msg.position.x, msg.position.y, msg.position.z]
        self.state[3:6] = [msg.orientation.x, msg.orientation.y, msg.orientation.z]
        self.plan_grasp()

    def depth_callback(self, msg):
        self.state[6] = np.random.rand()  # Dummy depth
        self.plan_grasp()

    def plan_grasp(self):
        if np.all(self.state == 0):
            return
        action = self.agent.act(self.state)
        grasp = Pose()
        grasp.position.x = action[0]
        grasp.position.y = action[1]
        grasp.position.z = action[2]
        grasp.orientation.x = action[3]
        grasp.orientation.y = action[4]
        grasp.orientation.z = action[5]
        self.grasp_pub.publish(grasp)

def main():
    rclpy.init()
    node = GraspPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()