import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from std_msgs.msg import String
import numpy as np

class EthicalMDP:
    def __init__(self):
        self.states = ['safe', 'hazardous']  # Simplified states
        self.actions = ['rescue', 'avoid', 'wait']
        self.transition_prob = {s: {a: {s2: 0.33 for s2 in self.states} for a in self.actions} for s in self.states}
        self.rewards = {
            'safe': {'rescue': 10, 'avoid': 0, 'wait': -1},
            'hazardous': {'rescue': -10, 'avoid': 5, 'wait': -2}
        }
        self.policy = self.value_iteration()

    def value_iteration(self, gamma=0.9, theta=1e-6):
        V = {s: 0.0 for s in self.states}
        while True:
            delta = 0
            for s in self.states:
                v = V[s]
                V[s] = max(sum(self.transition_prob[s][a][s2] * (self.rewards[s][a] + gamma * V[s2])
                               for s2 in self.states) for a in self.actions)
                delta = max(delta, abs(v - V[s]))
            if delta < theta:
                break
        policy = {s: max(self.actions, key=lambda a: sum(self.transition_prob[s][a][s2] * (self.rewards[s][a] + gamma * V[s2])
                                                         for s2 in self.states)) for s in self.states}
        return policy

class EthicalDecisionNode(Node):
    def __init__(self):
        super().__init__('ethical_decision_node')
        # Subscription
        self.hazard_sub = self.create_subscription(Detection3DArray, '/hazards', self.hazard_callback, 10)
        # Publisher
        self.action_pub = self.create_publisher(String, '/ethical_action', 10)
        # MDP
        self.mdp = EthicalMDP()
        self.current_state = 'safe'

    def hazard_callback(self, msg):
        # Dummy state update
        self.current_state = 'hazardous' if msg.detections else 'safe'
        action = self.mdp.policy[self.current_state]
        action_msg = String()
        action_msg.data = action
        self.action_pub.publish(action_msg)

def main():
    rclpy.init()
    node = EthicalDecisionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()