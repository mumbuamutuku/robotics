import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import Pose
import numpy as np
import matplotlib.pyplot as plt

class EvaluationKPIsNode(Node):
    def __init__(self):
        super().__init__('evaluation_kpis_node')
        # Subscriptions
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.detection_sub = self.create_subscription(Detection3DArray, '/obstacles', self.detection_callback, 10)
        self.grasp_sub = self.create_subscription(Pose, '/grasp_pose', self.grasp_callback, 10)
        # KPI storage
        self.path_lengths = []
        self.detections = []
        self.grasp_success = []

    def path_callback(self, msg):
        # Compute path length
        length = 0.0
        for i in range(1, len(msg.poses)):
            p1 = msg.poses[i-1].pose.position
            p2 = msg.poses[i].pose.position
            length += np.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        self.path_lengths.append(length)
        self.visualize_kpis()

    def detection_callback(self, msg):
        # Dummy precision/recall
        precision = np.random.rand()  # Simulated
        recall = np.random.rand()
        self.detections.append((precision, recall))
        self.visualize_kpis()

    def grasp_callback(self, msg):
        # Dummy grasp success
        success = np.random.choice([True, False])
        self.grasp_success.append(success)
        self.visualize_kpis()

    def visualize_kpis(self):
        plt.figure(figsize=(12, 4))
        
        # Path length
        plt.subplot(131)
        plt.plot(self.path_lengths)
        plt.title('Path Length')
        plt.xlabel('Trial')
        plt.ylabel('Length (m)')
        
        # Detection accuracy
        plt.subplot(132)
        if self.detections:
            precisions, recalls = zip(*self.detections)
            plt.scatter(precisions, recalls)
            plt.title('Detection Precision/Recall')
            plt.xlabel('Precision')
            plt.ylabel('Recall')
        
        # Grasp success
        plt.subplot(133)
        plt.plot(np.cumsum(self.grasp_success) / (np.arange(len(self.grasp_success)) + 1))
        plt.title('Grasp Success Rate')
        plt.xlabel('Trial')
        plt.ylabel('Success Rate')
        
        plt.tight_layout()
        plt.savefig('kpis.png')
        plt.close()

def main():
    rclpy.init()
    node = EvaluationKPIsNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()