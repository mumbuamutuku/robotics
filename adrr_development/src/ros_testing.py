import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from nav_msgs.msg import Path
from std_msgs.msg import String
import numpy as np

class TestingNode(Node):
    def __init__(self):
        super().__init__('testing_node')
        # Subscriptions
        self.detection_sub = self.create_subscription(Detection3DArray, '/obstacles', self.detection_callback, 10)
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.action_sub = self.create_subscription(String, '/ethical_action', self.action_callback, 10)
        # Metrics
        self.metrics = {'precision': [], 'path_length': [], 'ethical_correctness': []}

    def detection_callback(self, msg):
        # Dummy precision calculation
        precision = np.random.rand()  # Simulated
        self.metrics['precision'].append(precision)
        self.log_metrics()

    def path_callback(self, msg):
        # Compute path length
        length = 0.0
        for i in range(1, len(msg.poses)):
            p1 = msg.poses[i-1].pose.position
            p2 = msg.poses[i].pose.position
            length += np.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)
        self.metrics['path_length'].append(length)
        self.log_metrics()

    def action_callback(self, msg):
        # Dummy ethical correctness
        correctness = 1.0 if msg.data == 'rescue' else 0.5  # Simplified
        self.metrics['ethical_correctness'].append(correctness)
        self.log_metrics()

    def log_metrics(self):
        with open('test_report.txt', 'w') as f:
            for metric, values in self.metrics.items():
                if values:
                    f.write(f"{metric}: {np.mean(values):.2f} ± {np.std(values):.2f}\n")

def main():
    rclpy.init()
    node = TestingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()