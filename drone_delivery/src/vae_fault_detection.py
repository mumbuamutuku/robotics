import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float32
import numpy as np
import torch
import torch.nn as nn

class VAE(nn.Module):
    def __init__(self):
        super(VAE, self).__init__()
        self.encoder = nn.Sequential(nn.Linear(10, 16), nn.ReLU(), nn.Linear(16, 8))
        self.decoder = nn.Sequential(nn.Linear(8, 16), nn.ReLU(), nn.Linear(16, 10))
    
    def forward(self, x):
        z = self.encoder(x)
        return self.decoder(z)

class FaultDetectionNode(Node):
    def __init__(self):
        super().__init__('fault_detection_node')
        # Subscriptions
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/lidar/points', self.lidar_callback, 10)
        # Publisher
        self.fault_pub = self.create_publisher(Float32, '/fault_score', 10)
        # VAE model
        self.vae = VAE().eval()
        self.data = np.zeros(10)  # [imu, lidar_subset]

    def imu_callback(self, msg):
        self.data[:4] = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                         msg.angular_velocity.z]
        self.detect_fault()

    def lidar_callback(self, msg):
        self.data[4:] = np.random.rand(6)  # Dummy LiDAR subset
        self.detect_fault()

    def detect_fault(self):
        input_tensor = torch.from_numpy(self.data).float()
        with torch.no_grad():
            recon = self.vae(input_tensor)
            error = torch.mean((recon - input_tensor) ** 2).item()
        fault_msg = Float32()
        fault_msg.data = error
        self.fault_pub.publish(fault_msg)

def main():
    rclpy.init()
    node = FaultDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()