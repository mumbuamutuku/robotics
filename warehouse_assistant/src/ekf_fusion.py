import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np

class EKFFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_fusion_node')
        # Subscriptions
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.rgb_sub = self.create_subscription(Image, '/rgb/image_raw', self.rgb_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/fused_pose', 10)
        # EKF state
        self.state = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.covariance = np.eye(3) * 0.1

    def lidar_callback(self, msg):
        # Dummy measurement update
        measurement = np.array([self.state[0] + np.random.normal(0, 0.1),
                               self.state[1] + np.random.normal(0, 0.1)])
        self.update_ekf(measurement, np.eye(2) * 0.05)

    def imu_callback(self, msg):
        # Dummy angular velocity
        measurement = np.array([msg.angular_velocity.z])
        self.update_ekf(measurement, np.array([[0.01]]))

    def rgb_callback(self, msg):
        # Placeholder for RGB-D data
        pass

    def odom_callback(self, msg):
        # Dummy odometry
        measurement = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.update_ekf(measurement, np.eye(2) * 0.05)

    def update_ekf(self, measurement, meas_cov):
        # Simplified EKF update (prediction + correction)
        # Prediction
        dt = 0.1
        F = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # State transition
        self.state = F @ self.state
        self.covariance = F @ self.covariance @ F.T + np.eye(3) * 0.01
        
        # Update
        H = np.array([[1, 0, 0], [0, 1, 0]])  # Measurement model (for position)
        innovation = measurement - H @ self.state[:2]
        S = H @ self.covariance @ H.T + meas_cov
        K = self.covariance @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ innovation
        self.covariance = (np.eye(3) - K @ H) @ self.covariance
        
        self.publish_pose()

    def publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = self.state[0]
        pose_msg.pose.position.y = self.state[1]
        pose_msg.pose.orientation.z = np.sin(self.state[2] / 2)
        pose_msg.pose.orientation.w = np.cos(self.state[2] / 2)
        self.pose_pub.publish(pose_msg)

def main():
    rclpy.init()
    node = EKFFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()