import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np

class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        # Subscriptions
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/drone/map', 10)
        # Dummy state
        self.pose = np.array([0.0, 0.0, 0.0])  # [x, y, z]
        self.map = np.zeros((100, 100))  # Simplified 2D grid

    def camera_callback(self, msg):
        # Simulate ORB-SLAM3 feature extraction
        features = np.random.rand(100, 2)  # Dummy keypoints
        self.update_slam(features)

    def imu_callback(self, msg):
        # Simulate IMU orientation
        orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z])
        self.update_slam(orientation=orientation)

    def update_slam(self, features=None, orientation=None):
        # Simplified SLAM: Update pose and map
        if features is not None:
            self.pose += np.random.normal(0, 0.1, 3)  # Random walk for demo
        if orientation is not None:
            self.map += np.random.normal(0, 0.01, self.map.shape)  # Update map
        self.publish_pose()
        self.publish_map()

    def publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'world'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = self.pose[0]
        pose_msg.pose.position.y = self.pose[1]
        pose_msg.pose.position.z = self.pose[2]
        self.pose_pub.publish(pose_msg)

    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'world'
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.info.width = self.map.shape[0]
        map_msg.info.height = self.map.shape[1]
        map_msg.data = (self.map.flatten() * 100).astype(int).tolist()
        self.map_pub.publish(map_msg)

def main():
    rclpy.init()
    node = SlamNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()