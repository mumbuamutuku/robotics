import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np

class CartographerSlamNode(Node):
    def __init__(self):
        super().__init__('cartographer_slam_node')
        # Subscriptions
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/robot/pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        # Dummy state
        self.pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.map = np.zeros((100, 100))  # 2D grid

    def lidar_callback(self, msg):
        # Simulate Cartographer scan matching
        ranges = np.random.rand(360)  # Dummy LiDAR ranges
        self.update_slam(ranges)

    def imu_callback(self, msg):
        # Simulate IMU orientation
        orientation = np.array([msg.orientation.z])
        self.update_slam(orientation=orientation)

    def odom_callback(self, msg):
        # Simulate odometry update
        odom = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.update_slam(odom=odom)

    def update_slam(self, ranges=None, orientation=None, odom=None):
        # Simplified SLAM: Update pose and map
        if ranges is not None:
            self.pose += np.random.normal(0, 0.1, 3)  # Random walk
        if orientation is not None:
            self.map += np.random.normal(0, 0.01, self.map.shape)  # Update map
        self.publish_pose()
        self.publish_map()

    def publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = self.pose[0]
        pose_msg.pose.position.y = self.pose[1]
        pose_msg.pose.orientation.z = np.sin(self.pose[2] / 2)
        pose_msg.pose.orientation.w = np.cos(self.pose[2] / 2)
        self.pose_pub.publish(pose_msg)

    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.info.width = self.map.shape[0]
        map_msg.info.height = self.map.shape[1]
        map_msg.info.resolution = 0.1
        map_msg.data = (self.map.flatten() * 100).astype(int).tolist()
        self.map_pub.publish(map_msg)

def main():
    rclpy.init()
    node = CartographerSlamNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()