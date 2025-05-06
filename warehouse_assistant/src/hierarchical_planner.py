import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray
import numpy as np

class HierarchicalPlannerNode(Node):
    def __init__(self):
        super().__init__('hierarchical_planner_node')
        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/robot/pose', self.pose_callback, 10)
        self.obstacle_sub = self.create_subscription(Detection3DArray, '/obstacles', self.obstacle_callback, 10)
        # Publisher
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        # State
        self.map = None
        self.pose = np.array([0.0, 0.0])
        self.goal = np.array([10.0, 10.0])  # Dummy goal
        self.obstacles = []

    def map_callback(self, msg):
        self.map = np.array(msg.data).reshape(msg.info.height, msg.info.width)

    def pose_callback(self, msg):
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y])
        self.plan_path()

    def obstacle_callback(self, msg):
        self.obstacles = [(det.bbox.center.position.x, det.bbox.center.position.y)
                          for det in msg.detections]
        self.plan_path()

    def plan_path(self):
        if self.map is None:
            return
        
        # Simplified A* for global path
        path = [self.pose]
        for _ in range(10):  # Dummy path to goal
            step = (self.goal - path[-1]) * 0.1
            path.append(path[-1] + step)
        
        # DWA for local avoidance
        for i in range(1, len(path)):
            for obs in self.obstacles:
                dist = np.linalg.norm(path[i] - obs)
                if dist < 1.0:  # Avoid obstacle
                    path[i] += np.random.normal(0, 0.5, 2)  # Simplified avoidance
        
        self.publish_path(path)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for p in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

def main():
    rclpy.init()
    node = HierarchicalPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()