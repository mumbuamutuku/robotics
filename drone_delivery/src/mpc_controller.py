import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import cvxpy as cp

class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')
        # Subscriptions
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/drone/pose', self.pose_callback, 10)
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # State
        self.state = np.zeros(3)  # [x, y, z]
        self.reference = np.zeros(3)  # [x_ref, y_ref, z_ref]

    def pose_callback(self, msg):
        self.state = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def path_callback(self, msg):
        if msg.poses:
            self.reference = np.array([msg.poses[0].pose.position.x,
                                      msg.poses[0].pose.position.y,
                                      msg.poses[0].pose.position.z])
        self.compute_mpc()

    def compute_mpc(self):
        # Simplified MPC: Minimize tracking error and energy
        u = cp.Variable(3)  # Control inputs [vx, vy, vz]
        cost = cp.sum_squares(self.state + u - self.reference) + 0.1 * cp.sum_squares(u)
        prob = cp.Problem(cp.Minimize(cost), [cp.norm(u, 'inf') <= 1.0])
        prob.solve()
        cmd = Twist()
        cmd.linear.x = u.value[0] if u.value is not None else 0.0
        cmd.linear.y = u.value[1] if u.value is not None else 0.0
        cmd.linear.z = u.value[2] if u.value is not None else 0.0
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = MPCControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()