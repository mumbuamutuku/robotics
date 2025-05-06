import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
import numpy as np

class EnergySchedulerNode(Node):
    def __init__(self):
        super().__init__('energy_scheduler_node')
        # Subscription
        self.battery_sub = self.create_subscription(BatteryState, '/battery', self.battery_callback, 10)
        # Publishers
        self.yolo_pub = self.create_publisher(Bool, '/yolo_active', 10)
        self.ddpg_pub = self.create_publisher(Bool, '/ddpg_active', 10)
        # Model costs (Watts)
        self.model_costs = {'yolo': 10.0, 'ddpg': 5.0}
        self.battery_level = 1.0

    def battery_callback(self, msg):
        self.battery_level = msg.percentage
        self.schedule_models()

    def schedule_models(self):
        # Simple scheduler: Activate models if battery allows
        available_power = self.battery_level * 20.0  # Max 20W
        yolo_active = available_power >= self.model_costs['yolo']
        ddpg_active = available_power >= self.model_costs['yolo'] + self.model_costs['ddpg']
        
        # Publish activation signals
        yolo_msg = Bool()
        yolo_msg.data = yolo_active
        self.yolo_pub.publish(yolo_msg)
        
        ddpg_msg = Bool()
        ddpg_msg.data = ddpg_active
        self.ddpg_pub.publish(ddpg_msg)

def main():
    rclpy.init()
    node = EnergySchedulerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()