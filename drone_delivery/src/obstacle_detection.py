import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection3DArray, Detection3D
import numpy as np
import torch
from torchvision.models.detection import mobilenet_v3_small

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')
        # Subscriptions
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/lidar/points', self.lidar_callback, 10)
        # Publisher
        self.detection_pub = self.create_publisher(Detection3DArray, '/obstacles', 10)
        # MobileNetV3 model (simplified)
        self.model = mobilenet_v3_small(pretrained=True).eval()

    def camera_callback(self, msg):
        # Simulate image (dummy RGB)
        image = np.random.rand(224, 224, 3).astype(np.float32)
        image_tensor = torch.from_numpy(image).permute(2, 0, 1).unsqueeze(0)
        with torch.no_grad():
            detections = self.model(image_tensor)
        self.publish_detections(detections)

    def lidar_callback(self, msg):
        # Simulate PointNet (dummy point cloud)
        points = np.random.rand(1000, 3)  # [x, y, z]
        # Placeholder: Assume clustering for detection
        centroids = np.mean(points.reshape(10, 100, 3), axis=1)
        detections = [{'boxes': torch.tensor(centroids), 'scores': torch.ones(len(centroids))}]
        self.publish_detections(detections)

    def publish_detections(self, detections):
        det_array = Detection3DArray()
        det_array.header.frame_id = 'drone'
        det_array.header.stamp = self.get_clock().now().to_msg()
        for det in detections:
            for box, score in zip(det['boxes'], det['scores']):
                detection = Detection3D()
                detection.bbox.center.position.x = float(box[0])
                detection.bbox.center.position.y = float(box[1])
                detection.bbox.center.position.z = float(box[2])
                detection.bbox.size.x = 0.5  # Dummy size
                detection.bbox.size.y = 0.5
                detection.bbox.size.z = 0.5
                detection.results.append({'hypothesis': {'score': float(score)}})
                det_array.detections.append(detection)
        self.detection_pub.publish(det_array)

def main():
    rclpy.init()
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()