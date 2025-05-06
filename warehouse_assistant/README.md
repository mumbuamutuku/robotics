# Warehouse Assistant Robot

A comprehensive robotic system for warehouse automation, combining computer vision, SLAM, motion planning, and safety features.

## System Components

### Computer Vision
- **vit_finetune.py**: Fine-tunes a Vision Transformer (ViT) model for detecting warehouse items (e.g., boxes, packages) using RGB-D camera data.

### Localization and Mapping
- **cartographer_slam.py**: ROS Node implementing Cartographer-based SLAM to localize the robot and map the warehouse using LiDAR, IMU, and wheel odometry.
- **ekf_fusion.py**: Fuses LiDAR, RGB-D, IMU, and wheel encoder data using an Extended Kalman Filter (EKF) for robust localization.

### Motion Planning
- **hierarchical_planner.py**: Implements a hierarchical planner combining A* for global planning and Dynamic Window Approach (DWA) for local obstacle avoidance.
- **rl_grasp.py**: Trains a Deep Deterministic Policy Gradient (DDPG) agent to optimize robotic arm grasping for varied item shapes.

### Safety and Evaluation
- **safety_module.py**: Ensures safe human-robot interaction by adjusting paths and speeds based on costmaps and predicted human trajectories.
- **evaluation_kpis.py**: Logs and visualizes key performance indicators (KPIs) like navigation efficiency, detection accuracy, and grasp success.

## Requirements
- Ubuntu 20.04/22.04 (ROS 2 Humble installed)
- Python 3.8+
- Machine learning Libraries: PyTorch, TensorFlow, torchvision, OpenCV, NumPy, SciPy
- ROS 2 packages: rclpy, sensor_msgs, nav_msgs, geometry_msgs, vision_msgs, tf2_ros
- Navigation: nav2, moveit

## Installation
- Create a ROS 2 workspace: mkdir -p ~/warehouse_ws/src && cd ~/warehouse_ws
- Clone the repository: git clone https://github.com/mumbuamutuku/Warehouse_Assistant.git ~/warehouse_ws/src
- Build: colcon build
- Source: source install/setup.bash

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
