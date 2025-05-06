# Drone Delivery System

A comprehensive autonomous drone delivery system using ROS 2, computer vision, and machine learning for urban environments.

## Overview

This project implements a sophisticated drone delivery system capable of autonomous navigation, obstacle avoidance, and swarm coordination in urban environments. It combines state-of-the-art computer vision, reinforcement learning, and control systems to enable safe and efficient drone operations.

## Prerequisites

- **Environment**: Ubuntu 20.04/22.04 with ROS 2 Humble installed
- **Dependencies**:
  - Python 3.8+
  - ROS 2 packages:
    - rclpy
    - sensor_msgs
    - nav_msgs
    - geometry_msgs
    - std_msgs
  - Machine learning libraries:
    - torch
    - torchvision
    - numpy
  - Optimization: cvxpy
  - Simulation: AirSim or Gazebo (optional for testing)


## System Architecture

The system consists of several key components:

### 1. SLAM Node (`slam_node.py`)
- Visual-Inertial SLAM using ORB-SLAM3
- Purpose: Implements visual-inertial SLAM to localize the drone and map the urban environment using camera and IMU data

### 2. Obstacle Detection (`obstacle_detection.py`)
- MobileNetV3 and PointNet for Detection
- Purpose: Detects obstacles (e.g., buildings, birds) using camera (MobileNetV3) and LiDAR (PointNet) data

### 3. Path Planning (`rl_path_planning.py`)
- DDPG-Based Path Planner
- Purpose: Implements a Deep Deterministic Policy Gradient (DDPG) agent for real-time 3D path planning, optimizing for speed, safety, and energy

### 4. Swarm Coordination (`marl_swarm.py`)
- Decentralized MARL for Swarm Coordination
- Purpose: Implements a decentralized Multi-Agent Reinforcement Learning (MARL) system for coordinating 100+ drones using Proximal Policy Optimization (PPO)

### 5. Fault Detection (`vae_fault_detection.py`)
- VAE-Based Anomaly Detection
- Purpose: Detects sensor or actuator faults using a Variational Autoencoder (VAE) by identifying anomalies in sensor data

### 6. Trajectory Control (`mpc_controller.py`)
- MPC for Energy-Efficient Trajectories
- Purpose: Optimizes drone trajectories using Model Predictive Control (MPC) to balance energy consumption and delivery time

### 7. System Integration (`drone_architecture.py`)
- ROS 2-Based Modular AI Architecture
- Purpose: Integrates perception, planning, control, and fault detection modules using ROS 2 for a cohesive drone system

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


