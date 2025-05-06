# ADRR (Autonomous Disaster Response Robot)

An autonomous robot system for disaster response and rescue operations, featuring multimodal object detection, ethical decision-making, and energy-efficient navigation.

## Prerequisites

Environment: Ubuntu 20.04/22.04 with ROS 2 Humble installed.

Dependencies:
- Python 3.8+
- ROS 2 packages: rclpy, sensor_msgs, nav_msgs, geometry_msgs, vision_msgs, tf2_ros
- Machine learning libraries: torch, ultralytics (for YOLOv8), numpy
- Optimization: cvxpy
- Simulation: Gazebo (optional for testing)
- Testing: pytest, ros2_bag


## Project Structure

The project consists of several key components:

1. `yolo_finetune.py`: Fine-Tuning YOLOv8 for Multimodal Object Detection
   - Purpose: Fine-tunes a YOLOv8 model for detecting objects (e.g., survivors, debris, hazards) in flooded areas using multimodal data (RGB and depth images).

2. `ddpg_navigation.py`: DDPG Implementation for Navigation in Flooded Areas
   - Purpose: Implements a Deep Deterministic Policy Gradient (DDPG) agent for navigating through dynamic, flooded environments, avoiding obstacles and optimizing paths.

3. `ethical_mdp.py`: MDP-Based Ethical Decision-Making Framework
   - Purpose: Implements a Markov Decision Process (MDP) to make ethical decisions, prioritizing survivor rescue while minimizing harm (e.g., avoiding unstable structures).

4. `energy_scheduler.py`: Dynamic Model Scheduler for Energy Optimization
   - Purpose: Schedules AI model execution (e.g., YOLOv8, DDPG) to optimize energy consumption based on computational load and battery state.

5. `ros_testing.py`: ROS-Based Testing Framework for Performance Evaluation
   - Purpose: Implements a testing framework to evaluate ADRR performance (e.g., detection accuracy, navigation efficiency, ethical decisions) using ROS 2.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- YOLOv8 team for the object detection framework
- ROS 2 community for the robotics middleware
- Contributors and maintainers of the project