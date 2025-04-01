# Lep-Rover-Autonomous-Navigation
Autonomous Navigation for Leo Rover
# Leo Rover Autonomous Navigation (ROS 2)

This project enables **autonomous navigation** for a Leo Rover using **ROS 2 Humble**. It includes SLAM, path planning, motion control, IMU filtering, and multi-sensor fusion. The setup is designed for real robot deployment but can also be tested in simulation.
## 📌 Methodology

This system uses a modular ROS 2 architecture with the following components:

- **URDF + robot_state_publisher**: Publishes Leo Rover’s joint states and transforms.
- **RPLIDAR A2M12**: For 2D laser scanning.
- **SLAM Toolbox (online_async)**: Performs online mapping and localization using laser scans.
- **Nav2 stack**:
  - `planner_server`: Plans global path to goal
  - `controller_server`: Follows the path using velocity commands
  - `bt_navigator`: Executes navigation logic with behavior trees
  - `smoother_server`: Smooths paths before execution
- **robot_localization (EKF)**: Fuses IMU and odometry for better pose estimation.
- **explore_lite**: For autonomous area exploration.
- **map_saver_server**: Automatically saves the map after exploration.

> The launch file `my_leo.launch.py` integrates all modules for seamless startup.

## 📌 Demo Video
