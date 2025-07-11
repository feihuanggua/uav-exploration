# Multi-UAV Autonomous Exploration System Guide

## 1. Project Overview

This project is based on ROS and integrates PX4 multi-UAV simulation, ground-truth pose acquisition, coordinate transformation, multi-UAV communication, 3D visualization (RVIZ), and multi-UAV exploration and control modules. It supports collaborative autonomous exploration tasks for multiple UAVs.

---

## 2. Main Functional Modules

- **PX4 Multi-UAV Simulation**: Simulate multiple UAVs using PX4 firmware and Gazebo.
- **Ground-Truth Pose Acquisition**: Obtain real-time ground-truth poses of all UAVs for subsequent coordinate transformation and point cloud processing.
- **Coordinate Transformation**: Convert UAV poses in the simulation environment to a unified coordinate system for collaboration.
- **Multi-UAV Communication**: Synchronize status and task information among UAVs.
- **3D Visualization (RVIZ)**: Real-time display of exploration process, maps, trajectories, etc.
- **Multi-UAV Exploration & Control**: Support for autonomous exploration, obstacle avoidance, path planning, and control of multiple UAVs.

---

## 3. One-Click Launch Script

You can use the following bash script to launch the entire simulation and exploration workflow with one click:

```bash
#!/bin/bash
# See the full script in scripts/haha.sh
```

### Launch Workflow

1. **PX4 Multi-UAV Simulation**  
   Launch PX4 firmware and Gazebo to simulate multiple UAVs.
2. **Ground-Truth Pose Acquisition**  
   Run the script to obtain real-time ground-truth poses of all UAVs.
3. **Coordinate Transformation**  
   Start the coordinate transformation node to unify the coordinate system.
4. **Multi-UAV Communication**  
   Start the communication node for inter-UAV messaging.
5. **RVIZ Visualization**  
   Launch RVIZ for real-time visualization.
6. **Swarm Exploration Node**  
   Start the main multi-UAV exploration node.
7. **PX4 Controller**  
   Start the multi-UAV controller node.

### Command Details

- Enter the PX4_Firmware directory and launch PX4 multi-UAV simulation
```bash
cd ~/PX4_Firmware
roslaunch px4 multi_vehicle.launch
```

- Enter the XTDrone pose_ground_truth directory and acquire multi-UAV ground-truth poses
```bash
cd ~/XTDrone/sensing/pose_ground_truth/
bash get_multi_vehcle_local_pose_beta.sh
```

- Enter the XTDrone motion_planning/3d directory and perform multi-UAV coordinate transformation
```bash
cd ~/XTDrone/motion_planning/3d
python3 ego_swarm_transfer.py iris 2
```

- Enter the XTDrone communication directory and start multi-UAV communication
```bash
cd ~/XTDrone/communication
bash multi_vehicle_communication_beta.sh
```

- Enter the racer/catkin_ws directory, source ROS environment, and launch RVIZ visualization
```bash
cd ~/racer/catkin_ws/
source devel/setup.bash
roslaunch exploration_manager rviz.launch
```

- Enter the racer/catkin_ws directory, source ROS environment, and launch the main multi-UAV exploration node
```bash
cd ~/racer/catkin_ws/
source devel/setup.bash
roslaunch exploration_manager swarm_exploration.launch
```

- Enter the racer/catkin_ws directory, source ROS environment, and launch the PX4 multi-UAV controller
```bash
cd ~/racer/catkin_ws/
source devel/setup.bash
roslaunch px4ctrl multi_ctrl.launch
```

---

### Usage

1. Make sure all dependencies are installed (XTDrone, ROS, required Python packages, etc.).
2. Grant execute permission to the script:
   ```bash
   chmod +x scripts/haha.sh
   ```
3. Run the script:
   ```bash
   ./scripts/haha.sh
   ```
4. Press `Ctrl+C` to terminate all related processes at once.

---

## 4. Reference Directory Structure

```
catkin_ws/
  src/
    RACER/
      swarm_exploration/
        plan_env/
          src/map_ros.cpp
          include/plan_env/map_ros.h
        exploration_manager/
          launch/
            swarm_exploration.launch
            ...
  scripts/
    haha.sh
```

---

## 5. Contact & Support

If you have any questions, please contact the project maintainer or leave an issue.

---

**Wish you a smooth multi-UAV simulation and exploration!** 