# 多无人机自主探索系统说明

## 1. 项目简介

本工程基于 ROS，集成了 PX4 多机仿真、位姿真值获取、坐标转换、多机通讯、三维可视化（RVIZ）、多机探索与控制等模块，支持多无人机协同自主探索任务。

---

## 2. 主要功能模块

- **PX4 多机仿真**：通过 PX4 固件和 Gazebo 实现多架无人机的物理仿真。
- **位姿真值获取**：实时获取所有无人机的真实位姿，用于后续坐标转换与点云处理。
- **坐标转换**：将仿真环境下的无人机位姿转换为统一坐标系，便于多机协同。
- **多机通讯**：实现无人机间的状态、任务等信息同步。
- **三维可视化（RVIZ）**：实时展示探索过程、地图、轨迹等信息。
- **多机探索与控制**：支持多架无人机自主探索、避障、路径规划与控制。

---

## 3. 一键启动脚本说明

你可以使用如下 bash 脚本一键启动全部仿真与探索流程：

```bash
#!/bin/bash
# 详细脚本内容见本仓库/scripts/haha.sh
```

### 启动流程说明

1. **PX4 多机仿真**  
   启动 PX4 固件和 Gazebo，仿真多架无人机。
2. **多机位姿真值获取**  
   启动脚本获取所有无人机的真实位姿。
3. **坐标转换**  
   启动坐标转换节点，统一多机坐标系。
4. **多机通讯**  
   启动多机间的通信节点。
5. **RVIZ 可视化**  
   启动 RVIZ，实时可视化探索过程。
6. **swarm_exploration 启动**  
   启动多机探索主节点。
7. **PX4 控制器**  
   启动多机控制器节点。

### 脚本命令详解

```bash
cd ~/PX4_Firmware
roslaunch px4 multi_vehicle.launch
```
- 进入PX4_Firmware目录，启动PX4多机仿真环境


```bash
cd ~/XTDrone/sensing/pose_ground_truth/
bash get_multi_vehcle_local_pose_beta.sh
```
- 进入XTDrone的pose_ground_truth目录，获取多机位姿真值


```bash
cd ~/XTDrone/motion_planning/3d
python3 ego_swarm_transfer.py iris 2
```
- 进入XTDrone的motion_planning/3d目录，进行多机坐标转换


```bash
cd ~/XTDrone/communication
bash multi_vehicle_communication_beta.sh
```
- 进入XTDrone的communication目录，启动多机通信


```bash
cd ~/racer/catkin_ws/
source devel/setup.bash
roslaunch exploration_manager rviz.launch
```
- 进入racer/catkin_ws目录，加载ROS环境，启动RVIZ可视化


```bash
cd ~/racer/catkin_ws/
source devel/setup.bash
roslaunch exploration_manager swarm_exploration.launch
```
- 进入racer/catkin_ws目录，加载ROS环境，启动多机探索主节点


```bash
cd ~/racer/catkin_ws/
source devel/setup.bash
roslaunch px4ctrl multi_ctrl.launch
```
- 进入racer/catkin_ws目录，加载ROS环境，启动PX4多机控制器

---

### 使用方法

1. 确保所有依赖环境已配置好（XTDrone、ROS、相关Python依赖等）。
2. 赋予脚本执行权限：
   ```bash
   chmod +x scripts/haha.sh
   ```
3. 运行脚本：
   ```bash
   ./scripts/haha.sh
   ```
4. 按 `Ctrl+C` 可一键关闭所有相关进程。

---

## 4. 参考目录结构

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

## 5. 联系与支持

如有问题请联系项目维护者，或在 Issues 区留言。

---

**祝你多机仿真与探索顺利！** 