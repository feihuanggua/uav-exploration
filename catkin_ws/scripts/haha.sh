#!/bin/bash

# 记录所有子终端PID
TERMINAL_PIDS=()

# 捕获退出信号，关闭所有相关进程
function cleanup {
    echo -e "\033[31m[$(date +'%H:%M:%S')] [INFO] 关闭所有roslaunch/python/bash进程（发送SIGINT）...\033[0m"
    pkill -SIGINT -f roslaunch
    pkill -SIGINT -f python
    pkill -SIGINT -f bash
    for pid in "${TERMINAL_PIDS[@]}"; do
        kill $pid 2>/dev/null
    done
    exit 0
}
trap cleanup SIGINT SIGTERM

# 函数：带颜色和时间的日志输出
function log_info {
    echo -e "\033[32m[$(date +'%H:%M:%S')] [INFO]\033[0m $1"
}

# 1. 启动 PX4 多机仿真
log_info "Opening Terminal 1: PX4 Multi-Vehicle Simulation..."
gnome-terminal --title="PX4 Multi-Vehicle" -- bash -c "cd ~/PX4_Firmware && roslaunch px4 multi_vehicle.launch; exec bash" &
TERMINAL_PIDS+=($!)
sleep 5  # 等待PX4多机仿真完全启动

# 2. 获取多机位姿真值
log_info "Opening Terminal 2: Multi-Vehicle Pose Ground Truth..."
gnome-terminal --title="Multi-Vehicle Pose" -- bash -c "cd ~/XTDrone/sensing/pose_ground_truth/ && bash get_multi_vehcle_local_pose_beta.sh; exec bash" &
TERMINAL_PIDS+=($!)

# 3. 启动坐标转换
log_info "Opening Terminal 3: Ego Swarm Coordinate Transfer..."
gnome-terminal --title="Ego Swarm Transfer" -- bash -c "cd ~/XTDrone/motion_planning/3d && python3 ego_swarm_transfer.py iris 2; exec bash" &
TERMINAL_PIDS+=($!)

# 4. 启动多机通讯
log_info "Opening Terminal 4: Multi-Vehicle Communication..."
gnome-terminal --title="Multi-Vehicle Communication" -- bash -c "cd ~/XTDrone/communication && bash multi_vehicle_communication_beta.sh; exec bash" &
TERMINAL_PIDS+=($!)

# 5. 启动演示环境（RVIZ）
log_info "Opening Terminal 5: RVIZ..."
gnome-terminal --title="RVIZ" -- bash -c "cd ~/racer/catkin_ws/ && source devel/setup.bash && roslaunch exploration_manager rviz.launch; exec bash" &
TERMINAL_PIDS+=($!)
sleep 2

# 6. 启动模拟（swarm_exploration）
log_info "Opening Terminal 6: Swarm Exploration..."
gnome-terminal --title="Swarm Exploration" -- bash -c "cd ~/racer/catkin_ws/ && source devel/setup.bash && roslaunch exploration_manager swarm_exploration.launch; exec bash" &
TERMINAL_PIDS+=($!)

# 7. 启动控制器（多机）
log_info "Opening Terminal 7: PX4 Controller (Multi)..."
#gnome-terminal --title="PX4 Controller Multi" -- bash -c "cd ~/fuel_gazebo/catkin_ws/Fast-Exploration/ && source devel/setup.bash && roslaunch px4ctrl multi_ctrl.launch; exec bash" &
gnome-terminal --title="PX4 Controller Multi" -- bash -c "cd ~/racer/catkin_ws/ && source devel/setup.bash && roslaunch px4ctrl multi_ctrl.launch; exec bash" &
TERMINAL_PIDS+=($!)

log_info "All 7 terminals opened sequentially with delays!"

# 保持主脚本运行，等待Ctrl+C
while true; do
    sleep 1
done
