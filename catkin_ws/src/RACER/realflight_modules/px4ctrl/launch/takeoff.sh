#!/bin/bash

# 获取无人机编号
DRONE_ID=${DRONE_ID:-0}  # 默认0

# 定义上锁函数
function disarm() {
    echo "检测到中断，正在上锁..."
    rosservice call /iris_${DRONE_ID}/mavros/cmd/arming "value: false"
    exit 0
}

# 捕获 Ctrl+C (SIGINT) 信号
trap disarm SIGINT

# 起飞指令
sleep 3
rostopic pub -1 /iris_${DRONE_ID}_px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"

# 保持脚本运行，等待 Ctrl+C
while true; do
    sleep 1
done
