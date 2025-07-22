#!/bin/bash
# 用于丢弃多机探索相关通信话题的所有消息
# 使用方法：bash drop_swarm_topics.sh

rosrun topic_tools drop /planning/swarm_traj 1 &
rosrun topic_tools drop /swarm_expl/drone_state 1 &
rosrun topic_tools drop /swarm_expl/grid_tour 1 &
rosrun topic_tools drop /swarm_expl/hgrid 1 &
rosrun topic_tools drop /swarm_expl/pair_opt 1 &
rosrun topic_tools drop /swarm_expl/pair_opt_res 1 &

# 可选：等待所有后台进程启动
sleep 2

echo "已丢弃所有指定话题的消息（每条都丢弃，模拟断联）" 

#这里是纯命令
# rosrun topic_tools drop /planning/swarm_traj 1
# rosrun topic_tools drop /swarm_expl/drone_state 1
# rosrun topic_tools drop /swarm_expl/grid_tour 1
# rosrun topic_tools drop /swarm_expl/hgrid 1
# rosrun topic_tools drop /swarm_expl/pair_opt 1
# rosrun topic_tools drop /swarm_expl/pair_opt_res 1