#!/bin/bash
# 用于恢复多机探索相关通信（关闭所有drop节点）
# 使用方法：bash restore_swarm_topics.sh

# 查找所有drop节点并关闭
rosnode list | grep drop | xargs -r rosnode kill

echo "已关闭所有drop节点，通信已恢复。" 