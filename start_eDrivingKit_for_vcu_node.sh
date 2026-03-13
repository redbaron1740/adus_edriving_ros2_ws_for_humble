#!/bin/bash
# 스크립트가 위치한 경로를 BASE_DIR로 저장
BASE_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_DIR="$BASE_DIR/config"

# ROS2 환경 설정
source $BASE_DIR/install/setup.bash

echo "PCAN 노드를 시작합니다..."
ros2 run edrivingkit_pkg edrivingkit_for_vcu_node --ros-args --params-file $CONFIG_DIR/pcan_config.yaml #-r __node:=edrivingkit_pcan_node
