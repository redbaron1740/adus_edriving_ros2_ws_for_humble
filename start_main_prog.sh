#!/bin/bash
# 스크립트가 위치한 경로를 BASE_DIR로 저장
BASE_DIR="$(cd "$(dirname "$0")" && pwd)"

# ROS2 환경 설정
source $BASE_DIR/install/setup.bash

echo "메인 제어 프로그램을 시작합니다..."
ros2 run edrivingkit_pkg Main_Prog_node.py
