#!/bin/bash
echo "[터미널 3] Offboard 예제 실행 (Domain 0)..."

# Domain 0 사용 (PX4 SITL과 같음)
unset ROS_DOMAIN_ID

cd ~/offboard_ws
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash
source install/setup.bash
ros2 run offboard_example offboard_control
