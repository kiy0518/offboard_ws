#!/bin/bash
echo "[터미널 2] Micro XRCE-DDS Agent 시작 (Domain 0)..."

# Domain 0 사용 (기본값 - PX4와 같음)
unset ROS_DOMAIN_ID

# 라이브러리 경로 설정
export LD_LIBRARY_PATH=/usr/local/microxrcedds_agent-3.0.1/lib:/usr/local/microxrcedds_client-3.0.0/lib:/usr/local/microcdr-2.0.1/lib:$LD_LIBRARY_PATH

/usr/local/microxrcedds_agent-3.0.1/bin/MicroXRCEAgent udp4 -p 8888
