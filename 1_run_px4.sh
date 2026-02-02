#!/bin/bash
echo "[터미널 1] PX4 SITL 시작 (MAVLink 네트워크 활성화)..."

cd ~/PX4-Autopilot

# extras.txt가 있는지 확인
if [ -f build/px4_sitl_default/etc/extras.txt ]; then
    echo "✓ extras.txt 발견 - 네트워크 브로드캐스트 활성화됨"
else
    echo "⚠ extras.txt 없음 - 생성 중..."
    mkdir -p build/px4_sitl_default/etc
    cat > build/px4_sitl_default/etc/extras.txt << 'EXTRAS'
# GPS 시작 위치 설정 (대구)
param set SIM_GPS_INIT_LAT 35.905840
param set SIM_GPS_INIT_LON 128.802654
param set SIM_GPS_INIT_ALT 50.0

# MAVLink 네트워크 브로드캐스트 활성화
param set MAV_0_BROADCAST 1
param set MAV_1_BROADCAST 1

# 순항/최대 속도 설정
param set MPC_XY_CRUISE 3
param set MPC_XY_VEL_MAX 5

# QGC 연결용 (기본 포트 14550, MAVLink v1.0 호환)
mavlink start -x -u 14550 -r 4000000 -m config -f
param set MAV_PROTO_VER 1
EXTRAS
    echo "✓ extras.txt 생성 완료"
fi

# GPS 시작 위치 설정 (대구)
export PX4_HOME_LAT=35.905840
export PX4_HOME_LON=128.802654
export PX4_HOME_ALT=50.0

# PX4 SITL 실행
make px4_sitl gz_x500
