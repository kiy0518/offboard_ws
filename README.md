# PX4 Offboard 제어 학습 예제

PX4 공식 문서를 기반으로 만든 간단한 Offboard 제어 예제입니다.

## 🎯 학습 목표

이 예제를 통해 다음을 배울 수 있습니다:

1. **Heartbeat 지속 발행**: OffboardControlMode를 2Hz 이상 지속적으로 발행
2. **Offboard 모드 진입**: OFFBOARD 모드 전환 방법
3. **Arming 순서**: Heartbeat → OFFBOARD → ARM 순서
4. **위치 제어**: TrajectorySetpoint으로 간단한 takeoff

## 📋 코드 핵심 포인트

### 1. Heartbeat (가장 중요!)
```cpp
// 타이머: 100ms마다 (10Hz) - 절대 중단되면 안됨!
timer_ = this->create_wall_timer(100ms, 
    std::bind(&OffboardControl::timer_callback, this));

void timer_callback() {
    // ★★★ 항상 최우선으로 heartbeat 발행 ★★★
    publish_offboard_control_mode();
    
    // ... 그 다음에 다른 작업
}
```

### 2. Offboard 모드 진입 순서
```
1. Heartbeat 발행 시작 (10초간)
2. OFFBOARD 모드 전환
3. ARM (시동)
4. Setpoint 발행
```

### 3. QoS 설정
```cpp
// PX4 호환 QoS (센서 데이터 프로파일)
rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
```

## 🚀 실행 방법

### 준비물
- 3개의 터미널 창

### 실행 순서

**터미널 1: PX4 SITL**
```bash
cd ~/offboard_ws
./1_run_px4.sh
```

**터미널 2: Micro XRCE-DDS Agent**
```bash
cd ~/offboard_ws
./2_run_agent.sh
```

**터미널 3: Offboard 예제**
```bash
cd ~/offboard_ws
./3_run_offboard.sh
```

## 📊 예상 동작

1. **0-10초**: Heartbeat 발행, 준비 중
2. **10초**: OFFBOARD 모드 전환 요청
3. **15초**: ARM (시동) 요청
4. **15초 이후**: 5m 고도로 이륙 시작

## 🔍 관찰 포인트

### 로그에서 확인할 것
-  - 10Hz로 발행되는지
-  - OFFBOARD 전환 확인
-  - ARM 확인
-  - 고도 변화 확인

### PX4 SITL에서 확인할 것
- Commander: OFFBOARD mode 메시지
- Commander: Armed 메시지
- 드론이 실제로 이륙하는지

## ❌ 자주 발생하는 오류

### 1. Offboard mode rejected
→ Heartbeat가 충분히 발행되지 않았음. 10초 대기 후 재시도

### 2. Arming denied
→ OFFBOARD 모드가 아직 활성화되지 않음. 로그 확인

### 3. 드론이 움직이지 않음
→ Setpoint가 발행되고 있는지 확인 ( 로그)

## 📚 참고 문서

- [PX4 Offboard Control](https://docs.px4.io/v1.16/en/ros2/offboard_control)
- [PX4 ROS2 User Guide](https://docs.px4.io/v1.16/en/ros2/user_guide)

## 🎓 다음 단계

이 예제로 개념을 확실히 이해한 후:

1. **실험**: Heartbeat 주기 변경해보기 (2Hz 이하로 낮춰보기)
2. **확장**: 다른 위치로 이동 추가
3. **적용**: VIM4의 navigation 코드 수정

---

**핵심 교훈**: Heartbeat는 절대 중단되면 안 됩니다! 타이머를 cancel하지 말 것!
