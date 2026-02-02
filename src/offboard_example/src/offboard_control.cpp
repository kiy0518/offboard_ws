/**
 * @file offboard_control.cpp
 * @brief PX4 Offboard 제어 학습용 간단한 예제
 * 
 * PX4 공식 문서 기반:
 * - OffboardControlMode heartbeat를 2Hz 이상 지속적으로 발행 (10Hz 권장)
 * - TrajectorySetpoint으로 위치 제어
 * - Arming 전부터 메시지 발행 필요
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control"), offboard_setpoint_counter_(0)
    {
        // QoS 설정 (PX4 호환 - 매우 중요!)
        // PX4는 Best Effort + Volatile 조합을 요구함
        // sensor_data 프로파일 = BEST_EFFORT + VOLATILE + KEEP_LAST
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

        // 또는 명시적으로 설정:
        // auto qos = rclcpp::QoS(10)
        //     .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        //     .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
        //     .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        // Publishers
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", qos);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", qos);

        // Subscribers
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));
        
        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));

        vehicle_global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", qos,
            std::bind(&OffboardControl::vehicle_global_position_callback, this, std::placeholders::_1));

        // 타이머 주기 설정
        // PX4 요구사항: 2Hz 이상 (500ms 이하)
        // 권장: 10Hz (100ms) ~ 50Hz (20ms)
        constexpr auto TIMER_PERIOD = 100ms;  // 10Hz (변경 가능)
        timer_ = this->create_wall_timer(TIMER_PERIOD, std::bind(&OffboardControl::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "==============================================" );
        RCLCPP_INFO(this->get_logger(), "  Offboard Control 학습 예제 시작");
        RCLCPP_INFO(this->get_logger(), "==============================================" );
        RCLCPP_INFO(this->get_logger(), "1. 2초간 heartbeat 발행 (OFFBOARD 준비)");
        RCLCPP_INFO(this->get_logger(), "2. OFFBOARD 모드 전환 (2초)");
        RCLCPP_INFO(this->get_logger(), "3. ARM (시동) (4초)");
        RCLCPP_INFO(this->get_logger(), "4. 5m 이륙 및 호버링 (8초)");
        RCLCPP_INFO(this->get_logger(), "5. 헤딩 정렬 (10-16초)");
        RCLCPP_INFO(this->get_logger(), "6. 목표 위치로 이동 (17-37초)");
        RCLCPP_INFO(this->get_logger(), "7. RTL 모드로 전환하여 자동 귀환 및 착륙 (38초)");
        RCLCPP_INFO(this->get_logger(), "==============================================" );
    }

private:
    void timer_callback()
    {
        // ★★★ 핵심: 항상 heartbeat 발행 (2Hz 이상 필수) ★★★
        publish_offboard_control_mode();

        offboard_setpoint_counter_++;

        // 단계별 실행 (10Hz 기준)
        // PX4 요구사항: 최소 0.5초 heartbeat 필요
        // 타이밍: 2초 대기 → OFFBOARD → 2초 대기 → ARM → 이륙 → 호버링 → 회전 → 이동 → RTL
        constexpr uint64_t OFFBOARD_COUNT = 20;  // 2초 @ 10Hz
        constexpr uint64_t ARM_COUNT = 40;       // 4초 @ 10Hz
        constexpr uint64_t ROTATE_START = 100;   // 10초 @ 10Hz (회전 시작)
        constexpr uint64_t ROTATE_END = 160;     // 16초 @ 10Hz (회전 종료)
        constexpr uint64_t RTL_COUNT = 380;      // 38초 @ 10Hz (RTL 전환)

        if (offboard_setpoint_counter_ == OFFBOARD_COUNT) {
            // 2초 후: OFFBOARD 모드 전환
            RCLCPP_INFO(this->get_logger(), "\n[Step 1] OFFBOARD 모드로 전환 요청...");
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

        } else if (offboard_setpoint_counter_ == ARM_COUNT) {
            // 4초 후: ARM (시동)
            RCLCPP_INFO(this->get_logger(), "\n[Step 2] ARM (시동) 요청...");
            initial_yaw_ = current_yaw_;  // 이륙 시점 헤딩 기억
            arm();

        } else if (offboard_setpoint_counter_ == ROTATE_START) {
            // 10초 후: 헤딩 정렬 시작
            RCLCPP_INFO(this->get_logger(), "\n[Step 3] 목표 방향으로 헤딩 정렬 시작...");

        } else if (offboard_setpoint_counter_ == ROTATE_END) {
            // 16초 후: 이동 시작
            RCLCPP_INFO(this->get_logger(), "\n[Step 4] 목표 위치로 이동 시작...");

        } else if (offboard_setpoint_counter_ == RTL_COUNT) {
            // 37초 후: RTL 모드 전환 (자동 귀환 및 착륙)
            RCLCPP_INFO(this->get_logger(), "\n[Step 5] RTL 모드로 전환 - 자동 귀환 및 착륙 시작...");
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
            rtl_triggered_ = true;

        } else if (offboard_setpoint_counter_ > ARM_COUNT && !rtl_triggered_) {
            // ARM 이후 ~ RTL 전까지: 목표 위치로 Setpoint 발행
            publish_trajectory_setpoint();
        }

        // 진행 상황 출력
        if (offboard_setpoint_counter_ < OFFBOARD_COUNT && offboard_setpoint_counter_ % 5 == 0) {
            RCLCPP_INFO(this->get_logger(), "[준비 중] Heartbeat 발행 중... %ld/%ld (%.1f초)",
                        offboard_setpoint_counter_, OFFBOARD_COUNT, offboard_setpoint_counter_ / 10.0);
        }

        // RTL 중 실시간 GPS 위치 출력 (1초마다)
        if (rtl_triggered_ && offboard_setpoint_counter_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "[RTL] GPS(%.6f, %.6f) Alt: %.1fm (%.1fm) | Yaw: %.1f°",
                        current_lat_, current_lon_, -current_altitude_, current_alt_amsl_,
                        current_yaw_ * 180.0 / M_PI);
        }
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true;  // Position 제어 모드
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        // 목표 GPS 좌표 (대구 인근)
        // constexpr double TARGET_LAT = 35.905542;
        // constexpr double TARGET_LON = 128.802706;
        constexpr double TARGET_LAT = 35.905756;
        constexpr double TARGET_LON = 128.803066;
        constexpr double HOME_LAT = 35.905840;
        constexpr double HOME_LON = 128.802654;

        // GPS 차이를 로컬 NED 좌표로 변환 (미터)
        constexpr double DEG_TO_M_LAT = 111320.0;
        const double deg_to_m_lon = 111320.0 * cos(HOME_LAT * M_PI / 180.0);

        // 최종 목표 위치까지의 로컬 좌표 (NED)
        float final_target_north = (TARGET_LAT - HOME_LAT) * DEG_TO_M_LAT;
        float final_target_east = (TARGET_LON - HOME_LON) * deg_to_m_lon;

        // 목표 방향 계산
        float target_yaw = atan2(final_target_east, final_target_north);

        // 단계별 타이밍
        constexpr uint64_t ROTATE_START = 100;  // 10초: 회전 시작
        constexpr uint64_t ROTATE_END = 160;    // 16초: 회전 완료
        constexpr uint64_t MOVE_START = 170;    // 17초: 이동 시작 (회전 완료 후 1초 대기)

        // 제어 파라미터
        constexpr float MAX_YAW_RATE = 0.3f;     // 최대 회전 속도 (rad/s) ≈ 17도/초

        // 위치 및 헤딩 설정
        float enu_x, enu_y, enu_z;
        float yaw_setpoint = NAN;
        float yawspeed = 0.0f;
        float vx = NAN, vy = NAN, vz = NAN;  // 속도 제한

        if (offboard_setpoint_counter_ < ROTATE_START) {
            // 1단계: 호버링 (원점, 5m) - 헤딩 North 고정
            enu_x = 0.0f;
            enu_y = 0.0f;
            enu_z = 5.0f;
            yaw_setpoint = initial_yaw_;  // 이륙 시 헤딩 유지
            yawspeed = 0.0f;

        } else if (offboard_setpoint_counter_ >= ROTATE_START &&
                   offboard_setpoint_counter_ < ROTATE_END) {
            // 2단계: 천천히 회전 (yawspeed 사용)
            enu_x = 0.0f;
            enu_y = 0.0f;
            enu_z = 5.0f;

            // 현재 yaw와 목표 yaw 차이 계산
            float yaw_diff = target_yaw - current_yaw_;
            // -π ~ π 범위로 정규화
            while (yaw_diff > M_PI) yaw_diff -= 2.0f * M_PI;
            while (yaw_diff < -M_PI) yaw_diff += 2.0f * M_PI;

            // ★ PD 제어: P(오차 비례) + D(오차 변화율 = 댐핑)
            // K_P = 1.5: 이전보다 약간 높여서 응답성 향상
            // K_D = 0.3: 댐핑 역할로 목표 근처에서 부드럽게 정지
            // 오버슈트가 발생하면 K_D를 높이고, 반응이 너무 느리면 K_P를 높이면 됩니다.
            constexpr float K_P = 1.5f;   // 비례 게인
            constexpr float K_D = 0.9f;   // 미분 게인 (오버슈트 억제)
            constexpr float dt = 0.1f;    // 100ms (10Hz)

            float yaw_diff_deriv = (yaw_diff - prev_yaw_diff_) / dt;
            prev_yaw_diff_ = yaw_diff;

            yawspeed = K_P * yaw_diff + K_D * yaw_diff_deriv;

            // 최대/최소 속도 제한
            if (yawspeed > MAX_YAW_RATE) yawspeed = MAX_YAW_RATE;
            if (yawspeed < -MAX_YAW_RATE) yawspeed = -MAX_YAW_RATE;

            // Dead zone (아주 작은 오차는 무시)
            if (fabs(yaw_diff) < 0.01f) {  // 0.6도 이내
                yawspeed = 0.0f;
            }

            yaw_setpoint = NAN;  // yawspeed 사용 시 yaw는 NAN

        } else if (offboard_setpoint_counter_ < MOVE_START) {
            // 전환 구간 (ROTATE_END ~ MOVE_START): 호버링 유지
            enu_x = 0.0f;
            enu_y = 0.0f;
            enu_z = 5.0f;
            yaw_setpoint = target_yaw;
            yawspeed = 0.0f;

        } else if (offboard_setpoint_counter_ >= MOVE_START) {
            // 3단계: 목표 위치로 이동 (PX4 내부 위치 컨트롤러에 위임)
            enu_x = final_target_east;
            enu_y = final_target_north;
            enu_z = 5.0f;
            yaw_setpoint = target_yaw;
            yawspeed = 0.0f;
            // velocity는 NAN 유지 → PX4 자체 위치 컨트롤러가 감속/접근 처리
        }

        // ENU → NED 변환
        float ned_x = enu_y;
        float ned_y = enu_x;
        float ned_z = -enu_z;

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = {ned_x, ned_y, ned_z};
        msg.velocity = {vx, vy, vz};  // 속도 제한
        msg.yaw = yaw_setpoint;
        msg.yawspeed = yawspeed;

        trajectory_setpoint_pub_->publish(msg);

        // 진행 상황 출력 (GPS 좌표 포함)
        if (offboard_setpoint_counter_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "[Setpoint] GPS(%.6f, %.6f) Alt: %.1fm (%.1fm) | Pos(%.1f, %.1f, %.1f) | Vel(%.1f, %.1f) | Yaw: %.1f° | YawRate: %.2f",
                        current_lat_, current_lon_, -current_altitude_, current_alt_amsl_,
                        ned_x, ned_y, ned_z, vx, vy,
                        yaw_setpoint * 180.0 / M_PI, yawspeed);
        }
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;

        vehicle_command_pub_->publish(msg);
    }

    void arm()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    }

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        uint8_t old_nav_state = nav_state_;
        uint8_t old_arming_state = arming_state_;

        nav_state_ = msg->nav_state;
        arming_state_ = msg->arming_state;

        // 상태 변화 출력
        if (old_nav_state != nav_state_) {
            RCLCPP_INFO(this->get_logger(), "[상태 변경] nav_state: %d -> %d %s",
                        old_nav_state, nav_state_,
                        (nav_state_ == 14 ? "(OFFBOARD!)" : ""));
        }

        if (old_arming_state != arming_state_) {
            RCLCPP_INFO(this->get_logger(), "[상태 변경] arming_state: %d -> %d %s",
                        old_arming_state, arming_state_,
                        (arming_state_ == 2 ? "(ARMED!)" : "(DISARMED)"));
        }
    }

    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        current_x_ = msg->x;    // NED North (m)
        current_y_ = msg->y;    // NED East (m)
        current_altitude_ = msg->z;  // NED: Z는 아래 방향이 양수
        current_yaw_ = msg->heading;  // 현재 yaw (radians)
    }

    void vehicle_global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
    {
        current_lat_ = msg->lat;  // 현재 위도 (degrees)
        current_lon_ = msg->lon;  // 현재 경도 (degrees)
        current_alt_amsl_ = msg->alt;  // 평균 해수면 기준 고도 (meters)
    }

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    uint64_t offboard_setpoint_counter_;
    uint8_t nav_state_ = 0;
    uint8_t arming_state_ = 0;
    float current_x_ = 0.0;        // 현재 NED North 위치 (m)
    float current_y_ = 0.0;        // 현재 NED East 위치 (m)
    float current_altitude_ = 0.0;
    float current_yaw_ = 0.0;
    double current_lat_ = 0.0;      // 현재 위도 (degrees)
    double current_lon_ = 0.0;      // 현재 경도 (degrees)
    double current_alt_amsl_ = 0.0; // 현재 고도 AMSL (meters)
    bool rtl_triggered_ = false;    // RTL 모드 전환 플래그
    float prev_yaw_diff_ = 0.0f;    // 이전 yaw 오차 (PD 제어용)
    float initial_yaw_ = 0.0f;      // 이륙 시점 헤딩 (rad)
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
