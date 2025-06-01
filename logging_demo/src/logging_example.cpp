#include "rclcpp/rclcpp.hpp"
#include <string>
#include <bitset>
#include <thread>
#include <inttypes.h>  // for PRId64

using namespace std::chrono_literals;

class LoggingExampleNode : public rclcpp::Node
{
public:
  LoggingExampleNode()
  : Node("logging_example_node")
  {
    //  1. 기본 로깅
    RCLCPP_INFO(this->get_logger(), "▶ INFO: Node initialized.");
    RCLCPP_WARN(this->get_logger(), "▶ WARNING: Sample warning.");
    RCLCPP_ERROR(this->get_logger(), "▶ ERROR: Simulated error.");
    RCLCPP_DEBUG(this->get_logger(), "▶ DEBUG: Development debug message.");
    RCLCPP_FATAL(this->get_logger(), "▶ FATAL: Critical error encountered.");

    //   2. 로거 이름(tag) 기반 로깅
    RCLCPP_INFO(rclcpp::get_logger("motion_controller"), "▶ [motion_controller] Ready.");
    RCLCPP_WARN(rclcpp::get_logger("sensor_interface"), "▶ [sensor_interface] Unstable input.");

    //   3. 다양한 포맷 예시
    int a = 42;
    short s = 7;
    long l = 100000L;
    unsigned int u = 123;
    float f = 3.14159f;
    double d = 2.718281828;
    const char * msg = "Hello";
    std::string name = "ROS2";
    bool flag = true;
    int64_t big = 123456789012345;

    RCLCPP_INFO(this->get_logger(), "▶ int: %d, short: %hd, long: %ld, uint: %u", a, s, l, u);
    RCLCPP_INFO(this->get_logger(), "▶ float: %.2f, double: %.4lf", f, d);
    RCLCPP_INFO(this->get_logger(), "▶ char: %c, c-string: %s, std::string: %s", 'X', msg, name.c_str());
    RCLCPP_INFO(this->get_logger(), "▶ bool: %s", flag ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "▶ address: %p", static_cast<void*>(&a));
    RCLCPP_INFO(this->get_logger(), "▶ int64_t: %" PRId64, big);
    RCLCPP_INFO(this->get_logger(), "▶ hex: 0x%x, HEX: 0x%X, octal: %o", a, a, a);
    RCLCPP_INFO(this->get_logger(), "▶ binary: %s", std::bitset<8>(a).to_string().c_str());
    RCLCPP_INFO(this->get_logger(), "▶ formatted: |%6d|, |%.2f|, |%6.2f|", a, d, d);

    //   4. 조건부 로깅
    if (a > 40) {
      RCLCPP_WARN(this->get_logger(), "▶ Conditional warning: a is large (%d)", a);
    }

    //   5. 시간 및 스레드 ID 출력
    auto now = this->now();
    std::thread::id this_id = std::this_thread::get_id();
    RCLCPP_INFO(this->get_logger(), "▶ Time: %.3f sec | Thread ID: %zu", now.seconds(), std::hash<std::thread::id>{}(this_id));

    //   6. 타이머 기반 주기적 로그 + THROTTLE 로그
    timer_ = this->create_wall_timer(
      500ms,
      [this]() {
        static int count = 0;
        count++;

        RCLCPP_INFO(this->get_logger(), "▶ [TIMER] Loop count = %d", count);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
          "▶ [THROTTLE_INFO] Only every 3 seconds");
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "▶ [THROTTLE_WARN] Only every 5 seconds");
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 7000,
          "▶ [THROTTLE_ERROR] Only every 7 seconds");
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "▶ [THROTTLE_DEBUG] Only every 1 second");
        RCLCPP_FATAL_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "▶ [THROTTLE_FATAL] Only every 10 seconds");
      }
    );
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};
