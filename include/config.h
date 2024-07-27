#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

namespace config {
    namespace mavlink {
        HardwareSerial *serial = &Serial2;
        const uint32_t baudrate = 500000;
        const uint8_t system_id = 1;
        const uint8_t component_id = 0;
        const uint8_t steering_channel = 8;
        const uint8_t throttle_channel = 7;
        const uint8_t message_rate = 100;
    }  // namespace mavlink
    namespace PS4_controller {
        const char *mac = "A0:DD:6C:03:9A:EE";
    }  // namespace PS4_controller
    namespace PID {
        const float kp = 1;
        const float ki = 0.00;
        const float kd = 0.00;
        const float max_output = 100;
        const float integral_percentage = 30;
        const float low_pass_alpha = 0.1;
        const float high_pass_alpha = 0.1;
    }  // namespace PID
}
#endif  // CONFIG_H