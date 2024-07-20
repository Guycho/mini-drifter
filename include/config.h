#include <Arduino.h>
namespace config {
    namespace mavlink {
    const HardwareSerial *serial = &Serial2;
    const uint32_t baudrate = 500000;
    const uint8_t system_id = 1;
    const uint8_t component_id = 0;
    const uint8_t steering_channel = 8;
    const uint8_t throttle_channel = 7;
    const uint8_t message_rate = 100;
    }
}