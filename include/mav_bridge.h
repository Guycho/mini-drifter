#ifndef MAV_BRIDGE_H
#define MAV_BRIDGE_H

#include <Arduino.h>
#include <all/mavlink.h>

class MavBridge
{
public:
    MavBridge();
    ~MavBridge();

    void init(HardwareSerial *serial, uint32_t baudrate, uint8_t system_id, uint8_t component_id, uint8_t steering_channel, uint8_t throttle_channel, uint8_t message_rate);
    void run();
    float get_gyro_data();
    void set_steering(int steering_pct);
    void set_throttle(int throttle_pct);

private:
    void set_messages_rates();
    void set_message_rate(uint32_t msg_id, uint16_t message_rate_hz);

    void set_servo(uint8_t channel, uint16_t pwm);


    HardwareSerial *m_serial;
    uint32_t m_baudrate;
    uint8_t m_system_id;
    uint8_t m_component_id;
    uint8_t m_steering_channel;
    uint8_t m_throttle_channel;
    uint8_t m_message_rate;

    float m_gyro_data;
};

#endif // MAV_BRIDGE_H