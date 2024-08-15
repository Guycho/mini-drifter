#ifndef CONTROL_H
#define CONTROL_H

#include <Chrono.h>
#include <EEPROM.h>

#include "PID.h"
#include "input.h"
#include "mav_bridge.h"



class Control
{
public:
    Control();  // Constructor
    ~Control(); // Destructor

    void init(MavBridge * mav_bridge, PID *steering_pid, float gyro_input_max, float steering_input_max, float throttle_input_max); // Method to initialize control
    void update();

private:

    void trim_steering();
    void save_steering_trim_to_eeprom();
    float get_steering_trim_from_eeprom();
    void init_eeprom();
    PID *m_steering_pid;
    MavBridge *m_mav_bridge;  // Pointer to MavBridge object
    bool m_steering_mode;     // Steering mode
    float m_gyro_input_max;
    float m_gyro_input_min;
    float m_steering_input_max;
    float m_steering_input_min;
    float m_throttle_input_max;
    float m_throttle_input_min;
    float m_steering_trim;

    const bool OMEGA = true;
    const bool NORMAL = false;
    const uint8_t STEERING_TRIM_ADDR = 0;
    };

#endif // CONTROL_H