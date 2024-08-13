#ifndef CONTROL_H
#define CONTROL_H

#include <Chrono.h>

#include "PID.h"
#include "input.h"
#include "mav_bridge.h"


enum SteeringMode
{
    MANUAL = 0,
    OMEGA = 1,
};

class Control
{
public:
    Control();  // Constructor
    ~Control(); // Destructor

    void init(MavBridge * mav_bridge, PID *steering_pid, float gyro_input_max, float steering_input_max, float throttle_input_max); // Method to initialize control
    void update(float steering_input, float throttle_input);
    uint32_t get_last_loop_time();

private:
    Chrono *m_last_loop_timer;
    PID *m_steering_pid;
    MavBridge *m_mav_bridge;               // Pointer to MavBridge object
    SteeringMode m_steering_mode = MANUAL; // Steering mode
    float m_gyro_input_max;
    float m_gyro_input_min;
    float m_steering_input_max;
    float m_steering_input_min;
    float m_throttle_input_max;
    float m_throttle_input_min;

    uint32_t m_last_loop_time;
};

#endif // CONTROL_H