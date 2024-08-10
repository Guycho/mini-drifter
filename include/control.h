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

    void init(MavBridge *mav_bridge, PID *steering_pid); // Method to initialize control
    void update(float steering_input, float throttle_input);

private:
    Chrono m_timer;
    PID *m_steering_pid;
    MavBridge *m_mav_bridge;               // Pointer to MavBridge object
    SteeringMode m_steering_mode = MANUAL; // Steering mode
};

#endif // CONTROL_H