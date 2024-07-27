#ifndef CONTROL_H
#define CONTROL_H

#include <Chrono.h>
#include "mav_bridge.h"
#include "input.h"
#include "PID.h"


class AControl {
   public:
    AControl();   // Constructor
    ~AControl();  // Destructor

    void initialize(MavBridge *mav_bridge, PID *steering_pid, uint8_t steering_mode);  // Method to initialize control
    void update();      // Method to update control state
    void set_steering_mode(uint8_t steering_mode);  // Method to set steering
   private:

    Chrono m_timer;
    PID *m_steering_pid;
    MavBridge *m_mav_bridge;  // Pointer to MavBridge object
    uint8_t m_steering_mode;  // Steering mode
    
};

#endif  // CONTROL_H