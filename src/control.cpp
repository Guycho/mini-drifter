#include "control.h"
Control ::Control() {
    // Constructor implementation
}

Control::~Control() {
    // Destructor implementation
}

void Control::init(MavBridge *mav_bridge, PID *steering_pid) {
    m_mav_bridge = mav_bridge;
    m_steering_pid = steering_pid;
    m_timer.restart();  // Start the timer
}

void Control::update(float steering_input, float throttle_input) {
    // Update control state
    if (m_steering_pid && m_mav_bridge) {
        m_steering_mode = SteeringMode(get_steering_mode());

        float steering_output = steering_input;
        float throttle_output = throttle_input;

        // Compute the PID output
        if (m_steering_mode == SteeringMode::OMEGA) {
            float measured_value = m_mav_bridge->get_gyro_data();
            steering_output = m_steering_pid->compute(steering_input, measured_value);
        }
        // Apply the output to the steering mechanism
        m_mav_bridge->set_steering(steering_output);
        m_mav_bridge->set_throttle(throttle_output);
    }
}
