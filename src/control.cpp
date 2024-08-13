#include "control.h"
Control ::Control()
{
    // Constructor implementation
}

Control::~Control()
{
    // Destructor implementation
}

void Control::init(MavBridge *mav_bridge, PID *steering_pid, float gyro_input_max, float steering_input_max, float throttle_input_max)
{
    // Method to initialize control
    m_last_loop_timer = new Chrono(Chrono::MICROS);
    m_last_loop_timer->start();
    m_mav_bridge = mav_bridge;
    m_steering_pid = steering_pid;
    m_gyro_input_max = gyro_input_max;
    m_gyro_input_min = -gyro_input_max;
    m_steering_input_max = steering_input_max;
    m_steering_input_min = -steering_input_max;
    m_throttle_input_max = throttle_input_max;
    m_throttle_input_min = -throttle_input_max;
}

void Control::update(float steering_input, float throttle_input)
{
    // Update control state
    m_last_loop_timer->restart();
    if (m_steering_pid && m_mav_bridge)
    {
        m_steering_mode = SteeringMode(get_steering_mode());

        float steering_output = steering_input;
        float throttle_output = throttle_input;

        // Compute the PID output
        if (m_steering_mode == SteeringMode::OMEGA)
        {
            float measured_value =
                utils::calcs::map_float(m_mav_bridge->get_gyro_data(), m_gyro_input_min, m_gyro_input_max, -100, 100);
            steering_output = m_steering_pid->compute(steering_input, measured_value);
        }
        // Apply the output to the steering mechanism
        m_mav_bridge->set_steering(steering_output);
        m_mav_bridge->set_throttle(throttle_output);
    }
    m_last_loop_time = m_last_loop_timer->elapsed();
}

uint32_t Control::get_last_loop_time(){
    return m_last_loop_time;
}
