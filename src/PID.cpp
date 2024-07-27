#include "PID.h"

PID::PID(){
    // Constructor implementation
}

PID::~PID() {
    // Destructor implementation
}
void PID::init(float kp, float ki, float kd, float max_output, float integral_percentage, float low_pass_alpha, float high_Pass_alpha) {
    m_timer = new Chrono(Chrono::MICROS);
    m_timer->start();
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_min_output = -max_output;
    m_max_output = max_output;
    m_max_integral = ((integral_percentage / 1e2) * m_min_output) / m_ki;
    m_min_integral = -m_max_integral;
    m_low_pass_alpha = low_pass_alpha;
    m_high_pass_alpha = high_Pass_alpha;
}

float PID::compute(float set_point, float measured_value) {
    float error = set_point - measured_value;

    // Get the elapsed time in seconds
    float dt = m_timer->elapsed() / 1e6;
    m_timer->restart();  // Restart the timer for the next iteration

    // Update integral term with anti-windup
    m_integral += error * dt;
    if (m_integral > m_max_integral) m_integral = m_max_integral;
    if (m_integral < m_min_integral) m_integral = m_min_integral;

    // Calculate derivative term
    float derivative = (error - m_previous_error) / dt;
    m_previous_error = error;

    // Apply high-pass filter to the input
    float filtered_input = m_high_pass_alpha * (measured_value - m_previous_input);
    m_previous_input = measured_value;

    // Compute the PID output using the filtered input
    float output = m_kp * error + m_ki * m_integral + m_kd * derivative + filtered_input;

    // Apply low-pass filter to the output
    output = m_low_pass_alpha * output + (1 - m_low_pass_alpha) * m_previous_output;
    m_previous_output = output;

    // Clamp the final output
    if (output > m_max_output) output = m_max_output;
    if (output < m_min_output) output = m_min_output;

    return output;
}