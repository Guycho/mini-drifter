#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <Chrono.h>

class PID {
   public:
    PID();   // Constructor
    ~PID();  // Destructor

    void init(float kp, float ki, float kd, float max_output, float integral_percentage, float low_pass_alpha, float high_Pass_alpha);
    float compute(float set_point, float measured_value);  // Method to compute the PID output

   private:
    Chrono *m_timer;
    
    float m_kp;  // Proportional coefficient
    float m_ki;  // Integral coefficient
    float m_kd;  // Derivative coefficient

    float m_previous_error = 0;  // Previous error for derivative calculation
    float m_integral = 0;       // Integral of the error

    float m_min_integral;  // Minimum integral for anti-windup
    float m_max_integral;  // Maximum integral for anti-windup

    float m_min_output;  // Minimum output for anti-windup
    float m_max_output;  // Maximum output for anti-windup

    float m_low_pass_alpha;   // Low-pass filter coefficient
    float m_high_pass_alpha;  // High-pass filter coefficient

    float m_previous_output;  // Previous output for low-pass filter
    float m_previous_input;   // Previous input for high-pass filter
};

#endif  // PID_H