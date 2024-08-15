#include "control.h"
Control ::Control() {
    // Constructor implementation
}

Control::~Control() {
    // Destructor implementation
}

void Control::init(MavBridge *mav_bridge, PID *steering_pid, float gyro_input_max,
  float steering_input_max, float throttle_input_max) {
    // init_eeprom();
    // Method to initialize control
    m_mav_bridge = mav_bridge;
    m_steering_pid = steering_pid;
    m_gyro_input_max = gyro_input_max;
    m_gyro_input_min = -gyro_input_max;
    m_steering_input_max = steering_input_max;
    m_steering_input_min = -steering_input_max;
    m_throttle_input_max = throttle_input_max;
    m_throttle_input_min = -throttle_input_max;
}

void Control::update() {
    // Update control state
    if (m_steering_pid && m_mav_bridge) {
        m_mav_bridge->run();
        trim_steering();
        float steering_input = get_steering() + m_steering_trim;
        float throttle_input = get_throttle();
        // Get the steering mode
        if (get_steering_mode_toggle()) {
            m_steering_mode = !m_steering_mode;
        }
        if (get_arm_toggle()) {
            m_mav_bridge->toggle_arm();
            // save_steering_trim_to_eeprom();
        }
        if (m_mav_bridge->get_arm_state() == false) {
            m_mav_bridge->set_steering(m_steering_trim);
            m_mav_bridge->set_throttle(0);
            m_steering_pid->reset_pid();
            return;
        }
        float steering_output = steering_input;
        float throttle_output = throttle_input;

        // Compute the PID output
        if (m_steering_mode == OMEGA) {
            if (throttle_output == 0) {
                m_steering_pid->enable_integral(false);
            } else {
                m_steering_pid->enable_integral(true);
            }
            float measured_value = utils::calcs::map_float(m_mav_bridge->get_gyro_data(),
              m_gyro_input_min, m_gyro_input_max, -100, 100);
            steering_output = m_steering_pid->compute(steering_input, measured_value);
        } else {
            m_steering_pid->reset_pid();
        }
        // Apply the output to the steering mechanism
        m_mav_bridge->set_steering(steering_output);
        m_mav_bridge->set_throttle(throttle_output);
    }
}

void Control::trim_steering() {
    // Trim the steering mechanism
    float trim = get_steering_trim_from_input();
    if (trim == 1) {
        m_steering_trim += 1;
    } else if (trim == -1) {
        m_steering_trim -= 1;
    }
}

void Control::save_steering_trim_to_eeprom() {
    // Save the steering trim to EEPROM
    EEPROM.begin(512);
    float temp = get_steering_trim();
    if (m_steering_trim != temp) {
        EEPROM.put(STEERING_TRIM_ADDR, m_steering_trim);
        EEPROM.commit();  // Ensure data is written to EEPROM
    }
    EEPROM.end();
}

float Control::get_steering_trim_from_eeprom() {
    float temp;
    EEPROM.get(STEERING_TRIM_ADDR, temp);
    if (isnan(temp)) {
        temp = 0;
    }
    return temp;
}

void Control::init_eeprom() {
    EEPROM.begin(512);
    m_steering_trim = get_steering_trim_from_eeprom();
    EEPROM.end();
}

float Control::get_steering_trim() {
    return m_steering_trim;
}