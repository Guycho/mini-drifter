#include <Arduino.h>
#include <Chrono.h>

#include "PID.h"
#include "config.h"
#include "control.h"
#include "input.h"
#include "mav_bridge.h"

Chrono print_timer;

MavBridge my_mav_bridge;
Control my_control;
PID my_steering_pid;

void print_data();

void setup() {
    Serial.begin(9600);
    print_timer.start();
    init_ps4(config::PS4_controller::mac, config::PS4_controller::dead_band);
    my_mav_bridge.init(config::mavlink::serial, config::mavlink::baudrate,
      config::mavlink::system_id, config::mavlink::component_id, config::mavlink::steering_channel,
      config::mavlink::throttle_channel, config::mavlink::message_rate);
    my_steering_pid.init(config::PID::kp, config::PID::ki, config::PID::kd, config::PID::max_output,
      config::PID::integral_percentage, config::PID::low_pass_alpha, config::PID::high_pass_alpha,
      config::PID::use_filters);
    my_control.init(&my_mav_bridge, &my_steering_pid, config::control::gyro_input_max,
      config::control::steering_input_max, config::control::throttle_input_max);
}
void loop() {
    my_control.update();
#ifdef DEBUG
    print_data();
#endif
}

void print_data() {
    if (print_timer.hasPassed(250, true)) {
        float set_point, measured_value, kp_v, ki_v, kd_v, dt, error, integral;
        my_steering_pid.get_values(set_point, measured_value, kp_v, ki_v, kd_v, dt, error,
          integral);
        Serial.printf(
          "set_point: %f, measured_value: %f, kp: %f, ki: %f, kd: %f, dt: %f, error: %f, integral: "
          "%f",
          set_point, measured_value, kp_v, ki_v, kd_v, dt, error, integral);
          Serial.printf(" steering_trim: %f", my_control.get_steering_trim());
        Serial.println();
    }
}