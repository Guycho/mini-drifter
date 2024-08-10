#include <Arduino.h>
#include <Chrono.h>

#include "PID.h"
#include "config.h"
#include "input.h"
#include "control.h"
#include "mav_bridge.h"

#ifdef DEBUG
Chrono print_timer;
#endif
MavBridge my_mav_bridge;
Control my_control;
PID my_steering_pid;

void setup() {
    Serial.begin(9600);
#ifdef DEBUG
    print_timer.start();
#endif
    init(config::PS4_controller::mac);
    my_mav_bridge.init(config::mavlink::serial, config::mavlink::baudrate, config::mavlink::system_id, config::mavlink::component_id, config::mavlink::steering_channel, config::mavlink::throttle_channel, config::mavlink::message_rate);
    my_steering_pid.init(config::PID::kp, config::PID::ki, config::PID::kd, config::PID::max_output, config::PID::integral_percentage, config::PID::low_pass_alpha, config::PID::high_pass_alpha, config::PID::use_filters);
    my_control.init(&my_mav_bridge, &my_steering_pid);
}
void loop(){
    my_mav_bridge.run();
    my_control.update(get_steering(), get_throttle());
    float kp_v, ki_v, kd_v, dt, error, integral;

#ifdef DEBUG
    if(print_timer.hasPassed(250, true)){
        // my_steering_pid.get_values(kp_v, ki_v, kd_v, dt, error, integral);
        Serial.printf(" gyro: %f", my_mav_bridge.get_gyro_data(), 3);
        Serial.printf(" steering: %f", my_mav_bridge.get_steering_pct(), 3);
        Serial.printf(" throttle: %f", my_mav_bridge.get_throttle_pct(), 3);
        // Serial.printf(" kp_v: %f", kp_v, 3);
        // Serial.printf(" ki_v: %f", ki_v, 3);
        // Serial.printf(" kd_v: %f", kd_v, 3);
        // Serial.printf(" dt: %f", dt, 3);
        // Serial.printf(" error: %f", error, 3);
        // Serial.printf(" integral: %f", integral, 3);
        Serial.println();
    }
#endif
}
