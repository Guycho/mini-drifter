#include <Arduino.h>
#include <Chrono.h>

#include "PID.h"
#include "config.h"
#include "input.h"
#include "control.h"
#include "mav_bridge.h"

#ifdef DEBUG
Chrono print_timer;
Chrono loop_timer;
Chrono mav_timer;
Chrono control_timer;
uint32_t loop_count = 0;
uint32_t mav_time = 0;
uint32_t control_time = 0;
#endif
MavBridge my_mav_bridge;
Control my_control;
PID my_steering_pid;

void setup() {
    Serial.begin(9600);
#ifdef DEBUG
    print_timer.start();
    loop_timer.start();
    mav_timer.start();
    control_timer.start();
#endif
    init(config::PS4_controller::mac);
    my_mav_bridge.init(config::mavlink::serial, config::mavlink::baudrate, config::mavlink::system_id, config::mavlink::component_id, config::mavlink::steering_channel, config::mavlink::throttle_channel, config::mavlink::message_rate);
    my_steering_pid.init(config::PID::kp, config::PID::ki, config::PID::kd, config::PID::max_output, config::PID::integral_percentage, config::PID::low_pass_alpha, config::PID::high_pass_alpha, config::PID::use_filters);
    my_control.init(&my_mav_bridge, &my_steering_pid);
}
void loop(){
#ifdef DEBUG
    mav_timer.restart();
#endif
    my_mav_bridge.run();
#ifdef DEBUG
    mav_time = mav_time + mav_timer.elapsed();
    control_timer.restart();
#endif
    my_control.update(get_steering(), get_throttle());
#ifdef DEBUG
    control_time = control_time + control_timer.elapsed();
    float kp_v, ki_v, kd_v, dt, error, integral;
    loop_count++;
    if(print_timer.hasPassed(250, true)){
        float mean_loop_time = loop_timer.elapsed() / loop_count;
        float mean_mav_time = mav_time / loop_count;
        float mean_control_time = control_time / loop_count;
        loop_timer.restart();
        loop_count = 0;
        my_steering_pid.get_values(kp_v, ki_v, kd_v, dt, error, integral);
        Serial.printf(" mean Loop time: %f", mean_loop_time, 3);
        Serial.printf(" mean mav time: %f", mean_mav_time, 3);
        Serial.printf(" mean control time: %f", mean_control_time, 3);
        Serial.printf(" gyro: %f", my_mav_bridge.get_gyro_data(), 3);
        Serial.printf(" steering: %f", my_mav_bridge.get_steering_pct(), 3);
        Serial.printf(" throttle: %f", my_mav_bridge.get_throttle_pct(), 3);
        Serial.printf(" kp_v: %f", kp_v, 3);
        Serial.printf(" ki_v: %f", ki_v, 3);
        Serial.printf(" kd_v: %f", kd_v, 3);
        Serial.printf(" dt: %f", dt, 3);
        Serial.printf(" error: %f", error, 3);
        Serial.printf(" integral: %f", integral, 3);
        Serial.println();
        mav_time = 0;
        control_time = 0;
    }
#endif
}
