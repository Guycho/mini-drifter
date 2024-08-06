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
    my_steering_pid.init(config::PID::kp, config::PID::ki, config::PID::kd, config::PID::max_output, config::PID::integral_percentage, config::PID::low_pass_alpha, config::PID::high_pass_alpha);
    my_control.init(&my_mav_bridge, &my_steering_pid);
}
void loop(){
    my_mav_bridge.run();
    my_control.update(get_steering(), get_throttle());

#ifdef DEBUG
    if(print_timer.hasPassed(100, true)){
    Serial.printf(" gyro: %f", my_mav_bridge.get_gyro_data(), 3);
    Serial.printf(" steering: %f",my_mav_bridge.get_steering_pct(), 3);
    Serial.printf(" throttle: %f",my_mav_bridge.get_throttle_pct(), 3);
    Serial.println();
    }
#endif
}
