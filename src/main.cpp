#include <Arduino.h>
#include "config.h"
#include "input.h"
#include "mav_bridge.h"
MavBridge my_mav_bridge;

void setup() {
    Serial.begin(9600);
    init(config::PS4_controller::mac);
    my_mav_bridge.init(config::mavlink::serial, config::mavlink::baudrate, config::mavlink::system_id, config::mavlink::component_id, config::mavlink::steering_channel, config::mavlink::throttle_channel, config::mavlink::message_rate);
}
void loop(){
    my_mav_bridge.run();
    my_mav_bridge.set_steering(get_steering());
    my_mav_bridge.set_throttle(get_throttle());
}