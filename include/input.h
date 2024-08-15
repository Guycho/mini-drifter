#ifndef INPUT_CONTROLLER_H
#define INPUT_CONTROLLER_H

#include <PS4Controller.h>

#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_gap_bt_api.h"
#include "utils.h"

void init_ps4(const char* mac, float dead_band);
float get_throttle();
float get_steering();
bool get_steering_mode_toggle();
bool get_arm_toggle();
float get_steering_trim_from_input();


#endif  // INPUT_CONTROLLER_H