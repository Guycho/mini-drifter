#ifndef INPUT_CONTROLLER_H
#define INPUT_CONTROLLER_H

#include <PS4Controller.h>

#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_gap_bt_api.h"

void init(const char* mac);
int8_t get_throttle();
int8_t get_steering();
uint8_t get_steering_mode();


#endif  // INPUT_CONTROLLER_H