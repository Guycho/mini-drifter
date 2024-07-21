#include "input.h"

void init(const char* mac) {
    PS4.attach(controller_do);
    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisConnect);
    PS4.begin(mac);
    removePairedDevices();  // This helps to solve connection issues
}

void removePairedDevices() {
    uint8_t pairedDeviceBtAddr[20][6];
    int count = esp_bt_gap_get_bond_device_num();
    esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    for (int i = 0; i < count; i++) {
        esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
    }
}

void controller_do() {
    boolean sqd = PS4.event.button_down.square,
            squ = PS4.event.button_up.square,
            trd = PS4.event.button_down.triangle,
            tru = PS4.event.button_up.triangle,
            crd = PS4.event.button_down.cross,
            cru = PS4.event.button_up.cross,
            cid = PS4.event.button_down.circle,
            ciu = PS4.event.button_up.circle;

    boolean sq = PS4.Square(),
            tr = PS4.Triangle(),
            cr = PS4.Cross(),
            ci = PS4.Circle();

    int8_t lx = PS4.LStickX(),
           ly = PS4.LStickY(),
           rx = PS4.RStickX(),
           ry = PS4.RStickY();

    int16_t gx = PS4.GyrX(),
            gy = PS4.GyrY(),
            gz = PS4.GyrZ(),
            ax = PS4.AccX(),
            ay = PS4.AccY(),
            az = PS4.AccZ();
}

void printDeviceAddress() {
    const uint8_t* point = esp_bt_dev_get_address();
    for (int i = 0; i < 6; i++) {
        char str[3];
        sprintf(str, "%02x", (int)point[i]);
        Serial.print(str);
        if (i < 5) {
            Serial.print(":");
        }
    }
}

void onConnect() {
}

void onDisConnect() {
}
