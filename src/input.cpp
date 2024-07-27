#include "input.h"

uint32_t last_time = 0;
void on_connect();
void on_disConnect();
void controller_do();
void remove_paired_devices();
int8_t calc_throttle(uint8_t l2, uint8_t r2);
int8_t calc_steering(int8_t lx);

int8_t m_throttle;
int8_t m_steering;
uint8_t m_steering_mode;


void init(const char* mac) {
    PS4.attach(controller_do);
    PS4.attachOnConnect(on_connect);
    PS4.attachOnDisconnect(on_disConnect);
    PS4.begin(mac);
    remove_paired_devices();  // This helps to solve connection issues
}

int8_t get_throttle() {
    return m_throttle;
}
int8_t get_steering() {
    return m_steering;
}

uint8_t get_steering_mode() {
    return m_steering_mode;
}

void remove_paired_devices() {
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

    uint8_t l2 = PS4.L2Value(),
            r2 = PS4.R2Value();

    int16_t gx = PS4.GyrX(),
            gy = PS4.GyrY(),
            gz = PS4.GyrZ(),
            ax = PS4.AccX(),
            ay = PS4.AccY(),
            az = PS4.AccZ();

    if(trd) {
        m_steering_mode = 1;
    }
    else if(sqd) {
        m_steering_mode = 0;
    }

    m_throttle = calc_throttle(l2, r2);
    m_steering = calc_steering(lx);
}

int8_t calc_throttle(uint8_t l2, uint8_t r2) {
    return map((r2 - l2), -255, 255, -100, 100);
}

int8_t calc_steering(int8_t lx) {
    return map(lx, -127, 127, -100, 100);
}

void on_connect() {
}

void on_disConnect() {
}
