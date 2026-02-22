#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <pins.h>
#include <errors.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define CAM_I2C_ADDR 0x69

enum class B2BCommand : uint8_t {
    CAMERA0_OFF = 0,
    CAMERA0_ON = 1,
    CAMERA1_OFF = 2,
    CAMERA1_ON = 3,
    MUX_0 = 6,
    MUX_1 = 7,
};

struct cam_state_t {
    bool cam1_on = false;
    bool cam2_on = false;
    bool cam1_rec = false;
    bool cam2_rec = false;
    bool vmux_state = false;
    bool cam_ack = false;

    uint8_t encode() const {
        uint8_t dat = 0;
        dat |= (cam1_on) << 0;
        dat |= (cam2_on) << 1;
        dat |= (cam1_rec) << 2;
        dat |= (cam2_rec) << 3;
        dat |= (vmux_state) << 4;
        dat |= (cam_ack) << 5;
        return dat;
    }
};

class B2BHandler {
    static B2BHandler* instance; // singleton

    static void on_receive(int len);
    static void on_request();

    public:
    cam_state_t state;
    QueueHandle_t cmd_queue;

    int init();

    bool dequeue(uint8_t* cmd);
};
