#pragma once

#include <Arduino.h>
#include <pins.h>
#include <errors.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/i2c_slave.h"

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
    bool vtx_state = false;
    bool cam_ack = false;

    uint8_t encode() const {
        uint8_t dat = 0;
        dat |= (cam1_on) << 1;
        dat |= (cam2_on) << 0;
        dat |= (cam1_rec) << 3;
        dat |= (cam2_rec) << 2;
        dat |= (vtx_state) << 4;
        dat |= (vmux_state) << 5;
        dat |= (cam_ack) << 6;
        return dat;
    }
};

class B2BHandler {
    static B2BHandler* instance;

    static bool on_receive(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_rx_done_event_data_t *evt_data, void *arg);
    static bool on_request(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_request_event_data_t *evt_data, void *arg);

    public:
    cam_state_t state;
    QueueHandle_t cmd_queue;
    i2c_slave_dev_handle_t slave_handle;

    int init();
    void deinit();
    int reinit();

    bool dequeue(uint8_t* cmd);
};
