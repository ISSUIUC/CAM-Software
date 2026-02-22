#pragma once

#include <pins.h>
#include <Wire.h>
#include <tvp5151.h>
#include <errors.h>

#define TVP_DEVICE_ID 0x5151

class TVPController {
    tvp5151 tvp{TVP5151_PDN, TVP5151_RESET, TVP5151_ADDR, &Wire};

    public:
    int init() {

        for(uint8_t i = 0; i < YOUT_PIN_COUNT; i++) {
            pinMode(YOUT[i], INPUT);
        }

        if(!tvp.init()) {
            return CAM_TVP_FAILED_TO_INIT;
        }  

        uint16_t device_id = tvp.read_device_id();
        if(device_id != TVP_DEVICE_ID) {
            return CAM_TVP_FAILED_TO_COMMUNICATE;
        }

        if(!tvp.source_select(CAM1)) {
            return CAM_TVP_FAILED_TO_SS;
        }

        if(!tvp.set_ycbcr_output_enable(true)) {
            return CAM_TVP_FAILED_TO_SET_OUTPUT;
        }

        
        if(!tvp.set_clock_output_enable(true)) {
            return CAM_TVP_FAILED_TO_SET_CLOCK;
        }

        if (!tvp.set_avid_output_enable(true)) {
            return CAM_TVP_FAILED_TO_SET_AVID;
        }

        if (!tvp.set_yCbCr_output_format(true)) {
            return CAM_TVP_FAILED_TO_SET_FORMAT;
        }

        return CAM_OK;
    };

    bool source_select(CAM_SELECT camera) {
        return tvp.source_select(camera);
    };
};