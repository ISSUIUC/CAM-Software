#pragma once

#include <pins.h>
#include <Wire.h>
#include <tvp5151.h>
#include <errors.h>

#define TVP_DEVICE_ID 0x5151

class TVPController {
    public:
    tvp5151 tvp{TVP5151_PDN, TVP5151_RESET, TVP5151_ADDR, &Wire};

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

        if(!tvp.source_select(CAM2)) {  //make sure this is the right camera
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

    uint8_t tvp_locked() {
        bool vsync_locked;
        bool hsync_locked;
        bool color_locked;

        constexpr int MAX_TVP_LOCK_ATTEMPTS = 10;
        int num_attempts = 0;
        while ((!vsync_locked || !hsync_locked || !color_locked) && num_attempts < MAX_TVP_LOCK_ATTEMPTS)
        {
            vsync_locked = tvp.read_vertical_sync_lock_status();
            hsync_locked = tvp.read_horizontal_sync_lock_status();
            color_locked = tvp.read_color_subcarrier_lock_status();
            num_attempts++;
        }

        return vsync_locked && hsync_locked && color_locked;
    }

    bool source_select(CAM_SELECT camera) {
        return tvp.source_select(camera);
    };
};