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
        // Serial.print("Device ID: 0x");
        // Serial.println(device_id, HEX);
        if(device_id != TVP_DEVICE_ID) {
            return CAM_TVP_FAILED_TO_COMMUNICATE;
        }

        if(!tvp.source_select(CAM1)) {  //make sure this is the right camera
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

        // Check vertical line count - if this shows ~525 for NTSC, video IS being processed
        uint16_t line_count = tvp.read_vertical_line_count();
        Serial.print("Vertical Line Count: ");
        Serial.println(line_count);

        bool vsync_locked;
        bool hsync_locked;
        bool color_locked;

        while (!vsync_locked || !hsync_locked || !color_locked)
        {
            Serial.println("Waiting for TVP to lock...");

            vsync_locked = tvp.read_vertical_sync_lock_status();
            hsync_locked = tvp.read_horizontal_sync_lock_status();
            color_locked = tvp.read_color_subcarrier_lock_status();
            Serial.print("VSYNC: ");
            Serial.print(vsync_locked);
            Serial.print(", HSYNC: ");
            Serial.print(hsync_locked);
            Serial.print(", COL: ");
            Serial.println(color_locked);
        }

        delay(1000);
        Serial.println("Finished TVP init");

        return CAM_OK;
    };

    bool source_select(CAM_SELECT camera) {
        return tvp.source_select(camera);
    };
};