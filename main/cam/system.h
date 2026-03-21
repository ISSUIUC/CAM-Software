#pragma once
#include <Arduino.h>
#include <cam/tvpcontroller.h>
#include <cam/camera.h>
#include <cam/b2b.h>

#include "USB.h"
#include "USBCDC.h"

#include "DVP.h"
#include "tvpcontroller.h"
#include "jpeg.h"
#include "cam_radio/cam_radio.h"
#include "SPI.h"

#include "esp_cache.h"
#include "esp_video_init.h"
#include "../components/esp_video/private_include/esp_video.h"
#include "../components/esp_video/private_include/esp_video_device_internal.h"

// System file for CAM
// This file must expose `sys_begin()`, which will create all freeRTOS threads needed for the board.

/* Begin all system functions, including init. */
[[noreturn]] void sys_begin();

struct CAMSystems
{
    TVPController tvp;
    Cameras cameras;
    B2BHandler b2b;
    USBCDC *serial;
    esp_video *video;
    jpeg_encoder JPEG;
    CAMRadio radio;
};