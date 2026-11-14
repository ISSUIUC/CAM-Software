#pragma once

#ifndef DVP_DRIVER_H
#define DVP_DRIVER_H

#include "esp_video_init.h"
#include "../../components/esp_video/private_include/esp_video.h"
#include "../../components/esp_video/private_include/esp_video_device_internal.h"
#include "esp_cache.h"

#include <Arduino.h>

#define DVP_BUFFER_COUNT 2

esp_video* DVP_init();
void start_dvp_capture(esp_video *video);
esp_err_t restart_dvp_capture(esp_video *video);


#endif