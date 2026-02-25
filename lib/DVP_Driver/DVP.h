#pragma once

#ifndef DVP_DRIVER_H
#define DVP_DRIVER_H

#include "esp_video_init.h"
#include "../../components/esp_video/private_include/esp_video.h"
#include "../../components/esp_video/private_include/esp_video_device_internal.h"
#include "esp_cache.h"

#include <Arduino.h>

esp_video* DVP_init();
void start_dvp_cature(esp_video *video);
void on_frame_ready(uint32_t len, uint8_t *buf);


#endif