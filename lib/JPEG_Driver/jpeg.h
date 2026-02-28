#pragma once

#ifndef JPEG_DRIVER_H
#define JPEG_DRIVER_H

#include <Arduino.h>
#include "driver/jpeg_encode.h"
#include "../components/esp_video/private_include/esp_video.h"
#include "../components/esp_video/private_include/esp_video_device_internal.h"
#include "esp_cache.h"

class jpeg_encoder {

    private:
    jpeg_encode_engine_cfg_t encode_eng_cfg = {
        .intr_priority = 0,
        .timeout_ms = 40,
    };

    jpeg_encode_cfg_t enc_config = {
        .height = 240,
        .width = 720,
        .src_type = JPEG_ENCODE_IN_FORMAT_YUV422,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV422, // this works for some reason
        .image_quality = 20,
    };

    jpeg_encode_memory_alloc_cfg_t mem_cfg_output = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };

    jpeg_encode_memory_alloc_cfg_t mem_cfg_input = {
        .buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER,
    };

    int jpg_output_size = 720 * 240 * 2; // "in theory we could set this to some valid fraction of the original size, but rn idc" - Michael
    int merged_size = 720 * 240 * 2;

    jpeg_encoder_handle_t encoder_engine;

    size_t output_of_encoder_buffer_true_size;
    size_t merged_mem_size;

    uint8_t *merged_buf;


    public:
    uint32_t jpg_encoded_size = 0;
    uint8_t *jpg_encoder_output_buf;

    void init();
    
    void init_jpeg_engine();
    void init_jpeg_output_buf();
    void init_jpeg_merged_buffer();
    void merge_fields(bool a_odd, esp_video_buffer_element *elem_a, esp_video_buffer_element *elem_b);
    void clean_cache_and_memory();

    esp_err_t encode();
};

#endif