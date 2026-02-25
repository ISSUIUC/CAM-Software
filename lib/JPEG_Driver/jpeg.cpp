#include "jpeg.h"

void jpeg_encoder::init() {
    init_jpeg_engine();

    init_jpeg_output_buf();

    init_jpeg_merged_buffer();
}

void jpeg_encoder::init_jpeg_engine()
{
    jpeg_new_encoder_engine(&encode_eng_cfg, &encoder_engine);
}

void jpeg_encoder::init_jpeg_output_buf()
{
    jpg_encoder_output_buf = (uint8_t *)jpeg_alloc_encoder_mem(jpg_output_size, &mem_cfg_output, &output_of_encoder_buffer_true_size);
    if (!jpg_encoder_output_buf)
    {
        Serial.println("Jpg buf 1080 alloc memo failed");
        while (1)
            ;
    }
}

void jpeg_encoder::init_jpeg_merged_buffer()
{
    merged_buf = (uint8_t *)jpeg_alloc_encoder_mem(merged_size, &mem_cfg_input, &merged_mem_size);
    if (merged_buf == 0)
    {
        Serial.println("alloc fail while merging");
        while (1)
        {
        };
    }

    esp_cache_msync(merged_buf, merged_mem_size,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_INVALIDATE);
}

void jpeg_encoder::merge_fields(bool a_odd, esp_video_buffer_element *elem_a, esp_video_buffer_element *elem_b)
{
    uint8_t *odd_field = a_odd ? elem_a->buffer : elem_b->buffer;
    uint8_t *even_field = a_odd ? elem_b->buffer : elem_a->buffer;

    const int row_stride = 720 * 2;
    for (int i = 0; i < 240; i++)
    {
        memcpy(merged_buf + (2 * i) * row_stride, odd_field + i * row_stride, row_stride);
        uint8_t *dst = merged_buf + (2 * i + 1) * row_stride;
        uint8_t *src = even_field + i * row_stride;
        memcpy(dst, src, row_stride);
    }
}

void jpeg_encoder::clean_cache_and_memory()
{
    // Need to invalidate ESP cache for the frame to render properly.
    esp_cache_msync(merged_buf, merged_mem_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_INVALIDATE);
    esp_cache_msync(jpg_encoder_output_buf, output_of_encoder_buffer_true_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_INVALIDATE);
}

esp_err_t jpeg_encoder::encode()
{
    esp_err_t err = jpeg_encoder_process(encoder_engine, &enc_config, merged_buf, merged_size, jpg_encoder_output_buf, jpg_output_size, &jpg_encoded_size);
    return err;
}