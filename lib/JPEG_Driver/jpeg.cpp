#include "jpeg.h"

/**
 * Detect if a UYVY byte stream is misaligned by 1 byte.
 *
 * In correct UYVY, even-indexed bytes alternate U and V (large gradient
 * between same-parity neighbors at stride 2), while odd-indexed bytes
 * are sequential luma Y values (smooth, small gradient).  A 1-byte
 * offset swaps these roles, putting luma at even indices and chroma at
 * odd — which makes the JPEG encoder produce green/pink output.
 *
 * Returns 1 if a 1-byte shift is needed to restore alignment, 0 if OK.
 */
static int detect_uyvy_byte_offset(const uint8_t *buf, int buf_len)
{
    const int SAMPLE = 256;
    int n = (buf_len < SAMPLE) ? buf_len : SAMPLE;
    int32_t even_grad = 0;
    int32_t odd_grad = 0;

    for (int i = 0; i + 2 < n; i++)
    {
        int d = (int)buf[i + 2] - (int)buf[i];
        if (d < 0)
            d = -d;
        if (i & 1)
            odd_grad += d;
        else
            even_grad += d;
    }

    // Correct UYVY: even_grad >> odd_grad (U/V alternation vs smooth Y)
    // Misaligned:   even_grad << odd_grad (smooth Y at even, U/V at odd)
    return (even_grad < odd_grad) ? 1 : 0;
}

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

    // Detect UYVY byte misalignment that causes green/pink color artifacts.
    // If the DVP captured starting 1 byte off, luma and chroma swap roles.
    int offset = detect_uyvy_byte_offset(odd_field, 720 * 2);

    const int row_stride = 718 * 2;
    for (int i = 0; i < 240; i++)
    {
        // copy from original at 0, 720, ...
        // to 0, 718, ...
        int orig_pos = i * 2 * 720 + offset;
        int new_pos = i * 718 * 2;

        memcpy(merged_buf + new_pos, odd_field + orig_pos, row_stride);
        // uint8_t *dst = merged_buf + (2 * i + 1) * row_stride;
        // uint8_t *src = even_field + i * row_stride;
        // memcpy(dst, src, row_stride);
    }

    memset(merged_buf + 239 * 718 * 2, 0, 2 * 239);
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