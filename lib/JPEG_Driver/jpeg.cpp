#include "jpeg.h"

/**
 * Detect if a UYVY byte stream is misaligned by 1 byte.
 *
 * Two heuristics are combined:
 *
 *  (A) VARIANCE: Natural images have higher spatial variance in luma (Y)
 *      than chroma (U/V). Correct UYVY puts Y at odd bytes; if even bytes
 *      show more variance instead, the stream is shifted by 1.
 *
 *  (B) CHROMA NEUTRALITY: True U/V values cluster near 128. When luma gets
 *      misread as chroma it deviates from 128 by a scene-brightness-dependent
 *      amount — this is the direct cause of the green/pink cast you see.
 *      Whichever hypothesis produces "chroma" bytes closer to 128 wins.
 *
 * Returns 1 if a 1-byte shift is needed, 0 if OK.
 */
static int detect_uyvy_byte_offset(const uint8_t *buf, int buf_len)
{
    /* Use a full row (720 pixels × 2 bytes) — much more robust than 256 bytes */
    const int SAMPLE = 1440;
    int n = (buf_len < SAMPLE) ? buf_len : SAMPLE;

    /* ── (A) Variance of even vs odd byte positions ──────────────────────── */
    int64_t sum_e = 0, sumsq_e = 0;
    int64_t sum_o = 0, sumsq_o = 0;

    for (int i = 0; i < n; i++)
    {
        int32_t v = buf[i];
        if (i & 1)
        {
            sum_o += v;
            sumsq_o += v * v;
        }
        else
        {
            sum_e += v;
            sumsq_e += v * v;
        }
    }

    int h = n / 2;
    /* Scaled variance (proportional to true variance × h²), avoids division */
    int64_t var_e = sumsq_e * h - sum_e * sum_e;
    int64_t var_o = sumsq_o * h - sum_o * sum_o;
    /* Correct UYVY: Y at odd → var_o > var_e; offset: var_e > var_o */
    int vote_A = (var_e > var_o) ? 1 : 0;

    /* ── (B) Mean |chroma_byte − 128| for each hypothesis ───────────────── */
    int64_t dev_e = 0, dev_o = 0;
    int n4 = (n / 4) * 4; /* round down to UYVY 4-byte boundary */

    for (int i = 0; i < n4; i += 4)
    {
        /* Hypothesis: aligned  → U = buf[i],   V = buf[i+2]  (even bytes) */
        dev_e += abs((int)buf[i] - 128) + abs((int)buf[i + 2] - 128);
        /* Hypothesis: offset   → U = buf[i+1], V = buf[i+3]  (odd bytes)  */
        dev_o += abs((int)buf[i + 1] - 128) + abs((int)buf[i + 3] - 128);
    }
    /* Smaller deviation from 128 → that hypothesis is more likely correct */
    int vote_B = (dev_o < dev_e) ? 1 : 0;

    /* Both agree → confident. They disagree (borderline scene) → trust variance. */
    if (vote_A == vote_B)
        return vote_A;
    return vote_A; /* tiebreaker: variance is more scene-independent */
}

void jpeg_encoder::init()
{
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
        // hm
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