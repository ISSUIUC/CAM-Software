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
    const int ROW_BYTES = 1440;
    int total_rows = buf_len / ROW_BYTES;

    // Two rows from middle third instead of start of buffer
    int row_a = total_rows / 3;
    int row_b = total_rows * 2 / 3;
    const uint8_t *rows[2] = {buf + row_a * ROW_BYTES,
                              buf + row_b * ROW_BYTES};

    int64_t sum_e = 0, sumsq_e = 0, sum_o = 0, sumsq_o = 0;
    int64_t dev_e = 0, dev_o = 0;

    for (int r = 0; r < 2; r++)
    {
        const uint8_t *row = rows[r];
        int n4 = (ROW_BYTES / 4) * 4;
        for (int i = 0; i < ROW_BYTES; i++)
        {
            int32_t v = row[i];
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
        for (int i = 0; i < n4; i += 4)
        {
            dev_e += abs((int)row[i] - 128) + abs((int)row[i + 2] - 128);
            dev_o += abs((int)row[i + 1] - 128) + abs((int)row[i + 3] - 128);
        }
    }

    int64_t count = ROW_BYTES; // total even or odd bytes across both rows
    int64_t var_e = sumsq_e * count - sum_e * sum_e;
    int64_t var_o = sumsq_o * count - sum_o * sum_o;
    int vote_A = (var_e > var_o) ? 1 : 0;
    int vote_B = (dev_o < dev_e) ? 1 : 0;

    return (vote_A == vote_B) ? vote_A : vote_A; // variance wins ties
}

// Line stride search window in bytes: 720 px +/- 4 px
#define MIN_LINE_STRIDE (716 * 2)
#define MAX_LINE_STRIDE (724 * 2)

// A new stride is adopted once this many fields in a row agree on it
#define STRIDE_CONFIRM_FIELDS 3

/*
this measures how many bytes are stored per line on the dvp
Line-to-line distance is how many SCLK counts the tvp5151 had when avid is high
- bt656 mode: 1444 clock cycles (that is where the extra 4 bytes are)
- discrete sync mode is 1440

If we have the right stride there should be a vertical line somewhere
This vertical line is dark so its brightness would be low (y value)
so what we do is we take the sum of all the values in each column
then we measure the differences between each column

Something like this:
If like bright pixels were 100 and then dark pixels (the ones we are looking for are 10 pixels)
and then we have FOUR rows
correct guess:  column sums = 400 400 400 400 400  40 400 400 400
wrong guess:    column sums = 400 400 310 310 310 310 400 400 400

*/
static int measure_line_stride(const uint8_t *field, int field_len)
{
    const int FIRST_LINE = 60; // skip 1st line cuz it is a bit iffy sometimes
    const int LINES = 32;
    const int COLUMNS = 720 * 2;

    // make sure we aren't like going over the buffer we are given
    if ((FIRST_LINE + LINES) * MAX_LINE_STRIDE > field_len)
        return 0;

    // reposition pointer to be where we want to start (second lin)
    const uint8_t *block = field + FIRST_LINE * COLUMNS;

    uint32_t sharpness[MAX_LINE_STRIDE - MIN_LINE_STRIDE + 1]; // slots to put each stride's sharpness value
    uint16_t column_sum[COLUMNS];
    int best = 718 * 2; // what we have seen before lol

    for (int stride = MIN_LINE_STRIDE; stride <= MAX_LINE_STRIDE; stride++)
    {
        memset(column_sum, 0, sizeof(column_sum));
        for (int i = 0; i < LINES; i++)
        {
            const uint8_t *line = block + i * stride;
            for (int x = 0; x < COLUMNS; x++)
            {
                column_sum[x] += line[x];
            }
        }

        uint32_t edges = 0;
        for (int x = 4; x < COLUMNS; x++) // 4 bytes back is the same U/Y/V lane, 2 px left
        {
            edges += abs(column_sum[x] - column_sum[x - 4]);
        }

        sharpness[stride - MIN_LINE_STRIDE] = edges;
        if (edges > sharpness[best - MIN_LINE_STRIDE])
        {
            best = stride;
        }
    }

    return best;
}

//  SAV: From google:
//  "An ITU-R BT.656 SAV (Start of Active Video) code is a
//  four-byte timing reference sequence embedded directly into a
//  digital video data stream to mark where active pixel data begins
//  It is a 4 byte offset PER LINE
//  Fixed Preamble: The first three bytes of every SAV code are always FFh, 00h, and 00h"
//  Interpretation:
//  When the tvp is in bt656 mode, avid goes high 4 clocks early which means the dvp capture the 4 extra bytes
//  the end-of-video marker is sent after avid goes low so dvp never captures it
static bool is_sav(const uint8_t *p)
{
    return p[0] == 0xFF && p[1] == 0x00 && p[2] == 0x00;
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

void jpeg_encoder::update_line_stride(const uint8_t *field, int field_len)
{
    int measured = measure_line_stride(field, field_len);
    if (measured == 0)
        return;

    // we don't want to lock onto one stride with just one image we want at least THREE
    // frames/fields that have that stride
    if (measured != stride_candidate)
    {
        stride_candidate = measured;
        stride_candidate_fields = 0;
    }

    // so now we just mark the seen fields as ++
    stride_candidate_fields++;

    if (stride_candidate_fields >= STRIDE_CONFIRM_FIELDS)
    {
        line_stride = measured;
    }
}

void jpeg_encoder::merge_fields(bool a_odd, esp_video_buffer_element *elem_a, esp_video_buffer_element *elem_b)
{
    uint8_t *odd_field = a_odd ? elem_a->buffer : elem_b->buffer;
    // uint8_t *even_field = a_odd ? elem_b->buffer : elem_a->buffer;
    const int field_len = 720 * 480 * 2;

    update_line_stride(odd_field, field_len);

    // sav present means that we are in bt656 and then the offset would be 4 bytes (which matches with the
    // (720 * 2 - 718 * 2) = 2 * 2 = 4 "byte offset" magic number we had before)
    // we check the next line because this sav is for each line, and it confirms that our line stride is right
    sav_present = is_sav(odd_field) && is_sav(odd_field + line_stride);
    if (is_sav(odd_field) && is_sav(odd_field + line_stride))
    {
        uyvy_offset = 4;
    }
    else
    {
        // lowkey we just bash it out
        uyvy_offset = detect_uyvy_byte_offset(odd_field, 720 * 240 * 2);
    }

    // this code takes every other macropixel column to convert the image from 720 x 240 pixels to 360 x 240 pixels
    const int dst_row = 360 * 2;
    for (int i = 0; i < 240; i++)
    {
        const uint8_t *src = odd_field + uyvy_offset + i * line_stride;
        uint8_t *dst = merged_buf + i * dst_row;
        for (int j = 0; j < 180; j++) // 180 macro pixels (1 macro pixel = 2 pxiels) = 360 pixels
        {
            memcpy(dst + j * 4, src + j * 8, 4); // yuyv encodes 1 pixel every 2 bytes, 4 bytes every 2 pixels
        }
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