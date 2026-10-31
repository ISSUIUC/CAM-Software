#ifdef IS_CAM

#include <cam/system.h>
#include <errors.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#define THREAD_STACK_SIZE_DEFAULT 4096

USBCDC USBSerial;
CAMSystems sys;

// DVP frame-flow counters, defined in components/esp_video/src/device/esp_video_dvp_device.c
extern "C" volatile uint32_t g_dvp_eof_count;
extern "C" volatile uint32_t g_dvp_starved_count;

// VSYNC probe: watches the TVP's VSYNC pin independently of the DVP, so a dead capture can be told
// apart from a dead TVP. A field longer than ~30 ms (480 lines of AVID data) does not fit in one
// 720x480x2 DVP buffer, so long gaps are logged along with the DMA EOF count when they ended.
#define VSYNC_LONG_GAP_US 22000
#define VSYNC_GAP_LOG_LEN 16

struct vsync_gap_t
{
    int64_t end_us;
    int32_t gap_us;
    uint32_t eof_count;
};

static volatile uint32_t vsync_edges = 0;
static volatile int64_t vsync_last_us = 0;
static vsync_gap_t vsync_gap_log[VSYNC_GAP_LOG_LEN];
static volatile uint32_t vsync_gap_count = 0;

static void IRAM_ATTR vsync_isr(void *)
{
    int64_t now = esp_timer_get_time();
    if (vsync_last_us != 0 && now - vsync_last_us > VSYNC_LONG_GAP_US)
    {
        vsync_gap_t &gap = vsync_gap_log[vsync_gap_count % VSYNC_GAP_LOG_LEN];
        gap.end_us = now;
        gap.gap_us = (int32_t)(now - vsync_last_us);
        gap.eof_count = g_dvp_eof_count;
        vsync_gap_count = vsync_gap_count + 1;
    }
    vsync_last_us = now;
    vsync_edges = vsync_edges + 1;
}

static void start_vsync_probe()
{
    gpio_install_isr_service(0); // ESP_ERR_INVALID_STATE if already installed, which is fine
    gpio_set_intr_type((gpio_num_t)TVP5151_VSYNC, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)TVP5151_VSYNC, vsync_isr, nullptr);
}

static void print_new_vsync_gaps(CAMSystems *arg, uint32_t &printed)
{
    uint32_t count = vsync_gap_count;
    if (count - printed > VSYNC_GAP_LOG_LEN)
    {
        printed = count - VSYNC_GAP_LOG_LEN;
    }
    for (; printed < count; printed++)
    {
        const vsync_gap_t &gap = vsync_gap_log[printed % VSYNC_GAP_LOG_LEN];
        arg->serial->printf("[dvp] long VSYNC gap %.1f ms at t=%.3f s (dma eof %lu)\n",
                            gap.gap_us / 1000.0, gap.end_us / 1e6, (unsigned long)gap.eof_count);
    }
}

static void cmd_thread(CAMSystems *arg)
{
    uint8_t cmd;
    while (true)
    {
        if (arg->b2b.dequeue(&cmd))
        {
            arg->serial->print("CMD: ");
            arg->serial->println(cmd);
            switch (cmd)
            {
            case (uint8_t)B2BCommand::CAMERA0_OFF:
            {
                arg->serial->println("proc cmd: CAM1 OFF");
                camera_on_off(arg->cameras.cam1);
                delay(5000);
                arg->cameras.cam1.set_state(CAM_STATE_OFF);
                break;
            }
            case (uint8_t)B2BCommand::CAMERA0_ON:
            {
                arg->serial->println("proc cmd: CAM1 ON");
                arg->cameras.cam1.set_state(CAM_STATE_ON);
                break;
            }
            case (uint8_t)B2BCommand::CAMERA1_OFF:
            {
                arg->serial->println("proc cmd: CAM2 OFF");
                camera_on_off(arg->cameras.cam2);
                delay(5000);
                arg->cameras.cam2.set_state(CAM_STATE_OFF);
                break;
            }
            case (uint8_t)B2BCommand::CAMERA1_ON:
            {
                arg->serial->println("proc cmd: CAM2 ON");
                arg->cameras.cam2.set_state(CAM_STATE_ON);
                break;
            }
            case (uint8_t)B2BCommand::MUX_0:
                arg->serial->println("proc cmd: MUX_0");
                arg->tvp.source_select(CAM1);
                arg->b2b.state.vmux_state = false;
                break;
            case (uint8_t)B2BCommand::MUX_1:
                arg->serial->println("proc cmd: MUX_1");
                arg->tvp.source_select(CAM2);
                arg->b2b.state.vmux_state = true;
                break;
            default:
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void poll_thread(CAMSystems *arg)
{
    size_t current_mem_cam1 = 0;
    size_t cam1_consecutive_invalid = 0;
    size_t current_mem_cam2 = 0;
    size_t cam2_consecutive_invalid = 0;

    while (true)
    {
        struct read_mem_cap_data_return toReturn1;
        toReturn1 = read_mem_cap_data(arg->cameras.cam1);
        if (toReturn1.status == 1)
        {
            arg->serial->println("Cam1 valid");
            arg->b2b.state.cam1_on = true;
            cam1_consecutive_invalid = 0;
            if (current_mem_cam1 == 0)
            {
                current_mem_cam1 = toReturn1.mem_size;
            }
            else
            {
                arg->serial->print(current_mem_cam1);
                arg->serial->print("/");
                arg->serial->println(toReturn1.mem_size);
                if (current_mem_cam1 != toReturn1.mem_size)
                {
                    arg->b2b.state.cam1_rec = true;
                }
                else
                {
                    arg->b2b.state.cam1_rec = false;
                }
                current_mem_cam1 = toReturn1.mem_size;
            }
        }
        else
        {
            arg->serial->println("Cam1 Invalid");
            arg->serial->println(cam1_consecutive_invalid);
            cam1_consecutive_invalid++;
            if (cam1_consecutive_invalid >= 3)
            {
                arg->b2b.state.cam1_on = false;
                arg->b2b.state.cam1_rec = false;
            }
        }

        struct read_mem_cap_data_return toReturn2;
        toReturn2 = read_mem_cap_data(arg->cameras.cam2);
        if (toReturn2.status == 1)
        {
            arg->serial->println("CAM2 Valid");
            arg->b2b.state.cam2_on = true;
            cam2_consecutive_invalid = 0;
            if (current_mem_cam2 == 0)
            {
                current_mem_cam2 = toReturn2.mem_size;
            }
            else
            {
                arg->serial->print(current_mem_cam2);
                arg->serial->print("/");
                arg->serial->println(toReturn2.mem_size);
                if (current_mem_cam2 != toReturn2.mem_size)
                {
                    arg->b2b.state.cam2_rec = true;
                }
                else
                {
                    arg->b2b.state.cam2_rec = false;
                }
                current_mem_cam2 = toReturn2.mem_size;
            }
        }
        else
        {
            arg->serial->println("CAM2 Invalid");
            arg->serial->println(cam2_consecutive_invalid);
            cam2_consecutive_invalid++;
            if (cam2_consecutive_invalid >= 3)
            {
                arg->b2b.state.cam2_on = false;
                arg->b2b.state.cam2_rec = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

bool light_on_frame = false;
uint8_t frame_id = 0;

void on_frame_ready(uint32_t len, uint8_t *buf, CAMSystems *arg)
{
    arg->serial->println("on frame ready called!");
    if (len == 0)
        return;

    uint16_t n = frame_count(len);
    // arg->serial->print("[tx] frame count output: ");
    // arg->serial->println(n);
    if (n == 0)
        return;
    if (n > 100) // shouldn't happen... hmm
        return;

    light_on_frame = !light_on_frame;
    digitalWrite(LED_GREEN, light_on_frame);
    arg->b2b.state.vmux_state = !arg->b2b.state.vmux_state;

    LR2021Driver *driver = arg->radio.getDriver();

    uint8_t (*pool)[RADIO_PACKET_SIZE] = (uint8_t (*)[RADIO_PACKET_SIZE])
        heap_caps_malloc((size_t)n * RADIO_PACKET_SIZE, MALLOC_CAP_SPIRAM);

    if (!pool)
    {
        arg->serial->println("[tx] OOM building frame");
        return;
    }

    frame_id++;

    Frame frame = frame_build(frame_id, buf, len, pool, n);

    if (frame.valid)
    {
        arg->serial->println("[tx] Frame build successfully.");
        LR2021Error err = frame_transmit(*driver, frame);

        if (!err.ok())
        {
            arg->serial->print("[tx] burst failed: ");
            arg->serial->println(err.stageStr());
        }
    }

    if (frame_id % 10 == 0)
    {
        arg->serial->printf("[heap] frame=%lu free=%lu min=%lu\n",
                            (unsigned long)frame_id,
                            (unsigned long)esp_get_free_heap_size(),
                            (unsigned long)esp_get_minimum_free_heap_size());
    }
    // arg->serial->println("TX Done");

    heap_caps_free(pool);

    RSLayout layout;
    layout.compute(len, FRAG_DATA_SIZE);
    uint16_t numParity = layout.numBlocks * RS_PARITY_SHARDS;

    uint8_t *parity_flat = (uint8_t *)heap_caps_malloc(
        (uint32_t)numParity * FRAG_DATA_SIZE, MALLOC_CAP_SPIRAM);
    uint8_t (*pPool)[RADIO_PACKET_SIZE] = (uint8_t (*)[RADIO_PACKET_SIZE])
        heap_caps_malloc((size_t)numParity * RADIO_PACKET_SIZE, MALLOC_CAP_SPIRAM);

    if (parity_flat && pPool)
    {
        Frame pFrame = parity_frame_build(
            frame_id, buf, len, layout, parity_flat, pPool);
        if (pFrame.valid)
        {
            LR2021Error err = frame_transmit(*driver, pFrame);
            if (!err.ok())
                arg->serial->printf("[tx] parity burst failed: %s\n", err.stageStr());
        }
    }
    heap_caps_free(parity_flat);
    heap_caps_free(pPool);
}

// The TVP free-runs at ~60 fields/s even with no camera input, so this many empty 100 ms waits
// in a row means the DVP capture itself has stopped
#define DVP_STALL_WAITS 3

static void recover_stalled_dvp(CAMSystems *arg, uint32_t vsync_at_last_field, uint32_t eof_at_last_field)
{
    static uint32_t stalls = 0;
    uint32_t eof_now = g_dvp_eof_count;

    // vsync still counting + dma eof frozen = TVP is fine, the P4 capture DMA is dead.
    // A long VSYNC gap logged with this same dma eof count is the field that overflowed it.
    arg->serial->printf("[dvp] STALL #%lu: no field for %d ms | vsync +%lu, dma eof +%lu (frozen at %lu), starved %lu | TVP V%d H%d, %u lines\n",
                        (unsigned long)++stalls, DVP_STALL_WAITS * 100,
                        (unsigned long)(vsync_edges - vsync_at_last_field),
                        (unsigned long)(eof_now - eof_at_last_field), (unsigned long)eof_now,
                        (unsigned long)g_dvp_starved_count,
                        arg->tvp.tvp.read_vertical_sync_lock_status(),
                        arg->tvp.tvp.read_horizontal_sync_lock_status(),
                        arg->tvp.tvp.read_vertical_line_count());

    esp_err_t err = restart_dvp_capture(arg->video);
    if (err != ESP_OK)
    {
        arg->serial->printf("[dvp] capture restart failed: %s\n", esp_err_to_name(err));
    }
    else
    {
        arg->serial->println("[dvp] capture restarted");
    }
}

static void video_thread(CAMSystems *arg)
{
    uint32_t empty_waits = 0;
    uint32_t vsync_at_last_field = vsync_edges;
    uint32_t eof_at_last_field = g_dvp_eof_count;
    uint32_t vsync_gaps_printed = 0;

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        print_new_vsync_gaps(arg, vsync_gaps_printed);

        bool v_lock = arg->tvp.tvp.read_vertical_sync_lock_status();
        bool h_lock = arg->tvp.tvp.read_horizontal_sync_lock_status();
        bool avid_signal = digitalRead(TVP5151_AVID);
        arg->serial->print("V-Lock: ");
        arg->serial->print(v_lock);
        arg->serial->print(" H-Lock: ");
        arg->serial->print(h_lock);
        arg->serial->print(" AVID: ");
        arg->serial->println(avid_signal);

        arg->serial->println("field a and b start.");

        // Field A
        esp_video_buffer_element *elem_a = esp_video_recv_element(arg->video, V4L2_BUF_TYPE_VIDEO_CAPTURE, pdMS_TO_TICKS(100));
        if (!elem_a)
        {
            if (++empty_waits >= DVP_STALL_WAITS)
            {
                recover_stalled_dvp(arg, vsync_at_last_field, eof_at_last_field);
                empty_waits = 0;
                vsync_at_last_field = vsync_edges;
                eof_at_last_field = g_dvp_eof_count;
            }
            continue;
        }
        empty_waits = 0;
        vsync_at_last_field = vsync_edges;
        eof_at_last_field = g_dvp_eof_count;

        arg->serial->println("elem a recieved");

        bool a_odd = !arg->tvp.tvp.read_field_sequence_status();

        arg->serial->println("field a end");

        // Field B
        esp_video_buffer_element *elem_b = esp_video_recv_element(arg->video, V4L2_BUF_TYPE_VIDEO_CAPTURE, pdMS_TO_TICKS(100));
        if (!elem_b)
        {
            // clear A
            esp_video_queue_element(arg->video, V4L2_BUF_TYPE_VIDEO_CAPTURE, elem_a);
            continue;
        }

        // arg->serial->println("field a and b end.");

        bool b_odd = !arg->tvp.tvp.read_field_sequence_status();

        // arg->serial->println("JPEG.merge_fields() start.");

        arg->JPEG.merge_fields(a_odd, elem_a, elem_b);

        // arg->serial->println("JPEG.merge_fields() done.");

        esp_video_queue_element(arg->video, V4L2_BUF_TYPE_VIDEO_CAPTURE, elem_a);
        esp_video_queue_element(arg->video, V4L2_BUF_TYPE_VIDEO_CAPTURE, elem_b);

        arg->JPEG.clean_cache_and_memory();

        // arg->serial->println("JPEG.encode() start.");

        esp_err_t enc_ret = arg->JPEG.encode();

        // arg->serial->println("JPEG.encode() done.");

        if (enc_ret != ESP_OK)
        {
            arg->serial->print("JPEG encode failed: 0x");
            arg->serial->println(enc_ret, HEX);
        }
        else
        {
            arg->serial->print("JPEG compressed size: ");
            arg->serial->println(arg->JPEG.jpg_encoded_size);

            // arg->serial->print("*FRAME ");
            // arg->serial->println(arg->JPEG.jpg_encoded_size);

            // move to new file once radio is added

            if (!arg->tvp.tvp_locked())
            {
                digitalWrite(LED_RED, HIGH);
                arg->b2b.state.vtx_state = false;
                vTaskDelay(pdMS_TO_TICKS(10));
                arg->serial->println("help me");
                continue;
            }
            arg->b2b.state.vtx_state = true; // VTX state reports image phase lock (for now)

            digitalWrite(LED_RED, LOW);
            on_frame_ready(arg->JPEG.jpg_encoded_size, arg->JPEG.jpg_encoder_output_buf, arg);
            // arg->serial->println("**DONE");
        }
    }
}

#define I2C_RECOVERY_STATE_THRESHOLD 3000
#define I2C_RECOVERY_FALLBACK_TIME 180000 // 180 seconds
uint32_t LAST_I2C_COMM;
static void comm_thread(CAMSystems *arg)
{
    bool is_in_recovery_state = false;
    bool is_in_fallback_state = false;
    uint32_t time_entered_fallback_state = millis();
    uint32_t recovery_debounce = millis();
    uint32_t fail_detect_debounce = millis();

    while (true)
    {
        uint32_t cur_time = millis();
        // The other half of I2C stuff -- checking communication.
        if (is_in_recovery_state)
        {
            if (cur_time > recovery_debounce)
            {
                arg->serial->println("Attempting recovery...");
                recovery_debounce = cur_time + 5000; // Only attempt recovery every 5 seconds.
                // Attempt recovery.
                arg->b2b.reinit();
                arg->serial->println("Sleeping for recovery test...");
                delay(500);

                if (millis() - LAST_I2C_COMM <= I2C_RECOVERY_STATE_THRESHOLD)
                {
                    arg->serial->println("Successfully recovered!");
                    digitalWrite(LED_ORANGE, LOW);
                    fail_detect_debounce = millis() + 3000; // After successful recovery, we only detect fault after 3 more sec
                    is_in_recovery_state = false;
                    is_in_fallback_state = false;
                    digitalWrite(LED_RED, LOW);
                }
                else
                {
                    arg->b2b.deinit();
                    pinMode(B2B_I2C_SCL, INPUT);
                    pinMode(B2B_I2C_SDA, INPUT);
                    arg->serial->println("Failed to recover.");
                }
            }

            if (cur_time - time_entered_fallback_state > I2C_RECOVERY_FALLBACK_TIME && !is_in_fallback_state)
            {
                is_in_fallback_state = true;
                // arg->buzzer.play_tune(beep_beep, BEEP_LENGTH);
                delay(50);
                digitalWrite(CAM1_ON_OFF, HIGH);
                delay(50);
                digitalWrite(CAM2_ON_OFF, HIGH);
                delay(50);
                // digitalWrite(VTX_ON_OFF, HIGH);
                // digitalWrite(VIDEO_SELECT, LOW);

                // DESIRED_CAM_STATE.cam1_on = true;
                // DESIRED_CAM_STATE.cam2_on = true;
                // DESIRED_CAM_STATE.cam1_rec = true;
                // DESIRED_CAM_STATE.cam2_rec = true;
                // DESIRED_CAM_STATE.vtx_on = true;
                // DESIRED_CAM_STATE.vmux_state = false;
            }
        }

        if (cur_time - LAST_I2C_COMM > I2C_RECOVERY_STATE_THRESHOLD && !is_in_recovery_state && cur_time > fail_detect_debounce)
        {
            is_in_recovery_state = true;
            time_entered_fallback_state = cur_time;
            digitalWrite(LED_ORANGE, HIGH);
            arg->serial->println("I2C Comms not seen!");
            arg->b2b.deinit();

            pinMode(B2B_I2C_SCL, INPUT);
            pinMode(B2B_I2C_SDA, INPUT);
            arg->serial->println("Entered recovery mode");
        }
        // Serial.println("bruh5");
        delay(10);
        // Serial.println("bruh6");
    }
}

/* Begin all system data */
[[noreturn]] void sys_begin()
{

    digitalWrite(LED_ORANGE, HIGH);

    USB.begin();
    USBSerial.begin(115200);
    sys.serial = &USBSerial;

    // Power on camera BEFORE TVP init so it locks onto a real video signal
    sys.cameras.cam1.set_state(CAM_STATE_ON);
    // sys.cameras.cam2.set_state(CAM_STATE_ON);

    uint8_t tvp_res = sys.tvp.init();
    if (tvp_res != CAM_OK)
    {
        sys.serial->print("[sys_begin] tvp init failed: Code ");
        sys.serial->println(tvp_res);
        digitalWrite(LED_RED, HIGH);
        while (1)
        {
        }
    }
    sys.serial->println("[sys_begin] tvp init OK");

    uint8_t b2b_res = sys.b2b.init();
    if (b2b_res != CAM_OK)
    {
        sys.serial->print("B2B Error: ");
        sys.serial->println(b2b_res);
        digitalWrite(LED_GREEN, HIGH);
        while (1)
        {
        }
    }
    sys.serial->println("[sys_begin] b2b init done");

    sys.video = DVP_init();
    start_dvp_capture(sys.video);
    start_vsync_probe();

    sys.JPEG.init();

    // Radio begin
    CAMRadioStatus radio_status = sys.radio.init(SPI, sys.serial);
    if (radio_status != CAMRadioStatus::CAMRADIO_OK)
    {
        sys.serial->print("[sys_begin] Radio init fail: ");
        sys.serial->println(radio_status);
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_ORANGE, HIGH);
        while (1)
        {
        };
    }

    xTaskCreatePinnedToCore((TaskFunction_t)cmd_thread, "cmdq", THREAD_STACK_SIZE_DEFAULT, &sys, 10, nullptr, CORE_1);
    xTaskCreatePinnedToCore((TaskFunction_t)poll_thread, "poll", THREAD_STACK_SIZE_DEFAULT, &sys, 5, nullptr, CORE_1);
    xTaskCreatePinnedToCore((TaskFunction_t)comm_thread, "comm", THREAD_STACK_SIZE_DEFAULT, &sys, 8, nullptr, CORE_1);
    xTaskCreatePinnedToCore((TaskFunction_t)video_thread, "video", THREAD_STACK_SIZE_DEFAULT * 8, &sys, 5, nullptr, CORE_0);

    digitalWrite(LED_ORANGE, LOW);

    tone(BUZZER_PIN, 2700, 500);

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif
