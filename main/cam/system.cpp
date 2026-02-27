#ifdef IS_CAM

#include <cam/system.h>
#include <errors.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define THREAD_STACK_SIZE_DEFAULT 4096

USBCDC USBSerial;
CAMSystems sys;

static void cmd_thread(CAMSystems* arg) {
    uint8_t cmd;
    while (true) {
        if (arg->b2b.dequeue(&cmd)) {
            switch (cmd) {
            case (uint8_t)B2BCommand::CAMERA0_OFF: {
                arg->serial->println("proc cmd: CAM1 OFF");
                camera_on_off(arg->cameras.cam1);
                delay(5000);
                arg->cameras.cam1.set_state(CAM_STATE_OFF);
                break;}
            case (uint8_t)B2BCommand::CAMERA0_ON: {
                arg->serial->println("proc cmd: CAM1 ON");
                arg->cameras.cam1.set_state(CAM_STATE_ON);
                break;}
            case (uint8_t)B2BCommand::CAMERA1_OFF: {
                arg->serial->println("proc cmd: CAM2 OFF");
                camera_on_off(arg->cameras.cam2);
                delay(5000);
                arg->cameras.cam2.set_state(CAM_STATE_OFF);
                break;}
            case (uint8_t)B2BCommand::CAMERA1_ON: {
                arg->serial->println("proc cmd: CAM2 ON");
                arg->cameras.cam2.set_state(CAM_STATE_ON);
                break;}
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

static void poll_thread(CAMSystems* arg) {
    size_t current_mem_cam1 = 0;
    size_t cam1_consecutive_invalid = 0;
    size_t current_mem_cam2 = 0;
    size_t cam2_consecutive_invalid = 0;

    while (true) {
        struct read_mem_cap_data_return toReturn1;
        toReturn1 = read_mem_cap_data(arg->cameras.cam1);
        if (toReturn1.status == 1) {
            arg->b2b.state.cam1_on = true;
            cam1_consecutive_invalid = 0;
            if(current_mem_cam1 == 0) {
                current_mem_cam1 = toReturn1.mem_size;
            } else {
                if (current_mem_cam1 != toReturn1.mem_size) {
                    arg->b2b.state.cam1_rec = true;
                } else {
                    arg->b2b.state.cam1_rec = false;
                }
                current_mem_cam1 = toReturn1.mem_size;
            }
        } else {
            cam1_consecutive_invalid++;
            if(cam1_consecutive_invalid >= 3) {
                arg->b2b.state.cam1_on = false;
                arg->b2b.state.cam1_rec = false;
            }
        }

        struct read_mem_cap_data_return toReturn2;
        toReturn2 = read_mem_cap_data(arg->cameras.cam2);
        if (toReturn2.status == 1) {
            arg->b2b.state.cam2_on = true;
            cam2_consecutive_invalid = 0;
            if(current_mem_cam2 == 0) {
                current_mem_cam2 = toReturn2.mem_size;
            } else {
                if (current_mem_cam2 != toReturn2.mem_size) {
                    arg->b2b.state.cam2_rec = true;
                } else {
                    arg->b2b.state.cam2_rec = false;
                }
                current_mem_cam2 = toReturn2.mem_size;
            }
        } else {
            cam2_consecutive_invalid++;
            if(cam2_consecutive_invalid >= 3) {
                arg->b2b.state.cam2_on = false;
                arg->b2b.state.cam2_rec = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// void on_frame_ready(uint32_t len, uint8_t *buf, CAMSystems* arg)
// {
//     // TODO: implement radio transmission
//     arg->serial->print("Frame ready, size: ");
//     arg->serial->println(len);
// }

bool light_on_frame = false;

// Frame protocol magic: signals start of a new JPEG frame
// Header packet layout (64 useful bytes):
//   [0..7]   magic
//   [8..11]  JPEG size (little-endian uint32_t)
//   [12..63] zeros
static const uint8_t FRAME_MAGIC[8] = {0xCA, 0x3E, 0xBE, 0xEF, 0x57, 0xA1, 0xD0, 0x92};

void on_frame_ready(uint32_t len, uint8_t *buf, CAMSystems* arg)
{
    light_on_frame = !light_on_frame;
    digitalWrite(LED_GREEN, light_on_frame);

    const uint8_t PACKET_SIZE = 120;
    const uint8_t GOOD_PL = 64;

    // Send header packet with magic + JPEG size
    uint8_t header[PACKET_SIZE] = {0};
    memcpy(header, FRAME_MAGIC, 8);
    memcpy(header + 8, &len, sizeof(uint32_t));
    arg->radio.sendFast(header, PACKET_SIZE);

    // Send JPEG data
    uint32_t total_sent = 0;
    while(total_sent < len) {
        CAMRadioStatus txStatus = arg->radio.sendFast(buf + total_sent, PACKET_SIZE);
        total_sent += GOOD_PL;
    }
    
    

    // CODE BELOW IS FOR JPEG SERIAL OUTPUT!
    // uint32_t off = 0;
    // uint8_t tmp[512];

    // light_on_frame = !light_on_frame;
    // digitalWrite(LED_GREEN, light_on_frame);

    // while (off < len)
    // {
    //     uint32_t chunk = len - off;

    //     if (chunk > 512)
    //         chunk = 512;

    //     // idk why, but writing directly from spiram works for the first couple chunks, then the data gets
    //     // offset or shifted, making the rest of the image super grainy / makes the colors wrong.
    //     // To fix this we'll move the frame frmo spiram to program memory in 512 byte chunks before sending them.
    //     memcpy(tmp, buf + off, chunk);

    //     size_t wrote = arg->serial->write(tmp, chunk);
    //     arg->serial->flush();

    //     if (wrote > 0)
    //     {
    //         off += wrote;
    //     }
    //     else
    //     {
    //         vTaskDelay(pdMS_TO_TICKS(1));
    //     }
    // }
}

static void video_thread(CAMSystems* arg) {
    while (true) {

        // Field A
        esp_video_buffer_element *elem_a = esp_video_recv_element(arg->video, V4L2_BUF_TYPE_VIDEO_CAPTURE, portMAX_DELAY);
        if (!elem_a)
        {
            while (1)
            {
            };
        }
        bool a_odd = !arg->tvp.tvp.read_field_sequence_status();

        // Field B
        esp_video_buffer_element *elem_b = esp_video_recv_element(arg->video, V4L2_BUF_TYPE_VIDEO_CAPTURE, portMAX_DELAY);
        if (!elem_b)
        {
            while (1)
            {
            };
        }

        bool b_odd = !arg->tvp.tvp.read_field_sequence_status();

        arg->JPEG.merge_fields(a_odd, elem_a, elem_b);

        esp_video_queue_element(arg->video, V4L2_BUF_TYPE_VIDEO_CAPTURE, elem_a);
        esp_video_queue_element(arg->video, V4L2_BUF_TYPE_VIDEO_CAPTURE, elem_b);

        arg->JPEG.clean_cache_and_memory();

        esp_err_t enc_ret = arg->JPEG.encode();

        if (enc_ret != ESP_OK)
        {
            arg->serial->print("JPEG encode failed: 0x");
            arg->serial->println(enc_ret, HEX);
        }
        else
        {
            // arg->serial->print("JPEG compressed size: ");
            // arg->serial->println(arg->JPEG.jpg_encoded_size);

            // arg->serial->print("*FRAME ");
            // arg->serial->println(arg->JPEG.jpg_encoded_size);

            //move to new file once radio is added
            on_frame_ready(arg->JPEG.jpg_encoded_size, arg->JPEG.jpg_encoder_output_buf, arg);
            // arg->serial->println("**DONE");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* Begin all system data */
[[noreturn]] void sys_begin() {

    digitalWrite(LED_ORANGE, HIGH);

    USB.begin();
    USBSerial.begin(115200);
    sys.serial = &USBSerial;

    sys.cameras.cam1.set_state(true); // TEMPORARY!!

    uint8_t tvp_res = sys.tvp.init();
    if (tvp_res != CAM_OK) {
        sys.serial->print("[sys_begin] tvp init failed: Code "); sys.serial->println(tvp_res); 
        digitalWrite(LED_RED, HIGH);
        while (1) {}
    }
    sys.serial->println("[sys_begin] tvp init OK");

    uint8_t b2b_res = sys.b2b.init();
    if (b2b_res != CAM_OK) {
        sys.serial->print("B2B Error: ");
        sys.serial->println(b2b_res);
        digitalWrite(LED_GREEN, HIGH);
        while (1) {}
    }
    sys.serial->println("[sys_begin] b2b init done");
    
    sys.video = DVP_init();
    start_dvp_capture(sys.video);

    sys.JPEG.init();

    while(!sys.tvp.tvp_locked()) {
        digitalWrite(LED_RED, HIGH);
        bool vsync_locked = sys.tvp.tvp.read_vertical_sync_lock_status();
        bool hsync_locked = sys.tvp.tvp.read_horizontal_sync_lock_status();
        bool color_locked = sys.tvp.tvp.read_color_subcarrier_lock_status();

        sys.serial->print(vsync_locked); sys.serial->print(hsync_locked); sys.serial->println(color_locked);
        delay(100);
    }

    digitalWrite(LED_RED, LOW);

    // Radio begin
    CAMRadioStatus radio_status = sys.radio.init(SPI);
    if(radio_status != CAMRadioStatus::CAMRADIO_OK) {
        sys.serial->print("[sys_begin] Radio init fail: "); sys.serial->println(radio_status);
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_ORANGE, HIGH);
        while(1) {};
    }

    // From the old code -- seems like we discard some frames. I don't exactly know the logic behind it, but removing
    // this code corrupts video data. strange!
    for (int i = 0; i < 5; i++)
    {
        esp_video_buffer_element *discard = esp_video_recv_element(sys.video, V4L2_BUF_TYPE_VIDEO_CAPTURE, portMAX_DELAY);
        if (discard)
        {
            sys.serial->printf("Discarded frame %d (%lu bytes)\n", i, (unsigned long)discard->valid_size);
            esp_video_queue_element(sys.video, V4L2_BUF_TYPE_VIDEO_CAPTURE, discard);
        }
    }

    xTaskCreatePinnedToCore((TaskFunction_t) cmd_thread, "cmdq", THREAD_STACK_SIZE_DEFAULT, &sys, 7, nullptr, 0);
    xTaskCreatePinnedToCore((TaskFunction_t) poll_thread, "poll", THREAD_STACK_SIZE_DEFAULT, &sys, 5, nullptr, 0);
    xTaskCreatePinnedToCore((TaskFunction_t) video_thread, "video", THREAD_STACK_SIZE_DEFAULT, &sys, 5, nullptr, 0);

    digitalWrite(LED_ORANGE, LOW);
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif
