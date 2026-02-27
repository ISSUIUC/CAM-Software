#ifdef IS_EAGLE

#include <eagle/system.h>

#define THREAD_STACK_SIZE_DEFAULT 4096


EAGLESystems sys;

#ifdef USE_USB_DEBUG
USBCDC USBSerial;
#endif

bool bl_stat = false;

uint8_t *frame_buffer = nullptr;

// Must match CAM side
static const uint8_t FRAME_MAGIC[8] = {0xCA, 0x3E, 0xBE, 0xEF, 0x57, 0xA1, 0xD0, 0x92};
static const uint8_t HEADER_OVERHEAD = 12; // 8 magic + 4 size

enum RxState { WAITING_FOR_HEADER, RECEIVING_DATA };
RxState rx_state = WAITING_FOR_HEADER;
uint32_t jpeg_size = 0;
uint32_t bytes_received = 0;

void chunk_output(EAGLESystems* arg, uint8_t* buf, size_t len) {
    // CODE BELOW IS FOR JPEG SERIAL OUTPUT!
    uint32_t off = 0;
    uint8_t tmp[512];

    while (off < len)
    {
        uint32_t chunk = len - off;

        if (chunk > 512)
            chunk = 512;

        // idk why, but writing directly from spiram works for the first couple chunks, then the data gets
        // offset or shifted, making the rest of the image super grainy / makes the colors wrong.
        // To fix this we'll move the frame frmo spiram to program memory in 512 byte chunks before sending them.
        memcpy(tmp, buf + off, chunk);

        size_t wrote = arg->serial->write(tmp, chunk);
        arg->serial->flush();

        if (wrote > 0)
        {
            off += wrote;
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
};

static void receive_thread(EAGLESystems* arg) {
    si4463_t* handle = arg->radio.getHandle();
    static uint8_t rxBuffer[129];  // Static, max FIFO size is 129 bytes

    while(true) {
        uint8_t rxBytes = SI4463_GetRxFifoReceivedBytesFast(handle);

        if(rxBytes == 0) { continue; }

        if (SI4463_ReadRxFifoFast(handle, rxBuffer, rxBytes) == SI4463_OK) {

            if (rxBytes > 64) rxBytes = 64;  // bytes past 64 are unreliable

            if (rx_state == WAITING_FOR_HEADER) {
                // Check for magic header
                if (rxBytes >= HEADER_OVERHEAD && memcmp(rxBuffer, FRAME_MAGIC, 8) == 0) {
                    memcpy(&jpeg_size, &rxBuffer[8], sizeof(uint32_t));
                    bytes_received = 0;

                    if (jpeg_size > 0 && jpeg_size <= FRAME_SIZE_BYTES) {
                        // Extract JPEG data packed after the header
                        uint32_t header_data = rxBytes - HEADER_OVERHEAD;
                        if (header_data > jpeg_size) header_data = jpeg_size;
                        memcpy(frame_buffer, &rxBuffer[HEADER_OVERHEAD], header_data);
                        bytes_received = header_data;
                        rx_state = RECEIVING_DATA;
                    }
                }
            } else {
                // Accumulate JPEG data
                uint32_t remaining = jpeg_size - bytes_received;
                uint32_t to_copy = (rxBytes < remaining) ? rxBytes : remaining;
                memcpy(frame_buffer + bytes_received, rxBuffer, to_copy);
                bytes_received += to_copy;

                if (bytes_received >= jpeg_size) {
                    #ifdef USE_USB_DEBUG
                    arg->serial->print("*FRAME ");
                    arg->serial->println(jpeg_size);
                    chunk_output(arg, frame_buffer, jpeg_size);
                    arg->serial->println("**DONE");
                    #endif

                    bl_stat = !bl_stat;
                    digitalWrite(LED_BLUE, bl_stat);

                    rx_state = WAITING_FOR_HEADER;
                }
            }
        }
    }
}

/* Begin all system data */
[[noreturn]] void sys_begin() {

    digitalWrite(LED_ORANGE, HIGH);

    #ifndef USE_USB_DEBUG
    // Init UVC
    // Launch USB device task — it inits PHY + TinyUSB, then notifies us
    TaskHandle_t this_task = xTaskGetCurrentTaskHandle();
    xTaskCreatePinnedToCore(usb_device_task, "usbd", 8192,
                            (void *)this_task, configMAX_PRIORITIES - 1,
                            NULL, 0);

    // Wait for USB task to finish init (5s timeout)
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000)) == 0) {
        while (1) {
            digitalWrite(LED_RED, !digitalRead(LED_RED));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    #else
    USB.begin();
    USBSerial.begin(115200);
    sys.serial = &USBSerial;
    #endif

    // while(true) {
    //     if (tud_mounted()) {
    //         digitalWrite(LED_GREEN, HIGH);
    //     } else {
    //         digitalWrite(LED_GREEN, LOW);
    //     }

    //     if (!tud_video_n_streaming(0, 0)) {
    //         frame_num = 0;
    //         tx_busy = 0;
    //         digitalWrite(LED_BLUE, LOW);
    //         delay(100);
    //         continue;
    //     }

    //     if (tx_busy) {
    //         delay(1);
    //         continue;
    //     }

    //     digitalWrite(LED_BLUE, frame_num & 1);
    //     fill_color_bar(frame_buffer, frame_num);
    //     tx_busy = 1;
    //     tud_video_n_frame_xfer(0, 0, (void *)frame_buffer, FRAME_SIZE_BYTES);
    //     vTaskDelay(pdMS_TO_TICKS(interval_ms));
    //     delay(1000);

    // }

    frame_buffer = (uint8_t *)heap_caps_malloc(FRAME_SIZE_BYTES, MALLOC_CAP_SPIRAM);

    // Init radio
    CAMRadioStatus radio_status = sys.radio.init(SPI);
    if(radio_status != CAMRadioStatus::CAMRADIO_OK) {
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_ORANGE, LOW);
        while(1) {};
    }

    // Start RX mode
    CAMRadioStatus status = sys.radio.startRx();
    if (status != CAMRADIO_OK) {
        digitalWrite(LED_RED, HIGH);
        while(1) {};
    }

    xTaskCreatePinnedToCore((TaskFunction_t) receive_thread, "rxt", THREAD_STACK_SIZE_DEFAULT, &sys, 7, nullptr, 0);

    digitalWrite(LED_ORANGE, LOW);
    digitalWrite(LED_GREEN, HIGH);
    while(true) {
        delay(1000);
        // #ifdef USE_USB_DEBUG
        // sys.serial->println("Running...");
        // #endif
    }
}

#endif
