#ifdef IS_EAGLE

#include <eagle/system.h>

#define THREAD_STACK_SIZE_DEFAULT 4096


EAGLESystems sys;

#ifdef USE_USB_DEBUG
USBCDC USBSerial;
#endif

bool bl_stat = false;

uint8_t *frame_buffer = nullptr;

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
    Si4463Nuke* radio = arg->radio.getDriver();

    // Start receiving into the SPIRAM frame buffer
    radio->startRx(frame_buffer, FRAME_SIZE_BYTES);

    uint32_t lastStatusMs = 0;

    while (true) {
        radio->update();

        if (radio->available()) {
            uint32_t len = radio->getReceivedLength();
            int rssi = radio->getRSSI();

            #ifdef USE_USB_DEBUG
            arg->serial->printf("*FRAME %lu (rssi=%d, frags=%u)\n",
                                (unsigned long)len, rssi, radio->getRxTotal());
            chunk_output(arg, frame_buffer, len);
            arg->serial->println("**DONE");
            #endif

            bl_stat = !bl_stat;
            digitalWrite(LED_BLUE, bl_stat);

            // Restart RX for next frame
            radio->startRx(frame_buffer, FRAME_SIZE_BYTES);
        }

        // Periodic status print every 2 seconds
        #ifdef USE_USB_DEBUG
        if (millis() - lastStatusMs >= 2000) {
            lastStatusMs = millis();
            radio->printDebug(*arg->serial);
        }
        #endif

        // Yield to other tasks but keep polling fast
        taskYIELD();
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

    // Give USB CDC time to enumerate on host PC
    delay(2000);

    #ifdef USE_USB_DEBUG
    sys.serial->println("[eagle] booting...");
    sys.serial->printf("[eagle] SPI pins: SCK=%d MISO=%d MOSI=%d\n", SPI_SCK, SPI_MISO, SPI_MOSI);
    sys.serial->printf("[eagle] SI4463 pins: CS=%d SDN=%d INT=%d\n", SI4463_CS, SI4463_SDN, SI4463_INT);
    #endif

    frame_buffer = (uint8_t *)heap_caps_malloc(FRAME_SIZE_BYTES, MALLOC_CAP_SPIRAM);

    // Init radio
    CAMRadioStatus radio_status = sys.radio.init(SPI);
    if(radio_status != CAMRadioStatus::CAMRADIO_OK) {
        #ifdef USE_USB_DEBUG
        sys.serial->println("[eagle] Radio init FAILED");
        #endif
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_ORANGE, LOW);
        while(1) {};
    }

    #ifdef USE_USB_DEBUG
    sys.serial->println("[eagle] Radio init OK");
    #endif

    xTaskCreatePinnedToCore((TaskFunction_t) receive_thread, "rxt", THREAD_STACK_SIZE_DEFAULT * 2, &sys, 7, nullptr, 0);

    digitalWrite(LED_ORANGE, LOW);
    digitalWrite(LED_GREEN, HIGH);
    while(true) {
        delay(1000);
    }
}

#endif
