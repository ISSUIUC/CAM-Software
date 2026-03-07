#ifdef IS_EAGLE

#include <eagle/system.h>

#define THREAD_STACK_SIZE_DEFAULT 4096


EAGLESystems sys;

#ifdef USE_USB_DEBUG
USBCDC USBSerial;
#endif

bool bl_stat = false;

uint8_t *frame_buffer = nullptr;
uint8_t *output_buffer = nullptr;
uint32_t output_len = 0;
SemaphoreHandle_t output_sem = nullptr;
volatile bool output_busy = false;

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

// Output thread: runs on core 1, waits for semaphore, sends frame over serial.
// This keeps the receive thread (core 0) free to keep polling the radio.
static void output_thread(EAGLESystems* arg) {
    while (true) {
        // Wait for a frame to be ready
        if (xSemaphoreTake(output_sem, portMAX_DELAY) == pdTRUE) {
            uint32_t len = output_len;
            if (len > 0) {
                #ifdef USE_USB_DEBUG
                output_busy = true;
                arg->serial->printf("*FRAME %lu\n", (unsigned long)len);
                chunk_output(arg, output_buffer, len);
                arg->serial->println("**DONE");
                output_busy = false;
                #endif
            }
        }
    }
}

static void receive_thread(EAGLESystems* arg) {
    Si4463Nuke* radio = arg->radio.getDriver();

    // Start receiving into the SPIRAM frame buffer
    radio->startRx(frame_buffer, FRAME_SIZE_BYTES);

    uint32_t lastStatusMs = 0;
    uint32_t lastFrameMs = 0;
    uint32_t frameNum = 0;

    while (true) {
        radio->update();

        if (radio->available()) {
            uint32_t now = millis();
            uint32_t len = radio->getReceivedLength();
            uint32_t dt = now - lastFrameMs;
            lastFrameMs = now;
            frameNum++;

            bl_stat = !bl_stat;
            digitalWrite(LED_BLUE, bl_stat);

            uint8_t rsResult = radio->getRsResult();

            // Only output frames where RS succeeded (0=ok, 1=fixed). Skip corrupt frames.
            if (!output_busy && rsResult <= 1) {
                memcpy(output_buffer, frame_buffer, len);
                output_len = len;
                xSemaphoreGive(output_sem);
            }

            // Restart RX immediately - don't wait for output to finish
            radio->startRx(frame_buffer, FRAME_SIZE_BYTES);
        }

        // Periodic status print every 2 seconds (skip if output thread is sending a frame)
        #ifdef USE_USB_DEBUG
        if (!output_busy && millis() - lastStatusMs >= 2000) {
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

    frame_buffer  = (uint8_t *)heap_caps_malloc(FRAME_SIZE_BYTES, MALLOC_CAP_SPIRAM);
    output_buffer = (uint8_t *)heap_caps_malloc(FRAME_SIZE_BYTES, MALLOC_CAP_SPIRAM);
    output_sem    = xSemaphoreCreateBinary();

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

    // Receive thread on core 0 - high priority, never blocks on serial
    xTaskCreatePinnedToCore((TaskFunction_t) receive_thread, "rxt", THREAD_STACK_SIZE_DEFAULT * 2, &sys, 7, nullptr, 0);

    // Output thread on core 1 - handles slow serial output without blocking RX
    xTaskCreatePinnedToCore((TaskFunction_t) output_thread, "txout", THREAD_STACK_SIZE_DEFAULT * 2, &sys, 5, nullptr, 1);

    digitalWrite(LED_ORANGE, LOW);
    digitalWrite(LED_GREEN, HIGH);
    while(true) {
        delay(1000);
    }
}

#endif
