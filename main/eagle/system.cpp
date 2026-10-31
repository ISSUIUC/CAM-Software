#ifdef IS_EAGLE

#include <eagle/system.h>

#define THREAD_STACK_SIZE_DEFAULT 4096

EAGLESystems sys;

#ifdef USE_USB_DEBUG
USBCDC USBSerial;
#endif // ajdsfasf

static ReassemblyState s_reassembler;
static uint8_t *output_buffer = nullptr;
static uint32_t output_len = 0;
static SemaphoreHandle_t output_sem = nullptr;
static volatile bool output_busy = false;

bool bl_stat = false;

void chunk_output(EAGLESystems *arg, uint8_t *buf, size_t len)
{
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
static void output_thread(EAGLESystems *arg)
{
    while (true)
    {
        // Wait for a frame to be ready
        if (xSemaphoreTake(output_sem, portMAX_DELAY) == pdTRUE)
        {
            uint32_t len = output_len;
            if (len > 0)
            {
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

#if RADIO_MODULATION == MODULATION_FSK
#define RX_QUALITY_FMT "lqi=%.2f"
#define RX_QUALITY(status) (status).lqi
#else
#define RX_QUALITY_FMT "rssi_sync=%.1f"
#define RX_QUALITY(status) (status).rssiSync
#endif

static void receive_thread(EAGLESystems *arg)
{
    LR2021Driver *radio = arg->radio.getDriver();

    static uint8_t pkt_buf[RADIO_PACKET_SIZE];
    static LR2021PktStatus pkt_status;

#ifdef USE_USB_DEBUG
    uint32_t last_status_ms = 0;
#endif

#if RADIO_MODULATION == MODULATION_FLRC
    radio->startBurstReceive();
#endif

    while (true)
    {
        uint32_t now = (uint32_t)millis();
#if RADIO_MODULATION == MODULATION_FLRC
        LR2021Error result = radio->receiveBurstPacket(pkt_buf, RADIO_PACKET_SIZE, &pkt_status, RX_FRAME_TIMEOUT_MS);
#else
        LR2021Error result = radio->receive(pkt_buf, RADIO_PACKET_SIZE, &pkt_status);
#endif

        if (result.driverCode == LR2021_ERR_CRC_MISMATCH)
        {
            s_reassembler.stat_crc_drop++;
            if (reassembler_check_timeout(&s_reassembler, now) == INGEST_TIMEOUT)
                goto frame_ready;
            taskYIELD();
            continue;
        }

        if (result.driverCode == LR2021_ERR_RX_TIMEOUT)
        {
            if (reassembler_check_timeout(&s_reassembler, now) == INGEST_TIMEOUT)
                goto frame_ready;
            taskYIELD();
            continue;
        }

        if (!result.ok())
        {
#ifdef USE_USB_DEBUG
            if (!output_busy)
                arg->serial->printf("[rx] error: %s\n", result.stageStr());
#endif
            taskYIELD();
            continue;
        }

        {
            IngestResult ir = reassembler_ingest(&s_reassembler, pkt_buf, now);

            if (ir == INGEST_COMPLETE || ir == INGEST_TIMEOUT)
                goto frame_ready;

#ifdef USE_USB_DEBUG
            if (!output_busy && (millis() - last_status_ms >= 2000))
            {
                last_status_ms = millis();
                arg->serial->printf(
                    "[rx] id=0x%02X frags=%u/%u rssi=%.1f " RX_QUALITY_FMT " | drops=%lu resets=%lu\n",
                    s_reassembler.frame_id,
                    s_reassembler.frags_received,
                    s_reassembler.total_frags,
                    pkt_status.rssiAvg,
                    RX_QUALITY(pkt_status),
                    (unsigned long)s_reassembler.stat_crc_drop,
                    (unsigned long)s_reassembler.stat_frame_resets);
            }
#endif
            taskYIELD();
            continue;
        }

    frame_ready:
        {
            // completed_size is saved by ingest/check_timeout BEFORE reset clears total_size
            uint32_t len = s_reassembler.completed_size;
            if (len > 0 && !output_busy)
            {
                bl_stat = !bl_stat;
                digitalWrite(LED_BLUE, bl_stat);
                memcpy(output_buffer, s_reassembler.data_buf, len);
                output_len = len;
                xSemaphoreGive(output_sem);
            }
        }
        taskYIELD();
        continue;
    }
}

/* Begin all system data */
[[noreturn]] void sys_begin()
{

    digitalWrite(LED_ORANGE, HIGH);

#ifndef USE_USB_DEBUG
    // Init UVC
    // Launch USB device task — it inits PHY + TinyUSB, then notifies us
    TaskHandle_t this_task = xTaskGetCurrentTaskHandle();
    xTaskCreatePinnedToCore(usb_device_task, "usbd", 8192,
                            (void *)this_task, configMAX_PRIORITIES - 1,
                            NULL, 0);

    // Wait for USB task to finish init (5s timeout)
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000)) == 0)
    {
        while (1)
        {
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
    sys.serial->printf("[eagle] LR2021 pins: CS=%d INT=%d\n", LR2021_CS, LR2021_GPIO9);
#endif

    if (!reassembler_init(&s_reassembler, MAX_RX_FRAME_SIZE))
    {
        digitalWrite(LED_RED, HIGH);
        while (1)
        {
        }
    }

    output_buffer = (uint8_t *)heap_caps_malloc(FRAME_SIZE_BYTES, MALLOC_CAP_SPIRAM);
    output_sem = xSemaphoreCreateBinary();

    // Init radio
    CAMRadioStatus radio_status = sys.radio.init(SPI, sys.serial);
    if (radio_status != CAMRadioStatus::CAMRADIO_OK)
    {
#ifdef USE_USB_DEBUG
        sys.serial->println("[eagle] Radio init FAILED");
#endif
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_ORANGE, LOW);
        while (1)
        {
        };
    }

#ifdef USE_USB_DEBUG
    sys.serial->println("[eagle] Radio init OK");
#endif

    // Receive thread on core 0 high priority
    xTaskCreatePinnedToCore((TaskFunction_t)receive_thread, "rxt", THREAD_STACK_SIZE_DEFAULT * 2, &sys, 7, nullptr, 0);

    // Output thread on core 1, handles slow serial output without blocking RX
    xTaskCreatePinnedToCore((TaskFunction_t)output_thread, "txout", THREAD_STACK_SIZE_DEFAULT * 2, &sys, 5, nullptr, 1);

    digitalWrite(LED_ORANGE, LOW);
    digitalWrite(LED_GREEN, HIGH);
    while (true)
    {
        delay(1000);
    }
}

#endif
