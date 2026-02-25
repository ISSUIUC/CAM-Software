#ifdef IS_EAGLE

#include <eagle/system.h>

USBCDC USBSerial;
EAGLESystems sys;



/* Begin all system data */
[[noreturn]] void sys_begin() {
    digitalWrite(LED_RED, HIGH);

    
    USB.begin();
    USBSerial.begin(115200);


    sys.serial->println("EAGLE UVC test starting (direct TinyUSB init)...");


    sys.serial = &USBSerial;

    // Launch USB device task — it inits PHY + TinyUSB, then notifies us
    TaskHandle_t this_task = xTaskGetCurrentTaskHandle();
    xTaskCreatePinnedToCore(usb_device_task, "usbd", 8192,
                            (void *)this_task, configMAX_PRIORITIES - 1,
                            NULL, 0);


    // Wait for USB task to finish init (5s timeout)
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000)) == 0) {
        ESP_LOGE(TAG, "USB task failed to start!");
        while (1) {
            digitalWrite(LED_RED, !digitalRead(LED_RED));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    digitalWrite(LED_ORANGE, HIGH);

    sys.serial->println("USB ready. Waiting for host...");






    while(true) {
        if (tud_mounted()) {
            digitalWrite(LED_GREEN, HIGH);
        } else {
            digitalWrite(LED_GREEN, LOW);
        }

        if (!tud_video_n_streaming(0, 0)) {
            frame_num = 0;
            tx_busy = 0;
            digitalWrite(LED_BLUE, LOW);
            vTaskDelay(pdMS_TO_TICKS(100));
            return;
        }

        if (tx_busy) {
            vTaskDelay(pdMS_TO_TICKS(1));
            return;
        }

        digitalWrite(LED_BLUE, frame_num & 1);
        fill_color_bar(frame_buffer, frame_num);
        tx_busy = 1;
        tud_video_n_frame_xfer(0, 0, (void *)frame_buffer, FRAME_SIZE_BYTES);
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
            sys.serial->println("hi");
        delay(1000);





    }
}

#endif
