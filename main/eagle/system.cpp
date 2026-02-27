#ifdef IS_EAGLE

#include <eagle/system.h>

#define THREAD_STACK_SIZE_DEFAULT 4096


EAGLESystems sys;

#ifdef USE_USB_DEBUG
USBCDC USBSerial;
#endif

bool bl_stat = false;

uint32_t bytes_received;

static void receive_thread(EAGLESystems* arg) {
    si4463_t* handle = arg->radio.getHandle();
    static uint8_t rxBuffer[129];  // Static, max FIFO size is 129 bytes
    int i = 0;
    bool once = false; 

    while(true) {
        uint8_t rxBytes = SI4463_GetRxFifoReceivedBytesFast(handle);
        

        if (SI4463_ReadRxFifoFast(handle, rxBuffer, rxBytes) == SI4463_OK) {
            i++;

            constexpr uint32_t frame_size = FRAME_HEIGHT * FRAME_WIDTH * 2;

            // if(bytes_received + rxBytes < frame_size) {
            //     memcpy(frame_buffer + bytes_received, rxBuffer, rxBytes);
            // } else {
            //     memcpy(frame_buffer + bytes_received, rxBuffer, (frame_size - bytes_received));
            //     bytes_received = 0;
            //     // tud_video_n_frame_xfer(0, 0, (void *)frame_buffer, FRAME_SIZE_BYTES);
            //     serial.printf("Frame received: %u bytes\n", frame_size);    
            //     bl_stat = !bl_stat;
            //     digitalWrite(LED_BLUE, bl_stat);

            // }
            #ifdef USE_USB_DEBUG


            if(!once) {
                arg->serial->println("*FRAME"); 
                once = true;
            }
            
            // arg->serial->print("RX "); arg->serial->print(rxBytes); arg->serial->print(": "); 

            if(i<5359){
                arg->serial->print((char*)rxBuffer);
            }
            else{
                arg->serial->println("**DONE"); 
                i=0;
                once = false;
            }

            #endif
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
        #ifdef USE_USB_DEBUG
        sys.serial->println("Running...");
        #endif
    }
}

#endif
