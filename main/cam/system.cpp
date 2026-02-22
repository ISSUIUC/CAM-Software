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
                camera_on_off(*arg->cameras.cam1);
                delay(5000);
                digitalWrite(CAM1_ON_OFF, LOW);
                break;}
            case (uint8_t)B2BCommand::CAMERA0_ON: {
                arg->serial->println("proc cmd: CAM1 ON");
                digitalWrite(CAM1_ON_OFF, HIGH);
                break;}
            case (uint8_t)B2BCommand::CAMERA1_OFF: {
                arg->serial->println("proc cmd: CAM2 OFF");
                camera_on_off(*arg->cameras.cam2);
                delay(5000);
                digitalWrite(CAM2_ON_OFF, LOW);
                break;}
            case (uint8_t)B2BCommand::CAMERA1_ON: {
                arg->serial->println("proc cmd: CAM2 ON");
                digitalWrite(CAM2_ON_OFF, HIGH);
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
        toReturn1 = read_mem_cap_data(*arg->cameras.cam1);
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
        toReturn2 = read_mem_cap_data(*arg->cameras.cam2);
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

/* Begin all system data */
[[noreturn]] void sys_begin() {

    USB.begin();
    USBSerial.begin(115200);

    sys.serial = &USBSerial;

    uint8_t tvp_res = sys.tvp.init();
    if (tvp_res != CAM_OK) {
        sys.serial->print("TVP Error: ");
        sys.serial->println(tvp_res);
        while (1) {}
    }

    sys.cameras.cam1 = &Serial1;
    sys.cameras.cam2 = &Serial2;
    uint8_t cam_res = sys.cameras.init();
    if (cam_res != CAM_OK) {
        sys.serial->print("Camera Error: ");
        sys.serial->println(cam_res);
        while (1) {}
    }

    uint8_t b2b_res = sys.b2b.init();
    if (b2b_res != CAM_OK) {
        sys.serial->print("B2B Error: ");
        sys.serial->println(b2b_res);
        while (1) {}
    }

    xTaskCreatePinnedToCore((TaskFunction_t) cmd_thread, "cmdq", THREAD_STACK_SIZE_DEFAULT, &sys, 7, nullptr, 0);
    xTaskCreatePinnedToCore((TaskFunction_t) poll_thread, "poll", THREAD_STACK_SIZE_DEFAULT, &sys, 5, nullptr, 0);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif
