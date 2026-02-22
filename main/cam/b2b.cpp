#ifdef IS_CAM

#include <cam/b2b.h>

B2BHandler* B2BHandler::instance = nullptr;

void B2BHandler::on_receive(int len) {
    if (!instance) return;
    while (Wire1.available()) {
        uint8_t cmd = Wire1.read();
        xQueueSendFromISR(instance->cmd_queue, &cmd, nullptr);
        instance->state.cam_ack = !instance->state.cam_ack;
    }
}

void B2BHandler::on_request() {
    if (!instance) return;
    uint8_t buf[1] = { instance->state.encode() };
    Wire1.slaveWrite(buf, 1);
}

int B2BHandler::init() {
    cmd_queue = xQueueCreate(16, sizeof(uint8_t));
    if (!cmd_queue) {
        return CAM_B2B_FAILED_TO_INIT;
    }

    // this is lowk giga ugly but whatever
    instance = this;

    Wire1.onReceive(on_receive);
    Wire1.onRequest(on_request);
    Wire1.begin((uint8_t)CAM_I2C_ADDR, B2B_I2C_SDA, B2B_I2C_SCL, 0);

    return CAM_OK;
}

bool B2BHandler::dequeue(uint8_t* cmd) {
    return xQueueReceive(cmd_queue, cmd, 0) == pdTRUE;
}

#endif
