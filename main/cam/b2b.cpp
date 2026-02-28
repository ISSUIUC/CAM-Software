#ifdef IS_CAM

#include <cam/b2b.h>

extern uint32_t LAST_I2C_COMM;

B2BHandler* B2BHandler::instance = nullptr;

bool B2BHandler::on_receive(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_rx_done_event_data_t *evt_data, void *arg) {
    B2BHandler* self = (B2BHandler*)arg;
    LAST_I2C_COMM = millis();

    for (uint32_t i = 0; i < evt_data->length; i++) {
        uint8_t cmd = evt_data->buffer[i];
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(self->cmd_queue, &cmd, &xHigherPriorityTaskWoken);
        self->state.cam_ack = !self->state.cam_ack;
        if (xHigherPriorityTaskWoken) return true;
    }
    return false;
}

bool bbl = false;

bool B2BHandler::on_request(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_request_event_data_t *evt_data, void *arg) {
    B2BHandler* self = (B2BHandler*)arg;
    LAST_I2C_COMM = millis();

    bbl = !bbl;
    digitalWrite(LED_BLUE, bbl);

    uint8_t buf[3] = { self->state.encode(), 0, 0 };
    uint32_t write_len = 0;
    i2c_slave_write(i2c_slave, buf, 3, &write_len, 0);
    return false;
}

int B2BHandler::init() {
    cmd_queue = xQueueCreate(16, sizeof(uint8_t));
    if (!cmd_queue) {
        return CAM_B2B_FAILED_TO_INIT;
    }

    instance = this;

    i2c_slave_config_t conf = {};
    conf.i2c_port = I2C_NUM_1;
    conf.sda_io_num = (gpio_num_t)B2B_I2C_SDA;
    conf.scl_io_num = (gpio_num_t)B2B_I2C_SCL;
    conf.clk_source = I2C_CLK_SRC_DEFAULT;
    conf.send_buf_depth = 64;
    conf.receive_buf_depth = 64;
    conf.slave_addr = CAM_I2C_ADDR;

    esp_err_t err = i2c_new_slave_device(&conf, &slave_handle);
    if (err != ESP_OK) {
        return CAM_B2B_FAILED_TO_INIT;
    }

    i2c_slave_event_callbacks_t cbs = {};
    cbs.on_receive = on_receive;
    cbs.on_request = on_request;
    err = i2c_slave_register_event_callbacks(slave_handle, &cbs, this);
    if (err != ESP_OK) {
        i2c_del_slave_device(slave_handle);
        slave_handle = nullptr;
        return CAM_B2B_FAILED_TO_INIT;
    }

    LAST_I2C_COMM = millis();

    return CAM_OK;
}

void B2BHandler::deinit() {
    if (slave_handle) {
        i2c_del_slave_device(slave_handle);
        slave_handle = nullptr;
    }
}

int B2BHandler::reinit() {
    deinit();

    i2c_slave_config_t conf = {};
    conf.i2c_port = I2C_NUM_1;
    conf.sda_io_num = (gpio_num_t)B2B_I2C_SDA;
    conf.scl_io_num = (gpio_num_t)B2B_I2C_SCL;
    conf.clk_source = I2C_CLK_SRC_DEFAULT;
    conf.send_buf_depth = 64;
    conf.receive_buf_depth = 64;
    conf.slave_addr = CAM_I2C_ADDR;

    esp_err_t err = i2c_new_slave_device(&conf, &slave_handle);
    if (err != ESP_OK) {
        return CAM_B2B_FAILED_TO_INIT;
    }

    i2c_slave_event_callbacks_t cbs = {};
    cbs.on_receive = on_receive;
    cbs.on_request = on_request;
    err = i2c_slave_register_event_callbacks(slave_handle, &cbs, this);
    if (err != ESP_OK) {
        i2c_del_slave_device(slave_handle);
        slave_handle = nullptr;
        return CAM_B2B_FAILED_TO_INIT;
    }

    return CAM_OK;
}

bool B2BHandler::dequeue(uint8_t* cmd) {
    return xQueueReceive(cmd_queue, cmd, 0) == pdTRUE;
}

#endif
