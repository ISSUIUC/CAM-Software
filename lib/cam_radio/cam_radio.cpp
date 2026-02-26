#include <cam_radio.h>

CAMRadio::CAMRadio(SPIClass& _spi) {
    si_hal_set_spi(_spi);
    si_hal_arduino_init();
    si_hal_bind_arduino(&_r);
    setChannel(0);
}

void CAMRadio::setChannel(uint8_t new_ch) {
    _ch = new_ch;
}

CAMRadioStatus CAMRadio::init() {
    int8_t result = SI4463_Init(&_r);
    Serial.printf("[CAMRadio] SI4463_Init returned: %d\n", result);
    if(result != SI4463_OK) { return CAMRadioStatus::CAMRADIO_INIT_ERR; };

    // Clear any pending errors
    SI4463_ClearAllInterrupts(&_r);

    return CAMRadioStatus::CAMRADIO_OK;
}

CAMRadioStatus CAMRadio::startRx() {
    // Start RX mode with length=0 (variable length), stay in RX after timeout/valid/invalid packet
    int8_t result = SI4463_StartRx(&_r, 0, true, true, true);
    return (result == SI4463_OK) ? CAMRADIO_OK : CAMRADIO_INIT_ERR;
}

CAMRadioStatus CAMRadio::send(const uint8_t* data, uint8_t len) {
    int8_t result = SI4463_Transmit(&_r, data, len);
    return (result == SI4463_OK) ? CAMRADIO_OK : CAMRADIO_INIT_ERR;
}

CAMRadioStatus CAMRadio::sendFast(const uint8_t* data, uint8_t len) {
    int8_t result = SI4463_TransmitFast(&_r, data, len);
    return (result == SI4463_OK) ? CAMRADIO_OK : CAMRADIO_INIT_ERR;
}

void CAMRadio::getPartInfo(uint8_t* buf) {
    SI4463_GetPartInfo(&_r, buf);
}

si4463_state_t CAMRadio::getState() {
    return SI4463_GetCurrentState(&_r);
}

void CAMRadio::getChipStatus() {
    SI4463_GetChipStatus(&_r);
}

bool CAMRadio::checkCTS() {
    return digitalRead(SI4463_GPIO1) == HIGH;
}
