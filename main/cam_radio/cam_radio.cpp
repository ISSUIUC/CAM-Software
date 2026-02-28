#include <cam_radio/cam_radio.h>

CAMRadioStatus CAMRadio::init(SPIClass& spi) {
    Si4463Pins pins = {
        .spi   = &spi,
        .cs    = SI4463_CS,
        .sdn   = SI4463_SDN,
        .irq   = SI4463_INT,
#ifdef SI4463_GPIO0
        .gpio0 = SI4463_GPIO0,
#else
        .gpio0 = 0xFF,
#endif
#ifdef SI4463_GPIO1
        .gpio1 = SI4463_GPIO1,
#else
        .gpio1 = 0xFF,
#endif
    };

    if (!_radio.begin(pins)) {
        Serial.printf("[CAMRadio] init failed, err=%u\n", _radio.getLastError());
        return CAMRADIO_INIT_ERR;
    }

    // Print config for debug verification
    _radio.printConfig(Serial);
    _radio.printDebug(Serial);

    Serial.println("[CAMRadio] init OK (Si4463Nuke driver)");
    return CAMRADIO_OK;
}

CAMRadioStatus CAMRadio::send(const uint8_t* data, uint32_t len) {
    if (!_radio.tx(data, len)) {
        Serial.printf("[CAMRadio] tx failed, err=%u\n", _radio.getLastError());
        return CAMRADIO_INIT_ERR;
    }
    return CAMRADIO_OK;
}

void CAMRadio::startRx(uint8_t* buf, uint32_t bufLen) {
    _radio.startRx(buf, bufLen);
}

void CAMRadio::update() {
    _radio.update();
}

bool CAMRadio::isTxBusy() const {
    return _radio.isBusy();
}

bool CAMRadio::available() {
    return _radio.available();
}

uint32_t CAMRadio::getReceivedLength() const {
    return _radio.getReceivedLength();
}

int CAMRadio::getRSSI() const {
    return _radio.getRSSI();
}
