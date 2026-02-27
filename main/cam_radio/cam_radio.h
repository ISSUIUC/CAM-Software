#pragma once
#include "si4463_arduino_hal.h"

enum CAMRadioStatus {
    CAMRADIO_OK = 0,
    CAMRADIO_INIT_ERR = 1,
};

class CAMRadio {
    private:
    si4463_t _r;
    uint8_t _ch;

    public:
    void setChannel(uint8_t new_ch);
    CAMRadioStatus init(SPIClass& _spi);
    CAMRadioStatus startRx();
    CAMRadioStatus send(const uint8_t* data, uint8_t len);
    CAMRadioStatus sendFast(const uint8_t* data, uint8_t len);

    // Debug helpers
    void getPartInfo(uint8_t* buf);
    si4463_state_t getState();
    void getChipStatus();
    bool checkCTS();  // Check raw CTS pin state
    si4463_t* getHandle() { return &_r; }  // Direct access for debugging
};