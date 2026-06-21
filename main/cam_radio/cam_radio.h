#pragma once

#include "fsk.h"
#include "fsk_framer.h"
#include "fsk_reassembler.h"

#include <pins.h>
#include <SPI.h>

enum CAMRadioStatus
{
    CAMRADIO_OK = 0,
    CAMRADIO_INIT_ERR = 1,
};

class CAMRadio
{
private:
    LR2021FSKDriver _radio;

public:
    // Initialize radio hardware. Call after SPI.begin().
    CAMRadioStatus init(SPIClass &spi);

    // // Access underlying driver for debug/advanced use.
    LR2021FSKDriver *getDriver() { return &_radio; }
};
