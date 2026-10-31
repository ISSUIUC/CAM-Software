#pragma once

#include "modulation.h"
#include "framer.h"
#include "reassembler.h"

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
    LR2021Driver _radio;

public:
    // Initialize radio hardware. Call after SPI.begin().
    CAMRadioStatus init(SPIClass &spi, USBCDC *serial);

    // // Access underlying driver for debug/advanced use.
    LR2021Driver *getDriver() { return &_radio; }
};
