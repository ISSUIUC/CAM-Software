#include <cam_radio/cam_radio.h>

CAMRadioStatus CAMRadio::init(SPIClass &spi, USBCDC *serial)
{
    LR2021Error err = _radio.init(spi); // not _radio.begin(pins)
    if (err.driverCode != LR2021_ERR_NONE)
    {
        serial->printf("[radio] init failed: %s (err=%d, RadioLib code %d)\n", err.stageStr(), err.driverCode, err.radioLibCode);
#if RADIO_MODULATION == MODULATION_FLRC
        // BUSY stuck high (SPI_CMD_TIMEOUT) would hang the read
        if (err.radioLibCode != RADIOLIB_ERR_SPI_CMD_TIMEOUT)
            serial->printf("[radio] chip error flags: 0x%04X\n", _radio.readErrors());
#endif
        return CAMRADIO_INIT_ERR;
    }
    serial->println("[radio] init OK (LR2021 " RADIO_MODULATION_NAME " driver)");
    return CAMRADIO_OK;
}