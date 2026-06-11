#include <cam_radio/cam_radio.h>

CAMRadioStatus CAMRadio::init(SPIClass &spi)
{
    LR2021Error err = _radio.init(spi); // not _radio.begin(pins)
    if (err.driverCode != LR2021_ERR_NONE)
    {
        Serial.printf("[CAMRadio] init failed, err=%d\n", err.driverCode);
        return CAMRADIO_INIT_ERR;
    }
    Serial.println("[CAMRadio] init OK (LR2021 FSK driver)");
    return CAMRADIO_OK;
}