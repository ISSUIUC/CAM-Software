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

// CAMRadioStatus CAMRadio::send(const uint8_t *data, uint32_t len)
// {
//     // if (!_radio.tx(data, len))
//     // {
//     //     Serial.printf("[CAMRadio] tx failed, err=%u\n", _radio.getLastError());
//     //     return CAMRADIO_INIT_ERR;
//     // }
//     // return CAMRADIO_OK;
// }

// void CAMRadio::startRx(uint8_t *buf, uint32_t bufLen)
// {
//     //   _radio.startRx(buf, bufLen);
// }

// void CAMRadio::update()
// {
//     //  _radio.update();
// }

// bool CAMRadio::isTxBusy() const
// {
//     //  return _radio.isBusy();
// }

// bool CAMRadio::available()
// {
//     //   return _radio.available();
// }

// uint32_t CAMRadio::getReceivedLength() const
// {
//     //  return _radio.getReceivedLength();
// }

// int CAMRadio::getRSSI() const
// {
//     //  return _radio.getRSSI();
// }
