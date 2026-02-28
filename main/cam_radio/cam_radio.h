#pragma once
#include <Si4463Nuke.h>
#include <pins.h>
#include <SPI.h>

enum CAMRadioStatus {
    CAMRADIO_OK = 0,
    CAMRADIO_INIT_ERR = 1,
};

class CAMRadio {
private:
    Si4463Nuke _radio;

public:
    // Initialize radio hardware. Call after SPI.begin().
    CAMRadioStatus init(SPIClass& spi);

    // Transmit arbitrary-length data (>50KB supported, auto-fragmented).
    // Buffer must stay valid until isTxBusy() returns false.
    CAMRadioStatus send(const uint8_t* data, uint32_t len);

    // Start receiving into provided buffer.
    void startRx(uint8_t* buf, uint32_t bufLen);

    // Poll radio state machine. Call every loop iteration during TX or RX.
    void update();

    // True while a transmission is in progress.
    bool isTxBusy() const;

    // True once after a complete message has been received.
    bool available();

    // Length of last received message.
    uint32_t getReceivedLength() const;

    // RSSI of last received packet (dBm).
    int getRSSI() const;

    // Access underlying driver for debug/advanced use.
    Si4463Nuke* getDriver() { return &_radio; }
};
