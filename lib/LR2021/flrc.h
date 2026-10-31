#pragma once

#include <RadioLib.h>
#include <Arduino.h>
#include "radio.h"

#ifndef LR2021_FLRC_DRIVER
#define LR2021_FLRC_DRIVER

// Table 18-9
struct LR2021FlrcPktStatus
{
    uint16_t pktlen;
    float rssiAvg;  // dBm, e.g. -87.5
    float rssiSync; // dBm, e.g. -56.5
};

class LR2021FLRCDriver
{
private:
    SPIClass *_spi;
    SPISettings spiSettings;

    static LR2021FLRCDriver *_instance;

    /* Flags */
    volatile bool radioEvent = false;
    static void setFlag();

    /* IRQ */
    uint32_t readIRQ();

    /* Rx FIFO */
    void readRxFifo(uint8_t data[], uint8_t len);
    uint16_t readRxFifoLevel();
    uint8_t readAndClearRxFifoFlags();

    /* SPI */
    void spiWrite(const uint8_t cmd[], size_t len);
    void spiTransfer(const uint8_t txBuf[], uint8_t rxBuf[], size_t len);

    /* Other */
    LR2021FlrcPktStatus getFlrcPacketStatus();

    /* Init */
    void initTxPower();
    void initInterpolationRate();
    void initRSSICalibration();
    void initRxTxFallbackMode();

public:
    LR2021 *radio = nullptr;
    LR2021Error setIRQ();
    LR2021Error init(SPIClass &spi);

    LR2021FLRCDriver();
    LR2021Error transmit(uint8_t data[], uint8_t len);
    LR2021Error transmitBurst(uint8_t packets[][FLRC_MAX_PACKET_SIZE], uint16_t count, uint8_t len);
    LR2021Error receive(uint8_t data[], uint8_t len, LR2021FlrcPktStatus *outStatus);

    // Burst receive: start once, then call receiveBurstPacket() for each packet. The radio stays in Rx in between
    LR2021Error startBurstReceive();
    LR2021Error receiveBurstPacket(uint8_t data[], uint8_t len, LR2021FlrcPktStatus *outStatus, uint32_t timeoutMs);
    void stopBurstReceive();

    // Chip error flags (calibration, XOSC, PLL). Bits are RadioLib's RADIOLIB_LR2021_*_ERR. Waits on BUSY, so not
    // safe to call when init failed with RADIOLIB_ERR_SPI_CMD_TIMEOUT
    uint16_t readErrors();

    void transmitCallSign();
};

#endif
