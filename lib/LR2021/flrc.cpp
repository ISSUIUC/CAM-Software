#include "flrc.h"

LR2021FLRCDriver *LR2021FLRCDriver::_instance = nullptr;

LR2021FLRCDriver::LR2021FLRCDriver()
    : spiSettings(SPI_SPEED, MSBFIRST, SPI_MODE0)
{
}

LR2021Error LR2021FLRCDriver::init(SPIClass &spi)
{
    _spi = &spi;
    radio = new LR2021(new Module(LR2021_CS, LR2021_GPIO9, LR2021_NRST, LR2021_BUSY, *_spi, spiSettings));

    pinMode(LR2021_CS, OUTPUT);
    digitalWrite(LR2021_CS, HIGH);

    radio->irqDioNum = IRQ_PIN;

    int state = radio->beginFLRC(FREQ_FLRC, BITRATE_FLRC, CR_FLRC, POWER, PREAMBLE_LENGTH, SHAPING_FLRC, XTAL_MODE);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_FLRC_INIT_FAILED, state);

    state = radio->fixedPacketLengthMode(PAYLOAD_SIZE_FLRC);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_PKT_LEN_FAILED, state);

    state = radio->setSyncWord(SYNC_WORD_FLRC, 4);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_SYNC_WORD_FAILED, state);

    state = radio->setCRC(CRC_LENGTH_FLRC);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_CRC_CONFIG_FAILED, state);

    // Set RX boost gain mode
    state = radio->setRxBoostedGainMode(RX_BOOST_FLRC);
    if (state != RADIOLIB_ERR_NONE)
        return LR2021Error(LR2021_ERR_FLRC_INIT_RX_BOOST_FAILED, state);

    initRSSICalibration();
    initTxPower();
    initInterpolationRate();

    _instance = this;
    radio->setIrqAction(setFlag);

    setIRQ();

    initRxTxFallbackMode();

    return LR2021Error(LR2021_ERR_NONE, 0);
}

LR2021Error LR2021FLRCDriver::transmit(uint8_t data[], uint8_t len)
{
    radioEvent = false;
    // Start clean: a stale IRQ ends the wait early, and leftover FIFO bytes from a failed Tx would shift
    // every later packet while the CRC still passes
    uint8_t clearIrqCmd[] = {0x01, 0x16, 0xFF, 0xFF, 0xFF, 0xFF}; // ClearIrq all
    uint8_t clearTxCmd[] = {0x01, 0x1F};                          // ClearTxFifo
    spiWrite(clearIrqCmd, sizeof(clearIrqCmd));
    spiWrite(clearTxCmd, sizeof(clearTxCmd));

    // RadioTxFifo write opcode: 0x00 0x02. Whole packet in one write, it fits the 256-byte FIFO
    static uint8_t cmd[FLRC_MAX_PACKET_SIZE + 2] = {0x00, 0x02};
    memcpy(&cmd[2], data, len);
    spiWrite(cmd, len + 2);

    // SetTx, timeout 0 = none (DS Table 6-12), returns on TxDone
    uint8_t setTxCmd[] = {0x02, 0x0D, 0x00, 0x00, 0x00};
    spiWrite(setTxCmd, sizeof(setTxCmd));

    const unsigned long timeout = 3000 * 1000;
    unsigned long start = micros();
    while (true)
    {
        if (micros() - start > timeout)
        {
            uint8_t setFsCmd[] = {0x01, 0x29}; // SetFs: abort Tx, or the next SetTx is rejected (DS 6.3.6)
            spiWrite(setFsCmd, sizeof(setFsCmd));
            return LR2021Error{LR2021_ERR_TX_TIMEOUT, 0};
        }
        if (!radioEvent)
            taskYIELD();
        radioEvent = false;

        uint32_t irq = readIRQ(); // GetAndClearIrqStatus
        if (irq & (1UL << 19))    // TxDone bit 19
            return LR2021Error{LR2021_ERR_NONE, 0};
    }
}

// Sends packets[0] to packets[count - 1] back to back. The FIFO only holds one 255-byte packet, so each one is
// written right after the previous TxDone. That gap (~250 us) is EAGLE's window to start reading the last one out
LR2021Error LR2021FLRCDriver::transmitBurst(uint8_t packets[][FLRC_MAX_PACKET_SIZE], uint16_t count, uint8_t len)
{
    for (uint16_t i = 0; i < count; i++)
    {
        LR2021Error result = transmit(packets[i], len);
        if (!result.ok())
            return result;

        delayMicroseconds(BURST_GAP_US_FLRC);
    }
    return LR2021Error{LR2021_ERR_NONE, 0};
}

// Single-shot Rx, re-armed every call. A packet that lands while the host is between calls is dropped, never
// misreported. For back-to-back packets use startBurstReceive() and receiveBurstPacket()
LR2021Error LR2021FLRCDriver::receive(uint8_t data[], uint8_t len, LR2021FlrcPktStatus *outStatus)
{
    radioEvent = false;
    // Re-arm from a clean slate. SetRx is rejected while already in Rx (DS 6.3.5), so leave Rx first, then drop
    // any RxDone / FIFO data left from before, else an emptied FIFO gets returned as a good packet
    uint8_t setFsCmd[] = {0x01, 0x29};                            // SetFs
    uint8_t clearIrqCmd[] = {0x01, 0x16, 0xFF, 0xFF, 0xFF, 0xFF}; // ClearIrq all
    uint8_t clearRxCmd[] = {0x01, 0x1E};                          // ClearRxFifo
    uint8_t setRxCmd[] = {0x02, 0x0C, 0x00, 0x00, 0x00};          // SetRx single: back to FS fallback after RxDone
    spiWrite(setFsCmd, sizeof(setFsCmd));
    spiWrite(clearIrqCmd, sizeof(clearIrqCmd));
    spiWrite(clearRxCmd, sizeof(clearRxCmd));
    spiWrite(setRxCmd, sizeof(setRxCmd));

    const unsigned long timeout = 3000 * 1000;
    unsigned long start = micros();
    while (true)
    {
        if (micros() - start > timeout)
            return LR2021Error{LR2021_ERR_RX_TIMEOUT, 0};
        if (!radioEvent)
            taskYIELD();
        radioEvent = false;

        uint32_t irq = readIRQ();
        if (irq & (1UL << 18))
        {                          // RxDone bit 18
            if (irq & (1UL << 22)) // CrcError bit 22
                return LR2021Error{LR2021_ERR_CRC_MISMATCH, 0};

            readRxFifo(data, len);
            *outStatus = getFlrcPacketStatus();

            return LR2021Error{LR2021_ERR_NONE, 0};
        }
    }
}

// Continuous Rx (SetRx timeout 0xFFFFFF, DS 6.3.5): the radio keeps receiving while the host reads the last packet
// out, so back-to-back packets aren't missed during a re-arm like they are with receive()
LR2021Error LR2021FLRCDriver::startBurstReceive()
{
    radioEvent = false;
    uint8_t setFsCmd[] = {0x01, 0x29};                            // SetFs, SetRx is rejected while already in Rx
    uint8_t clearIrqCmd[] = {0x01, 0x16, 0xFF, 0xFF, 0xFF, 0xFF}; // ClearIrq all
    uint8_t clearRxCmd[] = {0x01, 0x1E};                          // ClearRxFifo
    uint8_t clearFifoFlagsCmd[] = {0x01, 0x14, 0xFF, 0xFF};       // ClearFifoIrqFlags, Rx and Tx
    uint8_t setRxCmd[] = {0x02, 0x0C, 0xFF, 0xFF, 0xFF};          // SetRx continuous
    spiWrite(setFsCmd, sizeof(setFsCmd));
    spiWrite(clearIrqCmd, sizeof(clearIrqCmd));
    spiWrite(clearRxCmd, sizeof(clearRxCmd));
    spiWrite(clearFifoFlagsCmd, sizeof(clearFifoFlagsCmd));
    spiWrite(setRxCmd, sizeof(setRxCmd));
    return LR2021Error{LR2021_ERR_NONE, 0};
}

// Next packet of a burst. RX_TIMEOUT means nothing came in timeoutMs, Rx keeps running either way.
// Has to be called promptly: the 256-byte FIFO holds one packet plus 1 byte, so reading has to start before the
// second byte of the next packet lands, ~250 us after RxDone. Once it starts, SPI empties the FIFO ~16x faster
// than the air fills it
LR2021Error LR2021FLRCDriver::receiveBurstPacket(uint8_t data[], uint8_t len, LR2021FlrcPktStatus *outStatus, uint32_t timeoutMs)
{
    unsigned long start = millis();
    while (true)
    {
        if (millis() - start > timeoutMs)
            return LR2021Error{LR2021_ERR_RX_TIMEOUT, 0};
        if (!radioEvent)
            taskYIELD();
        radioEvent = false;

        uint32_t irq = readIRQ();
        if (!(irq & (1UL << 18))) // RxDone bit 18. A bad packet sets CrcError bit 22 with it, same as in receive()
            continue;

        // Less than a packet in the FIFO: it overflowed while we were late and this packet lost bytes.
        // Restarting Rx empties the FIFO so the next packet starts at the front again
        if (readRxFifoLevel() < len)
        {
            startBurstReceive();
            return LR2021Error{LR2021_ERR_RX_FIFO_OVERFLOW, 0};
        }

        // Take the packet out even with a bad CRC, its bytes are in the FIFO either way (DS 5.3.1)
        readRxFifo(data, len);

        // Any overflow since the last packet means bytes went missing, so don't trust what we just read either
        if (readAndClearRxFifoFlags() & 0x10) // FifoOverflow
        {
            startBurstReceive();
            return LR2021Error{LR2021_ERR_RX_FIFO_OVERFLOW, 0};
        }

        if (irq & (1UL << 22))
            return LR2021Error{LR2021_ERR_CRC_MISMATCH, 0};

        *outStatus = getFlrcPacketStatus();
        return LR2021Error{LR2021_ERR_NONE, 0};
    }
}

void LR2021FLRCDriver::stopBurstReceive()
{
    uint8_t setFsCmd[] = {0x01, 0x29}; // SetFs: leave continuous Rx
    spiWrite(setFsCmd, sizeof(setFsCmd));
}

// SetDioIrqConfig: opcode 0x01 0x15  (DS Table 6-46 / §5.7)
// TxDone   = bit 19
// RxDone   = bit 18
LR2021Error LR2021FLRCDriver::setIRQ()
{
    uint32_t irqMask = (1UL << 19) | (1UL << 18);
    uint8_t setDioIrqCmd[] = {
        0x01, 0x15,
        IRQ_PIN,
        (uint8_t)((irqMask >> 24) & 0xFF),
        (uint8_t)((irqMask >> 16) & 0xFF),
        (uint8_t)((irqMask >> 8) & 0xFF),
        (uint8_t)(irqMask & 0xFF)};

    spiWrite(setDioIrqCmd, sizeof(setDioIrqCmd));

    return LR2021Error(LR2021_ERR_NONE, 0);
}

LR2021FlrcPktStatus LR2021FLRCDriver::getFlrcPacketStatus()
{
    // Phase 1: send opcode only (GetFlrcPacketStatus 0x02 0x4B)
    const uint8_t cbuffer[2] = {0x02, 0x4B};
    spiWrite(cbuffer, sizeof(cbuffer));

    // Phase 2: Table 18-9 read 7 bytes raw. FYI: chip prepends 2 stat bytes before the 5 data bytes
    // rx[2..3] = pkt_len MSB first
    // rx[4]    = rssi_avg integer part
    // rx[5]    = rssi_sync integer part
    // rx[6]    = flags: [7:4]=sw_num [2]=rssi_avg+0.5 [0]=rssi_sync+0.5
    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};
    spiTransfer(tx, rx, sizeof(rx));

    LR2021FlrcPktStatus s;
    s.pktlen = (uint16_t)(rx[2] << 8) | rx[3];
    s.rssiAvg = -(int16_t)rx[4] - (((rx[6] >> 2) & 0x01) ? 0.5f : 0.0f);
    s.rssiSync = -(int16_t)rx[5] - (((rx[6] >> 0) & 0x01) ? 0.5f : 0.0f);
    return s;
}

/* ---- helpers ---*/

uint32_t LR2021FLRCDriver::readIRQ()
{
    // GetAndClearIrqStatus 0x01 0x17 is a read command: the IRQs it cleared come back in a second frame
    // (DS Tables 6-48/6-49). Atomic, IRQs latching after the clear stay pending
    const uint8_t cmd[2] = {0x01, 0x17};
    spiWrite(cmd, sizeof(cmd));

    uint8_t tx[6] = {0};
    uint8_t rx[6] = {0};
    spiTransfer(tx, rx, sizeof(rx));
    return (uint32_t)rx[2] << 24 | (uint32_t)rx[3] << 16 |
           (uint32_t)rx[4] << 8 | (uint32_t)rx[5];
}

// ReadRadioRxFifo 0x00 0x01. The data comes back in the same frame, starting at byte 2
void LR2021FLRCDriver::readRxFifo(uint8_t data[], uint8_t len)
{
    uint8_t txBuf[FLRC_MAX_PACKET_SIZE + 2] = {0x00, 0x01};
    uint8_t rxBuf[FLRC_MAX_PACKET_SIZE + 2];
    spiTransfer(txBuf, rxBuf, len + 2);
    memcpy(data, &rxBuf[2], len);
}

// GetRxFifoLevel 0x01 0x1C: bytes waiting in the Rx FIFO. Answer comes in a second frame (DS Tables 6-56/6-57)
uint16_t LR2021FLRCDriver::readRxFifoLevel()
{
    const uint8_t cmd[2] = {0x01, 0x1C};
    spiWrite(cmd, sizeof(cmd));

    uint8_t tx[4] = {0};
    uint8_t rx[4] = {0};
    spiTransfer(tx, rx, sizeof(rx));
    return (uint16_t)(rx[2] << 8) | rx[3];
}

// GetAndClearFifoIrqFlags 0x01 0x2E, Rx FIFO flags: 0x08 full, 0x10 overflow, 0x20 underflow (DS Tables 6-50, 6-55).
// They latch whether or not the FIFO IRQs are enabled
uint8_t LR2021FLRCDriver::readAndClearRxFifoFlags()
{
    const uint8_t cmd[2] = {0x01, 0x2E};
    spiWrite(cmd, sizeof(cmd));

    uint8_t tx[4] = {0};
    uint8_t rx[4] = {0};
    spiTransfer(tx, rx, sizeof(rx));
    return rx[2]; // rx[3] is the Tx FIFO
}

// GetErrors 0x01 0x10: latched until ClearErrors or reset. Answer comes in a second frame, like the reads above
uint16_t LR2021FLRCDriver::readErrors()
{
    const uint8_t cmd[2] = {0x01, 0x10};
    spiWrite(cmd, sizeof(cmd));

    uint8_t tx[4] = {0};
    uint8_t rx[4] = {0};
    spiTransfer(tx, rx, sizeof(rx));
    return (uint16_t)(rx[2] << 8) | rx[3];
}

void LR2021FLRCDriver::spiWrite(const uint8_t cmd[], size_t len)
{
    while (digitalRead(LR2021_BUSY))
        ;
    _spi->beginTransaction(spiSettings);
    digitalWrite(LR2021_CS, LOW);
    _spi->transferBytes(cmd, nullptr, len);
    digitalWrite(LR2021_CS, HIGH);
    _spi->endTransaction();
}

void LR2021FLRCDriver::spiTransfer(const uint8_t txBuf[], uint8_t rxBuf[], size_t len)
{
    while (digitalRead(LR2021_BUSY))
        ;
    _spi->beginTransaction(spiSettings);
    digitalWrite(LR2021_CS, LOW);
    _spi->transferBytes(txBuf, rxBuf, len);
    digitalWrite(LR2021_CS, HIGH);
    _spi->endTransaction();
}

IRAM_ATTR void LR2021FLRCDriver::setFlag()
{
    if (_instance)
    {
        _instance->radioEvent = true;
    }
}

// Might not be necessary
void LR2021FLRCDriver::transmitCallSign()
{
    radio->variablePacketLengthMode(PAYLOAD_SIZE_FLRC);
    radio->startTransmit((uint8_t *)CALL_SIGN, strlen(CALL_SIGN)); // FLRC variable length needs >= 6 bytes
    delay(500);
    radio->fixedPacketLengthMode(PAYLOAD_SIZE_FLRC);
    delay(10);
    setIRQ(); // startTransmit remapped the DIO IRQs. Its leftover TxDone is cleared by the next transmit()
}

void LR2021FLRCDriver::initRSSICalibration()
{
    // 7.3.4 SetRssiCalibration
    // 0x02 and 0x05 opcode, LF path is 0x01 | HF path is 0x10 | HF & LF is 0x11
    uint8_t rssiCalCmd[3 + 3 * 27] = {0x02, 0x05, 0x01};
    for (int i = 0; i < 27; i++)
    {
        uint16_t gain = (uint16_t)RSSI_CAL_GAIN_LF[i] & 0x3FF; // 10 bit complelment. 0x3FF is 1111111111
        rssiCalCmd[3 + 3 * i] = gain >> 8;                     // upper two bits
        rssiCalCmd[4 + 3 * i] = gain & 0xFF;                   // 1111 1111 || lower 8 bits
        rssiCalCmd[5 + 3 * i] = RSSI_CAL_NF_LF[i];             // NF is
    }
    spiWrite(rssiCalCmd, sizeof(rssiCalCmd));
}

void LR2021FLRCDriver::initTxPower()
{
    // 7.4.3 SetPaConfig, then SetTxParams
    // SetPaConfig op code 0x02 0x02.
    // Byte 3: 0x0 (default): LF PA Full single-ended mode (FSM).
    // This option selects the power amplifier optimized for low-frequency operating
    // where 2 PAs branch running in phase.
    // 0x10 for the last byte is like "If the HF PA is not used, set the parameter to 16 (default)"
    uint8_t paConfigCmd[] = {0x02, 0x02, 0x00, (uint8_t)((PA_LF_DUTY_CYCLE << 4) | PA_LF_SLICES), 0x10};
    uint8_t txParamsCmd[] = {0x02, 0x03, (uint8_t)(POWER * 2), 0x05}; // 0.5 dB steps, 48 us ramp like RadioLib
    spiWrite(paConfigCmd, sizeof(paConfigCmd));
    spiWrite(txParamsCmd, sizeof(txParamsCmd));
}

void LR2021FLRCDriver::initInterpolationRate()
{
    // DAC uses 16 samples instead of 8 for low frequency things?
    // Tx modulator interpolates x16 instead of x8. Semtech's USP sets it for FLRC
    // below 2.08 Mbps. WriteRegMemMask32, MUST come after the last SetFlrcModulationParams
    // Opcode = 0x01 0x05
    // TABLE 6.2.2 WriteRegMemMask32
    // Addy: 0xF30904
    // Bit Mask: 8 (mask -> 00,00,01,00)
    // Value: 00, 00, bitrate < 2080 ? 01 : 00, 00
    uint8_t dblRateCmd[] = {0x01, 0x05, 0xF3, 0x09, 0x04, 0x00, 0x00, 0x01, 0x00,
                            0x00, 0x00, (BITRATE_FLRC < 2080) ? 0x01 : 0x00, 0x00};
    spiWrite(dblRateCmd, sizeof(dblRateCmd));
}

void LR2021FLRCDriver::initRxTxFallbackMode()
{
    // SetRxTxFallbackMode → FS (0x03), opcode 0x0206
    uint8_t fallbackCmd[] = {0x02, 0x06, 0x03};
    spiWrite(fallbackCmd, sizeof(fallbackCmd));
}