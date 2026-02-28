#include "Si4463Radio.h"

// Si4463 command bytes
#define CMD_NOP              0x00
#define CMD_PART_INFO        0x01
#define CMD_POWER_UP         0x02
#define CMD_FUNC_INFO        0x10
#define CMD_SET_PROPERTY     0x11
#define CMD_GET_PROPERTY     0x12
#define CMD_GPIO_PIN_CFG     0x13
#define CMD_FIFO_INFO        0x15
#define CMD_GET_INT_STATUS   0x20
#define CMD_GET_PH_STATUS    0x21
#define CMD_GET_MODEM_STATUS 0x22
#define CMD_GET_CHIP_STATUS  0x23
#define CMD_START_TX         0x31
#define CMD_START_RX         0x32
#define CMD_CHANGE_STATE     0x34
#define CMD_READ_CMD_BUFF    0x44
#define CMD_FRR_A_READ       0x50
#define CMD_FRR_B_READ       0x51
#define CMD_FRR_C_READ       0x53
#define CMD_FRR_D_READ       0x57
#define CMD_WRITE_TX_FIFO    0x66
#define CMD_READ_RX_FIFO     0x77

// Si4463 states from FRR
#define SI_STATE_NOCHANGE 0x00
#define SI_STATE_SLEEP    0x01
#define SI_STATE_SPI_ACTIVE 0x02
#define SI_STATE_READY    0x03
#define SI_STATE_READY2   0x04
#define SI_STATE_TX_TUNE  0x05
#define SI_STATE_RX_TUNE  0x06
#define SI_STATE_TX       0x07
#define SI_STATE_RX       0x08

// GPIO pin modes
#define GPIO_MODE_TX_FIFO_EMPTY  0x23
#define GPIO_MODE_RX_FIFO_FULL   0x22
#define GPIO_MODE_CTS            0x08

// ============================================================================
// SPI Layer
// ============================================================================

void Si4463Radio::spiWrite(uint8_t cmd, const uint8_t* data, uint8_t len) {
    _pins.spi->beginTransaction(_spiSettings);
    digitalWrite(_pins.cs, LOW);
    _pins.spi->transfer(cmd);
    for (uint8_t i = 0; i < len; i++) {
        _pins.spi->transfer(data[i]);
    }
    digitalWrite(_pins.cs, HIGH);
    _pins.spi->endTransaction();
}

void Si4463Radio::spiRead(uint8_t* data, uint8_t len) {
    // Poll READ_CMD_BUFF until CTS byte (0xFF) received
    uint32_t start = millis();
    while (millis() - start < 500) {
        _pins.spi->beginTransaction(_spiSettings);
        digitalWrite(_pins.cs, LOW);
        _pins.spi->transfer(CMD_READ_CMD_BUFF);
        uint8_t cts = _pins.spi->transfer(0x00);
        if (cts == 0xFF) {
            for (uint8_t i = 0; i < len; i++) {
                data[i] = _pins.spi->transfer(0x00);
            }
            digitalWrite(_pins.cs, HIGH);
            _pins.spi->endTransaction();
            return;
        }
        digitalWrite(_pins.cs, HIGH);
        _pins.spi->endTransaction();
        delayMicroseconds(100);
    }
}

bool Si4463Radio::waitCTS(uint16_t timeoutMs) {
    // GPIO1 is used for RX_FIFO_FULL after init, so we always use
    // SPI-based CTS polling (READ_CMD_BUFF returns 0xFF when ready).
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        _pins.spi->beginTransaction(_spiSettings);
        digitalWrite(_pins.cs, LOW);
        _pins.spi->transfer(CMD_READ_CMD_BUFF);
        uint8_t cts = _pins.spi->transfer(0x00);
        digitalWrite(_pins.cs, HIGH);
        _pins.spi->endTransaction();
        if (cts == 0xFF) return true;
        delayMicroseconds(50);
    }
    return false;
}

void Si4463Radio::sendCommand(const uint8_t* data, uint8_t len) {
    waitCTS();
    _pins.spi->beginTransaction(_spiSettings);
    digitalWrite(_pins.cs, LOW);
    for (uint8_t i = 0; i < len; i++) {
        _pins.spi->transfer(data[i]);
    }
    digitalWrite(_pins.cs, HIGH);
    _pins.spi->endTransaction();
}

void Si4463Radio::sendCommandAndRead(const uint8_t* cmd, uint8_t cmdLen, uint8_t* resp, uint8_t respLen) {
    sendCommand(cmd, cmdLen);
    spiRead(resp, respLen);
}

void Si4463Radio::readFRRs(uint8_t* data, uint8_t startReg) {
    uint8_t frrCmd;
    switch (startReg) {
        case 0: frrCmd = CMD_FRR_A_READ; break;
        case 1: frrCmd = CMD_FRR_B_READ; break;
        case 2: frrCmd = CMD_FRR_C_READ; break;
        case 3: frrCmd = CMD_FRR_D_READ; break;
        default: frrCmd = CMD_FRR_A_READ; break;
    }
    // FRR reads don't need CTS
    _pins.spi->beginTransaction(_spiSettings);
    digitalWrite(_pins.cs, LOW);
    _pins.spi->transfer(frrCmd);
    for (uint8_t i = 0; i < 4; i++) {
        data[i] = _pins.spi->transfer(0x00);
    }
    digitalWrite(_pins.cs, HIGH);
    _pins.spi->endTransaction();
}

uint8_t Si4463Radio::getFifoInfo(bool resetTx, bool resetRx) {
    uint8_t arg = 0;
    if (resetTx) arg |= 0x01;
    if (resetRx) arg |= 0x02;
    uint8_t cmd[2] = { CMD_FIFO_INFO, arg };
    uint8_t resp[2];
    sendCommandAndRead(cmd, 2, resp, 2);
    return resp[0]; // RX FIFO count
}

void Si4463Radio::setProperty(uint8_t group, uint8_t numProps, uint8_t startProp, const uint8_t* data) {
    uint8_t cmd[4 + 12]; // max 12 properties at once
    cmd[0] = CMD_SET_PROPERTY;
    cmd[1] = group;
    cmd[2] = numProps;
    cmd[3] = startProp;
    memcpy(&cmd[4], data, numProps);
    sendCommand(cmd, 4 + numProps);
}

void Si4463Radio::getProperty(uint8_t group, uint8_t numProps, uint8_t startProp, uint8_t* data) {
    uint8_t cmd[4] = { CMD_GET_PROPERTY, group, numProps, startProp };
    sendCommandAndRead(cmd, 4, data, numProps);
}

// ============================================================================
// Init Helpers
// ============================================================================

void Si4463Radio::powerOn() {
    uint8_t cmd[7] = {
        CMD_POWER_UP,
        0x01,  // boot options: EZConfig, no patch
        0x00,  // XTAL options
        (uint8_t)((RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ >> 24) & 0xFF),
        (uint8_t)((RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ >> 16) & 0xFF),
        (uint8_t)((RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ >> 8) & 0xFF),
        (uint8_t)(RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ & 0xFF)
    };
    sendCommand(cmd, 7);
    delay(100); // wait for power-up
    waitCTS(500);
}

void Si4463Radio::applyConfig() {
    uint8_t radioConfigArray[] = RADIO_CONFIGURATION_DATA_ARRAY;
    uint8_t* ptr = radioConfigArray;

    while (*ptr != 0x00) {
        uint8_t len = *ptr;
        ptr++;
        sendCommand(ptr, len);
        ptr += len;
        waitCTS(100);
    }
}

void Si4463Radio::configurePacketHandler() {
    // PKT_CRC_CONFIG = 0x00 (CRC off)
    uint8_t crc_off = 0x00;
    setProperty(0x12, 1, 0x00, &crc_off);

    // PKT_CONFIG1 = 0x20 (4GFSK bit)
    uint8_t pkt_config1 = 0x20;
    setProperty(0x12, 1, 0x06, &pkt_config1);

    // PKT_LEN = 0x3A (variable length, MSB first, 2-byte field, endianness)
    // PKT_LEN_FIELD_SOURCE = 0x01 (field 1 contains length)
    uint8_t pkt_len[2] = { 0x3A, 0x01 };
    setProperty(0x12, 2, 0x08, pkt_len);

    // Field 1: length=2 bytes (the length prefix), config with whitening + endian (no 4FSK)
    // PKT_FIELD_1_LENGTH = 0x0002
    // PKT_FIELD_1_CONFIG = 0x06 (whitening 0x02 | endian 0x04)
    uint8_t field1[4] = {
        0x00, 0x02,  // length = 2
        0x06,        // config: whitening + endian
        0x00         // CRC config
    };
    setProperty(0x12, 4, 0x0D, field1);

    // Field 2: length=0x1FFF (max), config with whitening (no 4FSK)
    // PKT_FIELD_2_LENGTH = 0x1FFF
    // PKT_FIELD_2_CONFIG = 0x02 (whitening)
    uint8_t field2[4] = {
        0x1F, 0xFF,  // length = 8191
        0x02,        // config: whitening
        0x00         // CRC config
    };
    setProperty(0x12, 4, 0x11, field2);

    // Field 3: length=0 (stop)
    uint8_t field3[2] = { 0x00, 0x00 };
    setProperty(0x12, 2, 0x15, field3);

    // RX fields mirror TX fields
    // RX Field 1: length=2
    uint8_t rxfield1[3] = { 0x00, 0x02, 0x06 };
    setProperty(0x12, 3, 0x21, rxfield1);

    // RX Field 2: length=0x1FFF
    uint8_t rxfield2[3] = { 0x1F, 0xFF, 0x02 };
    setProperty(0x12, 3, 0x25, rxfield2);
}

void Si4463Radio::configureGPIOs() {
    // GPIO0 = TX_FIFO_EMPTY (0x23)
    // GPIO1 = RX_FIFO_FULL  (0x22)
    // GPIO2 = default (0x20 = RX_STATE)
    // GPIO3 = default (0x21 = RX_DATA)
    // NIRQ  = CTS (0x08)
    // SDO   = default (0x0B)
    // GEN_CONFIG = 0x00
    uint8_t cmd[8] = {
        CMD_GPIO_PIN_CFG,
        GPIO_MODE_TX_FIFO_EMPTY,  // GPIO0
        GPIO_MODE_RX_FIFO_FULL,   // GPIO1
        0x20,                     // GPIO2 (RX_STATE, default)
        0x21,                     // GPIO3 (RX_DATA, default)
        GPIO_MODE_CTS,            // NIRQ = CTS
        0x0B,                     // SDO
        0x00                      // GEN_CONFIG
    };
    sendCommand(cmd, 8);
}

void Si4463Radio::configureFRRs() {
    // FRR_A = CURRENT_STATE (0x09)
    // FRR_B = LATCHED_RSSI  (0x0A)
    // FRR_C = disabled (0x00)
    // FRR_D = disabled (0x00)
    uint8_t frr[4] = { 0x09, 0x0A, 0x00, 0x00 };
    setProperty(0x02, 4, 0x00, frr);
}

void Si4463Radio::configureThresholds() {
    // PKT_TX_THRESHOLD (group 0x12, prop 0x0B) = TX_THRESH
    uint8_t txThresh = TX_THRESH;
    setProperty(0x12, 1, 0x0B, &txThresh);

    // PKT_RX_THRESHOLD (group 0x12, prop 0x0C) = RX_THRESH
    uint8_t rxThresh = RX_THRESH;
    setProperty(0x12, 1, 0x0C, &rxThresh);
}

void Si4463Radio::clearFIFO() {
    getFifoInfo(true, true);
}

void Si4463Radio::clearInterrupts() {
    uint8_t cmd[4] = { CMD_GET_INT_STATUS, 0x00, 0x00, 0x00 };
    uint8_t resp[8];
    sendCommandAndRead(cmd, 4, resp, 8);
}

bool Si4463Radio::verifyPartNumber() {
    uint8_t cmd[1] = { CMD_PART_INFO };
    uint8_t resp[8];
    sendCommandAndRead(cmd, 1, resp, 8);
    uint16_t part = ((uint16_t)resp[1] << 8) | resp[2];
    return (part == 0x4463);
}

// ============================================================================
// Public API
// ============================================================================

bool Si4463Radio::begin(Si4463PinConfig pins) {
    _pins = pins;
    _spiSettings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
    _state = STATE_IDLE;
    _available = false;
    _frameIdCounter = 0;
    _txData = nullptr;
    _txLen = 0;
    _txFragIndex = 0;
    _txFragTotal = 0;
    _fragBufPos = 0;
    _rxPacketLen = 0;
    _rxLenRead = false;
    _rxGpio1EdgeFlag = false;
    _rssi = 0;
    _rxBuf = nullptr;
    _rxBufSize = 0;
    _rxFrameId = 0xFF;
    _rxFragsReceived = 0;
    _rxFragTotal = 0;
    _rxTotalSize = 0;

    // Pin setup
    pinMode(_pins.cs, OUTPUT);
    digitalWrite(_pins.cs, HIGH);
    pinMode(_pins.sdn, OUTPUT);
    pinMode(_pins.irq, INPUT_PULLUP);
    pinMode(_pins.gpio0, INPUT);
    pinMode(_pins.gpio1, INPUT);

    // Power cycle: SDN HIGH -> delay -> SDN LOW -> wait for CTS
    digitalWrite(_pins.sdn, HIGH);
    delay(10);
    digitalWrite(_pins.sdn, LOW);
    delay(15); // Wait for GPIO1 (CTS) or 14ms per datasheet

    // Send NOP, wait CTS
    uint8_t nop = CMD_NOP;
    sendCommand(&nop, 1);
    if (!waitCTS(100)) return false;

    // POWER_UP
    powerOn();

    // Clear interrupts
    clearInterrupts();

    // Verify part number
    if (!verifyPartNumber()) return false;

    // Disable NIRQ interrupts: INT_CTL_ENABLE = 0x00
    uint8_t intCtl = 0x00;
    setProperty(0x01, 1, 0x00, &intCtl);

    // Apply WDS modem configuration
    applyConfig();

    // === RAW WDS TEST MODE ===
    // Skip ALL overrides to match old C driver behavior.
    // Old driver just: reset → applyConfig → clearInterrupts → startRx
    // and it WORKED for RX. One of our overrides breaks RX.
    clearInterrupts();
    clearFIFO();

    return true;
}

void Si4463Radio::setRxBuffer(uint8_t* buf, uint32_t maxSize) {
    _rxBuf = buf;
    _rxBufSize = maxSize;
}

bool Si4463Radio::tx(const uint8_t* data, uint32_t len) {
    if (len == 0) return false;

    _txData = data;
    _txLen = len;
    _txFragTotal = (len + FRAG_PAYLOAD - 1) / FRAG_PAYLOAD;
    if (_txFragTotal > 255) return false; // too large

    _txFragIndex = 0;
    _frameIdCounter++;

    startFragment(0);
    return true;
}

bool Si4463Radio::rx() {
    clearFIFO();
    clearInterrupts();

    _fragBufPos = 0;
    _rxPacketLen = 0;
    _rxLenRead = false;
    _rxGpio1EdgeFlag = false;

    // START_RX: match old C driver (all states -> RX)
    uint8_t cmd[8] = {
        CMD_START_RX,
        RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER,
        0x00,           // condition
        0x00, 0x00,     // RX length = 0 (variable)
        0x08,           // timeout -> RX
        0x08,           // valid -> RX
        0x08            // invalid -> RX
    };
    sendCommand(cmd, 8);

    _state = STATE_RX;
    return true;
}

void Si4463Radio::update() {
    // Read FRR A to get current chip state
    uint8_t frr[4];
    readFRRs(frr, 0);
    uint8_t chipState = frr[0] & 0x0F;

    // Reset RX edge detection flag when GPIO1 deasserts
    if (_rxGpio1EdgeFlag && digitalRead(_pins.gpio1) == LOW) {
        _rxGpio1EdgeFlag = false;
    }

    switch (_state) {
        case STATE_TX:
            // handleTX() will set STATE_TX_COMPLETE when all bytes are in FIFO
            handleTX();
            // Timeout fallback: if stuck for > 5 seconds, force complete
            if (_state == STATE_TX && millis() - _txStartTime > 5000) {
                _state = STATE_TX_COMPLETE;
            }
            break;

        case STATE_TX_COMPLETE:
            // All bytes have been written to FIFO.
            // Wait for chip to finish over-the-air transmission (chip goes to READY).
            if (chipState == SI_STATE_READY || chipState == SI_STATE_READY2) {
                if (_txFragIndex + 1 < _txFragTotal) {
                    // More fragments to send
                    _txFragIndex++;
                    startFragment(_txFragIndex);
                } else {
                    // All fragments sent
                    _state = STATE_IDLE;
                    _txData = nullptr;
                }
            }
            // Timeout fallback
            if (millis() - _txStartTime > 5000) {
                _state = STATE_IDLE;
                _txData = nullptr;
            }
            break;

        case STATE_RX:
            handleRX();
            break;

        case STATE_RX_COMPLETE:
            processReceivedFragment();
            break;

        case STATE_IDLE:
        default:
            break;
    }
}

bool Si4463Radio::avail() {
    if (_available) {
        _available = false;
        return true;
    }
    return false;
}

uint32_t Si4463Radio::getReceivedLength() const {
    return _rxTotalSize;
}

int Si4463Radio::getRSSI() const {
    return _rssi;
}

Si4463State Si4463Radio::getState() const {
    return _state;
}

bool Si4463Radio::isTxBusy() const {
    return (_state == STATE_TX || _state == STATE_TX_COMPLETE);
}

uint8_t Si4463Radio::readChipState() {
    uint8_t frr[4];
    readFRRs(frr, 0);
    return frr[0] & 0x0F;
}

uint8_t Si4463Radio::readRxFifoCount() {
    return getFifoInfo(false, false);
}

void Si4463Radio::printDebug(Print& out) {
    uint8_t frr[4];
    readFRRs(frr, 0);
    uint8_t chipState = frr[0] & 0x0F;
    uint8_t latchRssiRaw = frr[1];

    // GET_MODEM_STATUS: returns CURR_RSSI (real-time, not preamble-gated)
    uint8_t modemCmd[2] = { CMD_GET_MODEM_STATUS, 0xFF }; // 0xFF = don't clear pending
    uint8_t modemResp[8];
    sendCommandAndRead(modemCmd, 2, modemResp, 8);
    uint8_t currRssiRaw = modemResp[2];
    // modemResp[3] = latch_rssi, [4] = ant1_rssi, [5] = ant2_rssi

    uint8_t rxFifo = getFifoInfo(false, false);

    const char* chipStateStr = "?";
    switch (chipState) {
        case 0x01: chipStateStr = "SLEEP"; break;
        case 0x02: chipStateStr = "SPI_ACTIVE"; break;
        case 0x03: chipStateStr = "READY"; break;
        case 0x04: chipStateStr = "READY2"; break;
        case 0x05: chipStateStr = "TX_TUNE"; break;
        case 0x06: chipStateStr = "RX_TUNE"; break;
        case 0x07: chipStateStr = "TX"; break;
        case 0x08: chipStateStr = "RX"; break;
    }

    const char* drvStateStr = "?";
    switch (_state) {
        case STATE_IDLE: drvStateStr = "IDLE"; break;
        case STATE_TX: drvStateStr = "TX"; break;
        case STATE_TX_COMPLETE: drvStateStr = "TX_DONE"; break;
        case STATE_RX: drvStateStr = "RX"; break;
        case STATE_RX_COMPLETE: drvStateStr = "RX_DONE"; break;
    }

    int currRssi = (int)currRssiRaw / 2 - 134;
    int latchRssi = (int)latchRssiRaw / 2 - 134;

    out.printf("[radio] chip=%s(%u) drv=%s gpio0=%u gpio1=%u rxFifo=%u currRSSI=%d latchRSSI=%d fragPos=%u/%u lenRead=%u\n",
               chipStateStr, chipState, drvStateStr,
               digitalRead(_pins.gpio0), digitalRead(_pins.gpio1),
               rxFifo, currRssi, latchRssi,
               _fragBufPos, _rxPacketLen, _rxLenRead);
}

void Si4463Radio::printConfig(Print& out) {
    uint8_t globalCfg[1];
    getProperty(0x00, 1, 0x03, globalCfg);

    uint8_t syncCfg[5];
    getProperty(0x11, 5, 0x00, syncCfg);

    uint8_t pktCfg1[1];
    getProperty(0x12, 1, 0x06, pktCfg1);
    uint8_t pktLen[2];
    getProperty(0x12, 2, 0x08, pktLen);

    uint8_t field1[4];
    getProperty(0x12, 4, 0x0D, field1);
    uint8_t field2[4];
    getProperty(0x12, 4, 0x11, field2);

    uint8_t modemModType[1];
    getProperty(0x20, 1, 0x00, modemModType);

    uint8_t paPwr[1];
    getProperty(0x22, 1, 0x01, paPwr);

    // Frequency control: INTE, FRAC(3 bytes), CHANNEL_STEP(2 bytes)
    uint8_t freqCtrl[6];
    getProperty(0x40, 6, 0x00, freqCtrl);

    // MODEM_CLKGEN_BAND (determines PLL divider)
    uint8_t clkgenBand[1];
    getProperty(0x20, 1, 0x51, clkgenBand);

    // Preamble config
    uint8_t preamble[2];
    getProperty(0x10, 2, 0x00, preamble);
    // preamble[0]=PREAMBLE_TX_LENGTH, preamble[1]=PREAMBLE_CONFIG_STD_1

    out.printf("[cfg] GLOBAL=0x%02X SYNC=0x%02X:%02X%02X%02X%02X MOD=0x%02X PA=%u\n",
               globalCfg[0], syncCfg[0], syncCfg[1], syncCfg[2], syncCfg[3], syncCfg[4],
               modemModType[0], paPwr[0]);
    out.printf("[cfg] PKT_CFG1=0x%02X PKT_LEN=0x%02X%02X F1:len=%u,cfg=0x%02X F2:len=%u,cfg=0x%02X\n",
               pktCfg1[0], pktLen[0], pktLen[1],
               ((uint16_t)field1[0] << 8) | field1[1], field1[2],
               ((uint16_t)field2[0] << 8) | field2[1], field2[2]);
    out.printf("[cfg] FREQ: INTE=0x%02X FRAC=0x%02X%02X%02X STEP=0x%02X%02X BAND=0x%02X\n",
               freqCtrl[0], freqCtrl[1], freqCtrl[2], freqCtrl[3],
               freqCtrl[4], freqCtrl[5], clkgenBand[0]);
    out.printf("[cfg] PREAMBLE: txLen=%u cfg=0x%02X\n", preamble[0], preamble[1]);
}

// ============================================================================
// TX Engine
// ============================================================================

void Si4463Radio::startFragment(uint8_t index) {
    // Compute payload offset and size for this fragment
    uint32_t payloadOffset = (uint32_t)index * FRAG_PAYLOAD;
    uint32_t remaining = _txLen - payloadOffset;
    _txFragPayloadSize = (remaining > FRAG_PAYLOAD) ? FRAG_PAYLOAD : (uint16_t)remaining;
    _txFragTotalBytes = FRAG_HEADER_SIZE + _txFragPayloadSize;

    // Build 8-byte fragment header
    _txFragHeader[0] = _frameIdCounter;
    _txFragHeader[1] = index;
    _txFragHeader[2] = _txFragTotal;
    _txFragHeader[3] = 0x00; // reserved
    _txFragHeader[4] = (_txLen >> 24) & 0xFF;
    _txFragHeader[5] = (_txLen >> 16) & 0xFF;
    _txFragHeader[6] = (_txLen >> 8) & 0xFF;
    _txFragHeader[7] = _txLen & 0xFF;

    // Clear FIFO
    clearFIFO();

    // Write to TX FIFO: [2-byte packet length] [8-byte header] [payload start]
    _pins.spi->beginTransaction(_spiSettings);
    digitalWrite(_pins.cs, LOW);
    _pins.spi->transfer(CMD_WRITE_TX_FIFO);

    // 2-byte length prefix (total fragment size = header + payload)
    _pins.spi->transfer((_txFragTotalBytes >> 8) & 0xFF);
    _pins.spi->transfer(_txFragTotalBytes & 0xFF);

    // 8-byte fragment header
    for (uint8_t i = 0; i < FRAG_HEADER_SIZE; i++) {
        _pins.spi->transfer(_txFragHeader[i]);
    }

    // Fill remaining FIFO space with payload
    // FIFO has 129 bytes, we've used 2 (length) + 8 (header) = 10
    uint16_t initialPayload = FIFO_SIZE - 2 - FRAG_HEADER_SIZE; // 119
    if (initialPayload > _txFragPayloadSize) {
        initialPayload = _txFragPayloadSize;
    }

    uint32_t srcOffset = (uint32_t)index * FRAG_PAYLOAD;
    // Copy from caller's buffer via stack temporary (SPIRAM workaround)
    uint8_t tmpBuf[128];
    uint16_t written = 0;
    while (written < initialPayload) {
        uint16_t chunk = initialPayload - written;
        if (chunk > 128) chunk = 128;
        memcpy(tmpBuf, _txData + srcOffset + written, chunk);
        for (uint16_t i = 0; i < chunk; i++) {
            _pins.spi->transfer(tmpBuf[i]);
        }
        written += chunk;
    }

    digitalWrite(_pins.cs, HIGH);
    _pins.spi->endTransaction();

    _txFragBytesSent = 2 + FRAG_HEADER_SIZE + initialPayload; // total bytes written to FIFO
    _txStartTime = millis();

    // Issue START_TX with TX_LEN=0 (packet handler uses 2-byte length prefix from FIFO)
    // CONDITION=0x30: TXCOMPLETE_STATE=READY (0b0011), no retransmit, start immediately
    // Matches Terrapin reference driver exactly
    uint8_t startTxCmd[7] = {
        CMD_START_TX,
        RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER,
        0x30,        // go to READY after TX
        0x00, 0x00,  // TX_LEN=0: use packet handler field lengths
        0x00, 0x00   // TX_DELAY=0, NUM_REPEAT=0
    };
    sendCommand(startTxCmd, 7);

    _state = STATE_TX;
}

void Si4463Radio::handleTX() {
    // How many more bytes of this fragment's payload still need to enter the FIFO?
    uint16_t headerAndPrefix = 2 + FRAG_HEADER_SIZE;
    uint16_t payloadSent = (_txFragBytesSent > headerAndPrefix)
        ? (_txFragBytesSent - headerAndPrefix) : 0;
    uint16_t payloadRemaining = _txFragPayloadSize - payloadSent;

    if (payloadRemaining == 0) {
        _state = STATE_TX_COMPLETE;
        return;
    }

    // Aggressively fill FIFO whenever there's space.
    // GPIO0 = TX_FIFO_EMPTY: HIGH when FIFO below TX_THRESH, LOW when above.
    // Loop until FIFO is full (GPIO0 LOW) or all data written.
    // Writing TX_THRESH bytes at a time is safe — FIFO is 129 bytes,
    // so even in worst case (62 bytes remaining + 63 written = 125) we can't overflow.
    while (payloadRemaining > 0 && digitalRead(_pins.gpio0) == HIGH) {
        uint16_t toWrite = payloadRemaining;
        if (toWrite > TX_THRESH) toWrite = TX_THRESH;

        uint32_t srcOffset = (uint32_t)_txFragIndex * FRAG_PAYLOAD + payloadSent;

        // Write via stack temporary (SPIRAM workaround)
        uint8_t tmpBuf[TX_THRESH];
        memcpy(tmpBuf, _txData + srcOffset, toWrite);

        _pins.spi->beginTransaction(_spiSettings);
        digitalWrite(_pins.cs, LOW);
        _pins.spi->transfer(CMD_WRITE_TX_FIFO);
        for (uint16_t i = 0; i < toWrite; i++) {
            _pins.spi->transfer(tmpBuf[i]);
        }
        digitalWrite(_pins.cs, HIGH);
        _pins.spi->endTransaction();

        _txFragBytesSent += toWrite;
        payloadSent += toWrite;
        payloadRemaining = _txFragPayloadSize - payloadSent;
    }

    if (payloadRemaining == 0) {
        _state = STATE_TX_COMPLETE;
    }
}

// ============================================================================
// RX Engine
// ============================================================================

void Si4463Radio::handleRX() {
    // First threshold crossing: read RSSI and length prefix
    if (digitalRead(_pins.gpio1) == HIGH && !_rxGpio1EdgeFlag) {
        _rxGpio1EdgeFlag = true;

        // Read RSSI from FRR B
        uint8_t frr[4];
        readFRRs(frr, 0);
        _rssi = (int)frr[1] / 2 - 134;

        if (!_rxLenRead) {
            // Read 2-byte length prefix
            _pins.spi->beginTransaction(_spiSettings);
            digitalWrite(_pins.cs, LOW);
            _pins.spi->transfer(CMD_READ_RX_FIFO);
            uint8_t hi = _pins.spi->transfer(0x00);
            uint8_t lo = _pins.spi->transfer(0x00);
            _rxPacketLen = ((uint16_t)hi << 8) | lo;

            if (_rxPacketLen == 0 || _rxPacketLen > MAX_PACKET_LEN) {
                Serial.printf("[RX] bad length prefix: %u, discarding\n", _rxPacketLen);
                _rxPacketLen = 0;
                _rxLenRead = false;
                _fragBufPos = 0;
                digitalWrite(_pins.cs, HIGH);
                _pins.spi->endTransaction();
                rx(); // re-enter RX
                return;
            }
            _rxLenRead = true;

            // Read remaining available bytes (threshold - 2 already read)
            uint16_t toRead = RX_THRESH - 2;
            if (toRead > _rxPacketLen) toRead = _rxPacketLen;

            for (uint16_t i = 0; i < toRead; i++) {
                _fragBuf[_fragBufPos++] = _pins.spi->transfer(0x00);
            }
            digitalWrite(_pins.cs, HIGH);
            _pins.spi->endTransaction();
        } else {
            // Subsequent threshold crossing — read available data
            uint16_t remaining = _rxPacketLen - _fragBufPos;
            uint16_t toRead = (remaining > RX_THRESH) ? RX_THRESH : remaining;

            _pins.spi->beginTransaction(_spiSettings);
            digitalWrite(_pins.cs, LOW);
            _pins.spi->transfer(CMD_READ_RX_FIFO);
            for (uint16_t i = 0; i < toRead; i++) {
                _fragBuf[_fragBufPos++] = _pins.spi->transfer(0x00);
            }
            digitalWrite(_pins.cs, HIGH);
            _pins.spi->endTransaction();
        }
    }

    // Aggressive FIFO drain: poll FIFO_INFO and read whatever is available.
    // This prevents RX FIFO overflow on RTOS where update() may be delayed.
    if (_rxLenRead && _fragBufPos < _rxPacketLen) {
        uint8_t available = getFifoInfo(false, false);
        while (available > 0 && _fragBufPos < _rxPacketLen) {
            uint16_t remaining = _rxPacketLen - _fragBufPos;
            uint16_t toRead = (available > remaining) ? remaining : available;

            _pins.spi->beginTransaction(_spiSettings);
            digitalWrite(_pins.cs, LOW);
            _pins.spi->transfer(CMD_READ_RX_FIFO);
            for (uint16_t i = 0; i < toRead; i++) {
                _fragBuf[_fragBufPos++] = _pins.spi->transfer(0x00);
            }
            digitalWrite(_pins.cs, HIGH);
            _pins.spi->endTransaction();

            if (_fragBufPos >= _rxPacketLen) break;
            available = getFifoInfo(false, false);
        }
    }

    // Check if complete fragment received
    if (_rxLenRead && _fragBufPos >= _rxPacketLen) {
        _state = STATE_RX_COMPLETE;
    }
}

// ============================================================================
// Fragment Reassembly
// ============================================================================

void Si4463Radio::processReceivedFragment() {
    if (_rxPacketLen < FRAG_HEADER_SIZE) {
        Serial.printf("[RX] malformed pkt len=%u, re-entering RX\n", _rxPacketLen);
        rx();
        return;
    }

    // Parse 8-byte header
    uint8_t frameId   = _fragBuf[0];
    uint8_t fragIndex = _fragBuf[1];
    uint8_t fragTotal = _fragBuf[2];
    // _fragBuf[3] is reserved
    uint32_t totalSize = ((uint32_t)_fragBuf[4] << 24) |
                         ((uint32_t)_fragBuf[5] << 16) |
                         ((uint32_t)_fragBuf[6] << 8)  |
                         (uint32_t)_fragBuf[7];

    uint16_t payloadLen = _rxPacketLen - FRAG_HEADER_SIZE;

    Serial.printf("[RX] frag %u/%u frame=%u pktLen=%u payload=%u totalSize=%lu rssi=%d\n",
                  fragIndex + 1, fragTotal, frameId, _rxPacketLen, payloadLen, totalSize, _rssi);

    // If new frame_id, reset reassembly
    if (frameId != _rxFrameId || fragTotal != _rxFragTotal) {
        if (_rxFragsReceived > 0 && _rxFragsReceived < _rxFragTotal) {
            Serial.printf("[RX] dropped incomplete frame %u (%u/%u frags)\n",
                          _rxFrameId, _rxFragsReceived, _rxFragTotal);
        }
        _rxFrameId = frameId;
        _rxFragTotal = fragTotal;
        _rxFragsReceived = 0;
        _rxTotalSize = totalSize;
    }

    // Copy payload to output buffer at correct offset
    if (_rxBuf && fragIndex < fragTotal) {
        uint32_t dstOffset = (uint32_t)fragIndex * FRAG_PAYLOAD;
        uint32_t copyLen = payloadLen;
        if (dstOffset + copyLen > _rxBufSize) {
            copyLen = (_rxBufSize > dstOffset) ? (_rxBufSize - dstOffset) : 0;
            Serial.printf("[RX] WARNING: buffer overflow, clamped copy to %lu\n", copyLen);
        }
        if (copyLen > 0) {
            memcpy(_rxBuf + dstOffset, _fragBuf + FRAG_HEADER_SIZE, copyLen);
        }
        _rxFragsReceived++;
    }

    // Check if all fragments received
    if (_rxFragsReceived >= _rxFragTotal) {
        _available = true;
        // Clamp totalSize to what was actually received
        if (_rxTotalSize > _rxBufSize) _rxTotalSize = _rxBufSize;
        Serial.printf("[RX] FRAME COMPLETE: %lu bytes (%u fragments)\n", _rxTotalSize, _rxFragTotal);
        _state = STATE_IDLE;
    } else {
        // Re-enter RX for next fragment
        rx();
    }
}
