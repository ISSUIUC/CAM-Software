// Si4463Nuke - Nuclear option SI4463 driver
// Written from scratch based on AN625/AN633 register specs.
//
// Key design decisions vs. old broken drivers:
//   1. WDS config is loaded EXACTLY as generated - no packet handler overrides.
//   2. Status polling via FRR registers (no fragile GPIO edge detection).
//   3. Fixed-size packets (matching WDS Field 1 length) - no variable-length mode.
//   4. RX valid → READY state (prevents FIFO corruption from next packet).
//   5. SPIRAM-safe FIFO writes via stack bounce buffer.

#include "Si4463Nuke.h"

// WDS radio configuration.
// Change this include to switch between configs (HDR_I, LDR, etc.).
// The SI4463_PACKET_SIZE define must match the config's Field 1 length.
#include "../si4463/radio_config_Si4463_4gfsk_hdr_i.h"

// ============================================================================
// SI4463 SPI command bytes (from AN625)
// ============================================================================
#define CMD_NOP              0x00
#define CMD_PART_INFO        0x01
#define CMD_POWER_UP         0x02
#define CMD_SET_PROPERTY     0x11
#define CMD_GET_PROPERTY     0x12
#define CMD_GPIO_PIN_CFG     0x13
#define CMD_FIFO_INFO        0x15
#define CMD_GET_INT_STATUS   0x20
#define CMD_GET_MODEM_STATUS 0x22
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

// Chip states (from FRR or REQUEST_DEVICE_STATE)
#define SI_READY  0x03
#define SI_READY2 0x04
#define SI_TX     0x07
#define SI_RX     0x08

// PH_PEND bit masks (for FRR_A when configured as PH_PEND mode 4)
#define PH_PACKET_RX    0x10
#define PH_PACKET_SENT  0x20

// ============================================================================
// SPI Layer
// ============================================================================

bool Si4463Nuke::waitCTS(uint16_t timeoutMs) {
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        _pins.spi->beginTransaction(_spiCfg);
        digitalWrite(_pins.cs, LOW);
        _pins.spi->transfer(CMD_READ_CMD_BUFF);
        uint8_t cts = _pins.spi->transfer(0x00);
        digitalWrite(_pins.cs, HIGH);
        _pins.spi->endTransaction();
        if (cts == 0xFF) return true;
        delayMicroseconds(50);
    }
    _lastErr = SI_ERR_CTS_TIMEOUT;
    return false;
}

void Si4463Nuke::sendCmd(const uint8_t* data, uint8_t len) {
    if (!waitCTS()) return;
    _pins.spi->beginTransaction(_spiCfg);
    digitalWrite(_pins.cs, LOW);
    for (uint8_t i = 0; i < len; i++)
        _pins.spi->transfer(data[i]);
    digitalWrite(_pins.cs, HIGH);
    _pins.spi->endTransaction();
}

void Si4463Nuke::sendCmdResp(const uint8_t* cmd, uint8_t cmdLen,
                              uint8_t* resp, uint8_t respLen) {
    sendCmd(cmd, cmdLen);
    // Poll READ_CMD_BUFF until CTS, then read response in same transaction
    uint32_t start = millis();
    while (millis() - start < 300) {
        _pins.spi->beginTransaction(_spiCfg);
        digitalWrite(_pins.cs, LOW);
        _pins.spi->transfer(CMD_READ_CMD_BUFF);
        uint8_t cts = _pins.spi->transfer(0x00);
        if (cts == 0xFF) {
            for (uint8_t i = 0; i < respLen; i++)
                resp[i] = _pins.spi->transfer(0x00);
            digitalWrite(_pins.cs, HIGH);
            _pins.spi->endTransaction();
            return;
        }
        digitalWrite(_pins.cs, HIGH);
        _pins.spi->endTransaction();
        delayMicroseconds(100);
    }
    _lastErr = SI_ERR_CTS_TIMEOUT;
}

void Si4463Nuke::writeFifo(const uint8_t* data, uint8_t len) {
    // WRITE_TX_FIFO does NOT require CTS wait.
    // Bounce through stack buffer to handle SPIRAM source safely.
    uint8_t tmp[SI4463_PACKET_SIZE];
    if (len > SI4463_PACKET_SIZE) len = SI4463_PACKET_SIZE;
    memcpy(tmp, data, len);

    _pins.spi->beginTransaction(_spiCfg);
    digitalWrite(_pins.cs, LOW);
    _pins.spi->transfer(CMD_WRITE_TX_FIFO);
    for (uint8_t i = 0; i < len; i++)
        _pins.spi->transfer(tmp[i]);
    digitalWrite(_pins.cs, HIGH);
    _pins.spi->endTransaction();
}

void Si4463Nuke::readFifo(uint8_t* data, uint8_t len) {
    // READ_RX_FIFO does NOT require CTS wait.
    _pins.spi->beginTransaction(_spiCfg);
    digitalWrite(_pins.cs, LOW);
    _pins.spi->transfer(CMD_READ_RX_FIFO);
    for (uint8_t i = 0; i < len; i++)
        data[i] = _pins.spi->transfer(0x00);
    digitalWrite(_pins.cs, HIGH);
    _pins.spi->endTransaction();
}

uint8_t Si4463Nuke::readFRR(uint8_t reg) {
    // FRR reads do NOT require CTS wait.
    static const uint8_t frrCmd[] = {
        CMD_FRR_A_READ, CMD_FRR_B_READ, CMD_FRR_C_READ, CMD_FRR_D_READ
    };
    _pins.spi->beginTransaction(_spiCfg);
    digitalWrite(_pins.cs, LOW);
    _pins.spi->transfer(frrCmd[reg & 0x03]);
    uint8_t val = _pins.spi->transfer(0x00);
    digitalWrite(_pins.cs, HIGH);
    _pins.spi->endTransaction();
    return val;
}

void Si4463Nuke::readFRRAll(uint8_t out[4]) {
    // Read all 4 FRRs (A,B,C,D) in a single SPI transaction.
    _pins.spi->beginTransaction(_spiCfg);
    digitalWrite(_pins.cs, LOW);
    _pins.spi->transfer(CMD_FRR_A_READ);
    out[0] = _pins.spi->transfer(0x00);
    out[1] = _pins.spi->transfer(0x00);
    out[2] = _pins.spi->transfer(0x00);
    out[3] = _pins.spi->transfer(0x00);
    digitalWrite(_pins.cs, HIGH);
    _pins.spi->endTransaction();
}

void Si4463Nuke::clearFifo(bool tx, bool rx) {
    uint8_t flags = 0;
    if (tx) flags |= 0x01;
    if (rx) flags |= 0x02;
    uint8_t cmd[2] = {CMD_FIFO_INFO, flags};
    uint8_t resp[2];
    sendCmdResp(cmd, 2, resp, 2);
}

void Si4463Nuke::clearInterrupts(uint8_t* resp8) {
    uint8_t cmd[4] = {CMD_GET_INT_STATUS, 0x00, 0x00, 0x00};
    uint8_t local[8];
    sendCmdResp(cmd, 4, resp8 ? resp8 : local, 8);
}

void Si4463Nuke::setProp(uint8_t group, uint8_t count, uint8_t start,
                          const uint8_t* data) {
    uint8_t cmd[16];
    cmd[0] = CMD_SET_PROPERTY;
    cmd[1] = group;
    cmd[2] = count;
    cmd[3] = start;
    memcpy(cmd + 4, data, count);
    sendCmd(cmd, 4 + count);
}

void Si4463Nuke::getProp(uint8_t group, uint8_t count, uint8_t start,
                          uint8_t* data) {
    uint8_t cmd[4] = {CMD_GET_PROPERTY, group, count, start};
    sendCmdResp(cmd, 4, data, count);
}

// ============================================================================
// Initialization
// ============================================================================

void Si4463Nuke::hwReset() {
    digitalWrite(_pins.sdn, HIGH);
    delay(10);
    digitalWrite(_pins.sdn, LOW);
    delay(15); // POR requires ~6ms, use 15ms for margin
}

bool Si4463Nuke::loadConfig() {
    // Iterate through WDS-generated configuration array.
    // Format: [length][command bytes...][length][command bytes...]...[0x00]
    // The array includes POWER_UP as the first command.
    uint8_t cfgArr[] = RADIO_CONFIGURATION_DATA_ARRAY;
    uint8_t* p = cfgArr;

    while (*p != 0x00) {
        uint8_t len = *p++;
        if (!waitCTS(500)) return false;
        _pins.spi->beginTransaction(_spiCfg);
        digitalWrite(_pins.cs, LOW);
        for (uint8_t i = 0; i < len; i++)
            _pins.spi->transfer(p[i]);
        digitalWrite(_pins.cs, HIGH);
        _pins.spi->endTransaction();
        p += len;
    }
    return waitCTS(500);
}

void Si4463Nuke::setupFRRs() {
    // Configure Fast Response Registers for polling-based operation:
    //   FRR_A = PH_PEND          (mode 4) - PACKET_SENT / PACKET_RX detection
    //   FRR_B = CURRENT_STATE    (mode 9) - chip state machine
    //   FRR_C = LATCHED_RSSI     (mode 10)- signal strength
    //   FRR_D = MODEM_PEND       (mode 6) - sync detect, etc.
    uint8_t frr[4] = {0x04, 0x09, 0x0A, 0x06};
    setProp(0x02, 4, 0x00, frr);
}

bool Si4463Nuke::verifyChip() {
    uint8_t cmd = CMD_PART_INFO;
    uint8_t resp[8];
    sendCmdResp(&cmd, 1, resp, 8);
    uint16_t part = ((uint16_t)resp[1] << 8) | resp[2];
    if (part != 0x4463) {
        _lastErr = SI_ERR_BAD_PART;
        return false;
    }
    return true;
}

// ============================================================================
// Public API
// ============================================================================

bool Si4463Nuke::begin(Si4463Pins pins) {
    _pins    = pins;
    _spiCfg  = SPISettings(8000000, MSBFIRST, SPI_MODE0);
    _lastErr = SI_OK;
    _channel = 0;

    // Zero all state
    _txActive    = false;
    _txWaiting   = false;
    _txData      = nullptr;
    _txLen       = 0;
    _txFragIndex = 0;
    _txFragTotal = 0;
    _txFrameId   = 0;
    _rxActive    = false;
    _rxAvail     = false;
    _rxBuf       = nullptr;
    _rxBufLen    = 0;
    _rxTotalSize = 0;
    _rxFragTotal = 0;
    _rxFragsRcvd = 0;
    _rxFrameId   = 0xFF;
    _rssi        = -134;
    _frameIdGen  = 0;

    // Pin setup
    pinMode(_pins.cs, OUTPUT);
    digitalWrite(_pins.cs, HIGH);
    pinMode(_pins.sdn, OUTPUT);
    if (_pins.irq   != 0xFF) pinMode(_pins.irq,   INPUT_PULLUP);
    if (_pins.gpio0 != 0xFF) pinMode(_pins.gpio0,  INPUT);
    if (_pins.gpio1 != 0xFF) pinMode(_pins.gpio1,  INPUT);

    // Hardware reset
    hwReset();

    // Load full WDS configuration (includes POWER_UP, GPIO, modem, packet, freq)
    if (!loadConfig()) return false;

    // Verify Si4463 part number
    if (!verifyChip()) return false;

    // Configure FRRs for fast polling (WDS disables them by default)
    setupFRRs();

    // Clear all pending interrupts
    clearInterrupts();

    // Clear FIFOs
    clearFifo(true, true);

    return true;
}

void Si4463Nuke::setTxPower(uint8_t level) {
    setProp(0x22, 1, 0x01, &level);
}

// ============================================================================
// TX Engine
// ============================================================================

bool Si4463Nuke::tx(const uint8_t* data, uint32_t len) {
    if (_txActive) {
        _lastErr = SI_ERR_TX_BUSY;
        return false;
    }
    if (len == 0) return false;

    // Calculate fragment count
    uint32_t frags = (len + SI4463_FRAG_DATA - 1) / SI4463_FRAG_DATA;
    if (frags > 65535) {
        _lastErr = SI_ERR_TOO_LARGE;
        return false;
    }

    _txData      = data;
    _txLen       = len;
    _txFragTotal = (uint16_t)frags;
    _txFragIndex = 0;
    _txFrameId   = ++_frameIdGen;
    _txActive    = true;
    _txWaiting   = false;
    _rxActive    = false; // half-duplex: stop RX

    sendFragment(0);
    return true;
}

void Si4463Nuke::sendFragment(uint16_t idx) {
    // Build fixed-size packet: [8-byte header][payload, zero-padded to fill]
    uint8_t pkt[SI4463_PACKET_SIZE];
    memset(pkt, 0, SI4463_PACKET_SIZE);

    // 8-byte fragment header
    pkt[0] = _txFrameId;
    pkt[1] = 0x00;                        // reserved
    pkt[2] = (idx >> 8) & 0xFF;           // fragment index MSB
    pkt[3] = idx & 0xFF;                  // fragment index LSB
    pkt[4] = (_txLen >> 24) & 0xFF;       // total size MSB
    pkt[5] = (_txLen >> 16) & 0xFF;
    pkt[6] = (_txLen >> 8)  & 0xFF;
    pkt[7] = _txLen & 0xFF;              // total size LSB

    // Copy payload (last fragment may be shorter, rest stays zero-padded)
    uint32_t offset    = (uint32_t)idx * SI4463_FRAG_DATA;
    uint32_t remaining = _txLen - offset;
    uint16_t payLen    = (remaining > SI4463_FRAG_DATA)
                         ? SI4463_FRAG_DATA : (uint16_t)remaining;
    memcpy(pkt + SI4463_FRAG_HDR, _txData + offset, payLen);

    // Reset TX FIFO and clear pending interrupts
    clearFifo(true, false);
    clearInterrupts();

    // Write entire packet to FIFO (fits in 129-byte shared FIFO)
    writeFifo(pkt, SI4463_PACKET_SIZE);

    // START_TX: go to READY after completion, TX_LEN=0 (use field config)
    uint8_t cmd[7] = {
        CMD_START_TX,
        _channel,
        0x30,        // TXCOMPLETE_STATE = READY (0x03 << 4)
        0x00, 0x00,  // TX_LEN = 0 (use WDS field length)
        0x00, 0x00   // no delay, no repeat
    };
    sendCmd(cmd, 7);

    _txWaiting = true;
    _txStartMs = millis();
}

void Si4463Nuke::pollTxDone() {
    // Read FRR_A (PH_PEND) - fastest way to check PACKET_SENT
    uint8_t phPend = readFRR(0);

    if (phPend & PH_PACKET_SENT) {
        clearInterrupts();
        _txWaiting = false;

        _txFragIndex++;
        if (_txFragIndex < _txFragTotal) {
            sendFragment(_txFragIndex);
        } else {
            _txActive = false;
            _txData   = nullptr;
        }
        return;
    }

    // Also check chip state - if back to READY without PACKET_SENT flag,
    // the FRR might have been cleared. Accept it.
    uint8_t chipState = readFRR(1) & 0x0F;
    if (chipState == SI_READY || chipState == SI_READY2) {
        // Chip is done transmitting. Check if we missed the PACKET_SENT.
        // This can happen if clearInterrupts was called between the event
        // and our FRR read. Accept it and move on.
        clearInterrupts();
        _txWaiting = false;

        _txFragIndex++;
        if (_txFragIndex < _txFragTotal) {
            sendFragment(_txFragIndex);
        } else {
            _txActive = false;
            _txData   = nullptr;
        }
        return;
    }

    // Timeout: 5 seconds per fragment
    if (millis() - _txStartMs > 5000) {
        uint8_t forceReady[2] = {CMD_CHANGE_STATE, SI_READY};
        sendCmd(forceReady, 2);
        clearInterrupts();
        _txWaiting = false;
        _txActive  = false;
        _txData    = nullptr;
        _lastErr   = SI_ERR_TX_TIMEOUT;
    }
}

// ============================================================================
// RX Engine
// ============================================================================

void Si4463Nuke::startRx(uint8_t* buf, uint32_t bufLen) {
    _rxBuf       = buf;
    _rxBufLen    = bufLen;
    _rxTotalSize = 0;
    _rxFragTotal = 0;
    _rxFragsRcvd = 0;
    _rxFrameId   = 0xFF;
    _rxAvail     = false;
    _rxActive    = true;
    _txActive    = false; // half-duplex

    enterRx();
}

void Si4463Nuke::enterRx() {
    clearFifo(false, true);
    clearInterrupts();

    // START_RX:
    //   RXTIMEOUT_STATE = RX (keep listening on preamble timeout)
    //   RXVALID_STATE   = READY (stop after valid packet so we can safely read FIFO)
    //   RXINVALID_STATE = RX (keep listening on CRC error / invalid)
    uint8_t cmd[8] = {
        CMD_START_RX,
        _channel,
        0x00,        // condition: start immediately
        0x00, 0x00,  // RX_LEN = 0 (use WDS field config)
        SI_RX,       // RXTIMEOUT_STATE  = RX
        SI_READY,    // RXVALID_STATE    = READY
        SI_RX        // RXINVALID_STATE  = RX
    };
    sendCmd(cmd, 8);
}

void Si4463Nuke::pollRxDone() {
    // Read FRR_A (PH_PEND) for PACKET_RX
    uint8_t phPend = readFRR(0);

    if (!(phPend & PH_PACKET_RX)) {
        // Also check: chip went to READY (valid packet received)
        uint8_t chipState = readFRR(1) & 0x0F;
        if (chipState != SI_READY && chipState != SI_READY2) {
            return; // still in RX, nothing yet
        }
        // Chip is in READY but no PH flag - might have been cleared.
        // Fall through to read FIFO anyway.
    }

    // Read RSSI from FRR_C (LATCHED_RSSI)
    uint8_t rssiRaw = readFRR(2);
    _rssi = (int)rssiRaw / 2 - 134;

    // Read full packet from RX FIFO
    uint8_t pkt[SI4463_PACKET_SIZE];
    readFifo(pkt, SI4463_PACKET_SIZE);

    // Clear interrupts
    clearInterrupts();

    // Process the fragment
    processFragment(pkt);

    // Re-enter RX for next fragment (unless message is complete)
    if (_rxActive && !_rxAvail) {
        enterRx();
    }
}

void Si4463Nuke::processFragment(const uint8_t* pkt) {
    // Parse 8-byte header
    uint8_t  frameId  = pkt[0];
    // pkt[1] reserved
    uint16_t fragIdx  = ((uint16_t)pkt[2] << 8) | pkt[3];
    uint32_t totalSz  = ((uint32_t)pkt[4] << 24) | ((uint32_t)pkt[5] << 16) |
                         ((uint32_t)pkt[6] << 8)  | (uint32_t)pkt[7];

    // Sanity check
    if (totalSz == 0) return;

    uint16_t expectedFrags = (totalSz + SI4463_FRAG_DATA - 1) / SI4463_FRAG_DATA;

    // New frame? Reset reassembly.
    if (frameId != _rxFrameId) {
        _rxFrameId   = frameId;
        _rxFragTotal = expectedFrags;
        _rxFragsRcvd = 0;
        _rxTotalSize = totalSz;
    }

    // Compute actual payload length for this fragment
    uint32_t dstOffset = (uint32_t)fragIdx * SI4463_FRAG_DATA;
    uint16_t payLen    = SI4463_FRAG_DATA;
    if (fragIdx == expectedFrags - 1) {
        // Last fragment: actual payload is shorter
        payLen = (uint16_t)(totalSz - dstOffset);
    }

    // Bounds check and copy to output buffer
    if (_rxBuf && dstOffset + payLen <= _rxBufLen) {
        memcpy(_rxBuf + dstOffset, pkt + SI4463_FRAG_HDR, payLen);
        _rxFragsRcvd++;
    }

    // Check completion
    if (_rxFragsRcvd >= _rxFragTotal) {
        _rxAvail  = true;
        _rxActive = false;
        if (_rxTotalSize > _rxBufLen) _rxTotalSize = _rxBufLen;
    }
}

// ============================================================================
// Update loop
// ============================================================================

void Si4463Nuke::update() {
    if (_txActive && _txWaiting) {
        pollTxDone();
    }
    if (_rxActive) {
        pollRxDone();
    }
}

bool Si4463Nuke::available() {
    if (_rxAvail) {
        _rxAvail = false;
        return true;
    }
    return false;
}

// ============================================================================
// Debug
// ============================================================================

uint8_t Si4463Nuke::readChipState() {
    return readFRR(1) & 0x0F;
}

void Si4463Nuke::printDebug(Print& out) {
    uint8_t frr[4];
    readFRRAll(frr);

    uint8_t phPend    = frr[0];
    uint8_t chipState = frr[1] & 0x0F;
    uint8_t rssiRaw   = frr[2];
    uint8_t modemPend = frr[3];
    int rssi = (int)rssiRaw / 2 - 134;

    // Also get real-time RSSI from GET_MODEM_STATUS
    uint8_t mCmd[2]  = {CMD_GET_MODEM_STATUS, 0xFF}; // 0xFF = don't clear
    uint8_t mResp[8];
    sendCmdResp(mCmd, 2, mResp, 8);
    int currRssi = (int)mResp[2] / 2 - 134;

    const char* s = "?";
    switch (chipState) {
        case 0x01: s = "SLEEP";      break;
        case 0x02: s = "SPI_ACTIVE"; break;
        case 0x03: s = "READY";      break;
        case 0x04: s = "READY2";     break;
        case 0x05: s = "TX_TUNE";    break;
        case 0x06: s = "RX_TUNE";    break;
        case 0x07: s = "TX";         break;
        case 0x08: s = "RX";         break;
    }

    out.printf("[nuke] chip=%s(0x%02X) ph=0x%02X mdm=0x%02X "
               "rssi=%d/%ddBm tx=%d(%u/%u) rx=%d(%u/%u)\n",
               s, chipState, phPend, modemPend,
               rssi, currRssi,
               _txActive, _txFragIndex, _txFragTotal,
               _rxActive, _rxFragsRcvd, _rxFragTotal);
}

void Si4463Nuke::printConfig(Print& out) {
    // Global config
    uint8_t globalCfg;
    getProp(0x00, 1, 0x03, &globalCfg);

    // Sync word
    uint8_t sync[5];
    getProp(0x11, 5, 0x00, sync);

    // Packet handler
    uint8_t pktCfg1;
    getProp(0x12, 1, 0x06, &pktCfg1);
    uint8_t pktLen[2];
    getProp(0x12, 2, 0x08, pktLen);

    // Field 1
    uint8_t f1[4];
    getProp(0x12, 4, 0x0D, f1);

    // Modulation
    uint8_t modType;
    getProp(0x20, 1, 0x00, &modType);

    // PA power
    uint8_t paPwr;
    getProp(0x22, 1, 0x01, &paPwr);

    // Frequency
    uint8_t freq[6];
    getProp(0x40, 6, 0x00, freq);

    // Preamble
    uint8_t pre[2];
    getProp(0x10, 2, 0x00, pre);

    // FRR config
    uint8_t frr[4];
    getProp(0x02, 4, 0x00, frr);

    out.printf("[cfg] GLOBAL=0x%02X FIFO=%s MOD=0x%02X PA=%u\n",
               globalCfg,
               (globalCfg & 0x10) ? "shared-129" : "split-64",
               modType, paPwr);
    out.printf("[cfg] SYNC=0x%02X:%02X%02X%02X%02X\n",
               sync[0], sync[1], sync[2], sync[3], sync[4]);
    out.printf("[cfg] PKT_CFG1=0x%02X PKT_LEN=0x%02X%02X "
               "F1:len=%u cfg=0x%02X crc=0x%02X\n",
               pktCfg1, pktLen[0], pktLen[1],
               ((uint16_t)f1[0] << 8) | f1[1], f1[2], f1[3]);
    out.printf("[cfg] FREQ: INTE=0x%02X FRAC=0x%02X%02X%02X STEP=0x%02X%02X\n",
               freq[0], freq[1], freq[2], freq[3], freq[4], freq[5]);
    out.printf("[cfg] PREAMBLE: txLen=%u cfg=0x%02X\n", pre[0], pre[1]);
    out.printf("[cfg] FRR: A=%u B=%u C=%u D=%u\n",
               frr[0], frr[1], frr[2], frr[3]);
}
