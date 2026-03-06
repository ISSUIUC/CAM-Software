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
    _rxLastFragMs = 0;
    _parityBuf = (uint8_t*)heap_caps_malloc(RS_PARITY_BUF_SIZE, MALLOC_CAP_SPIRAM);
    _rxDataFragsRcvd = 0;
    memset(&_rsLayout, 0, sizeof(_rsLayout));
    _frameIdGen  = 0;
    _rxPollHits     = 0;
    _rxFrameResets  = 0;
    _rxCrcFail      = 0;
    _rsLastMissing  = 0;
    _rsLastRecovered = 0;
    _rsLastResult   = 0;
    _rsSelfTest     = (int8_t)rsSelfTest();

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

uint16_t Si4463Nuke::crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

static uint8_t headerCRC8(const uint8_t* hdr) {
    // CRC-8/MAXIM over bytes 0,2,3,4,5,6,7 (skip byte 1 = CRC slot)
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < SI4463_FRAG_HDR; i++) {
        if (i == 1) continue;  // skip CRC byte itself
        crc ^= hdr[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x31;
            else crc <<= 1;
        }
    }
    return crc;
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

    // Compute RS block layout
    _rsLayout.compute(len, SI4463_FRAG_USABLE);
    if (_rsLayout.totalFrags > 65535) {
        _lastErr = SI_ERR_TOO_LARGE;
        return false;
    }

    // Compute parity into pre-allocated PSRAM buffer
    if (!_parityBuf) {
        _lastErr = SI_ERR_TOO_LARGE;
        return false;
    }
    memset(_parityBuf, 0, RS_PARITY_BUF_SIZE);
    rsEncode(data, len, _parityBuf, _rsLayout, SI4463_FRAG_USABLE);

    _txData      = data;
    _txLen       = len;
    _txFragTotal = _rsLayout.totalFrags;
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
    pkt[1] = 0x00;                        // CRC slot (computed below)
    pkt[2] = (idx >> 8) & 0xFF;           // fragment index MSB
    pkt[3] = idx & 0xFF;                  // fragment index LSB
    pkt[4] = (_txLen >> 24) & 0xFF;       // total size MSB
    pkt[5] = (_txLen >> 16) & 0xFF;
    pkt[6] = (_txLen >> 8)  & 0xFF;
    pkt[7] = _txLen & 0xFF;              // total size LSB
    pkt[1] = headerCRC8(pkt);             // CRC-8 of header

    // Copy payload: data or parity fragment, then append CRC-16
    uint8_t* payload = pkt + SI4463_FRAG_HDR;
    if (!_rsLayout.isParity(idx)) {
        // Data fragment
        uint32_t offset    = (uint32_t)idx * SI4463_FRAG_USABLE;
        uint32_t remaining = _txLen - offset;
        uint16_t payLen    = (remaining > SI4463_FRAG_USABLE)
                             ? SI4463_FRAG_USABLE : (uint16_t)remaining;
        memcpy(payload, _txData + offset, payLen);
    } else {
        // Parity fragment
        uint16_t parityIdx = idx - _rsLayout.dataFrags;
        memcpy(payload, _parityBuf + (uint32_t)parityIdx * SI4463_FRAG_USABLE,
               SI4463_FRAG_USABLE);
    }
    // CRC-16 over the usable payload bytes
    uint16_t crc = crc16(payload, SI4463_FRAG_USABLE);
    payload[SI4463_FRAG_USABLE]     = (crc >> 8) & 0xFF;
    payload[SI4463_FRAG_USABLE + 1] = crc & 0xFF;

    // Reset TX FIFO only (skip clearInterrupts - saves one CTS round trip).
    // FIFO_INFO with TX reset flag, no response needed.
    {
        uint8_t fifoCmd[2] = {CMD_FIFO_INFO, 0x01};
        sendCmd(fifoCmd, 2);
    }

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
    // Read all FRRs in single transaction
    uint8_t frr[4];
    readFRRAll(frr);

    uint8_t phPend    = frr[0];
    uint8_t chipState = frr[1] & 0x0F;

    bool txDone = false;
    if (phPend & PH_PACKET_SENT) {
        txDone = true;
    } else if (chipState == SI_READY || chipState == SI_READY2) {
        // Chip is done transmitting but FRR was cleared. Accept it.
        txDone = true;
    }

    if (txDone) {
        clearInterrupts();
        _txWaiting = false;

        _txFragIndex++;
        if (_txFragIndex < _txFragTotal) {
            delayMicroseconds(2000);
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
    _rxDataFragsRcvd = 0;
    _rxFrameId   = 0xFF;
    _rxAvail     = false;
    _rxActive    = true;
    memset(_rxRecvBits, 0, sizeof(_rxRecvBits));
    memset(&_rsLayout, 0, sizeof(_rsLayout));
    _txActive    = false; // half-duplex

    // Note: intentionally NOT zeroing the buffer. During chunk_output,
    // the receive thread blocks and misses the start of the next frame.
    // Stale data from previous frames is better than zeros for JPEG decode
    // since consecutive video frames are visually similar.

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
    // Read all 4 FRRs in a single SPI transaction (fastest check)
    uint8_t frr[4];
    readFRRAll(frr);

    uint8_t phPend = frr[0];

    // ONLY trigger on PH_PACKET_RX flag. Do NOT use chip state as fallback.
    // After START_RX, the chip briefly stays in READY while transitioning
    // to RX_TUNE→RX. Using chip state as fallback would cause us to
    // read an empty FIFO (garbage) and double-count every packet.
    if (!(phPend & PH_PACKET_RX)) {
        return;
    }

    // RSSI from FRR_C (already read above)
    _rssi = (int)frr[2] / 2 - 134;

    _rxPollHits++;

    // --- CRITICAL HOT PATH: minimize time before re-entering RX ---
    // The TX sends fragments back-to-back. Every microsecond we spend
    // NOT in RX mode is a fragment we might miss.

    // 1. Read FIFO (no CTS wait - instant)
    uint8_t pkt[SI4463_PACKET_SIZE];
    readFifo(pkt, SI4463_PACKET_SIZE);

    // 2. Clear interrupts FIRST (while chip is in READY, before START_RX).
    //    This ensures PH_PEND flags are clean for the next packet detection.
    clearInterrupts();

    // 3. Reset RX FIFO (fast path - sendCmd, skip response read).
    //    Shared FIFO mode may need explicit reset even after full read.
    {
        uint8_t fifoCmd[2] = {CMD_FIFO_INFO, 0x02};
        sendCmd(fifoCmd, 2);
    }

    // 4. START_RX - re-enter receive mode
    {
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

    // 5. Process the fragment at leisure (memcpy to output buffer)
    processFragment(pkt);

    // If reassembly is complete, stop RX
    // (processFragment sets _rxAvail=true and _rxActive=false)
}

void Si4463Nuke::processFragment(const uint8_t* pkt) {
    // Verify header CRC-8 before trusting any header fields.
    {
        uint8_t expected = pkt[1];
        uint8_t computed = headerCRC8(pkt);
        if (computed != expected) { _rxCrcFail++; return; }
    }

    // Verify payload CRC-16 (detect bit corruption → clean RS erasure)
    {
        const uint8_t* payload = pkt + SI4463_FRAG_HDR;
        uint16_t expected = ((uint16_t)payload[SI4463_FRAG_USABLE] << 8)
                          | payload[SI4463_FRAG_USABLE + 1];
        uint16_t computed = crc16(payload, SI4463_FRAG_USABLE);
        if (computed != expected) { _rxCrcFail++; return; }
    }

    // Parse 8-byte header
    uint8_t  frameId  = pkt[0];
    uint16_t fragIdx  = ((uint16_t)pkt[2] << 8) | pkt[3];
    uint32_t totalSz  = ((uint32_t)pkt[4] << 24) | ((uint32_t)pkt[5] << 16) |
                         ((uint32_t)pkt[6] << 8)  | (uint32_t)pkt[7];

    // Sanity checks
    if (totalSz == 0 || totalSz > _rxBufLen) return;

    // Compute RS layout from totalSize
    RSLayout layout;
    layout.compute(totalSz, SI4463_FRAG_USABLE);
    if (fragIdx >= layout.totalFrags) return;

    // New frame? Reset reassembly.
    if (frameId != _rxFrameId) {
        if (_rxFrameId != 0xFF) _rxFrameResets++;
        _rxFrameId   = frameId;
        _rsLayout    = layout;
        _rxFragTotal = layout.totalFrags;
        _rxFragsRcvd = 0;
        _rxDataFragsRcvd = 0;
        _rxTotalSize = totalSz;
        memset(_rxRecvBits, 0, sizeof(_rxRecvBits));

        // Zero parity buffer for new frame
        if (_parityBuf) {
            uint32_t parBufSize = (uint32_t)layout.numBlocks * RS_PARITY_SHARDS * SI4463_FRAG_USABLE;
            memset(_parityBuf, 0, parBufSize);
        }
    }

    // Deduplicate: skip if we already received this fragment index
    uint16_t bitIdx = fragIdx / 8;
    uint8_t  bitMsk = 1 << (fragIdx % 8);
    if (bitIdx < sizeof(_rxRecvBits) && (_rxRecvBits[bitIdx] & bitMsk)) {
        return;
    }

    const uint8_t* payload = pkt + SI4463_FRAG_HDR;

    if (!_rsLayout.isParity(fragIdx)) {
        // Data fragment → write to _rxBuf
        uint32_t dstOffset = (uint32_t)fragIdx * SI4463_FRAG_USABLE;
        uint16_t payLen    = SI4463_FRAG_USABLE;
        if (fragIdx == _rsLayout.dataFrags - 1) {
            // Last data fragment may be shorter
            payLen = (uint16_t)(totalSz - dstOffset);
        }
        if (_rxBuf && dstOffset + payLen <= _rxBufLen) {
            memcpy(_rxBuf + dstOffset, payload, payLen);
            if (bitIdx < sizeof(_rxRecvBits)) _rxRecvBits[bitIdx] |= bitMsk;
            _rxFragsRcvd++;
            _rxDataFragsRcvd++;
            _rxLastFragMs = millis();
        }
    } else {
        // Parity fragment → write to _parityBuf
        if (_parityBuf) {
            uint16_t parityIdx = fragIdx - _rsLayout.dataFrags;
            uint32_t dstOffset = (uint32_t)parityIdx * SI4463_FRAG_USABLE;
            memcpy(_parityBuf + dstOffset, payload, SI4463_FRAG_USABLE);
            if (bitIdx < sizeof(_rxRecvBits)) _rxRecvBits[bitIdx] |= bitMsk;
            _rxFragsRcvd++;
            _rxLastFragMs = millis();
        }
    }

    // Check completion
    if (_rxFragsRcvd >= _rxFragTotal) {
        rsFinishDecode();
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

        // Timeout: if we've received at least one fragment but nothing
        // new for 150ms, the TX has moved on to the next frame.
        // Accept whatever we have - ~92% of a JPEG is still viewable.
        if (_rxFragsRcvd > 0 && !_rxAvail &&
            millis() - _rxLastFragMs > 150) {
            rsFinishDecode();
            _rxAvail  = true;
            _rxActive = false;
            if (_rxTotalSize > _rxBufLen) _rxTotalSize = _rxBufLen;
        }
    }
}

void Si4463Nuke::rsFinishDecode() {
    if (!_parityBuf || !_rxBuf) {
        _rsLastMissing = 0;
        _rsLastRecovered = 0;
        _rsLastResult = 3; // no buf
        return;
    }
    if (_rxDataFragsRcvd >= _rsLayout.dataFrags) {
        _rsLastMissing = 0;
        _rsLastRecovered = 0;
        _rsLastResult = 0; // no recovery needed
        return;
    }

    _rsLastMissing = _rsLayout.dataFrags - _rxDataFragsRcvd;

    _rsLastRecovered = rsDecode(_rxBuf, _rxTotalSize, _parityBuf,
             _rxRecvBits, sizeof(_rxRecvBits),
             _rsLayout, SI4463_FRAG_USABLE);

    _rsLastResult = (_rsLastRecovered > 0) ? 1 : 2;
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

    const char* rsStr = "?";
    switch (_rsLastResult) {
        case 0: rsStr = "ok";      break;
        case 1: rsStr = "fixed";   break;
        case 2: rsStr = "FAIL";    break;
        case 3: rsStr = "no-buf";  break;
    }

    out.printf("[nuke] chip=%s rssi=%d/%ddBm rx=%u/%u hits=%lu crc=%lu "
               "rs=%s miss=%u recov=%u selftest=%d\n",
               s, rssi, currRssi,
               _rxFragsRcvd, _rxFragTotal,
               (unsigned long)_rxPollHits,
               (unsigned long)_rxCrcFail,
               rsStr, _rsLastMissing, _rsLastRecovered,
               _rsSelfTest);
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
