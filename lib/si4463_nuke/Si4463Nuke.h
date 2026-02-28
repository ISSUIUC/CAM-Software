// Si4463Nuke - Nuclear option SI4463 driver
// Written from scratch based on AN625 register spec.
// Does NOT override WDS packet handler config (which broke previous drivers).
// Uses fixed-size packets matching WDS Field 1 length with software fragmentation.

#pragma once

#include <Arduino.h>
#include <SPI.h>

// Over-the-air packet size - MUST match WDS Field 1 length.
// HDR_I (4GFSK 500ksps 433MHz): 120 bytes
// LDR   (2GFSK 200kbps 433MHz): 64 bytes
#ifndef SI4463_PACKET_SIZE
#define SI4463_PACKET_SIZE 120
#endif

// Fragment header layout (8 bytes):
//   [0]    Frame ID      - unique per tx() call
//   [1]    Reserved      - 0x00
//   [2-3]  Fragment index - 16-bit big-endian, 0-based
//   [4-7]  Total size    - 32-bit big-endian, original payload length
#define SI4463_FRAG_HDR  8
#define SI4463_FRAG_DATA (SI4463_PACKET_SIZE - SI4463_FRAG_HDR)

struct Si4463Pins {
    SPIClass* spi;
    uint8_t cs;
    uint8_t sdn;
    uint8_t irq;   // NIRQ pin (0xFF if unused)
    uint8_t gpio0; // optional (0xFF if unused)
    uint8_t gpio1; // optional (0xFF if unused)
};

enum Si4463Err : uint8_t {
    SI_OK = 0,
    SI_ERR_CTS_TIMEOUT,
    SI_ERR_BAD_PART,
    SI_ERR_TX_BUSY,
    SI_ERR_TOO_LARGE,
    SI_ERR_TX_TIMEOUT,
};

class Si4463Nuke {
public:
    // Initialize hardware, load WDS config, verify chip. Returns false on failure.
    bool begin(Si4463Pins pins);

    // Transmit arbitrary-length data. Fragments automatically.
    // Max size: 65535 * SI4463_FRAG_DATA bytes (~7.3 MB for 120-byte packets).
    // Data buffer must remain valid until isBusy() returns false.
    // Returns false if already transmitting or data too large.
    bool tx(const uint8_t* data, uint32_t len);

    // Begin receiving into caller-provided buffer.
    // Buffer must remain valid until available() returns true.
    void startRx(uint8_t* buf, uint32_t bufLen);

    // Poll chip and drive TX/RX state machines. Call every loop iteration.
    void update();

    // True once after a complete multi-fragment message is reassembled.
    bool available();

    // Length of last received message (valid after available() returned true).
    uint32_t getReceivedLength() const { return _rxTotalSize; }

    // RSSI of last received packet in dBm.
    int getRSSI() const { return _rssi; }

    // True while TX is in progress.
    bool isBusy() const { return _txActive; }

    // TX/RX fragment progress (for progress bars).
    uint16_t getTxProgress() const { return _txFragIndex; }
    uint16_t getTxTotal()    const { return _txFragTotal; }
    uint16_t getRxProgress() const { return _rxFragsRcvd; }
    uint16_t getRxTotal()    const { return _rxFragTotal; }

    // Set RF channel (0-255). Takes effect on next tx()/startRx().
    void setChannel(uint8_t ch) { _channel = ch; }

    // Set TX power level (0x00=min, 0x7F=max +20dBm).
    void setTxPower(uint8_t level);

    // Last error code.
    Si4463Err getLastError() const { return _lastErr; }

    // Debug: raw chip state from FRR.
    uint8_t readChipState();

    // Debug: dump status to any Print stream.
    void printDebug(Print& out);

    // Debug: read and print key property values after init.
    void printConfig(Print& out);

private:
    // ---- SPI layer ----
    bool    waitCTS(uint16_t timeoutMs = 300);
    void    sendCmd(const uint8_t* data, uint8_t len);
    void    sendCmdResp(const uint8_t* cmd, uint8_t cmdLen,
                        uint8_t* resp, uint8_t respLen);
    void    writeFifo(const uint8_t* data, uint8_t len);
    void    readFifo(uint8_t* data, uint8_t len);
    uint8_t readFRR(uint8_t reg);            // reg 0=A,1=B,2=C,3=D
    void    readFRRAll(uint8_t out[4]);       // read A,B,C,D in one transaction
    void    clearFifo(bool tx, bool rx);
    void    clearInterrupts(uint8_t* resp8 = nullptr);
    void    setProp(uint8_t group, uint8_t count, uint8_t start,
                    const uint8_t* data);
    void    getProp(uint8_t group, uint8_t count, uint8_t start,
                    uint8_t* data);

    // ---- Init ----
    void hwReset();
    bool loadConfig();
    void setupFRRs();
    bool verifyChip();

    // ---- TX engine ----
    void sendFragment(uint16_t idx);
    void pollTxDone();

    // ---- RX engine ----
    void enterRx();
    void pollRxDone();
    void processFragment(const uint8_t* pkt);

    // ---- Hardware ----
    Si4463Pins  _pins;
    SPISettings _spiCfg;
    Si4463Err   _lastErr;
    uint8_t     _channel;

    // ---- TX state ----
    bool           _txActive;
    bool           _txWaiting;     // waiting for PACKET_SENT
    const uint8_t* _txData;
    uint32_t       _txLen;
    uint16_t       _txFragIndex;
    uint16_t       _txFragTotal;
    uint8_t        _txFrameId;
    uint32_t       _txStartMs;

    // ---- RX state ----
    bool     _rxActive;
    bool     _rxAvail;
    uint8_t* _rxBuf;
    uint32_t _rxBufLen;
    uint32_t _rxTotalSize;
    uint16_t _rxFragTotal;
    uint16_t _rxFragsRcvd;
    uint8_t  _rxFrameId;
    int      _rssi;
    uint32_t _rxLastFragMs;  // millis() of last valid fragment received

    // ---- Shared ----
    uint8_t  _frameIdGen;

    // ---- Debug counters ----
    uint32_t _rxPollHits;      // total fragments detected (across all frames)
    uint32_t _rxFrameResets;   // times reassembly was reset by new frameId
    uint32_t _rxCrcFail;       // packets rejected by header CRC
};
