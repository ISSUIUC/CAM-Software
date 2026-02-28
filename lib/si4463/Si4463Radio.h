#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "radio_config_Si4463_4gfsk_hdr_i.h"

enum Si4463State : uint8_t {
    STATE_IDLE,
    STATE_TX,
    STATE_TX_COMPLETE,
    STATE_RX,
    STATE_RX_COMPLETE
};

struct Si4463PinConfig {
    SPIClass* spi;
    uint8_t cs;
    uint8_t sdn;
    uint8_t irq;
    uint8_t gpio0;
    uint8_t gpio1;
};

class Si4463Radio {
public:
    static const uint16_t MAX_PACKET_LEN  = 8191;
    static const uint16_t FRAG_HEADER_SIZE = 8;
    static const uint16_t FRAG_PAYLOAD    = MAX_PACKET_LEN - FRAG_HEADER_SIZE; // 8183
    static const uint8_t  FIFO_SIZE       = 129;
    static const uint8_t  RX_THRESH       = 40;
    static const uint8_t  TX_THRESH       = 63;

    bool begin(Si4463PinConfig pins);

    // Set caller-provided buffer for RX reassembly (e.g. SPIRAM)
    void setRxBuffer(uint8_t* buf, uint32_t maxSize);

    // Start fragmented transmit (any size, fragments transparently)
    bool tx(const uint8_t* data, uint32_t len);

    // Enter receive mode
    bool rx();

    // Must call every loop iteration to pump FIFO and progress fragments
    void update();

    // True when a complete multi-fragment message is reassembled
    bool avail();

    uint32_t getReceivedLength() const;
    int getRSSI() const;
    Si4463State getState() const;
    bool isTxBusy() const;

    // Debug helpers
    uint8_t readChipState();     // raw FRR A value (chip-level state)
    uint8_t readRxFifoCount();   // bytes currently in RX FIFO
    void printDebug(Print& out);  // dump full diagnostic to any Print stream
    void printConfig(Print& out); // dump key property values after init

private:
    // --- SPI layer ---
    void spiWrite(uint8_t cmd, const uint8_t* data, uint8_t len);
    void spiRead(uint8_t* data, uint8_t len);
    bool waitCTS(uint16_t timeoutMs = 500);
    void sendCommand(const uint8_t* data, uint8_t len);
    void sendCommandAndRead(const uint8_t* cmd, uint8_t cmdLen, uint8_t* resp, uint8_t respLen);
    void readFRRs(uint8_t* data, uint8_t startReg = 0);
    uint8_t getFifoInfo(bool resetTx = false, bool resetRx = false);
    void setProperty(uint8_t group, uint8_t numProps, uint8_t startProp, const uint8_t* data);
    void getProperty(uint8_t group, uint8_t numProps, uint8_t startProp, uint8_t* data);

    // --- Init helpers ---
    void powerOn();
    void applyConfig();
    void configurePacketHandler();
    void configureGPIOs();
    void configureFRRs();
    void configureThresholds();
    void clearFIFO();
    void clearInterrupts();
    bool verifyPartNumber();

    // --- TX/RX engine ---
    void startFragment(uint8_t index);
    void handleTX();
    void handleRX();
    void processReceivedFragment();

    // --- Pin/SPI state ---
    Si4463PinConfig _pins;
    SPISettings _spiSettings;

    // --- State machine ---
    Si4463State _state;
    bool _available;

    // --- TX state (zero-copy) ---
    const uint8_t* _txData;
    uint32_t _txLen;
    uint8_t  _txFragIndex;
    uint8_t  _txFragTotal;
    uint8_t  _frameIdCounter;

    // TX FIFO tracking for current fragment
    uint16_t _txFragPayloadSize;  // payload bytes in this fragment
    uint16_t _txFragBytesSent;    // bytes of this fragment written to FIFO so far
    uint16_t _txFragTotalBytes;   // total bytes of this fragment (header + payload)
    uint8_t  _txFragHeader[FRAG_HEADER_SIZE];
    uint32_t _txStartTime;        // millis() when TX was started (timeout fallback)

    // --- RX state ---
    uint8_t  _fragBuf[MAX_PACKET_LEN]; // staging buffer for current fragment
    uint16_t _fragBufPos;              // bytes received into _fragBuf so far
    uint16_t _rxPacketLen;             // expected packet length from 2-byte prefix
    bool     _rxLenRead;               // have we read the 2-byte length prefix?
    bool     _rxGpio1EdgeFlag;
    int      _rssi;

    // RX reassembly
    uint8_t* _rxBuf;
    uint32_t _rxBufSize;
    uint8_t  _rxFrameId;
    uint8_t  _rxFragsReceived;
    uint8_t  _rxFragTotal;
    uint32_t _rxTotalSize;
};
