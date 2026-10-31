#pragma once

#include <RadioLib.h>
#include <Arduino.h>
#include "../../main/pins.h"

// Modulation CAMRadio, framer.h and reassembler.h run on (driver picked in modulation.h). CAM and EAGLE have to match
#define MODULATION_FSK 1
#define MODULATION_FLRC 2
#define RADIO_MODULATION MODULATION_FLRC

static const uint16_t PAYLOAD_SIZE_FLRC = 511;
static const uint8_t PAYLOAD_SIZE_FSK = 255;
#define FLRC_MAX_PACKET_SIZE 255

#define SPI_SPEED 16000000

static uint8_t SYNC_WORD_FLRC[] = {0x2D, 0x01, 0x4B, 0x1D};
static uint8_t SYNC_WORD_FSK[] = {0x01, 0x23, 0x45, 0x67,
                                  0x89, 0xAB, 0xCD, 0xEF};
static const char *CALL_SIGN = "KE2CNQ";
static const int IRQ_PIN = 9;

#define FREQ_915 915.0  // mhz
#define FREQ_434 434.0  // mhz
#define FREQ_447 446.85 // mhz

#define FREQ_FSK FREQ_447  // MHz
#define FREQ_FLRC FREQ_447 // MHz

#define BITRATE_FLRC 1300                   // kbps (kilo-BITS per second) raw. DS Rev 2.2 Table 3-8
#define CR_FLRC RADIOLIB_LR2021_FLRC_CR_3_4 // 1300 * 3/4 = 975 kbps effective (DS Table 18-2)
#define SHAPING_FLRC RADIOLIB_SHAPING_0_5   // Table 18.1.3: FLRC allows BT 0.5, 1.0 or off. Sensitivity specs use 0.5
#define CRC_LENGTH_FLRC 3                   // bytes: 0, 2, 3 or 4 (RadioLib setCRC)
#define RX_BOOST_FLRC 7                     // SetRxPath rx_boost 0..7. 7 = +1 dB LF sensitivity for +1.2 mA (DS Tables 3-6, 3-8)
#define BURST_GAP_US_FLRC 0                 // extra wait between burst packets. Raise it if EAGLE reports Rx FIFO overflows

#define BITRATE_FSK_434 1000
#define RX_BANDWIDTH_FSK_434 2222.22 // khz Table 11-2
#define FREQ_DEV_FSK_434 250.0       // khz

#define BITRATE_FSK_447 1000
#define RX_BANDWIDTH_FSK_447 2222.22
#define FREQ_DEV_FSK_447 250

#define BITRATE_FSK_915 2000
#define RX_BANDWIDTH_FSK_915 1111.10 // khz Table 11-2
#define FREQ_DEV_FSK_915 500         // khz

#define POWER 22           // dBm Tx Power
#define PREAMBLE_LENGTH 24 // bit
#define XTAL_MODE 0        // tcxoVoltage argument, setting to 0 brings it to xtal mode

// LF PA for POWER 22: one row of DS Rev 2.2 Table 7-20
#define PA_LF_DUTY_CYCLE 7
#define PA_LF_SLICES 7

// LF PA for POWER 22 | Table 7-20
#define PA_LF_DUTY_CYCLE 7
#define PA_LF_SLICES 7

// RSSI Calibration Table 7-8 LF 490 MHz
static const int16_t RSSI_CAL_GAIN_LF[27] = {                                                 // 0.25 dB steps, signed
                                             -46, -22, 2, 24, 44, 63, 84, 104, 138, 158, 181, // G1..G11
                                             200, 196, 208, 210, 211, 215, 217, 218,          // G12 at rx_boost 0..7
                                             224, 228, 231, 234, 237, 238, 240, 242};         // G13 at rx_boost 0..7
static const uint8_t RSSI_CAL_NF_LF[27] = {                                                   // noise figure, 0.25 dB steps
                                           221, 205, 190, 169, 147, 130, 113, 87, 46, 30, 18,
                                           13, 13, 13, 13, 13, 12, 12, 12,
                                           12, 12, 12, 12, 11, 11, 10, 10};

/* FIFO */
#define FIFO_TX_CHUNK_1 255    // bytes in first write (fills FIFO without overflow)
#define FIFO_TX_CHUNK_2 256    // bytes in second write (remainder of 511)
#define FIFO_TX_LOW_THRESH 128 // fire TxFifoLow IRQ when FIFO drops below this level

#define FIFO_RX_HIGH_THRESH 200 // fire RxFifo IRQ when RX FIFO fills above this

/* Error stuff */
#define LR2021_ERR_NONE 0
#define LR2021_ERR_GENERIC -6767
#define LR2021_ERR_SPI_INIT_FAILED -2001
#define LR2021_ERR_FLRC_INIT_FAILED -2002
#define LR2021_ERR_FSK_INIT_FAILED -2009
#define LR2021_ERR_SYNC_WORD_FAILED -2003
#define LR2021_ERR_CRC_CONFIG_FAILED -2004
#define LR2021_ERR_PKT_LEN_FAILED -2005
#define LR2021_ERR_TX_TIMEOUT -2006
#define LR2021_ERR_RX_TIMEOUT -2007
#define LR2021_ERR_CRC_MISMATCH -2008
#define LR2021_ERR_ADDRESS_FILTERING -2010
#define LR2021_ERR_DATASHAPING -2011
#define LR2021_ERR_FSK_FIXED_PACKET_MD -2012
#define LR2021_ERR_RX_FIFO_OVERFLOW -2013

struct LR2021Error
{
    int driverCode;
    int radioLibCode;

    bool ok() const { return driverCode == LR2021_ERR_NONE; }

    const char *stageStr() const
    {
        switch (driverCode)
        {
        case LR2021_ERR_NONE:
            return "OK";
        case LR2021_ERR_SPI_INIT_FAILED:
            return "SPI bus init failed";
        case LR2021_ERR_FSK_INIT_FAILED:
            return "radio.beginFSK failed";
        case LR2021_ERR_FLRC_INIT_FAILED:
            return "radio.beginFLRC() failed";
        case LR2021_ERR_SYNC_WORD_FAILED:
            return "radio.setSyncWord() failed";
        case LR2021_ERR_CRC_CONFIG_FAILED:
            return "radio.setCRC() failed";
        case LR2021_ERR_PKT_LEN_FAILED:
            return "radio.fixedPacketLengthMode() failed";
        case LR2021_ERR_TX_TIMEOUT:
            return "lr2021.transmit() tx timeout";
        case LR2021_ERR_RX_TIMEOUT:
            return "lr2021.receive() rx timeout";
        case LR2021_ERR_CRC_MISMATCH:
            return "lr2021.receive() crc mismatch";
        case LR2021_ERR_ADDRESS_FILTERING:
            return "radio.disableAddressFiltering(); failed";
        case LR2021_ERR_DATASHAPING:
            return "radio.setDataShaping(); failed";
        case LR2021_ERR_FSK_FIXED_PACKET_MD:
            return "radio.fixedPacketLengthMode(PAYLOAD_SIZE_FSK); failed";
        case LR2021_ERR_RX_FIFO_OVERFLOW:
            return "lr2021.receiveBurstPacket() rx fifo overflow, packet dropped";
        case LR2021_ERR_GENERIC:
            return "GL bro idk where this error is";
        default:
            return "unknown error";
        }
    }
};
