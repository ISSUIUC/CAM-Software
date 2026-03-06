#pragma once
#include <stdint.h>
#include <string.h>
#include "../RS/rs.hpp"

#define RS_DATA_SHARDS 192
#define RS_PARITY_SHARDS 63

static_assert(RS_DATA_SHARDS + RS_PARITY_SHARDS <= 255, "Block size exceeds GF(2^8)");

// Returns 0 on success, negative on failure.
static int rsSelfTest() {
    RS::ReedSolomon<RS_DATA_SHARDS, RS_PARITY_SHARDS> rs;
    uint8_t msg[RS_DATA_SHARDS];
    uint8_t ecc[RS_PARITY_SHARDS];
    uint8_t corrupted[RS_DATA_SHARDS];
    uint8_t decoded[RS_DATA_SHARDS];

    // Fill with known pattern
    for (int i = 0; i < RS_DATA_SHARDS; i++) msg[i] = (uint8_t)(i * 7 + 3);
    rs.EncodeBlock(msg, ecc);

    // Test 1: Clean decode (no errors) via Decode on combined buffer
    {
        uint8_t codeword[RS_DATA_SHARDS + RS_PARITY_SHARDS];
        memcpy(codeword, msg, RS_DATA_SHARDS);
        memcpy(codeword + RS_DATA_SHARDS, ecc, RS_PARITY_SHARDS);
        int ret = rs.Decode(codeword, decoded);
        if (ret != 0) return -1; // clean decode failed
        for (int i = 0; i < RS_DATA_SHARDS; i++)
            if (decoded[i] != msg[i]) return -2;
    }

    // Test 2: 30 erasures (known positions) — full erasure correction
    {
        memcpy(corrupted, msg, RS_DATA_SHARDS);
        uint8_t erase_pos[30];
        for (int i = 0; i < 30; i++) {
            erase_pos[i] = (uint8_t)(i * 6);
            corrupted[erase_pos[i]] = 0;
        }
        int ret = rs.DecodeBlock(corrupted, ecc, decoded, erase_pos, 30);
        if (ret != 0) return -100 - ret;
        for (int i = 0; i < RS_DATA_SHARDS; i++)
            if (decoded[i] != msg[i]) return -4;
    }

    // Test 3: 63 erasures (max capacity) — stress test
    {
        memcpy(corrupted, msg, RS_DATA_SHARDS);
        uint8_t erase_pos[63];
        for (int i = 0; i < 63; i++) {
            erase_pos[i] = (uint8_t)(i * 3);
            corrupted[erase_pos[i]] = 0;
        }
        int ret = rs.DecodeBlock(corrupted, ecc, decoded, erase_pos, 63);
        if (ret != 0) return -200 - ret;
        for (int i = 0; i < RS_DATA_SHARDS; i++)
            if (decoded[i] != msg[i]) return -5;
    }

    return 0;
}

struct RSLayout {
    uint16_t dataFrags;
    uint16_t numBlocks;
    uint16_t totalFrags;
    uint16_t lastBlockK;

    void compute(uint32_t totalSize, uint16_t fragUsable) {
        dataFrags = (totalSize + fragUsable - 1) / fragUsable;
        numBlocks = (dataFrags + RS_DATA_SHARDS - 1) / RS_DATA_SHARDS;
        lastBlockK = dataFrags - (numBlocks - 1) * RS_DATA_SHARDS;
        totalFrags = dataFrags + numBlocks * RS_PARITY_SHARDS;
    }

    bool isParity(uint16_t fragIdx) const { return fragIdx >= dataFrags; }

    uint16_t blockK(uint16_t block) const {
        return (block < numBlocks - 1) ? RS_DATA_SHARDS : lastBlockK;
    }
};

/**
 * @brief Encodes parity packets for an entire JPEG frame
 * @param data Raw payload of length `dataLen`
 * @param parityOut Output buffer of length `numBlocks * RS_PARITY_SHARDS * fragUsable`
 * @param layout Layout computed from `RSLayout`
 * @param fragUsable: Data bytes per fragment
 */
static void rsEncode(const uint8_t* data, uint32_t dataLen, uint8_t* parityOut, const RSLayout& layout, uint16_t fragUsable) {

    RS::ReedSolomon<RS_DATA_SHARDS, RS_PARITY_SHARDS> rs;
    uint8_t msg[RS_DATA_SHARDS];
    uint8_t ecc[RS_PARITY_SHARDS];

    for (uint16_t block = 0; block < layout.numBlocks; block++) {
        
        uint16_t k = layout.blockK(block);
        uint16_t blockDataStart = block * RS_DATA_SHARDS;

        for (uint16_t bp = 0; bp < fragUsable; bp++) {
            memset(msg, 0, RS_DATA_SHARDS);
            for (uint16_t i = 0; i < k; i++) {
                uint32_t off = (uint32_t)(blockDataStart + i) * fragUsable + bp;
                if (off < dataLen) msg[i] = data[off];
            }

            rs.EncodeBlock(msg, ecc);

            for (uint16_t p = 0; p < RS_PARITY_SHARDS; p++) {
                uint32_t dst = (uint32_t)(block * RS_PARITY_SHARDS + p) * fragUsable + bp;
                parityOut[dst] = ecc[p];
            }
        }
    }
}

// Decode (repair) a frame using RS erasure correction.
// dataBuf:      received data buffer (gaps for lost fragments)
// dataLen:      original data length
// rxParityBuf:  received parity buffer
// rxRecvBits:   bitfield of received fragment indices
// recvBitsSize: sizeof(rxRecvBits)
// layout:       pre-computed RS layout
// fragUsable:   usable data bytes per fragment
// Returns number of data fragments recovered.
static uint16_t rsDecode(uint8_t* dataBuf, uint32_t dataLen,
                         const uint8_t* rxParityBuf,
                         const uint8_t* rxRecvBits, uint16_t recvBitsSize,
                         const RSLayout& layout, uint16_t fragUsable) {
    RS::ReedSolomon<RS_DATA_SHARDS, RS_PARITY_SHARDS> rs;
    uint16_t recovered = 0;

    uint8_t msg[RS_DATA_SHARDS];
    uint8_t ecc[RS_PARITY_SHARDS];
    uint8_t decoded[RS_DATA_SHARDS];
    uint8_t erasures[RS_DATA_SHARDS + RS_PARITY_SHARDS];

    for (uint16_t block = 0; block < layout.numBlocks; block++) {
        uint16_t k = layout.blockK(block);
        uint16_t blockDataStart = block * RS_DATA_SHARDS;

        // Build erasure list
        uint8_t eraseCount = 0;
        uint8_t dataEraseCount = 0;
        for (uint16_t i = 0; i < k; i++) {
            uint16_t fi = blockDataStart + i;
            uint16_t byteIdx = fi / 8;
            uint8_t  bitMsk  = 1 << (fi % 8);
            if (byteIdx >= recvBitsSize || !(rxRecvBits[byteIdx] & bitMsk)) {
                erasures[eraseCount++] = (uint8_t)i;
                dataEraseCount++;
            }
        }
        for (uint16_t p = 0; p < RS_PARITY_SHARDS; p++) {
            uint16_t fi = layout.dataFrags + block * RS_PARITY_SHARDS + p;
            uint16_t byteIdx = fi / 8;
            uint8_t  bitMsk  = 1 << (fi % 8);
            if (byteIdx >= recvBitsSize || !(rxRecvBits[byteIdx] & bitMsk)) {
                erasures[eraseCount++] = (uint8_t)(RS_DATA_SHARDS + p);
            }
        }

        if (dataEraseCount == 0) continue;
        // Erasure correction: each known erasure costs 1 parity symbol
        if (eraseCount > RS_PARITY_SHARDS) continue;

        // Decode each byte position with known erasure positions
        bool blockOk = true;
        for (uint16_t bp = 0; bp < fragUsable; bp++) {
            memset(msg, 0, RS_DATA_SHARDS);
            for (uint16_t i = 0; i < k; i++) {
                uint16_t fi = blockDataStart + i;
                if ((fi / 8) < recvBitsSize && (rxRecvBits[fi / 8] & (1 << (fi % 8)))) {
                    uint32_t off = (uint32_t)(blockDataStart + i) * fragUsable + bp;
                    msg[i] = (off < dataLen) ? dataBuf[off] : 0;
                }
            }

            memset(ecc, 0, RS_PARITY_SHARDS);
            for (uint16_t p = 0; p < RS_PARITY_SHARDS; p++) {
                uint16_t fi = layout.dataFrags + block * RS_PARITY_SHARDS + p;
                if ((fi / 8) < recvBitsSize && (rxRecvBits[fi / 8] & (1 << (fi % 8)))) {
                    uint32_t off = (uint32_t)(block * RS_PARITY_SHARDS + p) * fragUsable + bp;
                    ecc[p] = rxParityBuf[off];
                }
            }

            int ret = rs.DecodeBlock(msg, ecc, decoded, erasures, eraseCount);
            if (ret != 0) {
                blockOk = false;
                break;
            }

            // Write recovered data symbols back
            for (uint8_t e = 0; e < dataEraseCount; e++) {
                uint16_t i = erasures[e];
                uint32_t off = (uint32_t)(blockDataStart + i) * fragUsable + bp;
                if (off < dataLen) dataBuf[off] = decoded[i];
            }
        }

        if (blockOk) recovered += dataEraseCount;
    }

    return recovered;
}
