#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "modulation.h" // LR2021Driver, RADIO_PACKET_SIZE

#include "RSCodec.h"

#define FRAG_HDR_SIZE 7u
#define FRAG_DATA_SIZE (RADIO_PACKET_SIZE - FRAG_HDR_SIZE)

struct Frame
{
    uint8_t (*pkts)[RADIO_PACKET_SIZE]; // packet array ready for transmitBurst()
    uint16_t count;                     // total number of fragments
    bool valid;                         // false if inputs were bad
};

static inline uint16_t frame_count(uint32_t data_len)
{
    if (data_len == 0)
        return 0;
    // equiv to ceil (total size / frag_data_size)
    return (uint16_t)((data_len + FRAG_DATA_SIZE - 1) / FRAG_DATA_SIZE);
}

static inline void build_fragment(
    uint8_t frame_id,
    uint16_t frag_index,
    uint32_t total_size,
    const uint8_t *src,
    uint8_t *out_pkt)
{
    // Header — 7 bytes
    out_pkt[0] = frame_id;
    out_pkt[1] = (uint8_t)(frag_index >> 8);
    out_pkt[2] = (uint8_t)(frag_index & 0xFF);
    out_pkt[3] = (uint8_t)(total_size >> 24);
    out_pkt[4] = (uint8_t)(total_size >> 16);
    out_pkt[5] = (uint8_t)(total_size >> 8);
    out_pkt[6] = (uint8_t)(total_size & 0xFF);

    // Payload
    uint8_t *payload = out_pkt + FRAG_HDR_SIZE;
    uint32_t offset = (uint32_t)frag_index * FRAG_DATA_SIZE;
    uint32_t remaining = (offset < total_size) ? (total_size - offset) : 0u;
    uint16_t copy_len = (remaining > FRAG_DATA_SIZE)
                            ? (uint16_t)FRAG_DATA_SIZE
                            : (uint16_t)remaining;

    memset(payload, 0, FRAG_DATA_SIZE); // zero-pad (important for last fragment)
    if (copy_len > 0)
        memcpy(payload, src + offset, copy_len);
}

static inline Frame frame_build(
    uint8_t frame_id,
    const uint8_t *data,
    uint32_t data_len,
    uint8_t (*pool)[RADIO_PACKET_SIZE],
    uint16_t count)
{
    Frame f = {nullptr, 0, false};
    if (!data || data_len == 0 || !pool || count == 0)
        return f;

    for (uint16_t i = 0; i < count; i++)
    {
        build_fragment(frame_id, i, data_len, data, pool[i]);
    }

    f.pkts = pool;
    f.count = count;
    f.valid = true;
    return f;
}

static inline Frame parity_frame_build(
    uint8_t frame_id,
    const uint8_t *data,
    uint32_t data_len,
    const RSLayout &layout,
    uint8_t *parity_flat_buf, // pre-allocated: layout.numBlocks * RS_PARITY_SHARDS * FRAG_DATA_SIZE
    uint8_t (*parityPool)[RADIO_PACKET_SIZE])
{
    Frame f = {nullptr, 0, false};
    uint16_t numParityFrags = layout.numBlocks * RS_PARITY_SHARDS;

    rsEncode(data, data_len, parity_flat_buf, layout, FRAG_DATA_SIZE);

    for (uint16_t p = 0; p < numParityFrags; p++)
    {
        uint16_t frag_index = layout.dataFrags + p;
        uint8_t *pkt = parityPool[p];
        // Same 7-byte header as data fragments
        pkt[0] = frame_id;
        pkt[1] = (uint8_t)(frag_index >> 8);
        pkt[2] = (uint8_t)(frag_index & 0xFF);
        pkt[3] = (uint8_t)(data_len >> 24);
        pkt[4] = (uint8_t)(data_len >> 16);
        pkt[5] = (uint8_t)(data_len >> 8);
        pkt[6] = (uint8_t)(data_len & 0xFF);
        memcpy(pkt + FRAG_HDR_SIZE,
               parity_flat_buf + (uint32_t)p * FRAG_DATA_SIZE,
               FRAG_DATA_SIZE);
    }

    f.pkts = parityPool;
    f.count = numParityFrags;
    f.valid = true;
    return f;
}

static inline LR2021Error frame_transmit(LR2021Driver &drv, const Frame &f)
{
    if (!f.valid || f.count == 0)
        return LR2021Error{LR2021_ERR_GENERIC, 0};
    return drv.transmitBurst(f.pkts, f.count, RADIO_PACKET_SIZE);
}
