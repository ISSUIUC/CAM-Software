#pragma once

#include <stdint.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "fsk_framer.h"

#define FSK_RX_FRAME_TIMEOUT_MS 150u

// Maximum JPEG frame size the reassembler will accept.
#define FSK_MAX_RX_FRAME_SIZE 2 * 720 * 240

#define FSK_RX_BYTE_FIELD_LENGTH 256u

static inline bool fsk_parse_header(
    const uint8_t *pkt,
    uint8_t *out_frame_id,
    uint16_t *out_frag_index,
    uint32_t *out_total_size)
{
    if (!pkt)
        return false;
    *out_frame_id = pkt[0];
    *out_frag_index = ((uint16_t)pkt[1] << 8) | pkt[2];
    *out_total_size = ((uint32_t)pkt[3] << 24) | ((uint32_t)pkt[4] << 16) | ((uint32_t)pkt[5] << 8) | (uint32_t)pkt[6];
    return true;
}

static inline bool fsk_reassemble_fragment(
    const uint8_t *pkt,
    uint16_t frag_index,
    uint32_t total_size,
    uint8_t *dst_buf,
    uint32_t dst_buf_len)
{
    if (!pkt || !dst_buf)
        return false;

    uint32_t offset = (uint32_t)frag_index * FSK_FRAG_DATA_SIZE;
    uint32_t remaining = (offset < total_size) ? (total_size - offset) : 0u;
    uint16_t copy_len = (remaining > FSK_FRAG_DATA_SIZE)
                            ? (uint16_t)FSK_FRAG_DATA_SIZE
                            : (uint16_t)remaining;

    if (copy_len == 0 || offset + copy_len > dst_buf_len)
        return false;

    memcpy(dst_buf + offset, pkt + FSK_FRAG_HDR_SIZE, copy_len);
    return true;
}

struct FskReassemblyState
{
    uint8_t frame_id;         // frame_id currently being assembled
    uint8_t last_complete_id; // frame_id of the last frame handed off (0xFF = none)
    bool active;              // true while assembling a frame

    // Size / progress
    uint32_t total_size;     // original payload length (from first fragment header)
    uint16_t total_frags;    // derived: fsk_frame_count(total_size)
    uint16_t frags_received; // unique fragments stored so far
    uint32_t completed_size; // total_size saved before reset — read this at frame_ready

    // Timing
    uint32_t last_frag_ms; // millis() of last successfully stored fragment

    // Received-fragment bitfield (one bit per fragment index)
    uint8_t recv_bits[FSK_RX_BYTE_FIELD_LENGTH];

    // Output buffer (allocated once in SPIRAM by fsk_reassembler_init)
    uint8_t *data_buf;
    uint32_t data_buf_len;

    // Debug counters
    uint32_t stat_crc_drop;     // packets dropped due to LR2021_ERR_CRC_MISMATCH
    uint32_t stat_frame_resets; // times a new frame_id interrupted an in-progress frame
    uint32_t stat_frags_total;  // total fragments successfully written across all frames

    // RS parity state: allocated once in init, persists across frames
    uint8_t *parity_buf; // SPIRAM: max_parity_frags * FSK_FRAG_DATA_SIZE
    uint32_t parity_buf_len;
    uint16_t parity_frags_received;

    // Layout is deterministic from total_size once known
    RSLayout rs_layout;
    bool rs_layout_valid;
};

enum FskIngestResult : uint8_t
{
    FSK_INGEST_OK = 0,       // fragment stored, frame still in progress
    FSK_INGEST_COMPLETE = 1, // all fragments received — frame ready in data_buf
    FSK_INGEST_TIMEOUT = 2,  // partial frame flushed after stale timeout
    FSK_INGEST_DUP = 3,      // duplicate fragment index, ignored
    FSK_INGEST_OVERFLOW = 4, // fragment write would overflow data_buf
    FSK_INGEST_BAD_HDR = 5,  // header parse failed or sanity check failed
};

static inline bool fsk_reassembler_init(FskReassemblyState *s, uint32_t buf_size)
{
    memset(s, 0, sizeof(*s));
    s->last_complete_id = 0xFF;
    s->data_buf = (uint8_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
    if (!s->data_buf)
        return false;
    s->data_buf_len = buf_size;

    // Max parity fragments: ceil(buf_size / FSK_FRAG_DATA_SIZE / RS_DATA_SHARDS) * RS_PARITY_SHARDS
    uint32_t max_data_frags = (buf_size + FSK_FRAG_DATA_SIZE - 1) / FSK_FRAG_DATA_SIZE;
    uint32_t max_blocks = (max_data_frags + RS_DATA_SHARDS - 1) / RS_DATA_SHARDS;
    uint32_t max_parity_frags = max_blocks * RS_PARITY_SHARDS;
    s->parity_buf_len = max_parity_frags * FSK_FRAG_DATA_SIZE;
    s->parity_buf = (uint8_t *)heap_caps_malloc(s->parity_buf_len, MALLOC_CAP_SPIRAM);
    if (!s->parity_buf)
    {
        heap_caps_free(s->data_buf);
        return false;
    }

    return true;
}

// Resets per-frame fields without touching the buffer or debug counters.
static inline void fsk_reassembler_reset(FskReassemblyState *s)
{
    s->active = false;
    s->frame_id = 0;
    s->total_size = 0;
    s->total_frags = 0;
    s->frags_received = 0;
    s->last_frag_ms = 0;
    s->parity_frags_received = 0;
    s->rs_layout_valid = false;
    memset(s->recv_bits, 0, FSK_RX_BYTE_FIELD_LENGTH);
}

// Returns true if fragment index was already received.
static inline bool fsk_fragment_seen(const FskReassemblyState *s, uint16_t idx)
{
    uint16_t byte_idx = idx / 8;
    uint8_t bit_mask = 1u << (idx % 8);
    if (byte_idx >= FSK_RX_BYTE_FIELD_LENGTH)
        return false;
    return (s->recv_bits[byte_idx] & bit_mask) != 0;
}

// Marks fragment index as received.
static inline void fsk_fragment_mark(FskReassemblyState *s, uint16_t idx)
{
    uint16_t byte_idx = idx / 8;
    uint8_t bit_mask = 1u << (idx % 8);
    if (byte_idx < FSK_RX_BYTE_FIELD_LENGTH)
        s->recv_bits[byte_idx] |= bit_mask;
}

static inline FskIngestResult fsk_reassembler_ingest(
    FskReassemblyState *s,
    const uint8_t *pkt,
    uint32_t now_ms)
{
    uint8_t incoming_id;
    uint16_t frag_index;
    uint32_t total_size;
    if (!fsk_parse_header(pkt, &incoming_id, &frag_index, &total_size))
        return FSK_INGEST_BAD_HDR;
    if (total_size == 0 || total_size > s->data_buf_len)
        return FSK_INGEST_BAD_HDR;

    uint16_t dataFrags = fsk_frame_count(total_size);
    bool is_parity = (frag_index >= dataFrags);

    // New frame ID (hermmm) start fresh (data frags only trigger this)
    if (!is_parity && (!s->active || incoming_id != s->frame_id))
    {
        if (s->active)
            s->stat_frame_resets++;
        fsk_reassembler_reset(s);
        s->active = true;
        s->frame_id = incoming_id;
        s->total_size = total_size;
        s->total_frags = dataFrags;
        s->rs_layout.compute(total_size, FSK_FRAG_DATA_SIZE);
        s->rs_layout_valid = true;
    }

    if (!s->active)
        return FSK_INGEST_BAD_HDR; // parity arrived before any data frag

    if (is_parity)
    {
        uint16_t parity_idx = frag_index - dataFrags;
        uint32_t parity_off = (uint32_t)parity_idx * FSK_FRAG_DATA_SIZE;
        if (!s->parity_buf || parity_off + FSK_FRAG_DATA_SIZE > s->parity_buf_len)
            return FSK_INGEST_OVERFLOW;
        if (fsk_fragment_seen(s, frag_index))
            return FSK_INGEST_DUP;
        memcpy(s->parity_buf + parity_off, pkt + FSK_FRAG_HDR_SIZE, FSK_FRAG_DATA_SIZE);
        fsk_fragment_mark(s, frag_index); // unified recv_bits covers data+parity indices
        s->parity_frags_received++;
        s->last_frag_ms = now_ms;
        return FSK_INGEST_OK;
    }

    // this is what happens to a data fragment
    if (frag_index >= s->total_frags)
        return FSK_INGEST_BAD_HDR;
    if (fsk_fragment_seen(s, frag_index))
        return FSK_INGEST_DUP;
    if (!fsk_reassemble_fragment(pkt, frag_index, total_size, s->data_buf, s->data_buf_len))
        return FSK_INGEST_OVERFLOW;

    fsk_fragment_mark(s, frag_index);
    s->frags_received++;
    s->last_frag_ms = now_ms;
    s->stat_frags_total++;

    if (s->frags_received >= s->total_frags)
    {
        s->last_complete_id = s->frame_id;
        s->completed_size = s->total_size;
        fsk_reassembler_reset(s);
        return FSK_INGEST_COMPLETE;
    }
    return FSK_INGEST_OK;
}
static inline FskIngestResult fsk_reassembler_check_timeout(
    FskReassemblyState *s,
    uint32_t now_ms)
{
    if (!s->active || s->frags_received == 0)
        return FSK_INGEST_OK;

    if (now_ms - s->last_frag_ms >= FSK_RX_FRAME_TIMEOUT_MS)
    {
        s->last_complete_id = s->frame_id;
        s->completed_size = s->total_size;

        if (s->frags_received < s->total_frags &&
            s->rs_layout_valid &&
            s->parity_frags_received > 0)
        {
            rsDecode(s->data_buf, s->total_size,
                     s->parity_buf,
                     s->recv_bits, FSK_RX_BYTE_FIELD_LENGTH,
                     s->rs_layout, FSK_FRAG_DATA_SIZE);
        }

        fsk_reassembler_reset(s);
        return FSK_INGEST_TIMEOUT;
    }
    return FSK_INGEST_OK;
}