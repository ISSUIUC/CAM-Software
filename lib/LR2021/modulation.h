#pragma once

#include "radio.h"

#if RADIO_MODULATION == MODULATION_FLRC
#include "flrc.h"
typedef LR2021FLRCDriver LR2021Driver;
typedef LR2021FlrcPktStatus LR2021PktStatus;
#define RADIO_MODULATION_NAME "FLRC"
#define RADIO_PACKET_SIZE FLRC_MAX_PACKET_SIZE
#elif RADIO_MODULATION == MODULATION_FSK
#include "fsk.h"
typedef LR2021FSKDriver LR2021Driver;
typedef LR2021FskPktStatus LR2021PktStatus;
#define RADIO_MODULATION_NAME "FSK"
#define RADIO_PACKET_SIZE PAYLOAD_SIZE_FSK
#else
#error "RADIO_MODULATION in radio.h has to be MODULATION_FSK or MODULATION_FLRC"
#endif
