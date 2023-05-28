// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#ifndef _lce_h_
#define _lce_h_

#include "oslmic.h"

#ifdef __cplusplus
extern "C"{
#endif

// Some keyids:
#define LCE_APPSKEY   (-2)
#define LCE_NWKSKEY   (-1)
#define LCE_MCGRP_0   ( 0)
#define LCE_MCGRP_MAX ( 2)

// Stream cipher categories (lce_cipher(..,cat,..):
// Distinct use of the AppSKey must use different key classes
// or plain text will leak:
enum {
    LCE_SCC_UP   = 0,     // std LoRaWAN uplink frame
    LCE_SCC_DN   = 1,     // std LoRaWAN downlink frame
    LCE_SCC_FUP  = 0x40,  // file upload
    LCE_SCC_DSE  = 0x41,  // data streaming engine
    LCE_SCC_ROSE = 0x42,  // reliable octet streaming engine
};

void lce_encKey0 (uint8_t* buf);
uint32_t lce_micKey0 (uint32_t devaddr, uint32_t seqno, uint8_t* pdu, int len);
bool lce_processJoinAccept (uint8_t* jacc, uint8_t jacclen, uint16_t devnonce);
void lce_addMicJoinReq (uint8_t* pdu, int len);
bool lce_verifyMic (int8_t keyid, uint32_t devaddr, uint32_t seqno, uint8_t* pdu, int len);
void lce_addMic (int8_t keyid, uint32_t devaddr, uint32_t seqno, uint8_t* pdu, int len);
void lce_cipher (int8_t keyid, uint32_t devaddr, uint32_t seqno, int cat, uint8_t* payload, int len);
#if defined(CFG_lorawan11)
void lce_loadSessionKeys (const uint8_t* nwkSKey, const uint8_t* nwkSKeyDn, const uint8_t* appSKey);
#else
void lce_loadSessionKeys (const uint8_t* nwkSKey, const uint8_t* appSKey);
#endif
void lce_init (void);


typedef struct lce_ctx_mcgrp {
    uint8_t nwkSKeyDn[16]; // network session key for down-link
    uint8_t appSKey[16];   // application session key
} lce_ctx_mcgrp_t;

typedef struct lce_ctx {
    uint8_t nwkSKey[16];   // network session key (LoRaWAN1.1: up-link only)
#if defined(CFG_lorawan11)
    uint8_t nwkSKeyDn[16]; // network session key for down-link
#endif
    uint8_t appSKey[16];   // application session key
    lce_ctx_mcgrp_t mcgroup[LCE_MCGRP_MAX];
} lce_ctx_t;


#ifdef __cplusplus
} // extern "C"
#endif

#endif // _lce_h_
