// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#include "oslmic.h"
#include "aes.h"
#include "lce.h"
#include "lmic.h"


bool lce_processJoinAccept (uint8_t* jacc, uint8_t jacclen, uint16_t devnonce) {
    if( (jacc[0] & HDR_FTYPE) != HDR_FTYPE_JACC || (jacclen != LEN_JA && jacclen != LEN_JAEXT) ) {
        return 0;
    }
    os_getNwkKey(AESkey);
    os_aes(AES_ENC, jacc+1, jacclen-1);

    jacclen -= 4;
    uint32_t mic1 = os_rmsbf4(jacc+jacclen);
#if defined(CFG_lorawan11)
    uint8_t optneg = jacc[OFF_JA_DLSET] & JA_DLS_OPTNEG;
    if( optneg ) {
        os_moveMem(jacc+OFF_JA_JOINNONCE+2, jacc+OFF_JA_JOINNONCE, jacclen-OFF_JA_JOINNONCE);
        os_wlsbf2(jacc+OFF_JA_JOINNONCE, devnonce);
        jacclen += 2;
    }
#endif
    os_getNwkKey(AESkey);
    uint32_t mic2 = os_aes(AES_MIC|AES_MICNOAUX, jacc, jacclen);
#if defined(CFG_lorawan11)
    if( optneg ) {  // Restore orig frame
        jacclen -= 2;
        os_moveMem(jacc+OFF_JA_JOINNONCE, jacc+OFF_JA_JOINNONCE+2, jacclen-OFF_JA_JOINNONCE);
        os_wlsbf4(jacc+jacclen, mic1);
    }
#endif
    if( mic1 != mic2 ) {
        return 0;
    }
    uint8_t* nwkskey = LMIC.lceCtx.nwkSKey;
    os_clearMem(nwkskey, 16);
    nwkskey[0] = 0x01;
    os_copyMem(nwkskey+1, &jacc[OFF_JA_JOINNONCE], LEN_JOINNONCE+LEN_NETID);
    os_wlsbf2(nwkskey+1+LEN_JOINNONCE+LEN_NETID, devnonce);
    os_copyMem(LMIC.lceCtx.appSKey, nwkskey, 16);
    LMIC.lceCtx.appSKey[0] = 0x02;
#if defined(CFG_lorawan11)
    os_copyMem(LMIC.lceCtx.nwkSKeyDn, nwkskey, 16);
    LMIC.lceCtx.nwkSKeyDn[0] = 0x03;
#endif

    os_getNwkKey(AESkey);
    os_aes(AES_ENC, nwkskey, 16);
#if defined(CFG_lorawan11)
    if( optneg ) {
        os_getNwkKey(AESkey);
        os_aes(AES_ENC, LMIC.lceCtx.nwkSKeyDn, 16);
        os_getAppKey(AESkey);
    } else {
        os_copyMem(LMIC.lceCtx.nwkSKeyDn, nwkskey, 16);
        os_getNwkKey(AESkey);
    }
#else
    os_getNwkKey(AESkey);
#endif
    os_aes(AES_ENC, LMIC.lceCtx.appSKey, 16);
    return 1;
}


void lce_addMicJoinReq (uint8_t* pdu, int len) {
    os_getNwkKey(AESkey);
    os_wmsbf4(pdu+len, os_aes(AES_MIC|AES_MICNOAUX, pdu, len));  // MSB because of internal structure of AES
}

void lce_encKey0 (uint8_t* buf) {
    os_clearMem(AESkey,16);
    os_aes(AES_ENC,buf,16);
}

static void micB0 (uint32_t devaddr, uint32_t seqno, uint8_t cat, int len) {
    os_clearMem(AESaux,16);
    AESaux[0]  = 0x49;
    AESaux[5]  = cat;
    AESaux[15] = len;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
}

bool lce_verifyMic (int8_t keyid, uint32_t devaddr, uint32_t seqno, uint8_t* pdu, int len) {
    micB0(devaddr, seqno, 1, len);
    const uint8_t* key;
    if( keyid == LCE_NWKSKEY ) {
#if defined(CFG_lorawan11)
        key = LMIC.lceCtx.nwkSKeyDn;
#else
        key = LMIC.lceCtx.nwkSKey;
#endif
    }
    else if( keyid >= LCE_MCGRP_0 && keyid < LCE_MCGRP_0+LCE_MCGRP_MAX ) {
        key = LMIC.lceCtx.mcgroup[keyid - LCE_MCGRP_0].nwkSKeyDn;
    }
    else {
        // Illegal key index
        return 0;
    }
    os_copyMem(AESkey,key,16);
    return os_aes(AES_MIC, pdu, len) == os_rmsbf4(pdu+len);
}

void lce_addMic (int8_t keyid, uint32_t devaddr, uint32_t seqno, uint8_t* pdu, int len) {
    if( keyid != LCE_NWKSKEY ) {
        return; // Illegal key index
    }
    micB0(devaddr, seqno, 0, len);
    const uint8_t* key = LMIC.lceCtx.nwkSKey;
    os_copyMem(AESkey,key,16);
    // MSB because of internal structure of AES
    os_wmsbf4(pdu+len, os_aes(AES_MIC, pdu, len));
}

uint32_t lce_micKey0 (uint32_t devaddr, uint32_t seqno, uint8_t* pdu, int len) {
    micB0(devaddr, seqno, 0, len);
    os_clearMem(AESkey,16);
    // MSB because of internal structure of AES
    uint8_t mic[4];
    os_wmsbf4(mic, os_aes(AES_MIC, pdu, len));
    return os_rlsbf4(mic);
}

void lce_cipher (int8_t keyid, uint32_t devaddr, uint32_t seqno, int cat, uint8_t* payload, int len) {
    if(len <= 0 || (cat==LCE_SCC_UP && (LMIC.opmode & OP_NOCRYPT)) ) {
        return;
    }
    const uint8_t* key;
    if( keyid == LCE_NWKSKEY ) {
#if defined(CFG_lorawan11)
        key = cat==LCE_SCC_DN ? LMIC.lceCtx.nwkSKeyDn : LMIC.lceCtx.nwkSKey;
#else
        key = LMIC.lceCtx.nwkSKey;
#endif
    }
    else if( keyid == LCE_APPSKEY ) {
        key = LMIC.lceCtx.appSKey;
    }
    else if( keyid >= LCE_MCGRP_0 && keyid < LCE_MCGRP_0+LCE_MCGRP_MAX ) {
        key = LMIC.lceCtx.mcgroup[keyid - LCE_MCGRP_0].appSKey;
        cat = LCE_SCC_DN;
    }
    else {
        // Illegal key index
        os_clearMem(payload,len);
        return;
    }
    micB0(devaddr, seqno, cat, 1);
    AESaux[0]  = 0x01;
    os_copyMem(AESkey,key,16);
    os_aes(AES_CTR, payload, len);
}


#if defined(CFG_lorawan11)
void lce_loadSessionKeys (const uint8_t* nwkSKey, const uint8_t* nwkSKeyDn, const uint8_t* appSKey)
#else
void lce_loadSessionKeys (const uint8_t* nwkSKey, const uint8_t* appSKey)
#endif
{
    if( nwkSKey != (uint8_t*)0 )
        os_copyMem(LMIC.lceCtx.nwkSKey, nwkSKey, 16);
#if defined(CFG_lorawan11)
    if( nwkSKeyDn != (uint8_t*)0 )
        os_copyMem(LMIC.lceCtx.nwkSKeyDn, nwkSKeyDn, 16);
#endif
    if( appSKey != (uint8_t*)0 )
        os_copyMem(LMIC.lceCtx.appSKey, appSKey, 16);
}


void lce_init (void) {
    os_clearMem(&LMIC.lceCtx, sizeof(LMIC.lceCtx));
}
