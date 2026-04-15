/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file gptp_msg.h
 * @brief gPTP (IEEE 802.1AS) message format definitions
 *
 * Extends IEEE 1588-2008 PTP message format with gPTP (IEEE 802.1AS) specifics:
 *  - transportSpecific = 0x1 (SdoId = 1, bit7 of messagetype byte set to 1)
 *  - P2P delay messages: PDelayReq (0x2), PDelayResp (0x3), PDelayRespFollowUp (0xA)
 *
 * All multi-byte fields are big-endian (network byte order).
 */

#pragma once

#include <stdint.h>

/* =========================================================================
 * EtherType and multicast MAC constants
 * ========================================================================= */

/** EtherType for all PTP / gPTP frames (same as standard PTP) */
#define GPTP_ETHERTYPE              0x88F7U

/** gPTP multicast destination MAC: IEEE 802.1 bridge-reserved address.
 *  Does NOT cross bridges, ensuring P2P peer-to-peer semantics. */
#define GPTP_MCAST_MAC              {0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E}

/* =========================================================================
 * Message type constants
 * =========================================================================
 * The high nibble of messagetype byte carries transportSpecific:
 *   gPTP: transportSpecific = 0x1  →  byte = 0x10 | msgtype
 * ========================================================================= */

/** Mask to extract 4-bit message type */
#define GPTP_MSGTYPE_MASK           0x0FU

/** Mask to extract transportSpecific (high nibble) */
#define GPTP_TRANSPORT_SPECIFIC_MASK 0xF0U

/** gPTP transportSpecific value (SdoId = 1) encoded in high nibble */
#define GPTP_TRANSPORT_SPECIFIC     0x10U

/* Event messages (time-sensitive, hardware timestamp needed) */
#define GPTP_MSGTYPE_SYNC           0x0U   /**< 0x10 on wire */
#define GPTP_MSGTYPE_PDELAY_REQ     0x2U   /**< 0x12 on wire */
#define GPTP_MSGTYPE_PDELAY_RESP    0x3U   /**< 0x13 on wire */

/* General messages (no hardware timestamp needed) */
#define GPTP_MSGTYPE_FOLLOW_UP      0x8U   /**< 0x18 on wire */
#define GPTP_MSGTYPE_PDELAY_RESP_FUP 0xAU  /**< 0x1A on wire */
#define GPTP_MSGTYPE_ANNOUNCE       0xBU   /**< 0x1B on wire */
#define GPTP_MSGTYPE_SIGNALING      0xCU   /**< 0x1C on wire (optional) */

/** Helper: build the first byte of a gPTP message header */
#define GPTP_MSG_BYTE0(msgtype)     (GPTP_TRANSPORT_SPECIFIC | ((msgtype) & GPTP_MSGTYPE_MASK))

/** Helper: check if a received byte0 is a gPTP event message */
#define GPTP_IS_EVENT_MSG(byte0)    (((byte0) & GPTP_MSGTYPE_MASK) < 8)

/* =========================================================================
 * Header flags
 * ========================================================================= */

/** flags[0] bit1: TWO_STEP – sender will follow up with precise timestamp */
#define GPTP_FLAGS0_TWOSTEP         (1U << 1)

/** flags[1] bit3: PTP_TIMESCALE – clock uses PTP timescale (TAI) */
#define GPTP_FLAGS1_PTP_TIMESCALE   (1U << 3)

/* =========================================================================
 * Control field values (IEEE 1588 legacy, ignored in PTPv2 but must be set)
 * ========================================================================= */
#define GPTP_CTRL_SYNC              0x00U
#define GPTP_CTRL_DELAY_REQ         0x01U
#define GPTP_CTRL_FOLLOW_UP         0x02U
#define GPTP_CTRL_DELAY_RESP        0x03U
#define GPTP_CTRL_MANAGEMENT        0x04U
#define GPTP_CTRL_OTHER             0x05U

/* =========================================================================
 * Ethernet frame header length
 * ========================================================================= */
#define GPTP_ETH_HEADER_LEN         14U     /**< 6 dst + 6 src + 2 ethertype */

/* =========================================================================
 * Message structures (packed, big-endian)
 * All sizes match IEEE 1588-2008 / IEEE 802.1AS wire format.
 * ========================================================================= */

/**
 * @brief Common header for all gPTP message types (34 bytes)
 *
 * The first byte on the wire encodes both transportSpecific (high nibble = 0x1)
 * and messageType (low nibble). Use GPTP_MSG_BYTE0(type) to construct it.
 */
struct gptp_header_s {
    uint8_t  messagetype;         /**< [7:4]=transportSpecific(0x1) [3:0]=messageType */
    uint8_t  version;             /**< PTP version = 2 */
    uint8_t  messagelength[2];    /**< Total message length in bytes (big-endian) */
    uint8_t  domain;              /**< PTP domain number */
    uint8_t  reserved1;
    uint8_t  flags[2];            /**< flags[0]: TWO_STEP etc. flags[1]: timescale */
    uint8_t  correction[8];       /**< Correction field (scaled ns = ns × 2^16, big-endian) */
    uint8_t  reserved2[4];
    uint8_t  sourceidentity[8];   /**< Clock identity (EUI-64) */
    uint8_t  sourceportindex[2];  /**< Source port number (big-endian) */
    uint8_t  sequenceid[2];       /**< Sequence ID (big-endian) */
    uint8_t  controlfield;        /**< Legacy control field */
    uint8_t  logmessageinterval;  /**< log2 of message interval in seconds */
} __attribute__((packed));        /* 34 bytes */

/**
 * @brief Sync message (44 bytes total)
 *
 * In TWO_STEP mode: origintimestamp = 0; precise T1 is in FollowUp.
 * In ONE_STEP mode: origintimestamp = T1 (filled by hardware).
 */
struct gptp_sync_s {
    struct gptp_header_s header;
    uint8_t origintimestamp[10];  /**< 48-bit seconds + 32-bit nanoseconds (all big-endian) */
} __attribute__((packed));        /* 44 bytes */

/**
 * @brief Follow_Up message (44 bytes total)
 *
 * Carries the precise T1 TX timestamp of the preceding Sync.
 */
struct gptp_follow_up_s {
    struct gptp_header_s header;
    uint8_t origintimestamp[10];  /**< Precise T1: actual Sync TX hardware timestamp */
} __attribute__((packed));        /* 44 bytes */

/**
 * @brief Announce message (64 bytes total)
 *
 * Used by BMCA to elect the grandmaster clock.
 */
struct gptp_announce_s {
    struct gptp_header_s header;
    uint8_t origintimestamp[10];  /**< Current time estimate (usually zero) */
    uint8_t utcoffset[2];         /**< Current UTC offset (TAI-UTC seconds) */
    uint8_t reserved;
    uint8_t gm_priority1;         /**< Grandmaster priority1 (lower = better) */
    uint8_t gm_quality[4];        /**< [0]=clockClass [1]=clockAccuracy [2:3]=offsetScaledLogVariance */
    uint8_t gm_priority2;         /**< Grandmaster priority2 */
    uint8_t gm_identity[8];       /**< Grandmaster clock identity (EUI-64) */
    uint8_t stepsremoved[2];      /**< Topology distance from grandmaster */
    uint8_t timesource;           /**< Time source type code */
} __attribute__((packed));        /* 64 bytes */

/**
 * @brief PDelayReq message (54 bytes total)
 *
 * Sent by PDelay Requester every 250ms. origintimestamp is zero in TWO_STEP mode.
 */
struct gptp_pdelay_req_s {
    struct gptp_header_s header;
    uint8_t origintimestamp[10];  /**< T1 hint (zero in TWO_STEP, HW-filled in ONE_STEP) */
    uint8_t reserved[10];
} __attribute__((packed));        /* 54 bytes */

/**
 * @brief PDelayResp message (54 bytes total)
 *
 * Sent by PDelay Responder in reply to PDelayReq.
 * requestreceipttimestamp = T2 (RX timestamp of the PDelayReq on responder).
 * In TWO_STEP: correctionField = 0; precise T3 is in PDelayRespFollowUp.
 * In ONE_STEP: correctionField contains (T3 - T2) residence time.
 */
struct gptp_pdelay_resp_s {
    struct gptp_header_s header;
    uint8_t requestreceipttimestamp[10]; /**< T2: hardware RX timestamp of PDelayReq */
    uint8_t requestingportidentity[10];  /**< sourcePortIdentity from PDelayReq */
} __attribute__((packed));              /* 54 bytes */

/**
 * @brief PDelayRespFollowUp message (54 bytes total)
 *
 * Sent by PDelay Responder after PDelayResp in TWO_STEP mode.
 * responseorigintimestamp = T3 (TX timestamp of the PDelayResp on responder).
 */
struct gptp_pdelay_resp_fup_s {
    struct gptp_header_s header;
    uint8_t responseorigintimestamp[10]; /**< T3: hardware TX timestamp of PDelayResp */
    uint8_t requestingportidentity[10];  /**< sourcePortIdentity from the original PDelayReq */
} __attribute__((packed));              /* 54 bytes */

/* =========================================================================
 * Timestamp conversion helpers (inline)
 * ========================================================================= */

/**
 * @brief Convert struct timespec to PTP on-wire timestamp (10 bytes, big-endian)
 *
 * Wire format: 48-bit seconds (bytes 0-5) + 32-bit nanoseconds (bytes 6-9).
 */
static inline void gptp_ts_to_wire(const struct timespec *ts, uint8_t *wire)
{
    /* Seconds: 6 bytes big-endian (assume tv_sec fits 32-bit for embedded) */
    wire[0] = 0;
    wire[1] = 0;
    wire[2] = (uint8_t)((uint32_t)ts->tv_sec >> 24);
    wire[3] = (uint8_t)((uint32_t)ts->tv_sec >> 16);
    wire[4] = (uint8_t)((uint32_t)ts->tv_sec >>  8);
    wire[5] = (uint8_t)((uint32_t)ts->tv_sec >>  0);
    /* Nanoseconds: 4 bytes big-endian */
    wire[6] = (uint8_t)((uint32_t)ts->tv_nsec >> 24);
    wire[7] = (uint8_t)((uint32_t)ts->tv_nsec >> 16);
    wire[8] = (uint8_t)((uint32_t)ts->tv_nsec >>  8);
    wire[9] = (uint8_t)((uint32_t)ts->tv_nsec >>  0);
}

/**
 * @brief Convert PTP on-wire timestamp (10 bytes, big-endian) to struct timespec
 */
static inline void gptp_wire_to_ts(const uint8_t *wire, struct timespec *ts)
{
    ts->tv_sec  = ((time_t)wire[2] << 24) | ((time_t)wire[3] << 16)
                | ((time_t)wire[4] <<  8) | ((time_t)wire[5]);
    ts->tv_nsec = ((long)wire[6] << 24) | ((long)wire[7] << 16)
                | ((long)wire[8] <<  8) | ((long)wire[9]);
}

/**
 * @brief Convert PTP on-wire correctionField (8 bytes big-endian, scaled ns) to nanoseconds
 *
 * correctionField is stored as (nanoseconds × 2^16) in a signed 64-bit integer.
 */
static inline int64_t gptp_correction_to_ns(const uint8_t *corr)
{
    int64_t val = 0;
    for (int i = 0; i < 8; i++) {
        val = (val << 8) | corr[i];
    }
    return val >> 16;   /* divide by 2^16 to get nanoseconds */
}

/**
 * @brief Encode nanoseconds into PTP correctionField (8 bytes big-endian, scaled ns)
 */
static inline void gptp_ns_to_correction(int64_t ns, uint8_t *corr)
{
    int64_t val = ns << 16;  /* multiply by 2^16 */
    for (int i = 7; i >= 0; i--) {
        corr[i] = (uint8_t)(val & 0xFF);
        val >>= 8;
    }
}

/**
 * @brief Compute (a - b) in nanoseconds from two timespec values
 */
static inline int64_t gptp_timespec_diff_ns(const struct timespec *a,
                                             const struct timespec *b)
{
    return ((int64_t)a->tv_sec - (int64_t)b->tv_sec) * 1000000000LL
         + ((int64_t)a->tv_nsec - (int64_t)b->tv_nsec);
}

/**
 * @brief Compute elapsed milliseconds from a past monotonic timestamp to now
 */
static inline int64_t gptp_elapsed_ms(const struct timespec *past)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return gptp_timespec_diff_ns(&now, past) / 1000000LL;
}

/**
 * @brief Get uint16 from two bytes big-endian
 */
static inline uint16_t gptp_get_u16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

/**
 * @brief Put uint16 into two bytes big-endian
 */
static inline void gptp_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}
