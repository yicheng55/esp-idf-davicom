/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file gptp.c
 * @brief gPTP (IEEE 802.1AS) daemon implementation for ESP-IDF
 *
 * Architecture:
 *  - Single FreeRTOS task runs the gPTP state machine
 *  - L2TAP VFS (/dev/net/tap) provides raw Ethernet frame access
 *  - Hardware timestamps from DM9058 PTP engine via L2TAP IREC
 *  - PI controller for frequency-domain clock synchronization
 *
 * gPTP vs standard PTP differences implemented here:
 *  - Multicast MAC: 01:80:C2:00:00:0E (bridge-reserved, does not cross bridges)
 *  - transportSpecific = 0x1 in all message headers (SdoId = 1)
 *  - P2P delay mechanism: PDelayReq / PDelayResp / PDelayRespFollowUp
 *  - Sync interval: 125 ms (log = -3)
 *  - PDelay interval: 250 ms (log = -2)
 *  - Any node responds to PDelayReq regardless of MASTER/SLAVE role
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_eth_driver.h"
#include "esp_eth_mac_spi.h"
#include "esp_vfs_l2tap.h"
#include "lwip/prot/ethernet.h"

#include "gptp.h"
#include "gptp_msg.h"
#include "esp_eth_time.h"

/* =========================================================================
 * Build-time guard
 * ========================================================================= */
#ifndef CONFIG_GPTP_ENABLE
/* Component is compiled but daemon is disabled; provide stub gptp_start only */
esp_err_t gptp_start(const char *interface, gptp_handle_t *handle)
{
    (void)interface;
    if (handle) *handle = NULL;
    return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t gptp_stop(gptp_handle_t handle) { (void)handle; return ESP_OK; }
esp_err_t gptp_get_status(gptp_handle_t handle, gptp_status_t *s)
{ (void)handle; (void)s; return ESP_ERR_NOT_SUPPORTED; }
#else

/* =========================================================================
 * Logging
 * ========================================================================= */
static const char *TAG = "gptp";

#if CONFIG_GPTP_LOG_ENABLE
#  define GTPD(fmt, ...)  ESP_LOGD(TAG, fmt, ##__VA_ARGS__)
#  define GTPI(fmt, ...)  ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#  define GTPW(fmt, ...)  ESP_LOGW(TAG, fmt, ##__VA_ARGS__)
#  define GTPE(fmt, ...)  ESP_LOGE(TAG, fmt, ##__VA_ARGS__)
#else
#  define GTPD(fmt, ...)  ((void)0)
#  define GTPI(fmt, ...)  ((void)0)
#  define GTPW(fmt, ...)  ((void)0)
#  define GTPE(fmt, ...)  ((void)0)
#endif

/* =========================================================================
 * Constants
 * ========================================================================= */

#define GPTP_SYNC_INTERVAL_MS       125     /**< 2^-3 seconds */
#define GPTP_PDELAY_INTERVAL_MS     250     /**< 2^-2 seconds */
#define GPTP_ANNOUNCE_INTERVAL_MS   1000    /**< 2^0 seconds */
#define GPTP_ANNOUNCE_TIMEOUT_MS    3000    /**< 3 × 1 s */

/** Maximum step-correction threshold (ns). Large offsets get settime(), small get adjtime(). */
#define GPTP_SETTIME_THRESHOLD_NS   ((int64_t)CONFIG_GPTP_SETTIME_THRESHOLD_MS * 1000000LL)

/** Maximum frequency adjustment range: ±512 ppm */
#define GPTP_ADJ_FREQ_MAX           512000

/** Maximum raw frame buffer (Ethernet header + largest PTP message) */
#define GPTP_FRAME_MAX              (GPTP_ETH_HEADER_LEN + 128)

/* =========================================================================
 * PI controller state (same approach as ptpd.c)
 * ========================================================================= */
typedef struct {
    int32_t kp;           /**< Proportional gain */
    int32_t ki;           /**< Integral gain denominator */
    int32_t drift_acc;    /**< Integrator accumulator (ppb) */
} gptp_pi_t;

/* =========================================================================
 * Main daemon state
 * ========================================================================= */
struct gptp_state_s {
    /* ---- control ---- */
    volatile bool       stop;
    TaskHandle_t        task_handle;

    /* ---- L2TAP socket ---- */
    int                 ptp_socket;
    esp_eth_handle_t    eth_handle;
    uint8_t             intf_hw_addr[ETH_ADDR_LEN];

    /* ---- own identity (filled from MAC) ---- */
    struct gptp_announce_s own_identity;   /**< Own clock parameters for BMCA */

    /* ---- BMCA state ---- */
    bool                is_master;             /**< Current role */
    struct gptp_announce_s best_master;        /**< Best master seen */
    bool                best_master_valid;
    struct timespec     last_announce_rx;      /**< CLOCK_MONOTONIC, for timeout */

    /* ---- sequence numbers ---- */
    uint16_t            pdelay_seq;
    uint16_t            sync_seq;
    uint16_t            announce_seq;
    uint16_t            recv_pdelay_req_seq;   /**< Last PDelayReq seq we responded to */

    /* ---- PDelay Requester (we send PDelayReq, measure T1 and T4) ---- */
    struct timespec     pdelay_t1;             /**< TX timestamp of last PDelayReq */
    struct timespec     pdelay_t4;             /**< RX timestamp of PDelayResp */
    uint16_t            pdelay_outstanding_seq;/**< Sequence ID we're waiting response for */
    bool                waiting_pdelay_resp;
    bool                waiting_pdelay_resp_fup;
    struct timespec     pdelay_t2_from_resp;   /**< T2 reported in PDelayResp body */
    struct timespec     last_pdelay_req_tx;    /**< CLOCK_MONOTONIC, for interval */

    /* ---- PDelay Responder (we receive PDelayReq and send PDelayResp) ---- */
    struct timespec     resp_t2;               /**< RX timestamp of incoming PDelayReq */
    uint8_t             req_port_identity[10]; /**< sourcePortIdentity from PDelayReq */

    /* ---- Sync Slave (we receive Sync and FollowUp) ---- */
    struct timespec     sync_t2;               /**< RX timestamp of Sync */
    uint16_t            sync_seq_expected;
    bool                waiting_followup;
    struct gptp_sync_s  twostep_sync;          /**< Buffered Sync for TWO_STEP */
    struct timespec     twostep_t2;            /**< RX timestamp for buffered Sync */

    /* ---- Sync Master (we send Sync and FollowUp) ---- */
    struct timespec     last_sync_tx;          /**< CLOCK_MONOTONIC, for interval */
    struct timespec     last_announce_tx;      /**< CLOCK_MONOTONIC, for interval */

    /* ---- Clock servo ---- */
    long                peer_path_delay_ns;    /**< Latest peerMeanPathDelay */
    int64_t             last_remote_ns;        /**< For PI feed-forward */
    int64_t             last_local_ns;
    int64_t             last_offset_ns;
    gptp_pi_t           pi;

    /* ---- status (protected by mutex) ---- */
    SemaphoreHandle_t   status_mutex;
    gptp_status_t       status;
};

/* =========================================================================
 * L2TAP I/O helpers (same pattern as ptpd.c)
 * ========================================================================= */

/**
 * @brief Send an Ethernet frame via L2TAP and optionally retrieve TX timestamp.
 *
 * @param state     Daemon state
 * @param ptp_msg   PTP message payload (without Ethernet header)
 * @param ptp_len   PTP payload length
 * @param tx_ts     If non-NULL, filled with hardware TX timestamp
 * @return          Bytes sent on success, negative on error
 */
static int gptp_net_send(struct gptp_state_s *state,
                         const void *ptp_msg, uint16_t ptp_len,
                         struct timespec *tx_ts)
{
    /* Build Ethernet frame: 14-byte header + PTP payload */
    uint8_t eth_frame[GPTP_FRAME_MAX];
    if (ptp_len + GPTP_ETH_HEADER_LEN > sizeof(eth_frame)) {
        GTPE("gptp_net_send: payload too large (%u)", ptp_len);
        return -1;
    }

    /* Destination MAC: gPTP multicast */
    static const uint8_t gptp_mcast[ETH_ADDR_LEN] = GPTP_MCAST_MAC;
    memcpy(eth_frame + 0, gptp_mcast, ETH_ADDR_LEN);
    /* Source MAC: our interface MAC */
    memcpy(eth_frame + 6, state->intf_hw_addr, ETH_ADDR_LEN);
    /* EtherType: 0x88F7 */
    eth_frame[12] = (uint8_t)(GPTP_ETHERTYPE >> 8);
    eth_frame[13] = (uint8_t)(GPTP_ETHERTYPE & 0xFF);
    /* PTP payload */
    memcpy(eth_frame + GPTP_ETH_HEADER_LEN, ptp_msg, ptp_len);

    /* Prepare L2TAP extended buffer with IREC timestamp request */
    union {
        uint8_t buff[L2TAP_IREC_SPACE(sizeof(struct timespec))];
        l2tap_irec_hdr_t align;
    } irec_u;

    l2tap_extended_buff_t ext;
    ext.info_recs_len  = sizeof(irec_u.buff);
    ext.info_recs_buff = irec_u.buff;
    ext.buff           = eth_frame;
    ext.buff_len       = GPTP_ETH_HEADER_LEN + ptp_len;

    l2tap_irec_hdr_t *ts_irec = L2TAP_IREC_FIRST(&ext);
    ts_irec->len  = L2TAP_IREC_LEN(sizeof(struct timespec));
    ts_irec->type = L2TAP_IREC_TIME_STAMP;

    int ret = write(state->ptp_socket, &ext, 0);

    if (ret > 0 && tx_ts && ts_irec->type == L2TAP_IREC_TIME_STAMP) {
        *tx_ts = *(struct timespec *)ts_irec->data;
    }

    return ret;
}

/**
 * @brief Receive an Ethernet frame via L2TAP and extract PTP payload + RX timestamp.
 *
 * @param state     Daemon state
 * @param ptp_msg   Output buffer for PTP payload (without Ethernet header)
 * @param ptp_max   Maximum bytes to store in ptp_msg
 * @param rx_ts     If non-NULL, filled with hardware RX timestamp
 * @return          PTP payload bytes on success, 0 if no frame, negative on error
 */
static int gptp_net_recv(struct gptp_state_s *state,
                         void *ptp_msg, uint16_t ptp_max,
                         struct timespec *rx_ts)
{
    uint8_t eth_frame[GPTP_FRAME_MAX];

    union {
        uint8_t buff[L2TAP_IREC_SPACE(sizeof(struct timespec))];
        l2tap_irec_hdr_t align;
    } irec_u;

    l2tap_extended_buff_t ext;
    ext.info_recs_len  = sizeof(irec_u.buff);
    ext.info_recs_buff = irec_u.buff;
    ext.buff           = eth_frame;
    ext.buff_len       = sizeof(eth_frame);

    l2tap_irec_hdr_t *ts_irec = L2TAP_IREC_FIRST(&ext);
    ts_irec->len  = L2TAP_IREC_LEN(sizeof(struct timespec));
    ts_irec->type = L2TAP_IREC_TIME_STAMP;

    int ret = read(state->ptp_socket, &ext, 0);
    if (ret <= 0) {
        return ret;
    }

    if (rx_ts && ts_irec->type == L2TAP_IREC_TIME_STAMP) {
        *rx_ts = *(struct timespec *)ts_irec->data;
    }

    /* Strip Ethernet header, copy PTP payload */
    int ptp_len = ret - GPTP_ETH_HEADER_LEN;
    if (ptp_len <= 0 || ptp_len > (int)ptp_max) {
        return -1;
    }
    memcpy(ptp_msg, eth_frame + GPTP_ETH_HEADER_LEN, ptp_len);
    return ptp_len;
}

/* =========================================================================
 * Clock helpers
 * ========================================================================= */

static void gptp_gettime(struct timespec *ts)
{
    if (esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, ts) != 0) {
        clock_gettime(CLOCK_REALTIME, ts);   /* fallback */
    }
}

static void gptp_settime(struct gptp_state_s *state, const struct timespec *ts)
{
    (void)state;
    esp_eth_clock_settime(CLOCK_PTP_SYSTEM, ts);
}

/* =========================================================================
 * Message helpers
 * ========================================================================= */

/**
 * @brief Fill common gPTP header fields into a message buffer.
 *
 * @param hdr       Pointer to the gptp_header_s to fill
 * @param msgtype   4-bit message type (GPTP_MSGTYPE_*)
 * @param msglen    Total message length in bytes
 * @param state     Daemon state (for sourceidentity etc.)
 * @param seq       Sequence ID to use
 * @param ctrl      Control field value
 * @param log_interval  logMessageInterval
 * @param two_step  Set TWO_STEP flag in flags[0]
 */
static void gptp_fill_header(struct gptp_header_s *hdr,
                             uint8_t msgtype, uint16_t msglen,
                             struct gptp_state_s *state,
                             uint16_t seq, uint8_t ctrl,
                             int8_t log_interval, bool two_step)
{
    memset(hdr, 0, sizeof(*hdr));
    hdr->messagetype = GPTP_MSG_BYTE0(msgtype);
    hdr->version     = 2;
    gptp_put_u16(hdr->messagelength, msglen);
    hdr->domain      = CONFIG_GPTP_DOMAIN;
    if (two_step) {
        hdr->flags[0] = GPTP_FLAGS0_TWOSTEP;
    }
    hdr->flags[1] = GPTP_FLAGS1_PTP_TIMESCALE;
    memcpy(hdr->sourceidentity,   state->own_identity.header.sourceidentity, 8);
    memcpy(hdr->sourceportindex,  state->own_identity.header.sourceportindex, 2);
    gptp_put_u16(hdr->sequenceid, seq);
    hdr->controlfield       = ctrl;
    hdr->logmessageinterval = (uint8_t)(int8_t)log_interval;
}

static uint16_t gptp_get_seq(const struct gptp_header_s *hdr)
{
    return gptp_get_u16(hdr->sequenceid);
}

/* =========================================================================
 * PDelay Requester functions
 * ========================================================================= */

/**
 * @brief Issue a PDelayReq message.  Records T1 (TX hardware timestamp).
 */
static void gptp_issue_pdelay_req(struct gptp_state_s *state)
{
    struct gptp_pdelay_req_s req;
    memset(&req, 0, sizeof(req));

    state->pdelay_seq++;
    gptp_fill_header(&req.header, GPTP_MSGTYPE_PDELAY_REQ, sizeof(req),
                     state, state->pdelay_seq,
                     GPTP_CTRL_OTHER, -2, false);

    struct timespec tx_ts;
    memset(&tx_ts, 0, sizeof(tx_ts));

    int ret = gptp_net_send(state, &req, sizeof(req), &tx_ts);
    if (ret < 0) {
        GTPE("PDelayReq send failed: %d", errno);
        return;
    }

    /* If hardware timestamp returned, use it; otherwise fall back to SW */
    if (tx_ts.tv_sec == 0 && tx_ts.tv_nsec == 0) {
        gptp_gettime(&tx_ts);
    }
    state->pdelay_t1              = tx_ts;
    state->pdelay_outstanding_seq = state->pdelay_seq;
    state->waiting_pdelay_resp    = true;
    state->waiting_pdelay_resp_fup = false;

    clock_gettime(CLOCK_MONOTONIC, &state->last_pdelay_req_tx);
    clock_gettime(CLOCK_MONOTONIC, &state->status.last_pdelay_resp); /* update monotonic timer */

    GTPD("Sent PDelayReq seq=%u", state->pdelay_seq);
}

/**
 * @brief Handle incoming PDelayResp.
 *
 * Validates that it matches our outstanding PDelayReq, records T4 and T2.
 * In TWO_STEP mode sets waiting_pdelay_resp_fup; in ONE_STEP computes delay directly.
 */
static void gptp_handle_pdelay_resp(struct gptp_state_s *state,
                                    const struct gptp_pdelay_resp_s *resp,
                                    const struct timespec *rx_ts)
{
    if (!state->waiting_pdelay_resp) {
        return;
    }
    uint16_t seq = gptp_get_seq(&resp->header);
    if (seq != state->pdelay_outstanding_seq) {
        GTPD("PDelayResp seq mismatch: got %u expected %u", seq, state->pdelay_outstanding_seq);
        return;
    }
    /* Verify that the resp is addressed to us (requestingportidentity matches our identity) */
    if (memcmp(resp->requestingportidentity,
               state->own_identity.header.sourceidentity, 8) != 0) {
        return;   /* not for us */
    }

    /* T4: RX timestamp of PDelayResp on Requester side */
    if (rx_ts && (rx_ts->tv_sec != 0 || rx_ts->tv_nsec != 0)) {
        state->pdelay_t4 = *rx_ts;
    } else {
        gptp_gettime(&state->pdelay_t4);
    }
    clock_gettime(CLOCK_MONOTONIC, &state->status.last_pdelay_resp);

    /* T2: requestReceiptTimestamp from PDelayResp body */
    gptp_wire_to_ts(resp->requestreceipttimestamp, &state->pdelay_t2_from_resp);

    state->waiting_pdelay_resp = false;

#if CONFIG_GPTP_TWO_STEP
    /* Wait for PDelayRespFollowUp to get T3 */
    state->waiting_pdelay_resp_fup = true;
    GTPD("PDelayResp received seq=%u, waiting for FUP", seq);
#else
    /* ONE_STEP: T3-T2 is encoded in correctionField, compute path delay now */
    int64_t correction_ns = gptp_correction_to_ns(resp->header.correction);
    int64_t t4_ns = (int64_t)state->pdelay_t4.tv_sec * 1000000000LL + state->pdelay_t4.tv_nsec;
    int64_t t1_ns = (int64_t)state->pdelay_t1.tv_sec * 1000000000LL + state->pdelay_t1.tv_nsec;
    int64_t delay_ns = (t4_ns - t1_ns - correction_ns) / 2;
    state->peer_path_delay_ns = (long)delay_ns;
    GTPI("peerMeanPathDelay = %ld ns (ONE_STEP)", state->peer_path_delay_ns);
    xSemaphoreTake(state->status_mutex, portMAX_DELAY);
    state->status.path_delay_ns = state->peer_path_delay_ns;
    xSemaphoreGive(state->status_mutex);
    state->waiting_pdelay_resp_fup = false;
#endif
}

/**
 * @brief Handle incoming PDelayRespFollowUp (TWO_STEP mode).
 *
 * Gets T3 and computes peerMeanPathDelay = ((T2-T1) + (T4-T3)) / 2 - correctionField.
 */
static void gptp_handle_pdelay_resp_fup(struct gptp_state_s *state,
                                        const struct gptp_pdelay_resp_fup_s *fup)
{
    if (!state->waiting_pdelay_resp_fup) {
        return;
    }
    uint16_t seq = gptp_get_seq(&fup->header);
    if (seq != state->pdelay_outstanding_seq) {
        return;
    }
    if (memcmp(fup->requestingportidentity,
               state->own_identity.header.sourceidentity, 8) != 0) {
        return;
    }

    /* T3: responseOriginTimestamp from PDelayRespFollowUp */
    struct timespec pdelay_t3;
    gptp_wire_to_ts(fup->responseorigintimestamp, &pdelay_t3);

    /* correctionField from FUP (should be 0 in typical implementations) */
    int64_t correction_ns = gptp_correction_to_ns(fup->header.correction);

    /* TWO_STEP calculation:
     * Tab = T2 - T1  (Responder perspective: ingress relative to Requester TX)
     * Tba = T4 - T3  (Requester perspective: ingress relative to Responder TX)
     * peerMeanPathDelay = (Tab + Tba) / 2 - correctionField
     */
    int64_t t1_ns = (int64_t)state->pdelay_t1.tv_sec * 1000000000LL + state->pdelay_t1.tv_nsec;
    int64_t t2_ns = (int64_t)state->pdelay_t2_from_resp.tv_sec * 1000000000LL
                  + state->pdelay_t2_from_resp.tv_nsec;
    int64_t t3_ns = (int64_t)pdelay_t3.tv_sec * 1000000000LL + pdelay_t3.tv_nsec;
    int64_t t4_ns = (int64_t)state->pdelay_t4.tv_sec * 1000000000LL + state->pdelay_t4.tv_nsec;

    int64_t tab = t2_ns - t1_ns;
    int64_t tba = t4_ns - t3_ns;
    int64_t delay_ns = (tab + tba) / 2 - correction_ns;

    /* Sanity check: ignore negative or unrealistically large values */
    if (delay_ns < 0 || delay_ns > 100000000LL) {
        GTPW("peerMeanPathDelay out of range: %lld ns, ignored", (long long)delay_ns);
        state->waiting_pdelay_resp_fup = false;
        return;
    }

    state->peer_path_delay_ns = (long)delay_ns;

    GTPI("peerMeanPathDelay = %ld ns (T1=%lld T2=%lld T3=%lld T4=%lld CF=%lld)",
         state->peer_path_delay_ns,
         (long long)t1_ns, (long long)t2_ns,
         (long long)t3_ns, (long long)t4_ns,
         (long long)correction_ns);

    xSemaphoreTake(state->status_mutex, portMAX_DELAY);
    state->status.path_delay_ns = state->peer_path_delay_ns;
    xSemaphoreGive(state->status_mutex);

    state->waiting_pdelay_resp_fup = false;
}

/* =========================================================================
 * PDelay Responder functions
 * ========================================================================= */

/**
 * @brief Handle incoming PDelayReq and send PDelayResp (+ FUP in TWO_STEP).
 *
 * Any gPTP node must respond to PDelayReq regardless of MASTER/SLAVE role.
 */
static void gptp_handle_pdelay_req(struct gptp_state_s *state,
                                   const struct gptp_pdelay_req_s *req,
                                   const struct timespec *rx_ts)
{
    uint16_t seq = gptp_get_seq(&req->header);

    /* T2: RX timestamp of PDelayReq on Responder side */
    if (rx_ts && (rx_ts->tv_sec != 0 || rx_ts->tv_nsec != 0)) {
        state->resp_t2 = *rx_ts;
    } else {
        gptp_gettime(&state->resp_t2);
    }
    state->recv_pdelay_req_seq = seq;

    /* Build PDelayResp */
    struct gptp_pdelay_resp_s resp;
    memset(&resp, 0, sizeof(resp));

    /* The Responder increments its own sequence counter in sourceidentity/portindex
     * but the sequenceId in PDelayResp must ECHO the PDelayReq sequenceId */
#ifdef CONFIG_GPTP_TWO_STEP
    const bool resp_two_step = true;
#else
    const bool resp_two_step = false;
#endif
    gptp_fill_header(&resp.header, GPTP_MSGTYPE_PDELAY_RESP, sizeof(resp),
                     state, seq,
                     GPTP_CTRL_OTHER, -2, resp_two_step);

    /* requestReceiptTimestamp = T2 */
    gptp_ts_to_wire(&state->resp_t2, resp.requestreceipttimestamp);

    /* requestingPortIdentity = sourcePortIdentity from PDelayReq */
    memcpy(resp.requestingportidentity,     req->header.sourceidentity, 8);
    memcpy(resp.requestingportidentity + 8, req->header.sourceportindex, 2);

    struct timespec tx_ts;
    memset(&tx_ts, 0, sizeof(tx_ts));

    int ret = gptp_net_send(state, &resp, sizeof(resp), &tx_ts);
    if (ret < 0) {
        GTPE("PDelayResp send failed: %d", errno);
        return;
    }

    /* T3: TX timestamp of PDelayResp on Responder side */
    if (tx_ts.tv_sec == 0 && tx_ts.tv_nsec == 0) {
        gptp_gettime(&tx_ts);
    }

    GTPD("Sent PDelayResp seq=%u", seq);

#if CONFIG_GPTP_TWO_STEP
    /* TWO_STEP: send PDelayRespFollowUp containing T3 */
    struct gptp_pdelay_resp_fup_s fup;
    memset(&fup, 0, sizeof(fup));
    gptp_fill_header(&fup.header, GPTP_MSGTYPE_PDELAY_RESP_FUP, sizeof(fup),
                     state, seq,
                     GPTP_CTRL_OTHER, -2, false);

    /* responseOriginTimestamp = T3 */
    gptp_ts_to_wire(&tx_ts, fup.responseorigintimestamp);

    /* requestingPortIdentity: same as in PDelayResp */
    memcpy(fup.requestingportidentity, resp.requestingportidentity, 10);

    /* correctionField: encode residence time (T3 - T2) if available */
    int64_t residence_ns = gptp_timespec_diff_ns(&tx_ts, &state->resp_t2);
    gptp_ns_to_correction(residence_ns, fup.header.correction);

    ret = gptp_net_send(state, &fup, sizeof(fup), NULL);
    if (ret < 0) {
        GTPE("PDelayRespFollowUp send failed: %d", errno);
    } else {
        GTPD("Sent PDelayRespFollowUp seq=%u residence=%lldns", seq, (long long)residence_ns);
    }
#else
    /* ONE_STEP: correctionField in PDelayResp already contains T3-T2 */
    (void)tx_ts;
#endif
}

/* =========================================================================
 * Sync Master functions
 * ========================================================================= */

/**
 * @brief Issue Sync (and FollowUp in TWO_STEP) as gPTP master.
 */
static void gptp_issue_sync(struct gptp_state_s *state)
{
    struct gptp_sync_s sync;
    memset(&sync, 0, sizeof(sync));

    state->sync_seq++;
#ifdef CONFIG_GPTP_TWO_STEP
    bool two_step = true;
#else
    bool two_step = false;
#endif
    gptp_fill_header(&sync.header, GPTP_MSGTYPE_SYNC, sizeof(sync),
                     state, state->sync_seq,
                     GPTP_CTRL_SYNC, -3, two_step);

#if !CONFIG_GPTP_TWO_STEP
    /* ONE_STEP: get current time as estimate for origintimestamp */
    struct timespec ts;
    gptp_gettime(&ts);
    gptp_ts_to_wire(&ts, sync.origintimestamp);
#endif

    struct timespec tx_ts;
    memset(&tx_ts, 0, sizeof(tx_ts));

    int ret = gptp_net_send(state, &sync, sizeof(sync), &tx_ts);
    if (ret < 0) {
        GTPE("Sync send failed: %d", errno);
        return;
    }

    clock_gettime(CLOCK_MONOTONIC, &state->last_sync_tx);
    clock_gettime(CLOCK_MONOTONIC, &state->status.last_transmitted_sync);

#if CONFIG_GPTP_TWO_STEP
    /* Get actual TX timestamp; fall back to SW */
    if (tx_ts.tv_sec == 0 && tx_ts.tv_nsec == 0) {
        gptp_gettime(&tx_ts);
    }

    /* Send FollowUp with precise TX timestamp */
    struct gptp_follow_up_s fup;
    memset(&fup, 0, sizeof(fup));
    gptp_fill_header(&fup.header, GPTP_MSGTYPE_FOLLOW_UP, sizeof(fup),
                     state, state->sync_seq,
                     GPTP_CTRL_FOLLOW_UP, -3, false);
    gptp_ts_to_wire(&tx_ts, fup.origintimestamp);

    ret = gptp_net_send(state, &fup, sizeof(fup), NULL);
    if (ret < 0) {
        GTPE("FollowUp send failed: %d", errno);
    } else {
        GTPD("Sent Sync+FollowUp seq=%u T1=%lld.%09ld",
             state->sync_seq, (long long)tx_ts.tv_sec, tx_ts.tv_nsec);
    }
#endif
}

/* =========================================================================
 * Sync Slave functions (clock servo)
 * ========================================================================= */

/**
 * @brief PI controller + frequency adjustment via DM9058 ioctl.
 *
 * offset_ns = T2 - T1 - peerMeanPathDelay (positive means local clock is behind master).
 */
static void gptp_apply_clock_correction(struct gptp_state_s *state,
                                        int64_t offset_ns,
                                        const struct timespec *remote_ts,
                                        const struct timespec *local_ts)
{
    int64_t abs_offset = (offset_ns < 0) ? -offset_ns : offset_ns;

    /* Large offset → step clock */
    if (GPTP_SETTIME_THRESHOLD_NS > 0 && abs_offset > GPTP_SETTIME_THRESHOLD_NS) {
        struct timespec new_time;
        gptp_gettime(&new_time);
        /* new_time = remote_ts + (now - local_ts) */
        int64_t elapsed_ns = gptp_timespec_diff_ns(&new_time, local_ts);
        new_time.tv_sec  = remote_ts->tv_sec;
        new_time.tv_nsec = remote_ts->tv_nsec;
        new_time.tv_nsec += (long)(elapsed_ns % 1000000000LL);
        new_time.tv_sec  += elapsed_ns / 1000000000LL;
        if (new_time.tv_nsec >= 1000000000L) { new_time.tv_sec++; new_time.tv_nsec -= 1000000000L; }
        if (new_time.tv_nsec < 0)            { new_time.tv_sec--; new_time.tv_nsec += 1000000000L; }

        gptp_settime(state, &new_time);
        GTPW("Clock step: offset %+lld ns → settime()", (long long)offset_ns);

        /* Reset PI state */
        state->last_remote_ns = 0;
        state->last_local_ns  = 0;
        state->last_offset_ns = 0;
        state->pi.drift_acc   = 0;
        return;
    }

    /* Small offset → PI frequency adjustment */
    int64_t remote_ns = (int64_t)remote_ts->tv_sec * 1000000000LL + remote_ts->tv_nsec;
    int64_t local_ns  = (int64_t)local_ts->tv_sec  * 1000000000LL + local_ts->tv_nsec;

    if (state->last_remote_ns == 0 || state->last_local_ns == 0) {
        state->last_remote_ns = remote_ns;
        state->last_local_ns  = local_ns;
        state->last_offset_ns = offset_ns;
        return;
    }

    int64_t local_delta = local_ns - state->last_local_ns;
    if (local_delta <= 0) {
        state->last_remote_ns = remote_ns;
        state->last_local_ns  = local_ns;
        return;
    }

    /* Integrator update */
    state->pi.drift_acc += (int32_t)(offset_ns / state->pi.ki);
    if (state->pi.drift_acc >  GPTP_ADJ_FREQ_MAX) state->pi.drift_acc =  GPTP_ADJ_FREQ_MAX;
    if (state->pi.drift_acc < -GPTP_ADJ_FREQ_MAX) state->pi.drift_acc = -GPTP_ADJ_FREQ_MAX;

    /* Feed-forward: tick difference between master and slave */
    int64_t remote_delta = remote_ns - state->last_remote_ns;
    int64_t tick_diff    = remote_delta - local_delta;
    int64_t ff_ppb  = (tick_diff * 1000000000LL / local_delta) * 20 / 100;

    int64_t pi_ppb  = (int64_t)(offset_ns / state->pi.kp + state->pi.drift_acc)
                    * 1000000000LL / local_delta;
    int64_t adj_ppb = pi_ppb + ff_ppb;

    if (adj_ppb >  GPTP_ADJ_FREQ_MAX) adj_ppb =  GPTP_ADJ_FREQ_MAX;
    if (adj_ppb < -GPTP_ADJ_FREQ_MAX) adj_ppb = -GPTP_ADJ_FREQ_MAX;

    int32_t adj_i32 = (int32_t)adj_ppb;
    esp_err_t err = esp_eth_ioctl(state->eth_handle,
                                  ETH_MAC_DM9058_CMD_ADJ_PTP_FREQ, &adj_i32);
    if (err != ESP_OK) {
        GTPW("ADJ_PTP_FREQ failed: %s", esp_err_to_name(err));
    }

    GTPW("offset=%+lld ns adj=%+ld ppb drift_acc=%+ld path_delay=%ld ns",
         (long long)offset_ns, (long)adj_ppb,
         (long)state->pi.drift_acc, state->peer_path_delay_ns);

    state->last_remote_ns = remote_ns;
    state->last_local_ns  = local_ns;
    state->last_offset_ns = offset_ns;

    xSemaphoreTake(state->status_mutex, portMAX_DELAY);
    state->status.offset_ns   = offset_ns;
    state->status.drift_ppb   = (long)adj_ppb;
    state->status.path_delay_ns = state->peer_path_delay_ns;
    xSemaphoreGive(state->status_mutex);
}

/**
 * @brief Handle incoming Sync message (slave role).
 *
 * Records T2 (hardware RX timestamp). In TWO_STEP mode buffers the message
 * and waits for FollowUp to get T1. In ONE_STEP applies correction immediately.
 */
static void gptp_handle_sync(struct gptp_state_s *state,
                             const struct gptp_sync_s *sync,
                             const struct timespec *rx_ts)
{
    /* Ignore Sync from ourselves */
    if (memcmp(sync->header.sourceidentity,
               state->own_identity.header.sourceidentity, 8) == 0) {
        return;
    }
    /* Ignore if we are master and received our own Sync echo */
    if (state->is_master) {
        return;
    }
    /* Check that Sync is from the current best master */
    if (state->best_master_valid &&
        memcmp(sync->header.sourceidentity,
               state->best_master.header.sourceidentity, 8) != 0) {
        return;
    }

    /* T2: RX timestamp of Sync */
    if (rx_ts && (rx_ts->tv_sec != 0 || rx_ts->tv_nsec != 0)) {
        state->sync_t2 = *rx_ts;
    } else {
        gptp_gettime(&state->sync_t2);
    }

    clock_gettime(CLOCK_MONOTONIC, &state->status.last_received_sync);

    bool two_step = !!(sync->header.flags[0] & GPTP_FLAGS0_TWOSTEP);
    uint16_t seq  = gptp_get_seq(&sync->header);

    if (two_step) {
        /* Buffer Sync, wait for FollowUp to get T1 */
        state->twostep_sync = *sync;
        state->twostep_t2   = state->sync_t2;
        state->sync_seq_expected = seq;
        state->waiting_followup  = true;
        GTPD("Sync TWO_STEP seq=%u, waiting FollowUp", seq);
    } else {
        /* ONE_STEP: T1 is in origintimestamp */
        struct timespec t1;
        gptp_wire_to_ts(sync->origintimestamp, &t1);
        int64_t correction_ns = gptp_correction_to_ns(sync->header.correction);

        /* offsetFromMaster = T2 - T1 - correctionField - peerMeanPathDelay */
        int64_t offset_ns = gptp_timespec_diff_ns(&state->sync_t2, &t1)
                          - correction_ns - state->peer_path_delay_ns;

        gptp_apply_clock_correction(state, offset_ns, &t1, &state->sync_t2);
        state->waiting_followup = false;

        xSemaphoreTake(state->status_mutex, portMAX_DELAY);
        state->status.clock_source_valid = true;
        xSemaphoreGive(state->status_mutex);
    }
}

/**
 * @brief Handle incoming FollowUp (TWO_STEP slave mode).
 */
static void gptp_handle_follow_up(struct gptp_state_s *state,
                                  const struct gptp_follow_up_s *fup)
{
    if (!state->waiting_followup) {
        return;
    }
    uint16_t seq = gptp_get_seq(&fup->header);
    if (seq != state->sync_seq_expected) {
        GTPD("FollowUp seq mismatch: got %u expected %u", seq, state->sync_seq_expected);
        return;
    }
    /* Check source identity matches buffered Sync */
    if (memcmp(fup->header.sourceidentity,
               state->twostep_sync.header.sourceidentity, 8) != 0) {
        return;
    }

    /* T1: precise TX timestamp from FollowUp */
    struct timespec t1;
    gptp_wire_to_ts(fup->origintimestamp, &t1);

    /* Combined correctionField: CF_sync + CF_followup */
    int64_t cf_sync    = gptp_correction_to_ns(state->twostep_sync.header.correction);
    int64_t cf_fup     = gptp_correction_to_ns(fup->header.correction);
    int64_t correction = cf_sync + cf_fup;

    /* offsetFromMaster = T2 - T1 - correctionField - peerMeanPathDelay */
    int64_t offset_ns = gptp_timespec_diff_ns(&state->twostep_t2, &t1)
                      - correction - state->peer_path_delay_ns;

    GTPD("FollowUp seq=%u T1=%lld.%09ld T2=%lld.%09ld offset=%+lld ns",
         seq,
         (long long)t1.tv_sec, t1.tv_nsec,
         (long long)state->twostep_t2.tv_sec, state->twostep_t2.tv_nsec,
         (long long)offset_ns);

    gptp_apply_clock_correction(state, offset_ns, &t1, &state->twostep_t2);
    state->waiting_followup = false;

    xSemaphoreTake(state->status_mutex, portMAX_DELAY);
    state->status.clock_source_valid = true;
    xSemaphoreGive(state->status_mutex);
}

/* =========================================================================
 * Announce / BMCA functions
 * ========================================================================= */

/**
 * @brief Issue gPTP Announce message (master role).
 */
static void gptp_issue_announce(struct gptp_state_s *state)
{
    struct gptp_announce_s ann;
    memset(&ann, 0, sizeof(ann));

    state->announce_seq++;
    gptp_fill_header(&ann.header, GPTP_MSGTYPE_ANNOUNCE, sizeof(ann),
                     state, state->announce_seq,
                     GPTP_CTRL_OTHER, 0, false);

    struct timespec ts;
    gptp_gettime(&ts);
    gptp_ts_to_wire(&ts, ann.origintimestamp);

    /* Copy our own clock parameters */
    ann.gm_priority1 = state->own_identity.gm_priority1;
    memcpy(ann.gm_quality, state->own_identity.gm_quality, 4);
    ann.gm_priority2 = state->own_identity.gm_priority2;
    memcpy(ann.gm_identity, state->own_identity.gm_identity, 8);
    ann.stepsremoved[0] = 0;
    ann.stepsremoved[1] = 0;
    ann.timesource = state->own_identity.timesource;

    gptp_net_send(state, &ann, sizeof(ann), NULL);
    clock_gettime(CLOCK_MONOTONIC, &state->last_announce_tx);
    GTPD("Sent Announce seq=%u", state->announce_seq);
}

/**
 * @brief Return true if clock A is a better master than clock B.
 *
 * Simplified BMCA per IEEE 802.1AS: compare priority1, clockClass,
 * clockAccuracy, offsetScaledLogVariance, priority2, then clockIdentity.
 */
static bool gptp_is_better_master(const struct gptp_announce_s *a,
                                  const struct gptp_announce_s *b)
{
    if (a->gm_priority1 != b->gm_priority1) return a->gm_priority1 < b->gm_priority1;
    if (a->gm_quality[0] != b->gm_quality[0]) return a->gm_quality[0] < b->gm_quality[0];
    if (a->gm_quality[1] != b->gm_quality[1]) return a->gm_quality[1] < b->gm_quality[1];
    if (a->gm_quality[2] != b->gm_quality[2]) return a->gm_quality[2] < b->gm_quality[2];
    if (a->gm_quality[3] != b->gm_quality[3]) return a->gm_quality[3] < b->gm_quality[3];
    if (a->gm_priority2 != b->gm_priority2)   return a->gm_priority2 < b->gm_priority2;
    return memcmp(a->gm_identity, b->gm_identity, 8) < 0;
}

/**
 * @brief Handle incoming Announce: update best master and BMCA.
 */
static void gptp_handle_announce(struct gptp_state_s *state,
                                 const struct gptp_announce_s *ann)
{
    /* Ignore our own Announces */
    if (memcmp(ann->header.sourceidentity,
               state->own_identity.header.sourceidentity, 8) == 0) {
        return;
    }

    clock_gettime(CLOCK_MONOTONIC, &state->last_announce_rx);
    clock_gettime(CLOCK_MONOTONIC, &state->status.last_received_announce);

    /* Check if this is better than our current best master */
    bool update = false;
    if (!state->best_master_valid) {
        update = true;
    } else if (gptp_is_better_master(ann, &state->best_master)) {
        update = true;
    }

    if (update) {
        state->best_master = *ann;
        state->best_master_valid = true;
        GTPI("New best master: identity=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X priority1=%u",
             ann->gm_identity[0], ann->gm_identity[1], ann->gm_identity[2], ann->gm_identity[3],
             ann->gm_identity[4], ann->gm_identity[5], ann->gm_identity[6], ann->gm_identity[7],
             ann->gm_priority1);

        memcpy(state->status.master_identity, ann->gm_identity, 8);
    }
}

/**
 * @brief Run BMCA: decide MASTER or SLAVE role based on current best master.
 *
 * Called periodically (every ANNOUNCE_INTERVAL) and after announce timeout.
 */
static void gptp_bmca(struct gptp_state_s *state)
{
#if CONFIG_GPTP_SLAVE_ONLY
    bool should_be_master = false;
#else
    /* Check announce timeout */
    bool master_alive = false;
    if (state->best_master_valid) {
        int64_t elapsed = gptp_elapsed_ms(&state->last_announce_rx);
        master_alive = (elapsed < GPTP_ANNOUNCE_TIMEOUT_MS);
        if (!master_alive) {
            GTPW("Best master timed out (%.0lld ms since last Announce)", (long long)elapsed);
            state->best_master_valid    = false;
            state->waiting_followup     = false;
            state->last_remote_ns       = 0;
            state->last_local_ns        = 0;
            xSemaphoreTake(state->status_mutex, portMAX_DELAY);
            state->status.clock_source_valid = false;
            xSemaphoreGive(state->status_mutex);
        }
    }

    bool should_be_master;
    if (!master_alive) {
        should_be_master = true;   /* No master found → become master */
    } else {
        /* Compare our own identity against best master */
        should_be_master = gptp_is_better_master(&state->own_identity, &state->best_master);
    }
#endif /* CONFIG_GPTP_SLAVE_ONLY */

    if (should_be_master && !state->is_master) {
        GTPI("Became gPTP MASTER");
        state->is_master = true;
        /* Reset slave state */
        state->waiting_followup = false;
        state->last_remote_ns   = 0;
        state->last_local_ns    = 0;
        xSemaphoreTake(state->status_mutex, portMAX_DELAY);
        state->status.is_master = true;
        state->status.clock_source_valid = false;
        xSemaphoreGive(state->status_mutex);
    } else if (!should_be_master && state->is_master) {
        GTPI("Became gPTP SLAVE");
        state->is_master = false;
        xSemaphoreTake(state->status_mutex, portMAX_DELAY);
        state->status.is_master = false;
        xSemaphoreGive(state->status_mutex);
    }
}

/* =========================================================================
 * Message dispatcher
 * ========================================================================= */

static void gptp_process_message(struct gptp_state_s *state,
                                 const uint8_t *ptp_msg, int ptp_len,
                                 const struct timespec *rx_ts)
{
    if (ptp_len < (int)sizeof(struct gptp_header_s)) {
        return;
    }

    const struct gptp_header_s *hdr = (const struct gptp_header_s *)ptp_msg;

    /* Validate transportSpecific = 0x1 (gPTP SdoId) */
    if ((hdr->messagetype & GPTP_TRANSPORT_SPECIFIC_MASK) != GPTP_TRANSPORT_SPECIFIC) {
        GTPD("Ignoring non-gPTP frame (ts=0x%X)", hdr->messagetype >> 4);
        return;
    }

    /* Validate domain */
    if (hdr->domain != CONFIG_GPTP_DOMAIN) {
        return;
    }

    uint8_t msgtype = hdr->messagetype & GPTP_MSGTYPE_MASK;

    switch (msgtype) {
    case GPTP_MSGTYPE_SYNC:
        if (ptp_len >= (int)sizeof(struct gptp_sync_s)) {
            gptp_handle_sync(state, (const struct gptp_sync_s *)ptp_msg, rx_ts);
        }
        break;

    case GPTP_MSGTYPE_FOLLOW_UP:
        if (ptp_len >= (int)sizeof(struct gptp_follow_up_s)) {
            gptp_handle_follow_up(state, (const struct gptp_follow_up_s *)ptp_msg);
        }
        break;

    case GPTP_MSGTYPE_PDELAY_REQ:
        if (ptp_len >= (int)sizeof(struct gptp_pdelay_req_s)) {
            gptp_handle_pdelay_req(state, (const struct gptp_pdelay_req_s *)ptp_msg, rx_ts);
        }
        break;

    case GPTP_MSGTYPE_PDELAY_RESP:
        if (ptp_len >= (int)sizeof(struct gptp_pdelay_resp_s)) {
            gptp_handle_pdelay_resp(state, (const struct gptp_pdelay_resp_s *)ptp_msg, rx_ts);
        }
        break;

    case GPTP_MSGTYPE_PDELAY_RESP_FUP:
        if (ptp_len >= (int)sizeof(struct gptp_pdelay_resp_fup_s)) {
            gptp_handle_pdelay_resp_fup(state, (const struct gptp_pdelay_resp_fup_s *)ptp_msg);
        }
        break;

    case GPTP_MSGTYPE_ANNOUNCE:
        if (ptp_len >= (int)sizeof(struct gptp_announce_s)) {
            gptp_handle_announce(state, (const struct gptp_announce_s *)ptp_msg);
        }
        break;

    default:
        GTPD("Unknown gPTP msgtype 0x%X, ignored", msgtype);
        break;
    }
}

/* =========================================================================
 * Initialization / Teardown
 * ========================================================================= */

#define GPTP_ETHERTYPE_U16 ((uint16_t)GPTP_ETHERTYPE)

static esp_err_t gptp_socket_init(struct gptp_state_s *state, const char *interface)
{
    state->ptp_socket = open("/dev/net/tap", 0);
    if (state->ptp_socket < 0) {
        GTPE("Failed to open /dev/net/tap: %d", errno);
        return ESP_FAIL;
    }

    if (ioctl(state->ptp_socket, L2TAP_S_INTF_DEVICE, interface) < 0) {
        GTPE("L2TAP_S_INTF_DEVICE failed: %d", errno);
        return ESP_FAIL;
    }

    uint16_t filter = GPTP_ETHERTYPE_U16;
    if (ioctl(state->ptp_socket, L2TAP_S_RCV_FILTER, &filter) < 0) {
        GTPE("L2TAP_S_RCV_FILTER failed: %d", errno);
        return ESP_FAIL;
    }

    if (ioctl(state->ptp_socket, L2TAP_G_DEVICE_DRV_HNDL, &state->eth_handle) < 0) {
        GTPE("L2TAP_G_DEVICE_DRV_HNDL failed: %d", errno);
        return ESP_FAIL;
    }

    if (ioctl(state->ptp_socket, L2TAP_S_TIMESTAMP_EN) < 0) {
        GTPE("L2TAP_S_TIMESTAMP_EN failed: %d", errno);
        return ESP_FAIL;
    }

    /* Get MAC address */
    esp_eth_ioctl(state->eth_handle, ETH_CMD_G_MAC_ADDR, state->intf_hw_addr);

    /* Add gPTP multicast MAC filter */
    uint8_t gptp_mcast[ETH_ADDR_LEN] = GPTP_MCAST_MAC;
    esp_eth_ioctl(state->eth_handle, ETH_CMD_ADD_MAC_FILTER, gptp_mcast);

    GTPI("gPTP socket init: MAC=%02X:%02X:%02X:%02X:%02X:%02X",
         state->intf_hw_addr[0], state->intf_hw_addr[1], state->intf_hw_addr[2],
         state->intf_hw_addr[3], state->intf_hw_addr[4], state->intf_hw_addr[5]);

    return ESP_OK;
}

static void gptp_socket_deinit(struct gptp_state_s *state)
{
    if (state->eth_handle) {
        uint8_t gptp_mcast[ETH_ADDR_LEN] = GPTP_MCAST_MAC;
        esp_eth_ioctl(state->eth_handle, ETH_CMD_DEL_MAC_FILTER, gptp_mcast);
    }
    if (state->ptp_socket >= 0) {
        close(state->ptp_socket);
        state->ptp_socket = -1;
    }
}

static void gptp_init_own_identity(struct gptp_state_s *state)
{
    struct gptp_announce_s *id = &state->own_identity;
    memset(id, 0, sizeof(*id));

    id->header.version = 2;
    id->header.domain  = CONFIG_GPTP_DOMAIN;

    /* Build EUI-64 clock identity from EUI-48 MAC (insert FF:FE at bytes 3-4) */
    id->header.sourceidentity[0] = state->intf_hw_addr[0];
    id->header.sourceidentity[1] = state->intf_hw_addr[1];
    id->header.sourceidentity[2] = state->intf_hw_addr[2];
    id->header.sourceidentity[3] = 0xFF;
    id->header.sourceidentity[4] = 0xFE;
    id->header.sourceidentity[5] = state->intf_hw_addr[3];
    id->header.sourceidentity[6] = state->intf_hw_addr[4];
    id->header.sourceidentity[7] = state->intf_hw_addr[5];
    id->header.sourceportindex[0] = 0;
    id->header.sourceportindex[1] = 1;

    /* Clock quality parameters (from Kconfig) */
    id->gm_priority1  = CONFIG_GPTP_PRIORITY1;
    id->gm_quality[0] = CONFIG_GPTP_CLOCK_CLASS;
    id->gm_quality[1] = CONFIG_GPTP_CLOCK_ACCURACY;
    id->gm_quality[2] = 0xFF;  /* No offsetScaledLogVariance estimate */
    id->gm_quality[3] = 0xFF;
    id->gm_priority2  = CONFIG_GPTP_PRIORITY2;
    id->timesource    = CONFIG_GPTP_CLOCK_SOURCE;

    /* Grandmaster identity = our clock identity */
    memcpy(id->gm_identity, id->header.sourceidentity, 8);
}

/* =========================================================================
 * Main daemon task
 * ========================================================================= */

static void gptp_task(void *arg)
{
    struct gptp_state_s *state = (struct gptp_state_s *)arg;

    GTPI("gPTP daemon started (interface initialized)");

    /* Initialize periodic transmission baselines */
    clock_gettime(CLOCK_MONOTONIC, &state->last_pdelay_req_tx);
    clock_gettime(CLOCK_MONOTONIC, &state->last_sync_tx);
    clock_gettime(CLOCK_MONOTONIC, &state->last_announce_tx);
    clock_gettime(CLOCK_MONOTONIC, &state->last_announce_rx);

    /* Stagger initial transmissions slightly to avoid burst */
    vTaskDelay(pdMS_TO_TICKS(50));

    while (!state->stop) {
        /* ---- Poll L2TAP socket for incoming frames (non-blocking) ---- */
        struct pollfd pfd = {
            .fd = state->ptp_socket,
            .events = POLLIN,
        };
        int ready = poll(&pfd, 1, 0);  /* timeout=0: non-blocking */

        if (ready > 0 && (pfd.revents & POLLIN)) {
            uint8_t ptp_buf[128];
            struct timespec rx_ts;
            memset(&rx_ts, 0, sizeof(rx_ts));

            int len = gptp_net_recv(state, ptp_buf, sizeof(ptp_buf), &rx_ts);
            if (len > 0) {
                gptp_process_message(state, ptp_buf, len, &rx_ts);
            }
        }

        /* ---- Periodic transmission timers ---- */

        /* PDelayReq: every 250ms (always, regardless of role) */
        if (gptp_elapsed_ms(&state->last_pdelay_req_tx) >= GPTP_PDELAY_INTERVAL_MS) {
            gptp_issue_pdelay_req(state);
        }

        /* Sync + FollowUp: only as MASTER, every 125ms */
        if (state->is_master &&
            gptp_elapsed_ms(&state->last_sync_tx) >= GPTP_SYNC_INTERVAL_MS) {
            gptp_issue_sync(state);
        }

        /* Announce: only as MASTER, every 1000ms */
        if (state->is_master &&
            gptp_elapsed_ms(&state->last_announce_tx) >= GPTP_ANNOUNCE_INTERVAL_MS) {
            gptp_issue_announce(state);
        }

        /* BMCA: evaluate role every ANNOUNCE_INTERVAL */
        if (gptp_elapsed_ms(&state->last_announce_tx) >= GPTP_ANNOUNCE_INTERVAL_MS ||
            (state->best_master_valid &&
             gptp_elapsed_ms(&state->last_announce_rx) > GPTP_ANNOUNCE_TIMEOUT_MS)) {
            gptp_bmca(state);
        }

        /* Yield to other tasks */
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    GTPI("gPTP daemon stopping");
    gptp_socket_deinit(state);
    vSemaphoreDelete(state->status_mutex);
    free(state);
    vTaskDelete(NULL);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

esp_err_t gptp_start(const char *interface, gptp_handle_t *handle)
{
    if (!interface || !handle) {
        return ESP_ERR_INVALID_ARG;
    }

    struct gptp_state_s *state = calloc(1, sizeof(*state));
    if (!state) {
        return ESP_ERR_NO_MEM;
    }

    state->ptp_socket = -1;
    state->stop       = false;

    /* Initialize PI controller */
    state->pi.kp = 5;
    state->pi.ki = 50;
    state->pi.drift_acc = 0;

    /* Create status mutex */
    state->status_mutex = xSemaphoreCreateMutex();
    if (!state->status_mutex) {
        free(state);
        return ESP_ERR_NO_MEM;
    }

    /* Initialize L2TAP socket */
    esp_err_t err = gptp_socket_init(state, interface);
    if (err != ESP_OK) {
        vSemaphoreDelete(state->status_mutex);
        free(state);
        return err;
    }

    /* Initialize own clock identity (requires MAC address from socket) */
    gptp_init_own_identity(state);

    /* Start with SLAVE role; BMCA will promote to MASTER if needed */
    state->is_master = false;

    /* Create the daemon task */
    BaseType_t ret = xTaskCreate(gptp_task, "gptp_daemon",
                                 CONFIG_GPTP_TASK_STACK_SIZE,
                                 state,
                                 CONFIG_GPTP_TASK_PRIORITY,
                                 &state->task_handle);
    if (ret != pdPASS) {
        gptp_socket_deinit(state);
        vSemaphoreDelete(state->status_mutex);
        free(state);
        return ESP_ERR_NO_MEM;
    }

    *handle = state;
    GTPI("gPTP daemon started on interface '%s'", interface);
    return ESP_OK;
}

esp_err_t gptp_stop(gptp_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    handle->stop = true;
    /* The task frees its own memory on exit */
    return ESP_OK;
}

esp_err_t gptp_get_status(gptp_handle_t handle, gptp_status_t *status)
{
    if (!handle || !status) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(handle->status_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *status = handle->status;
    xSemaphoreGive(handle->status_mutex);
    return ESP_OK;
}

#endif /* CONFIG_GPTP_ENABLE */
