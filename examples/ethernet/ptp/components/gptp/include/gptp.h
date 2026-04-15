/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file gptp.h
 * @brief gPTP (IEEE 802.1AS) daemon public API
 *
 * Provides a gPTP daemon that implements IEEE 802.1AS over raw L2 Ethernet
 * using the ESP-IDF L2TAP VFS interface. Supports both master and slave roles
 * with hardware timestamps via DM9058 PTP engine.
 *
 * Key differences from standard PTP (ptpd):
 *  - L2 transport only (multicast MAC 01:80:C2:00:00:0E)
 *  - P2P (Peer-to-Peer) delay mechanism mandatory
 *  - transportSpecific = 0x1 (SdoId)
 *  - 125ms Sync interval, 250ms PDelay interval
 *  - PDelayReq/PDelayResp/PDelayRespFollowUp messages
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to a running gPTP daemon instance.
 */
typedef struct gptp_state_s *gptp_handle_t;

/**
 * @brief gPTP daemon status snapshot.
 */
typedef struct {
    /** True if this node is currently locked to a remote master clock */
    bool clock_source_valid;

    /** True if this node is currently acting as gPTP master */
    bool is_master;

    /** Latest measured clock offset from master (nanoseconds, signed).
     *  Positive = local clock is ahead of master. */
    int64_t offset_ns;

    /** Latest measured peer mean path delay (nanoseconds).
     *  Average one-way link delay to the adjacent gPTP node. */
    int64_t path_delay_ns;

    /** Estimated clock drift rate (parts per billion).
     *  Positive = remote clock runs faster than local before adjustment. */
    long drift_ppb;

    /** Monotonic timestamp of last received Sync message */
    struct timespec last_received_sync;

    /** Monotonic timestamp of last received Announce message */
    struct timespec last_received_announce;

    /** Monotonic timestamp of last PDelayResp received */
    struct timespec last_pdelay_resp;

    /** Monotonic timestamp of last transmitted Sync (master mode) */
    struct timespec last_transmitted_sync;

    /** Identity of current best master clock (8-byte EUI-64) */
    uint8_t master_identity[8];
} gptp_status_t;

/**
 * @brief Start the gPTP daemon and bind it to the specified network interface.
 *
 * Creates a FreeRTOS task that runs the gPTP state machine, including:
 *  - PDelay Requester: sends PDelayReq every 250ms, computes peer path delay
 *  - PDelay Responder: responds to any PDelayReq received (regardless of role)
 *  - Sync/FollowUp: master sends every 125ms; slave receives and adjusts clock
 *  - Announce/BMCA: runs best-master clock algorithm; determines MASTER/SLAVE role
 *
 * @param[in]  interface  Network interface name (e.g. "ETH_0")
 * @param[out] handle     Returned handle for use with gptp_stop/gptp_get_status
 *
 * @return
 *   - ESP_OK on success
 *   - ESP_ERR_NO_MEM if memory allocation fails
 *   - ESP_FAIL if socket or hardware initialization fails
 */
esp_err_t gptp_start(const char *interface, gptp_handle_t *handle);

/**
 * @brief Stop a running gPTP daemon.
 *
 * Signals the gPTP task to stop and waits for it to exit.
 * Releases all resources (socket, MAC filters, task stack).
 *
 * @param[in] handle  Handle returned by gptp_start()
 *
 * @return
 *   - ESP_OK on success
 *   - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t gptp_stop(gptp_handle_t handle);

/**
 * @brief Query current status from a running gPTP daemon.
 *
 * Thread-safe. Can be called from any task.
 *
 * @param[in]  handle  Handle returned by gptp_start()
 * @param[out] status  Filled with current daemon status
 *
 * @return
 *   - ESP_OK on success
 *   - ESP_ERR_INVALID_ARG if handle or status is NULL
 *   - ESP_ERR_TIMEOUT if daemon is not responding
 */
esp_err_t gptp_get_status(gptp_handle_t handle, gptp_status_t *status);

#ifdef __cplusplus
}
#endif
