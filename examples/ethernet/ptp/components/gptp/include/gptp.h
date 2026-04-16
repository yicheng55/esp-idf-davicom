/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public API for gPTP (IEEE 802.1AS) daemon component.
 *
 * Usage:
 *   gptp_config_t cfg = GPTP_DEFAULT_CONFIG("ETH_DEF");
 *   gptp_start(&cfg);
 *   ...
 *   gptp_status_t st;
 *   gptp_status(&st);
 *   ...
 *   gptp_stop();
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------
 *  gPTP configuration
 * -------------------------------------------------------- */
typedef struct {
    const char *ifaceName;         /**< Ethernet interface name, e.g. "ETH_DEF" */
    int         domainNumber;      /**< PTP domain (default 0) */
    uint8_t     priority1;         /**< Priority1 for BMCA (default 128) */
    uint8_t     priority2;         /**< Priority2 for BMCA (default 128) */
    uint8_t     clockClass;        /**< Clock class (default 248 = slave-only capable) */
    int8_t      syncInterval;      /**< log2(sync interval) default -3 = 125 ms */
    int8_t      announceInterval;  /**< log2(announce interval) default 0 = 1 s */
    bool        slaveOnly;         /**< Force slave-only mode (default false) */
    bool        twoStepFlag;       /**< Use two-step mode for sync (default false) */
    int         stackSize;         /**< FreeRTOS task stack size in bytes (default 8192) */
    int         taskPriority;      /**< FreeRTOS task priority (default 10) */
} gptp_config_t;

/** Default configuration initializer */
#define GPTP_DEFAULT_CONFIG(_iface) {   \
    .ifaceName       = (_iface),        \
    .domainNumber    = 0,               \
    .priority1       = 128,             \
    .priority2       = 128,             \
    .clockClass      = 248,             \
    .syncInterval    = -3,              \
    .announceInterval = 0,              \
    .slaveOnly       = false,           \
    .twoStepFlag     = false,           \
    .stackSize       = 8192,            \
    .taskPriority    = 10,              \
}

/* --------------------------------------------------------
 *  gPTP runtime status
 * -------------------------------------------------------- */
typedef enum {
    GPTP_STATE_INITIALIZING = 1,
    GPTP_STATE_FAULTY       = 2,
    GPTP_STATE_DISABLED     = 3,
    GPTP_STATE_LISTENING    = 4,
    GPTP_STATE_PRE_MASTER   = 5,
    GPTP_STATE_MASTER       = 6,
    GPTP_STATE_PASSIVE      = 7,
    GPTP_STATE_UNCALIBRATED = 8,
    GPTP_STATE_SLAVE        = 9,
} gptp_port_state_t;

typedef struct {
    gptp_port_state_t portState;       /**< Current port state */
    int64_t  offsetFromMaster_ns;      /**< Offset from master (nanoseconds) */
    int64_t  peerMeanPathDelay_ns;     /**< Mean path delay to peer (nanoseconds) */
    uint8_t  grandmasterIdentity[8];   /**< Grandmaster clock identity (EUI-64) */
    uint8_t  clockIdentity[8];         /**< Our clock identity (EUI-64) */
    bool     isSlave;                  /**< True when in SLAVE state */
    bool     isMaster;                 /**< True when in MASTER state */
} gptp_status_t;

/* --------------------------------------------------------
 *  Public API
 * -------------------------------------------------------- */

/**
 * @brief Start the gPTP daemon.
 *
 * Creates a FreeRTOS task that runs the gPTP state machine.
 * Must only be called once. Call gptp_stop() before re-starting.
 *
 * @param config  Pointer to configuration (copied internally, may be stack-allocated).
 * @return        0 on success, negative errno on failure.
 */
int gptp_start(const gptp_config_t *config);

/**
 * @brief Query current gPTP status.
 *
 * Thread-safe snapshot of the daemon's runtime state.
 *
 * @param[out] status  Filled with current state on success.
 * @return             0 on success, -1 if daemon is not running.
 */
int gptp_status(gptp_status_t *status);

/**
 * @brief Stop the gPTP daemon.
 *
 * Signals the FreeRTOS task to shut down and waits for it to finish.
 *
 * @return  0 on success, -1 if daemon was not running.
 */
int gptp_stop(void);

#ifdef __cplusplus
}
#endif
