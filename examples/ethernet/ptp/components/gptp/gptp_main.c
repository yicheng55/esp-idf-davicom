/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * gPTP (IEEE 802.1AS) daemon – FreeRTOS task + public API.
 *
 * Architecture:
 *   gptp_start()  → creates FreeRTOS task → gptp_task()
 *   gptp_task()   → PTPd_Init() → main loop
 *   Main loop:    catchAlarm(tick_ms)
 *                 → gptp_net_poll_and_queue()   (drain L2TAP fd into queues)
 *                 → doState()                   (protocol state machine)
 *
 * The loop runs at ~10 ms per iteration (100 Hz), which is fine for
 * gPTP Sync at 125 ms and PDelay at 250 ms.
 */

#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "gptp_defs.h"
#include "gptp_protocol.h"
#include "gptp_bmc.h"
#include "gptp_timer.h"
#include "gptp_servo.h"
#include "gptp_net.h"
#include "gptp_arith.h"
#include "gptp.h"

/* ============================================================
 *  Internal state
 * ============================================================ */

#define GPTP_TASK_NAME        "gptp_task"
#define GPTP_LOOP_PERIOD_MS   10u          /* main loop interval: 10 ms */

/* Maximum foreign master records (statically allocated) */
#define GPTP_MAX_FOREIGN_RECORDS  5

static PtpClock          s_ptpClock;
static RunTimeOpts        s_rtOpts;
static ForeignMasterRecord s_foreignRecords[GPTP_MAX_FOREIGN_RECORDS];

static TaskHandle_t       s_task_handle    = NULL;
static volatile bool      s_running        = false;
static SemaphoreHandle_t  s_status_mutex   = NULL;

/* Snapshot of status for gptp_status() */
static gptp_status_t      s_status_cache;

/* ============================================================
 *  Internal helpers
 * ============================================================ */

static void update_status_cache(void)
{
    if (!s_status_mutex) return;
    if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return;

    PtpClock *clk = &s_ptpClock;
    s_status_cache.portState          = (gptp_port_state_t)clk->portDS.portState;
    s_status_cache.offsetFromMaster_ns =
        (int64_t)clk->currentDS.offsetFromMaster.seconds * 1000000000LL
        + clk->currentDS.offsetFromMaster.nanoseconds;
    s_status_cache.peerMeanPathDelay_ns =
        (int64_t)clk->portDS.peerMeanPathDelay.seconds * 1000000000LL
        + clk->portDS.peerMeanPathDelay.nanoseconds;
    memcpy(s_status_cache.grandmasterIdentity, clk->parentDS.grandmasterIdentity,
           CLOCK_IDENTITY_LENGTH);
    memcpy(s_status_cache.clockIdentity, clk->defaultDS.clockIdentity,
           CLOCK_IDENTITY_LENGTH);
    s_status_cache.isSlave  = (clk->portDS.portState == PTP_SLAVE);
    s_status_cache.isMaster = (clk->portDS.portState == PTP_MASTER);

    xSemaphoreGive(s_status_mutex);
}

/* Build RunTimeOpts from user-supplied gptp_config_t */
static void build_rtOpts(const gptp_config_t *cfg)
{
    memset(&s_rtOpts, 0, sizeof(s_rtOpts));

    /* Interface */
    strncpy(s_rtOpts.ifaceName, cfg->ifaceName, IFACE_NAME_LENGTH - 1);

    /* Domain / priorities */
    s_rtOpts.domainNumber = (UInteger8)cfg->domainNumber;
    s_rtOpts.priority1    = cfg->priority1;
    s_rtOpts.priority2    = cfg->priority2;
    s_rtOpts.slaveOnly    = cfg->slaveOnly;

    /* Clock quality */
    s_rtOpts.clockQuality.clockClass    = cfg->clockClass;
    s_rtOpts.clockQuality.clockAccuracy = DEFAULT_CLOCK_ACCURACY;
    s_rtOpts.clockQuality.offsetScaledLogVariance = DEFAULT_CLOCK_VARIANCE;

    /* Intervals (802.1AS defaults) */
    s_rtOpts.syncInterval     = cfg->syncInterval;
    s_rtOpts.announceInterval = cfg->announceInterval;

    /* Forced 802.1AS profile */
    s_rtOpts.delayMechanism  = P2P;
    s_rtOpts.transportType   = TRANSPORT_IEEE_802_1AS;

    /* UTC offset */
    s_rtOpts.currentUtcOffset = DEFAULT_UTC_OFFSET;

    /* Foreign records limit */
    s_rtOpts.maxForeignRecords = GPTP_MAX_FOREIGN_RECORDS;

    /* PI servo defaults */
    s_rtOpts.servo.ap           = DEFAULT_AP;
    s_rtOpts.servo.ai           = DEFAULT_AI;
    s_rtOpts.servo.sDelay       = DEFAULT_DELAY_S;
    s_rtOpts.servo.sOffset      = DEFAULT_OFFSET_S;
    s_rtOpts.servo.noAdjust     = NO_ADJUST;
    s_rtOpts.servo.noResetClock = DEFAULT_NO_RESET_CLOCK;
}

/* Initialise PtpClock – equivalent of PTPd_Init() */
static bool gptp_clock_init(const gptp_config_t *cfg)
{
    memset(&s_ptpClock, 0, sizeof(s_ptpClock));

    /* Wire foreign-master records */
    s_ptpClock.foreignMasterDS.records  = s_foreignRecords;
    s_ptpClock.foreignMasterDS.capacity = GPTP_MAX_FOREIGN_RECORDS;
    s_ptpClock.rtOpts                   = &s_rtOpts;

    /* twoStepFlag from config */
    s_ptpClock.defaultDS.twoStepFlag = cfg->twoStepFlag;

    initTimer();

    /* Network init – opens L2TAP fd, sets MAC filter, retrieves hw addr */
    if (!netInit(&s_ptpClock.netPath, &s_ptpClock)) {
        ESP_LOGE(GPTP_TAG, "gptp_clock_init: netInit failed");
        return false;
    }

    /* Data-set init – derives clock identity from MAC */
    initData(&s_ptpClock);

    /* PI servo / HAL init */
    initClock(&s_ptpClock);

    /* Kick off state machine */
    toState(&s_ptpClock, PTP_INITIALIZING);

    ESP_LOGI(GPTP_TAG, "gPTP init OK, clockId=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
             s_ptpClock.defaultDS.clockIdentity[0], s_ptpClock.defaultDS.clockIdentity[1],
             s_ptpClock.defaultDS.clockIdentity[2], s_ptpClock.defaultDS.clockIdentity[3],
             s_ptpClock.defaultDS.clockIdentity[4], s_ptpClock.defaultDS.clockIdentity[5],
             s_ptpClock.defaultDS.clockIdentity[6], s_ptpClock.defaultDS.clockIdentity[7]);

    return true;
}

/* ============================================================
 *  FreeRTOS task
 * ============================================================ */
static void gptp_task(void *arg)
{
    const gptp_config_t *cfg = (const gptp_config_t *)arg;

    ESP_LOGI(GPTP_TAG, "gPTP task started, iface=%s", cfg->ifaceName);

    if (!gptp_clock_init(cfg)) {
        ESP_LOGE(GPTP_TAG, "gPTP init failed – task exiting");
        s_running = false;
        vTaskDelete(NULL);
        return;
    }

    /* Enter LISTENING state to start the state machine */
    toState(&s_ptpClock, PTP_LISTENING);

    TickType_t last_wake = xTaskGetTickCount();

    while (s_running) {
        /* Advance software timers by one loop period */
        catchAlarm(GPTP_LOOP_PERIOD_MS);

        /* Drain all available frames from the L2TAP fd into queues */
        gptp_net_poll_and_queue(&s_ptpClock.netPath);

        /* Run one step of the protocol state machine */
        doState(&s_ptpClock);

        /* Refresh public status snapshot */
        update_status_cache();

        /* Sleep until next period */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(GPTP_LOOP_PERIOD_MS));
    }

    /* Cleanup */
    netShutdown(&s_ptpClock.netPath);
    ESP_LOGI(GPTP_TAG, "gPTP task stopped");

    s_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ============================================================
 *  Public API
 * ============================================================ */
int gptp_start(const gptp_config_t *config)
{
    if (!config || !config->ifaceName) {
        return -EINVAL;
    }

    if (s_running) {
        ESP_LOGW(GPTP_TAG, "gptp_start: already running");
        return -EALREADY;
    }

    /* Build runtime options */
    build_rtOpts(config);

    /* Create status mutex */
    if (!s_status_mutex) {
        s_status_mutex = xSemaphoreCreateMutex();
        if (!s_status_mutex) {
            ESP_LOGE(GPTP_TAG, "gptp_start: failed to create mutex");
            return -ENOMEM;
        }
    }

    memset(&s_status_cache, 0, sizeof(s_status_cache));
    s_status_cache.portState = GPTP_STATE_INITIALIZING;

    s_running = true;

    BaseType_t ret = xTaskCreate(
        gptp_task,
        GPTP_TASK_NAME,
        (uint32_t)config->stackSize,
        (void *)config,          /* config is in caller's scope – safe if static */
        (UBaseType_t)config->taskPriority,
        &s_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(GPTP_TAG, "gptp_start: xTaskCreate failed");
        s_running = false;
        return -ENOMEM;
    }

    ESP_LOGI(GPTP_TAG, "gPTP daemon started (iface=%s, prio1=%u, domain=%u)",
             config->ifaceName, config->priority1, config->domainNumber);
    return 0;
}

int gptp_status(gptp_status_t *out)
{
    if (!out) return -EINVAL;
    if (!s_running || !s_status_mutex) {
        memset(out, 0, sizeof(*out));
        return -1;
    }

    if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return -1;
    }
    *out = s_status_cache;
    xSemaphoreGive(s_status_mutex);
    return 0;
}

int gptp_stop(void)
{
    if (!s_running) {
        return -1;
    }

    s_running = false;

    /* Wait for the task to exit (up to 2 seconds) */
    for (int i = 0; i < 200 && s_task_handle != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (s_status_mutex) {
        vSemaphoreDelete(s_status_mutex);
        s_status_mutex = NULL;
    }

    ESP_LOGI(GPTP_TAG, "gPTP daemon stopped");
    return 0;
}
