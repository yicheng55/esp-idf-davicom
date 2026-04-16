/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * gPTP clock servo (PI controller) and HAL functions.
 * Ported from ptpd-2.0.0 dep/servo.c + dep/sys_time.c,
 * adapted for ESP-IDF esp_eth_clock_* API.
 */

#include <time.h>
#include "gptp_defs.h"
#include "gptp_arith.h"
#include "gptp_timer.h"
#include "gptp_servo.h"
#include "gptp_net.h"
#include "esp_eth_time.h"

/* ============================================================
 *  HAL – clock access via esp_eth_clock_*
 * ============================================================ */

void getTime(TimeInternal *time)
{
    struct timespec ts = {0, 0};
    esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &ts);
    time->seconds     = (Integer32)ts.tv_sec;
    time->nanoseconds = (Integer32)ts.tv_nsec;
}

void setTime(const TimeInternal *time)
{
    struct timespec ts = {
        .tv_sec  = (time_t)time->seconds,
        .tv_nsec = (long)  time->nanoseconds,
    };
    esp_eth_clock_settime(CLOCK_PTP_SYSTEM, &ts);
    ESP_LOGI(GPTP_TAG, "setTime: %d.%09d", time->seconds, time->nanoseconds);
}

Boolean adjFreq(Integer32 adj_ppb)
{
    /* Clamp to ±ADJ_FREQ_MAX */
    if (adj_ppb >  ADJ_FREQ_MAX) adj_ppb =  ADJ_FREQ_MAX;
    if (adj_ppb < -ADJ_FREQ_MAX) adj_ppb = -ADJ_FREQ_MAX;

    /* esp_eth_clock_adjtime uses a frequency scale ratio:
     *   freq_scale = 1.0 + ppb / 1e9
     * e.g. +500 ppb → freq_scale = 1.0000005 (run 0.05 ppm fast) */
    esp_eth_clock_adj_param_t param = {
        .mode       = ETH_CLK_ADJ_FREQ_SCALE,
        .freq_scale = 1.0 + (double)adj_ppb / 1e9,
    };
    int ret = esp_eth_clock_adjtime(CLOCK_PTP_SYSTEM, &param);
    return (ret == 0) ? TRUE : FALSE;
}

UInteger32 getRand(UInteger32 upperBound)
{
    if (upperBound == 0) return 0;
    return (UInteger32)(rand() % (int)upperBound);
}

UInteger32 safeGetRand(UInteger32 upperBound)
{
    if (upperBound == 0) return 0;
    return (UInteger32)(rand() % (int)upperBound);
}

/* ============================================================
 *  Exponential smoothing filter (from servo.c)
 * ============================================================ */
static void filter(Integer32 *nsec_current, Filter *filt)
{
    Integer32 s, s2;

    filt->n++;

    if (filt->n == 1) {
        filt->y_prev = *nsec_current;
        filt->y_sum  = *nsec_current;
        filt->s_prev = 0;
    }

    s = filt->s;
    if ((1 << s) > filt->n) {
        s = floorLog2((UInteger32)filt->n);
    } else {
        filt->n = 1 << s;
    }

    s2 = 30 - floorLog2((UInteger32)max(abs(filt->y_prev), abs(*nsec_current)));
    s  = min(s, s2);

    if (filt->s_prev > s)      filt->y_sum >>= (filt->s_prev - s);
    else if (filt->s_prev < s) filt->y_sum <<= (s - filt->s_prev);

    filt->y_sum  += *nsec_current - filt->y_prev;
    filt->y_prev  = filt->y_sum >> s;
    filt->s_prev  = s;

    *nsec_current = filt->y_prev;
}

/* ============================================================
 *  Clock servo init
 * ============================================================ */
void initClock(PtpClock *ptpClock)
{
    DBG("initClock");

    ptpClock->Tms.seconds = ptpClock->Tms.nanoseconds = 0;
    ptpClock->observedDrift = 0;

    ptpClock->owd_filt.n = 0;
    ptpClock->owd_filt.s = ptpClock->servo.sDelay;

    ptpClock->ofm_filt.n = 0;
    ptpClock->ofm_filt.s = ptpClock->servo.sOffset;

    if (DEFAULT_PARENTS_STATS) {
        ptpClock->slv_filt.n = 0;
        ptpClock->slv_filt.s = 6;
        ptpClock->offsetHistory[0] = 0;
        ptpClock->offsetHistory[1] = 0;
    }

    ptpClock->waitingForFollowUp           = FALSE;
    ptpClock->waitingForPDelayRespFollowUp = FALSE;

    ptpClock->pdelay_t1.seconds = ptpClock->pdelay_t1.nanoseconds = 0;
    ptpClock->pdelay_t2.seconds = ptpClock->pdelay_t2.nanoseconds = 0;
    ptpClock->pdelay_t3.seconds = ptpClock->pdelay_t3.nanoseconds = 0;
    ptpClock->pdelay_t4.seconds = ptpClock->pdelay_t4.nanoseconds = 0;

    ptpClock->parentDS.parentStats = FALSE;
    ptpClock->parentDS.observedParentClockPhaseChangeRate = 0;
    ptpClock->parentDS.observedParentOffsetScaledLogVariance = 0;

    if (!ptpClock->servo.noAdjust)
        adjFreq(0);

    netEmptyEventQ(&ptpClock->netPath);
}

/* ============================================================
 *  Offset update (11.2) – called after Sync/FollowUp
 * ============================================================ */
void updateOffset(PtpClock *ptpClock,
                  const TimeInternal *syncEventIngressTimestamp,
                  const TimeInternal *preciseOriginTimestamp,
                  const TimeInternal *correctionField)
{
    DBGV("updateOffset");

    subTime(&ptpClock->Tms, syncEventIngressTimestamp, preciseOriginTimestamp);
    DBG_PRINTF("T2-T1 (before CF): %ds %+010dns",
               ptpClock->Tms.seconds, ptpClock->Tms.nanoseconds);

    subTime(&ptpClock->Tms, &ptpClock->Tms, correctionField);
    ptpClock->currentDS.offsetFromMaster = ptpClock->Tms;

    /* P2P: subtract peerMeanPathDelay */
    switch (ptpClock->portDS.delayMechanism) {
    case E2E:
        subTime(&ptpClock->currentDS.offsetFromMaster,
                &ptpClock->currentDS.offsetFromMaster,
                &ptpClock->currentDS.meanPathDelay);
        break;
    case P2P:
        subTime(&ptpClock->currentDS.offsetFromMaster,
                &ptpClock->currentDS.offsetFromMaster,
                &ptpClock->portDS.peerMeanPathDelay);
        break;
    default:
        break;
    }

    DBG_PRINTF("offsetFromMaster: %ds %+010dns",
               ptpClock->currentDS.offsetFromMaster.seconds,
               ptpClock->currentDS.offsetFromMaster.nanoseconds);

    if (ptpClock->currentDS.offsetFromMaster.seconds != 0) {
        if (ptpClock->portDS.portState == PTP_SLAVE)
            setFlag(ptpClock->events, SYNCHRONIZATION_FAULT);
        return;
    }

    filter(&ptpClock->currentDS.offsetFromMaster.nanoseconds, &ptpClock->ofm_filt);

    int32_t absOff = abs(ptpClock->currentDS.offsetFromMaster.nanoseconds);
    if (absOff < DEFAULT_CALIBRATED_OFFSET_NS) {
        if (ptpClock->portDS.portState == PTP_UNCALIBRATED)
            setFlag(ptpClock->events, MASTER_CLOCK_SELECTED);
    } else if (absOff > DEFAULT_UNCALIBRATED_OFFSET_NS) {
        if (ptpClock->portDS.portState == PTP_SLAVE)
            setFlag(ptpClock->events, SYNCHRONIZATION_FAULT);
    }
}

/* ============================================================
 *  E2E delay update (11.3) – called after DelayResp
 * ============================================================ */
void updateDelay(PtpClock *ptpClock,
                 const TimeInternal *delayEventEgressTimestamp,
                 const TimeInternal *receiveTimestamp,
                 const TimeInternal *correctionField)
{
    if (ptpClock->ofm_filt.n == 0) {
        DBGV("updateDelay: Tms not yet valid");
        return;
    }

    subTime(&ptpClock->Tsm, receiveTimestamp, delayEventEgressTimestamp);
    subTime(&ptpClock->Tsm, &ptpClock->Tsm, correctionField);

    addTime(&ptpClock->currentDS.meanPathDelay, &ptpClock->Tms, &ptpClock->Tsm);
    div2Time(&ptpClock->currentDS.meanPathDelay);

    if (ptpClock->currentDS.meanPathDelay.seconds == 0) {
        filter(&ptpClock->currentDS.meanPathDelay.nanoseconds, &ptpClock->owd_filt);
    }

    DBG_PRINTF("E2E meanPathDelay: %ds %dns",
               ptpClock->currentDS.meanPathDelay.seconds,
               ptpClock->currentDS.meanPathDelay.nanoseconds);
}

/* ============================================================
 *  P2P peer delay update – called after PDelayResp[FollowUp]
 * ============================================================ */
void updatePeerDelay(PtpClock *ptpClock,
                     const TimeInternal *correctionField,
                     Boolean twoStep)
{
    DBGV("updatePeerDelay: twoStep=%d", (int)twoStep);

    if (twoStep) {
        TimeInternal Tab, Tba;
        subTime(&Tab, &ptpClock->pdelay_t2, &ptpClock->pdelay_t1);
        subTime(&Tba, &ptpClock->pdelay_t4, &ptpClock->pdelay_t3);
        addTime(&ptpClock->portDS.peerMeanPathDelay, &Tab, &Tba);
    } else {
        /* ONE_STEP: correctionField already contains (t3-t2) from responder */
        subTime(&ptpClock->portDS.peerMeanPathDelay,
                &ptpClock->pdelay_t4, &ptpClock->pdelay_t1);
    }

    subTime(&ptpClock->portDS.peerMeanPathDelay,
            &ptpClock->portDS.peerMeanPathDelay, correctionField);
    div2Time(&ptpClock->portDS.peerMeanPathDelay);

    if (ptpClock->portDS.peerMeanPathDelay.seconds == 0) {
        filter(&ptpClock->portDS.peerMeanPathDelay.nanoseconds, &ptpClock->owd_filt);
    }

    ESP_LOGI(GPTP_TAG, "peerMeanPathDelay: %ds %dns",
             ptpClock->portDS.peerMeanPathDelay.seconds,
             ptpClock->portDS.peerMeanPathDelay.nanoseconds);
}

/* ============================================================
 *  PI servo: apply clock adjustment
 * ============================================================ */
void updateClock(PtpClock *ptpClock)
{
    Integer32 adj;
    TimeInternal timeTmp;
    Integer32    offsetNorm;
    Integer32    absOffset;

    DBGV("updateClock");

    absOffset = abs(ptpClock->currentDS.offsetFromMaster.nanoseconds);

    if (ptpClock->currentDS.offsetFromMaster.seconds != 0 || absOffset > MAX_ADJ_OFFSET_NS) {
        /* Large offset: jump the clock */
        if (!ptpClock->servo.noAdjust) {
            if (!ptpClock->servo.noResetClock) {
                getTime(&timeTmp);
                subTime(&timeTmp, &timeTmp, &ptpClock->currentDS.offsetFromMaster);
                setTime(&timeTmp);
                initClock(ptpClock);
            } else {
                adj = (ptpClock->currentDS.offsetFromMaster.nanoseconds > 0)
                      ? ADJ_FREQ_MAX : -ADJ_FREQ_MAX;
                adjFreq(-adj);
            }
        }
        return;
    }

    /* PI controller */
    offsetNorm = ptpClock->currentDS.offsetFromMaster.nanoseconds;
    if (ptpClock->portDS.logSyncInterval > 0)
        offsetNorm >>= ptpClock->portDS.logSyncInterval;
    else if (ptpClock->portDS.logSyncInterval < 0)
        offsetNorm <<= -ptpClock->portDS.logSyncInterval;

    ptpClock->observedDrift += offsetNorm / ptpClock->servo.ai;
    if (ptpClock->observedDrift >  ADJ_FREQ_MAX) ptpClock->observedDrift =  ADJ_FREQ_MAX;
    if (ptpClock->observedDrift < -ADJ_FREQ_MAX) ptpClock->observedDrift = -ADJ_FREQ_MAX;

    if (!ptpClock->servo.noAdjust) {
        adj = offsetNorm / ptpClock->servo.ap + ptpClock->observedDrift;
        adjFreq(-adj);
    }

    ESP_LOGI(GPTP_TAG,
             "updateClock: offset=%dns peerDelay=%dns drift=%d",
             ptpClock->currentDS.offsetFromMaster.nanoseconds,
             ptpClock->portDS.peerMeanPathDelay.nanoseconds,
             ptpClock->observedDrift);
}
