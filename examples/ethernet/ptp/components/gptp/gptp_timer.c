/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * gPTP software timer management.
 * Ported from ptpd-2.0.0 dep/timer.c, adapted for FreeRTOS/ESP-IDF.
 *
 * The timer is driven by calling catchAlarm(delta_ms) from the gPTP task
 * loop each iteration.  timerExpired() then checks whether any interval
 * has elapsed.
 */

#include "gptp_defs.h"
#include "gptp_timer.h"

static volatile uint32_t s_elapsed_ms = 0;

void catchAlarm(UInteger32 time_ms)
{
    s_elapsed_ms += time_ms;
    DBGVV("catchAlarm: elapsed %u", (unsigned)s_elapsed_ms);
}

void initTimer(void)
{
    DBG("initTimer");
    s_elapsed_ms = 0;
}

void timerUpdate(IntervalTimer *itimer)
{
    int     i;
    int32_t delta = (int32_t)s_elapsed_ms;
    s_elapsed_ms  = 0;

    if (delta <= 0) return;

    for (i = 0; i < TIMER_ARRAY_SIZE; i++) {
        if (itimer[i].interval > 0) {
            itimer[i].left -= delta;
            if (itimer[i].left <= 0) {
                itimer[i].left   = itimer[i].interval;
                itimer[i].expire = TRUE;
                DBGV("timerUpdate: timer %d expired", i);
            }
        }
    }
}

void timerStop(UInteger16 index, IntervalTimer *itimer)
{
    if (index >= TIMER_ARRAY_SIZE) return;
    itimer[index].interval = 0;
    itimer[index].expire   = FALSE;
}

void timerStart(UInteger16 index, UInteger32 interval_ms, IntervalTimer *itimer)
{
    if (index >= TIMER_ARRAY_SIZE) return;
    itimer[index].expire   = FALSE;
    itimer[index].left     = (Integer32)interval_ms;
    itimer[index].interval = (Integer32)interval_ms;
    DBGV("timerStart: timer %d set to %u ms", (int)index, (unsigned)interval_ms);
}

Boolean timerExpired(UInteger16 index, IntervalTimer *itimer)
{
    timerUpdate(itimer);
    if (index >= TIMER_ARRAY_SIZE) return FALSE;
    if (!itimer[index].expire)     return FALSE;
    itimer[index].expire = FALSE;
    return TRUE;
}
