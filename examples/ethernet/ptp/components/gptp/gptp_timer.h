/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "gptp_defs.h"

void    catchAlarm(UInteger32 time_ms);
void    initTimer(void);
void    timerUpdate(IntervalTimer *itimer);
void    timerStop(UInteger16 index, IntervalTimer *itimer);
void    timerStart(UInteger16 index, UInteger32 interval_ms, IntervalTimer *itimer);
Boolean timerExpired(UInteger16 index, IntervalTimer *itimer);
