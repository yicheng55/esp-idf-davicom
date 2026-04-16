/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "gptp_defs.h"

/* HAL */
void       getTime(TimeInternal *time);
void       setTime(const TimeInternal *time);
Boolean    adjFreq(Integer32 adj_ppb);
UInteger32 getRand(UInteger32 upperBound);
UInteger32 safeGetRand(UInteger32 upperBound);

/* Servo */
void initClock(PtpClock *ptpClock);
void updateOffset(PtpClock *ptpClock,
                  const TimeInternal *syncEventIngressTimestamp,
                  const TimeInternal *preciseOriginTimestamp,
                  const TimeInternal *correctionField);
void updateDelay(PtpClock *ptpClock,
                 const TimeInternal *delayEventEgressTimestamp,
                 const TimeInternal *receiveTimestamp,
                 const TimeInternal *correctionField);
void updatePeerDelay(PtpClock *ptpClock,
                     const TimeInternal *correctionField,
                     Boolean twoStep);
void updateClock(PtpClock *ptpClock);
