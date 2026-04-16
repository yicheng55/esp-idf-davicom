/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "gptp_defs.h"

/* State machine public API */
void toState(PtpClock *ptpClock, UInteger8 state);
void doState(PtpClock *ptpClock);
