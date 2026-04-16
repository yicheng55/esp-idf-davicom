/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "gptp_defs.h"

void     EUI48toEUI64(const Octet *eui48, Octet *eui64);
void     initData(PtpClock *ptpClock);
Boolean  isSamePortIdentity(const PortIdentity *A, const PortIdentity *B);
void     addForeign(PtpClock *ptpClock, const MsgHeader *header, const MsgAnnounce *announce);
void     m1(PtpClock *ptpClock);
void     p1(PtpClock *ptpClock);
void     s1(PtpClock *ptpClock, const MsgHeader *header, const MsgAnnounce *announce);
void     copyD0(MsgHeader *header, MsgAnnounce *announce, PtpClock *ptpClock);
Integer8 bmcDataSetComparison(MsgHeader *headerA, MsgAnnounce *announceA,
                              MsgHeader *headerB, MsgAnnounce *announceB,
                              PtpClock  *ptpClock);
UInteger8 bmcStateDecision(MsgHeader *header, MsgAnnounce *announce, PtpClock *ptpClock);
UInteger8 bmc(PtpClock *ptpClock);
