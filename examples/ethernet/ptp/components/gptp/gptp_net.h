/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "gptp_defs.h"

/* Called from gPTP task loop to drain the L2TAP fd into queues */
void      gptp_net_poll_and_queue(NetPath *netPath);

/* Standard ptpd network API */
Boolean   netInit(NetPath *netPath, PtpClock *ptpClock);
Boolean   netShutdown(NetPath *netPath);
Integer32 netSelect(NetPath *netPath, const TimeInternal *timeout);
ssize_t   netRecvEvent(NetPath *netPath, Octet *buf, TimeInternal *time);
ssize_t   netRecvGeneral(NetPath *netPath, Octet *buf, TimeInternal *time);
ssize_t   netSendEvent(NetPath *netPath, const Octet *buf, UInteger16 length, TimeInternal *time);
ssize_t   netSendGeneral(NetPath *netPath, const Octet *buf, UInteger16 length);
ssize_t   netSendPeerEvent(NetPath *netPath, const Octet *buf, UInteger16 length, TimeInternal *time);
ssize_t   netSendPeerGeneral(NetPath *netPath, const Octet *buf, UInteger16 length);
void      netEmptyEventQ(NetPath *netPath);
