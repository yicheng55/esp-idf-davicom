/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * gPTP arithmetic + message pack/unpack declarations.
 */

#pragma once
#include "gptp_defs.h"

/* Time arithmetic */
void     scaledNanosecondsToInternalTime(const Integer64 *scaledNanoseconds, TimeInternal *internal);
void     fromInternalTime(const TimeInternal *internal, Timestamp *external);
void     toInternalTime(TimeInternal *internal, const Timestamp *external);
void     normalizeTime(TimeInternal *r);
void     addTime(TimeInternal *r, const TimeInternal *x, const TimeInternal *y);
void     subTime(TimeInternal *r, const TimeInternal *x, const TimeInternal *y);
void     div2Time(TimeInternal *r);
Integer32 floorLog2(UInteger32 n);

static inline Integer32 gptp_max(Integer32 a, Integer32 b) { return a > b ? a : b; }
static inline Integer32 gptp_min(Integer32 a, Integer32 b) { return a < b ? a : b; }
/* Override min/max used in servo.c */
#define max(a, b) gptp_max(a, b)
#define min(a, b) gptp_min(a, b)

/* Message pack */
void msgPackHeader(const PtpClock *ptpClock, Octet *buf);
void msgPackAnnounce(const PtpClock *ptpClock, Octet *buf);
void msgPackSync(const PtpClock *ptpClock, Octet *buf, const Timestamp *originTimestamp);
void msgPackFollowUp(const PtpClock *ptpClock, Octet *buf, const Timestamp *preciseOriginTimestamp);
void msgPackDelayReq(const PtpClock *ptpClock, Octet *buf, const Timestamp *originTimestamp);
void msgPackDelayResp(const PtpClock *ptpClock, Octet *buf, const MsgHeader *header, const Timestamp *receiveTimestamp);
void msgPackPDelayReq(const PtpClock *ptpClock, Octet *buf, const Timestamp *originTimestamp);
void msgPackPDelayResp(Octet *buf, const MsgHeader *header, const Timestamp *requestReceiptTimestamp);
void msgPackPDelayRespFollowUp(Octet *buf, const MsgHeader *header, const Timestamp *responseOriginTimestamp);

/* Message unpack */
void msgUnpackHeader(const Octet *buf, MsgHeader *header);
void msgUnpackAnnounce(const Octet *buf, MsgAnnounce *announce);
void msgUnpackSync(const Octet *buf, MsgSync *sync);
void msgUnpackFollowUp(const Octet *buf, MsgFollowUp *follow);
void msgUnpackDelayReq(const Octet *buf, MsgDelayReq *delayreq);
void msgUnpackDelayResp(const Octet *buf, MsgDelayResp *resp);
void msgUnpackPDelayReq(const Octet *buf, MsgPDelayReq *pdelayreq);
void msgUnpackPDelayResp(const Octet *buf, MsgPDelayResp *presp);
void msgUnpackPDelayRespFollowUp(const Octet *buf, MsgPDelayRespFollowUp *prespfollow);
