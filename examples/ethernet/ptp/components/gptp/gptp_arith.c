/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * gPTP arithmetic and message pack/unpack.
 * Ported from ptpd-2.0.0: arith.c + dep/msg.c
 */

#include "gptp_defs.h"
#include "gptp_arith.h"

/* ============================================================
 *  Time arithmetic (arith.c)
 * ============================================================ */

void scaledNanosecondsToInternalTime(const Integer64 *scaledNanoseconds, TimeInternal *internal)
{
    int     sign;
    int64_t ns = *scaledNanoseconds;

    if (ns < 0) { ns = -ns; sign = -1; }
    else         { sign =  1; }

    ns >>= 16; /* drop sub-nanosecond fraction */

    internal->seconds     = (Integer32)(sign * (ns / 1000000000LL));
    internal->nanoseconds = (Integer32)(sign * (ns % 1000000000LL));
}

void fromInternalTime(const TimeInternal *internal, Timestamp *external)
{
    if ((internal->seconds & ~INT_MAX) || (internal->nanoseconds & ~INT_MAX)) {
        DBGV("fromInternalTime: negative value cannot be converted to Timestamp");
        return;
    }
    external->secondsField.lsb  = (uint32_t)internal->seconds;
    external->secondsField.msb  = 0;
    external->nanosecondsField  = (uint32_t)internal->nanoseconds;
}

void toInternalTime(TimeInternal *internal, const Timestamp *external)
{
    if (external->secondsField.lsb < (uint32_t)INT_MAX) {
        internal->seconds     = (Integer32)external->secondsField.lsb;
        internal->nanoseconds = (Integer32)external->nanosecondsField;
    } else {
        DBGV("toInternalTime: seconds field overflow");
    }
}

void normalizeTime(TimeInternal *r)
{
    r->seconds     += r->nanoseconds / 1000000000;
    r->nanoseconds -= (r->nanoseconds / 1000000000) * 1000000000;

    if (r->seconds > 0 && r->nanoseconds < 0) {
        r->seconds--;
        r->nanoseconds += 1000000000;
    } else if (r->seconds < 0 && r->nanoseconds > 0) {
        r->seconds++;
        r->nanoseconds -= 1000000000;
    }
}

void addTime(TimeInternal *r, const TimeInternal *x, const TimeInternal *y)
{
    r->seconds     = x->seconds     + y->seconds;
    r->nanoseconds = x->nanoseconds + y->nanoseconds;
    normalizeTime(r);
}

void subTime(TimeInternal *r, const TimeInternal *x, const TimeInternal *y)
{
    r->seconds     = x->seconds     - y->seconds;
    r->nanoseconds = x->nanoseconds - y->nanoseconds;
    normalizeTime(r);
}

void div2Time(TimeInternal *r)
{
    r->nanoseconds += (r->seconds % 2) * 1000000000;
    r->seconds     /= 2;
    r->nanoseconds /= 2;
    normalizeTime(r);
}

Integer32 floorLog2(UInteger32 n)
{
    int pos = 0;
    if (n == 0) return -1;
    if (n >= (1U << 16)) { n >>= 16; pos += 16; }
    if (n >= (1U <<  8)) { n >>=  8; pos +=  8; }
    if (n >= (1U <<  4)) { n >>=  4; pos +=  4; }
    if (n >= (1U <<  2)) { n >>=  2; pos +=  2; }
    if (n >= (1U <<  1)) {           pos +=  1; }
    return pos;
}

/* ============================================================
 *  Message pack / unpack (dep/msg.c)
 * ============================================================ */

void msgUnpackHeader(const Octet *buf, MsgHeader *header)
{
    Integer32  msb;
    UInteger32 lsb;

    header->transportSpecific = (uint8_t)(((uint8_t)buf[0]) >> 4);
    header->messageType       = (uint8_t)(((uint8_t)buf[0]) & 0x0F);
    header->versionPTP        = (uint8_t)(((uint8_t)buf[1]) & 0x0F);
    header->messageLength     = flip16(*(const UInteger16 *)(buf + 2));
    header->domainNumber      = (uint8_t)buf[4];
    memcpy(header->flagField, buf + 6, FLAG_FIELD_LENGTH);

    memcpy(&msb, buf +  8, 4);
    memcpy(&lsb, buf + 12, 4);
    header->correctionfield  = (Integer64)flip32(msb);
    header->correctionfield <<= 32;
    header->correctionfield  += (Integer64)flip32(lsb);

    memcpy(header->sourcePortIdentity.clockIdentity, buf + 20, CLOCK_IDENTITY_LENGTH);
    header->sourcePortIdentity.portNumber = flip16(*(const UInteger16 *)(buf + 28));
    header->sequenceId        = flip16(*(const UInteger16 *)(buf + 30));
    header->controlField      = (uint8_t)buf[32];
    header->logMessageInterval = (int8_t) buf[33];
}

void msgPackHeader(const PtpClock *ptpClock, Octet *buf)
{
    Nibble transportSpecific = DEFAULT_TRANSPORT_SPECIFIC_1588;
    if (ptpClock && ptpClock->rtOpts &&
        ptpClock->rtOpts->transportType == TRANSPORT_IEEE_802_1AS) {
        transportSpecific = DEFAULT_TRANSPORT_SPECIFIC_8021AS;
    }
    buf[0] = (uint8_t)((transportSpecific & 0x0F) << 4);
    buf[1] = (uint8_t)(ptpClock->portDS.versionNumber & 0x0F);
    buf[4] = (uint8_t)ptpClock->defaultDS.domainNumber;

    if (ptpClock->defaultDS.twoStepFlag) {
        buf[6] = FLAG0_TWO_STEP;
    } else {
        buf[6] = 0;
    }
    buf[7] = 0;

    memset(buf + 8, 0, 8); /* correction field = 0 */

    memcpy(buf + 20, ptpClock->portDS.portIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
    *(UInteger16 *)(buf + 28) = flip16(ptpClock->portDS.portIdentity.portNumber);
    buf[33] = 0x7F; /* logMessageInterval: default (Table 24) */
}

void msgPackAnnounce(const PtpClock *ptpClock, Octet *buf)
{
    buf[0] = (buf[0] & 0xF0) | ANNOUNCE;
    *(UInteger16 *)(buf + 2)  = flip16(ANNOUNCE_LENGTH);
    *(UInteger16 *)(buf + 30) = flip16(ptpClock->sentAnnounceSequenceId);
    buf[32] = CTRL_OTHER;
    buf[33] = (int8_t)ptpClock->portDS.logAnnounceInterval;

    memset(buf + 34, 0, 10); /* originTimestamp */
    *(Integer16 *)(buf + 44)  = flip16(ptpClock->timePropertiesDS.currentUtcOffset);
    buf[47] = ptpClock->parentDS.grandmasterPriority1;
    buf[48] = ptpClock->defaultDS.clockQuality.clockClass;
    buf[49] = ptpClock->defaultDS.clockQuality.clockAccuracy;
    *(UInteger16 *)(buf + 50) = flip16(ptpClock->defaultDS.clockQuality.offsetScaledLogVariance);
    buf[52] = ptpClock->parentDS.grandmasterPriority2;
    memcpy(buf + 53, ptpClock->parentDS.grandmasterIdentity, CLOCK_IDENTITY_LENGTH);
    *(UInteger16 *)(buf + 61) = flip16(ptpClock->currentDS.stepsRemoved);
    buf[63] = ptpClock->timePropertiesDS.timeSource;
}

void msgUnpackAnnounce(const Octet *buf, MsgAnnounce *announce)
{
    announce->originTimestamp.secondsField.msb = flip16(*(const UInteger16 *)(buf + 34));
    announce->originTimestamp.secondsField.lsb = flip32(*(const UInteger32 *)(buf + 36));
    announce->originTimestamp.nanosecondsField  = flip32(*(const UInteger32 *)(buf + 40));
    announce->currentUtcOffset                  = flip16(*(const Integer16  *)(buf + 44));
    announce->grandmasterPriority1              = (uint8_t)buf[47];
    announce->grandmasterClockQuality.clockClass    = (uint8_t)buf[48];
    announce->grandmasterClockQuality.clockAccuracy = (uint8_t)buf[49];
    announce->grandmasterClockQuality.offsetScaledLogVariance = flip16(*(const UInteger16 *)(buf + 50));
    announce->grandmasterPriority2              = (uint8_t)buf[52];
    memcpy(announce->grandmasterIdentity, buf + 53, CLOCK_IDENTITY_LENGTH);
    announce->stepsRemoved = flip16(*(const UInteger16 *)(buf + 61));
    announce->timeSource   = (uint8_t)buf[63];
}

void msgPackSync(const PtpClock *ptpClock, Octet *buf, const Timestamp *originTimestamp)
{
    buf[0] = (buf[0] & 0xF0) | SYNC;
    *(UInteger16 *)(buf + 2)  = flip16(SYNC_LENGTH);
    *(UInteger16 *)(buf + 30) = flip16(ptpClock->sentSyncSequenceId);
    buf[32] = CTRL_SYNC;
    buf[33] = (int8_t)ptpClock->portDS.logSyncInterval;
    memset(buf + 8, 0, 8);

    *(UInteger16 *)(buf + 34) = flip16(originTimestamp->secondsField.msb);
    *(UInteger32 *)(buf + 36) = flip32(originTimestamp->secondsField.lsb);
    *(UInteger32 *)(buf + 40) = flip32(originTimestamp->nanosecondsField);
}

void msgUnpackSync(const Octet *buf, MsgSync *sync)
{
    sync->originTimestamp.secondsField.msb = flip16(*(const UInteger16 *)(buf + 34));
    sync->originTimestamp.secondsField.lsb = flip32(*(const UInteger32 *)(buf + 36));
    sync->originTimestamp.nanosecondsField  = flip32(*(const UInteger32 *)(buf + 40));
}

void msgPackFollowUp(const PtpClock *ptpClock, Octet *buf, const Timestamp *preciseOriginTimestamp)
{
    buf[0] = (buf[0] & 0xF0) | FOLLOW_UP;
    *(UInteger16 *)(buf + 2)  = flip16(FOLLOW_UP_LENGTH);
    *(UInteger16 *)(buf + 30) = flip16((UInteger16)(ptpClock->sentSyncSequenceId - 1));
    buf[32] = CTRL_FOLLOW_UP;
    buf[33] = (int8_t)ptpClock->portDS.logSyncInterval;

    *(UInteger16 *)(buf + 34) = flip16(preciseOriginTimestamp->secondsField.msb);
    *(UInteger32 *)(buf + 36) = flip32(preciseOriginTimestamp->secondsField.lsb);
    *(UInteger32 *)(buf + 40) = flip32(preciseOriginTimestamp->nanosecondsField);
}

void msgUnpackFollowUp(const Octet *buf, MsgFollowUp *follow)
{
    follow->preciseOriginTimestamp.secondsField.msb = flip16(*(const UInteger16 *)(buf + 34));
    follow->preciseOriginTimestamp.secondsField.lsb = flip32(*(const UInteger32 *)(buf + 36));
    follow->preciseOriginTimestamp.nanosecondsField  = flip32(*(const UInteger32 *)(buf + 40));
}

void msgPackDelayReq(const PtpClock *ptpClock, Octet *buf, const Timestamp *originTimestamp)
{
    buf[0] = (buf[0] & 0xF0) | DELAY_REQ;
    *(UInteger16 *)(buf + 2)  = flip16(DELAY_REQ_LENGTH);
    *(UInteger16 *)(buf + 30) = flip16(ptpClock->sentDelayReqSequenceId);
    buf[32] = CTRL_DELAY_REQ;
    buf[33] = 0x7F;
    memset(buf + 8, 0, 8);

    *(UInteger16 *)(buf + 34) = flip16(originTimestamp->secondsField.msb);
    *(UInteger32 *)(buf + 36) = flip32(originTimestamp->secondsField.lsb);
    *(UInteger32 *)(buf + 40) = flip32(originTimestamp->nanosecondsField);
}

void msgUnpackDelayReq(const Octet *buf, MsgDelayReq *delayreq)
{
    delayreq->originTimestamp.secondsField.msb = flip16(*(const UInteger16 *)(buf + 34));
    delayreq->originTimestamp.secondsField.lsb = flip32(*(const UInteger32 *)(buf + 36));
    delayreq->originTimestamp.nanosecondsField  = flip32(*(const UInteger32 *)(buf + 40));
}

void msgPackDelayResp(const PtpClock *ptpClock, Octet *buf,
                      const MsgHeader *header, const Timestamp *receiveTimestamp)
{
    buf[0] = (buf[0] & 0xF0) | DELAY_RESP;
    *(UInteger16 *)(buf + 2) = flip16(DELAY_RESP_LENGTH);
    memset(buf + 8, 0, 8);

    *(Integer32 *)(buf +  8) = flip32((Integer32)(header->correctionfield >> 32));
    *(Integer32 *)(buf + 12) = flip32((Integer32)(header->correctionfield));

    *(UInteger16 *)(buf + 30) = flip16(header->sequenceId);
    buf[32] = CTRL_DELAY_RESP;
    buf[33] = (int8_t)ptpClock->portDS.logMinDelayReqInterval;

    *(UInteger16 *)(buf + 34) = flip16(receiveTimestamp->secondsField.msb);
    *(UInteger32 *)(buf + 36) = flip32(receiveTimestamp->secondsField.lsb);
    *(UInteger32 *)(buf + 40) = flip32(receiveTimestamp->nanosecondsField);
    memcpy(buf + 44, header->sourcePortIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
    *(UInteger16 *)(buf + 52) = flip16(header->sourcePortIdentity.portNumber);
}

void msgUnpackDelayResp(const Octet *buf, MsgDelayResp *resp)
{
    resp->receiveTimestamp.secondsField.msb = flip16(*(const UInteger16 *)(buf + 34));
    resp->receiveTimestamp.secondsField.lsb = flip32(*(const UInteger32 *)(buf + 36));
    resp->receiveTimestamp.nanosecondsField  = flip32(*(const UInteger32 *)(buf + 40));
    memcpy(resp->requestingPortIdentity.clockIdentity, buf + 44, CLOCK_IDENTITY_LENGTH);
    resp->requestingPortIdentity.portNumber = flip16(*(const UInteger16 *)(buf + 52));
}

void msgPackPDelayReq(const PtpClock *ptpClock, Octet *buf, const Timestamp *originTimestamp)
{
    buf[0] = (buf[0] & 0xF0) | PDELAY_REQ;
    *(UInteger16 *)(buf + 2)  = flip16(PDELAY_REQ_LENGTH);
    *(UInteger16 *)(buf + 30) = flip16(ptpClock->sentPDelayReqSequenceId);
    buf[32] = CTRL_OTHER;
    buf[33] = 0x7F;
    memset(buf + 8, 0, 8);

    *(UInteger16 *)(buf + 34) = flip16(originTimestamp->secondsField.msb);
    *(UInteger32 *)(buf + 36) = flip32(originTimestamp->secondsField.lsb);
    *(UInteger32 *)(buf + 40) = flip32(originTimestamp->nanosecondsField);
    memset(buf + 44, 0, 10); /* reserved */
}

void msgUnpackPDelayReq(const Octet *buf, MsgPDelayReq *pdelayreq)
{
    pdelayreq->originTimestamp.secondsField.msb = flip16(*(const UInteger16 *)(buf + 34));
    pdelayreq->originTimestamp.secondsField.lsb = flip32(*(const UInteger32 *)(buf + 36));
    pdelayreq->originTimestamp.nanosecondsField  = flip32(*(const UInteger32 *)(buf + 40));
}

void msgPackPDelayResp(Octet *buf, const MsgHeader *header,
                       const Timestamp *requestReceiptTimestamp)
{
    buf[0] = (buf[0] & 0xF0) | PDELAY_RESP;
    *(UInteger16 *)(buf + 2) = flip16(PDELAY_RESP_LENGTH);
    memset(buf + 8, 0, 8);
    *(UInteger16 *)(buf + 30) = flip16(header->sequenceId);
    buf[32] = CTRL_OTHER;
    buf[33] = 0x7F;

    *(UInteger16 *)(buf + 34) = flip16(requestReceiptTimestamp->secondsField.msb);
    *(UInteger32 *)(buf + 36) = flip32(requestReceiptTimestamp->secondsField.lsb);
    *(UInteger32 *)(buf + 40) = flip32(requestReceiptTimestamp->nanosecondsField);
    memcpy(buf + 44, header->sourcePortIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
    *(UInteger16 *)(buf + 52) = flip16(header->sourcePortIdentity.portNumber);
}

void msgUnpackPDelayResp(const Octet *buf, MsgPDelayResp *presp)
{
    presp->requestReceiptTimestamp.secondsField.msb = flip16(*(const UInteger16 *)(buf + 34));
    presp->requestReceiptTimestamp.secondsField.lsb = flip32(*(const UInteger32 *)(buf + 36));
    presp->requestReceiptTimestamp.nanosecondsField  = flip32(*(const UInteger32 *)(buf + 40));
    memcpy(presp->requestingPortIdentity.clockIdentity, buf + 44, CLOCK_IDENTITY_LENGTH);
    presp->requestingPortIdentity.portNumber = flip16(*(const UInteger16 *)(buf + 52));
}

void msgPackPDelayRespFollowUp(Octet *buf, const MsgHeader *header,
                               const Timestamp *responseOriginTimestamp)
{
    buf[0] = (buf[0] & 0xF0) | PDELAY_RESP_FOLLOW_UP;
    *(UInteger16 *)(buf + 2) = flip16(PDELAY_RESP_FOLLOW_UP_LENGTH);
    *(UInteger16 *)(buf + 30) = flip16(header->sequenceId);
    buf[32] = CTRL_OTHER;
    buf[33] = 0x7F;

    /*
     * correctionField SHALL be 0 per IEEE 802.1AS-2020 §11.4.4.3 and
     * IEEE 1588-2019 §11.4.4c.  The peer-delay formula at the requester is:
     *   peerDelay = ((T4-T1) - (T3-T2) - CF_resp - CF_respFU) / 2
     * For a two-step responder, CF_resp=0 and CF_respFU=0; the values T2 and T3
     * are carried explicitly in the message bodies, so there is nothing to encode
     * in the correctionField.
     */
    memset(buf + 8, 0, 8);

    *(UInteger16 *)(buf + 34) = flip16(responseOriginTimestamp->secondsField.msb);
    *(UInteger32 *)(buf + 36) = flip32(responseOriginTimestamp->secondsField.lsb);
    *(UInteger32 *)(buf + 40) = flip32(responseOriginTimestamp->nanosecondsField);
    memcpy(buf + 44, header->sourcePortIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
    *(UInteger16 *)(buf + 52) = flip16(header->sourcePortIdentity.portNumber);
}

void msgUnpackPDelayRespFollowUp(const Octet *buf, MsgPDelayRespFollowUp *prespfollow)
{
    prespfollow->responseOriginTimestamp.secondsField.msb = flip16(*(const UInteger16 *)(buf + 34));
    prespfollow->responseOriginTimestamp.secondsField.lsb = flip32(*(const UInteger32 *)(buf + 36));
    prespfollow->responseOriginTimestamp.nanosecondsField  = flip32(*(const UInteger32 *)(buf + 40));
    memcpy(prespfollow->requestingPortIdentity.clockIdentity, buf + 44, CLOCK_IDENTITY_LENGTH);
    prespfollow->requestingPortIdentity.portNumber = flip16(*(const UInteger16 *)(buf + 52));
}
