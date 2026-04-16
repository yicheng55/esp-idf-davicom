/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * gPTP Best Master Clock Algorithm.
 * Ported from ptpd-2.0.0 bmc.c with minimal changes.
 */

#include "gptp_defs.h"
#include "gptp_bmc.h"
#include "gptp_servo.h"

/* EUI-48 → EUI-64 conversion */
void EUI48toEUI64(const Octet *eui48, Octet *eui64)
{
    eui64[0] = eui48[0]; eui64[1] = eui48[1]; eui64[2] = eui48[2];
    eui64[3] = 0xff;     eui64[4] = 0xfe;
    eui64[5] = eui48[3]; eui64[6] = eui48[4]; eui64[7] = eui48[5];
}

void initData(PtpClock *ptpClock)
{
    RunTimeOpts *rtOpts = ptpClock->rtOpts;
    DBG("initData");

    ptpClock->defaultDS.twoStepFlag = DEFAULT_TWO_STEP_FLAG;

    /* Clock identity from MAC (EUI-48 → EUI-64) */
    if (CLOCK_IDENTITY_LENGTH == 8 && PTP_UUID_LENGTH == 6) {
        EUI48toEUI64(ptpClock->portUuidField, ptpClock->defaultDS.clockIdentity);
    } else {
        memcpy(ptpClock->defaultDS.clockIdentity, ptpClock->portUuidField, CLOCK_IDENTITY_LENGTH);
    }

    ptpClock->defaultDS.numberPorts = NUMBER_PORTS;
    ptpClock->defaultDS.clockQuality.clockAccuracy = rtOpts->clockQuality.clockAccuracy;
    ptpClock->defaultDS.clockQuality.clockClass    = rtOpts->clockQuality.clockClass;
    ptpClock->defaultDS.clockQuality.offsetScaledLogVariance = rtOpts->clockQuality.offsetScaledLogVariance;
    ptpClock->defaultDS.priority1    = rtOpts->priority1;
    ptpClock->defaultDS.priority2    = rtOpts->priority2;
    ptpClock->defaultDS.domainNumber = rtOpts->domainNumber;
    ptpClock->defaultDS.slaveOnly    = rtOpts->slaveOnly;

    /* Port DS */
    memcpy(ptpClock->portDS.portIdentity.clockIdentity,
           ptpClock->defaultDS.clockIdentity, CLOCK_IDENTITY_LENGTH);
    ptpClock->portDS.portIdentity.portNumber    = NUMBER_PORTS;
    ptpClock->portDS.logMinDelayReqInterval     = DEFAULT_DELAYREQ_INTERVAL;
    ptpClock->portDS.peerMeanPathDelay.seconds  = 0;
    ptpClock->portDS.peerMeanPathDelay.nanoseconds = 0;
    ptpClock->portDS.logAnnounceInterval        = rtOpts->announceInterval;
    ptpClock->portDS.announceReceiptTimeout     =
        (rtOpts->transportType == TRANSPORT_IEEE_802_1AS)
            ? DEFAULT_ANNOUNCE_RECEIPT_TIMEOUT_8021AS
            : DEFAULT_ANNOUNCE_RECEIPT_TIMEOUT;
    ptpClock->portDS.logSyncInterval            = rtOpts->syncInterval;
    ptpClock->portDS.delayMechanism             = rtOpts->delayMechanism;
    ptpClock->portDS.logMinPdelayReqInterval    =
        (rtOpts->transportType == TRANSPORT_IEEE_802_1AS)
            ? DEFAULT_PDELAYREQ_INTERVAL_8021AS
            : DEFAULT_PDELAYREQ_INTERVAL;
    ptpClock->portDS.versionNumber = VERSION_PTP;

    ptpClock->foreignMasterDS.count    = 0;
    ptpClock->foreignMasterDS.capacity = rtOpts->maxForeignRecords;

    ptpClock->inboundLatency.seconds  = 0;
    ptpClock->inboundLatency.nanoseconds  = DEFAULT_INBOUND_LATENCY;
    ptpClock->outboundLatency.seconds = 0;
    ptpClock->outboundLatency.nanoseconds = DEFAULT_OUTBOUND_LATENCY;

    ptpClock->servo.sDelay     = rtOpts->servo.sDelay;
    ptpClock->servo.sOffset    = rtOpts->servo.sOffset;
    ptpClock->servo.ai         = rtOpts->servo.ai;
    ptpClock->servo.ap         = rtOpts->servo.ap;
    ptpClock->servo.noAdjust   = rtOpts->servo.noAdjust;
    ptpClock->servo.noResetClock = rtOpts->servo.noResetClock;
}

Boolean isSamePortIdentity(const PortIdentity *A, const PortIdentity *B)
{
    return (Boolean)(
        0 == memcmp(A->clockIdentity, B->clockIdentity, CLOCK_IDENTITY_LENGTH)
        && A->portNumber == B->portNumber);
}

void addForeign(PtpClock *ptpClock, const MsgHeader *header, const MsgAnnounce *announce)
{
    int     i, j;
    Boolean found = FALSE;

    j = ptpClock->foreignMasterDS.best;

    for (i = 0; i < ptpClock->foreignMasterDS.count; i++) {
        if (isSamePortIdentity(&header->sourcePortIdentity,
                               &ptpClock->foreignMasterDS.records[j].foreignMasterPortIdentity)) {
            ptpClock->foreignMasterDS.records[j].foreignMasterAnnounceMessages++;
            ptpClock->foreignMasterDS.records[j].header   = *header;
            ptpClock->foreignMasterDS.records[j].announce = *announce;
            found = TRUE;
            break;
        }
        j = (j + 1) % ptpClock->foreignMasterDS.count;
    }

    if (!found) {
        if (ptpClock->foreignMasterDS.count < ptpClock->foreignMasterDS.capacity)
            ptpClock->foreignMasterDS.count++;

        j = ptpClock->foreignMasterDS.i;
        memcpy(ptpClock->foreignMasterDS.records[j].foreignMasterPortIdentity.clockIdentity,
               header->sourcePortIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
        ptpClock->foreignMasterDS.records[j].foreignMasterPortIdentity.portNumber =
            header->sourcePortIdentity.portNumber;
        ptpClock->foreignMasterDS.records[j].foreignMasterAnnounceMessages = 0;
        ptpClock->foreignMasterDS.records[j].header   = *header;
        ptpClock->foreignMasterDS.records[j].announce = *announce;

        ptpClock->foreignMasterDS.i =
            (ptpClock->foreignMasterDS.i + 1) % ptpClock->foreignMasterDS.capacity;
    }
}

/* Enter Master state (Table 13 in spec) */
void m1(PtpClock *ptpClock)
{
    DBGV("bmc: m1");
    ptpClock->currentDS.stepsRemoved = 0;
    ptpClock->currentDS.offsetFromMaster.seconds = 0;
    ptpClock->currentDS.offsetFromMaster.nanoseconds = 0;
    ptpClock->currentDS.meanPathDelay.seconds = 0;
    ptpClock->currentDS.meanPathDelay.nanoseconds = 0;

    memcpy(ptpClock->parentDS.parentPortIdentity.clockIdentity,
           ptpClock->defaultDS.clockIdentity, CLOCK_IDENTITY_LENGTH);
    ptpClock->parentDS.parentPortIdentity.portNumber = 0;
    memcpy(ptpClock->parentDS.grandmasterIdentity,
           ptpClock->defaultDS.clockIdentity, CLOCK_IDENTITY_LENGTH);
    ptpClock->parentDS.grandmasterClockQuality = ptpClock->defaultDS.clockQuality;
    ptpClock->parentDS.grandmasterPriority1    = ptpClock->defaultDS.priority1;
    ptpClock->parentDS.grandmasterPriority2    = ptpClock->defaultDS.priority2;

    ptpClock->timePropertiesDS.currentUtcOffset       = ptpClock->rtOpts->currentUtcOffset;
    ptpClock->timePropertiesDS.currentUtcOffsetValid  = DEFAULT_UTC_VALID;
    ptpClock->timePropertiesDS.leap59                 = FALSE;
    ptpClock->timePropertiesDS.leap61                 = FALSE;
    ptpClock->timePropertiesDS.timeTraceable          = DEFAULT_TIME_TRACEABLE;
    ptpClock->timePropertiesDS.frequencyTraceable     = DEFAULT_FREQUENCY_TRACEABLE;
    ptpClock->timePropertiesDS.ptpTimescale           = (Boolean)(DEFAULT_TIMESCALE == PTP_TIMESCALE);
    ptpClock->timePropertiesDS.timeSource             = DEFAULT_TIME_SOURCE;
}

/* Enter Passive state */
void p1(PtpClock *ptpClock) { DBGV("bmc: p1"); (void)ptpClock; }

/* Enter Slave state (Table 16 in spec) */
void s1(PtpClock *ptpClock, const MsgHeader *header, const MsgAnnounce *announce)
{
    Boolean isFromCurrentParent;
    DBGV("bmc: s1");

    ptpClock->currentDS.stepsRemoved = announce->stepsRemoved + 1;

    isFromCurrentParent = isSamePortIdentity(&ptpClock->parentDS.parentPortIdentity,
                                             &header->sourcePortIdentity);
    if (!isFromCurrentParent)
        setFlag(ptpClock->events, MASTER_CLOCK_CHANGED);

    memcpy(ptpClock->parentDS.parentPortIdentity.clockIdentity,
           header->sourcePortIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
    ptpClock->parentDS.parentPortIdentity.portNumber = header->sourcePortIdentity.portNumber;
    memcpy(ptpClock->parentDS.grandmasterIdentity,
           announce->grandmasterIdentity, CLOCK_IDENTITY_LENGTH);
    ptpClock->parentDS.grandmasterClockQuality = announce->grandmasterClockQuality;
    ptpClock->parentDS.grandmasterPriority1    = announce->grandmasterPriority1;
    ptpClock->parentDS.grandmasterPriority2    = announce->grandmasterPriority2;

    ptpClock->timePropertiesDS.currentUtcOffset      = announce->currentUtcOffset;
    ptpClock->timePropertiesDS.currentUtcOffsetValid = getFlag(header->flagField[1], FLAG1_UTC_OFFSET_VALID);
    ptpClock->timePropertiesDS.leap59                = getFlag(header->flagField[1], FLAG1_LEAP59);
    ptpClock->timePropertiesDS.leap61                = getFlag(header->flagField[1], FLAG1_LEAP61);
    ptpClock->timePropertiesDS.timeTraceable         = getFlag(header->flagField[1], FLAG1_TIME_TRACEABLE);
    ptpClock->timePropertiesDS.frequencyTraceable    = getFlag(header->flagField[1], FLAG1_FREQUENCY_TRACEABLE);
    ptpClock->timePropertiesDS.ptpTimescale          = getFlag(header->flagField[1], FLAG1_PTP_TIMESCALE);
    ptpClock->timePropertiesDS.timeSource            = announce->timeSource;
}

/* Copy local data set into header+announce (Table 12) */
void copyD0(MsgHeader *header, MsgAnnounce *announce, PtpClock *ptpClock)
{
    announce->grandmasterPriority1       = ptpClock->defaultDS.priority1;
    memcpy(announce->grandmasterIdentity, ptpClock->defaultDS.clockIdentity, CLOCK_IDENTITY_LENGTH);
    announce->grandmasterClockQuality    = ptpClock->defaultDS.clockQuality;
    announce->grandmasterPriority2       = ptpClock->defaultDS.priority2;
    announce->stepsRemoved               = 0;
    memcpy(header->sourcePortIdentity.clockIdentity,
           ptpClock->defaultDS.clockIdentity, CLOCK_IDENTITY_LENGTH);
}

#define A_better_then_B            1
#define B_better_then_A           -1
#define A_better_by_topology_then_B  1
#define B_better_by_topology_then_A -1
#define ERROR_1 0
#define ERROR_2 0

#define COMPARE_AB_RETURN_BETTER(cond, msg)                         \
    if ((announceA->cond) > (announceB->cond)) return B_better_then_A;  \
    if ((announceB->cond) > (announceA->cond)) return A_better_then_B;

Integer8 bmcDataSetComparison(MsgHeader *headerA, MsgAnnounce *announceA,
                              MsgHeader *headerB, MsgAnnounce *announceB,
                              PtpClock  *ptpClock)
{
    int   grandmasterIdentityComp;
    short comp;

    grandmasterIdentityComp = memcmp(announceA->grandmasterIdentity,
                                     announceB->grandmasterIdentity,
                                     CLOCK_IDENTITY_LENGTH);

    if (grandmasterIdentityComp != 0) {
        COMPARE_AB_RETURN_BETTER(grandmasterPriority1, "priority1");
        COMPARE_AB_RETURN_BETTER(grandmasterClockQuality.clockClass, "clockClass");
        COMPARE_AB_RETURN_BETTER(grandmasterClockQuality.clockAccuracy, "clockAccuracy");
        COMPARE_AB_RETURN_BETTER(grandmasterClockQuality.offsetScaledLogVariance, "variance");
        COMPARE_AB_RETURN_BETTER(grandmasterPriority2, "priority2");
        return (grandmasterIdentityComp > 0) ? B_better_then_A : A_better_then_B;
    }

    /* Same grandmaster identity – compare topology (stepsRemoved) */
    if (announceA->stepsRemoved > (UInteger16)(announceB->stepsRemoved + 1))
        return B_better_then_A;
    if (announceB->stepsRemoved > (UInteger16)(announceA->stepsRemoved + 1))
        return A_better_then_B;

    if (announceA->stepsRemoved > announceB->stepsRemoved) {
        comp = (short)memcmp(headerA->sourcePortIdentity.clockIdentity,
                             ptpClock->portDS.portIdentity.clockIdentity,
                             CLOCK_IDENTITY_LENGTH);
        return (comp > 0) ? B_better_then_A
             : (comp < 0) ? B_better_by_topology_then_A
             : ERROR_1;
    } else if (announceA->stepsRemoved < announceB->stepsRemoved) {
        comp = (short)memcmp(headerB->sourcePortIdentity.clockIdentity,
                             ptpClock->portDS.portIdentity.clockIdentity,
                             CLOCK_IDENTITY_LENGTH);
        return (comp > 0) ? A_better_then_B
             : (comp < 0) ? A_better_by_topology_then_B
             : ERROR_1;
    }

    comp = (short)memcmp(headerA->sourcePortIdentity.clockIdentity,
                         headerB->sourcePortIdentity.clockIdentity,
                         CLOCK_IDENTITY_LENGTH);
    return (comp > 0) ? B_better_by_topology_then_A
         : (comp < 0) ? A_better_by_topology_then_B
         : ERROR_2;
}

UInteger8 bmcStateDecision(MsgHeader *header, MsgAnnounce *announce, PtpClock *ptpClock)
{
    int comp;

    if (!ptpClock->foreignMasterDS.count && ptpClock->portDS.portState == PTP_LISTENING)
        return PTP_LISTENING;

    copyD0(&ptpClock->msgTmpHeader, &ptpClock->msgTmp.announce, ptpClock);
    comp = bmcDataSetComparison(&ptpClock->msgTmpHeader, &ptpClock->msgTmp.announce,
                                header, announce, ptpClock);
    DBGV("bmcStateDecision: comp=%d", comp);

    if (ptpClock->defaultDS.clockQuality.clockClass < 128) {
        if (comp == A_better_then_B) { m1(ptpClock); return PTP_MASTER; }
        else                         { p1(ptpClock); return PTP_PASSIVE; }
    } else {
        if (comp == A_better_then_B) { m1(ptpClock); return PTP_MASTER; }
        else                         { s1(ptpClock, header, announce); return PTP_SLAVE; }
    }
}

UInteger8 bmc(PtpClock *ptpClock)
{
    Integer16 i, best;

    for (i = 1, best = 0; i < ptpClock->foreignMasterDS.count; i++) {
        if (bmcDataSetComparison(&ptpClock->foreignMasterDS.records[i].header,
                                 &ptpClock->foreignMasterDS.records[i].announce,
                                 &ptpClock->foreignMasterDS.records[best].header,
                                 &ptpClock->foreignMasterDS.records[best].announce,
                                 ptpClock) < 0) {
            best = i;
        }
    }
    ptpClock->foreignMasterDS.best = best;
    return bmcStateDecision(&ptpClock->foreignMasterDS.records[best].header,
                            &ptpClock->foreignMasterDS.records[best].announce,
                            ptpClock);
}
