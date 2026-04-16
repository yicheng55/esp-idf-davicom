/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * gPTP (IEEE 802.1AS) protocol state machine.
 * Ported from ptpd-2.0.0 protocol.c with ESP-IDF adaptations.
 *
 * Key gPTP specifics vs. standard PTP:
 *  - Delay mechanism: P2P only (PDelay_Req/Resp/RespFollowUp)
 *  - No E2E delay requests in slave state
 *  - PDelay runs in ALL states (LISTENING, MASTER, SLAVE, PASSIVE)
 */

#include "gptp_defs.h"
#include "gptp_protocol.h"
#include "gptp_arith.h"
#include "gptp_bmc.h"
#include "gptp_timer.h"
#include "gptp_servo.h"
#include "gptp_net.h"

/* --------------------------------------------------------
 *  Forward declarations (internal)
 * -------------------------------------------------------- */
static void handle(PtpClock *);
static void handleAnnounce(PtpClock *, Boolean);
static void handleSync(PtpClock *, TimeInternal *, Boolean);
static void handleFollowUp(PtpClock *, Boolean);
static void handleDelayReq(PtpClock *, TimeInternal *, Boolean);
static void handleDelayResp(PtpClock *, Boolean);
static void handlePDelayReq(PtpClock *, TimeInternal *, Boolean);
static void handlePDelayResp(PtpClock *, TimeInternal *, Boolean);
static void handlePDelayRespFollowUp(PtpClock *, Boolean);
static void handleManagement(PtpClock *, Boolean);
static void handleSignaling(PtpClock *, Boolean);

static void issueDelayReqTimerExpired(PtpClock *);
static void issueAnnounce(PtpClock *);
static void issueSync(PtpClock *);
static void issueFollowup(PtpClock *, const TimeInternal *);
static void issueDelayReq(PtpClock *);
static void issueDelayResp(PtpClock *, const TimeInternal *, const MsgHeader *);
static void issuePDelayReq(PtpClock *);
static void issuePDelayResp(PtpClock *, TimeInternal *, const MsgHeader *);
static void issuePDelayRespFollowUp(PtpClock *, const TimeInternal *, const MsgHeader *);
static Boolean doInit(PtpClock *);

/* ============================================================
 *  toState: state transition
 * ============================================================ */
void toState(PtpClock *ptpClock, UInteger8 state)
{
    ptpClock->messageActivity = TRUE;

    /* Actions on leaving current state */
    switch (ptpClock->portDS.portState) {
    case PTP_MASTER:
        initClock(ptpClock);
        timerStop(SYNC_INTERVAL_TIMER,     ptpClock->itimer);
        timerStop(ANNOUNCE_INTERVAL_TIMER, ptpClock->itimer);
        timerStop(PDELAYREQ_INTERVAL_TIMER,ptpClock->itimer);
        break;

    case PTP_UNCALIBRATED:
    case PTP_SLAVE:
        if (state == PTP_UNCALIBRATED || state == PTP_SLAVE) break;
        timerStop(ANNOUNCE_RECEIPT_TIMER, ptpClock->itimer);
        timerStop(PDELAYREQ_INTERVAL_TIMER, ptpClock->itimer);
        timerStop(DELAYREQ_INTERVAL_TIMER, ptpClock->itimer);
        initClock(ptpClock);
        break;

    case PTP_PASSIVE:
        initClock(ptpClock);
        timerStop(PDELAYREQ_INTERVAL_TIMER, ptpClock->itimer);
        timerStop(ANNOUNCE_RECEIPT_TIMER,   ptpClock->itimer);
        break;

    case PTP_LISTENING:
        initClock(ptpClock);
        timerStop(ANNOUNCE_RECEIPT_TIMER,   ptpClock->itimer);
        timerStop(PDELAYREQ_INTERVAL_TIMER, ptpClock->itimer);
        break;

    case PTP_PRE_MASTER:
        initClock(ptpClock);
        timerStop(QUALIFICATION_TIMEOUT, ptpClock->itimer);
        break;

    default:
        break;
    }

    /* Actions on entering new state */
    switch (state) {
    case PTP_INITIALIZING:
        DBG("→ PTP_INITIALIZING");
        ptpClock->portDS.portState   = PTP_INITIALIZING;
        ptpClock->recommendedState   = PTP_INITIALIZING;
        break;

    case PTP_FAULTY:
        DBG("→ PTP_FAULTY");
        ptpClock->portDS.portState = PTP_FAULTY;
        break;

    case PTP_DISABLED:
        DBG("→ PTP_DISABLED");
        ptpClock->portDS.portState = PTP_DISABLED;
        break;

    case PTP_LISTENING:
        DBG("→ PTP_LISTENING");
        timerStart(ANNOUNCE_RECEIPT_TIMER,
                   (UInteger32)(ptpClock->portDS.announceReceiptTimeout) *
                   (UInteger32)pow2ms(ptpClock->portDS.logAnnounceInterval),
                   ptpClock->itimer);
        /* 802.1AS: continue PDelay measurement even in LISTENING */
        timerStart(PDELAYREQ_INTERVAL_TIMER,
                   safeGetRand((UInteger32)pow2ms(ptpClock->portDS.logMinPdelayReqInterval + 1)),
                   ptpClock->itimer);
        ptpClock->portDS.portState = PTP_LISTENING;
        ptpClock->recommendedState = PTP_LISTENING;
        break;

    case PTP_PRE_MASTER:
        DBG("→ PTP_PRE_MASTER (transitioning to MASTER)");
        /* Fall through to MASTER for ordinary clock */
        /* FALLTHROUGH */
    case PTP_MASTER:
        DBG("→ PTP_MASTER");
        ptpClock->portDS.logMinDelayReqInterval = DEFAULT_DELAYREQ_INTERVAL;
        timerStart(SYNC_INTERVAL_TIMER,
                   (UInteger32)pow2ms(ptpClock->portDS.logSyncInterval),
                   ptpClock->itimer);
        timerStart(ANNOUNCE_INTERVAL_TIMER,
                   (UInteger32)pow2ms(ptpClock->portDS.logAnnounceInterval),
                   ptpClock->itimer);
        /* P2P: PDelay runs in MASTER too */
        timerStart(PDELAYREQ_INTERVAL_TIMER,
                   safeGetRand((UInteger32)pow2ms(ptpClock->portDS.logMinPdelayReqInterval + 1)),
                   ptpClock->itimer);
        ptpClock->portDS.portState = PTP_MASTER;
        break;

    case PTP_PASSIVE:
        DBG("→ PTP_PASSIVE");
        timerStart(ANNOUNCE_RECEIPT_TIMER,
                   (UInteger32)(ptpClock->portDS.announceReceiptTimeout) *
                   (UInteger32)pow2ms(ptpClock->portDS.logAnnounceInterval),
                   ptpClock->itimer);
        timerStart(PDELAYREQ_INTERVAL_TIMER,
                   safeGetRand((UInteger32)pow2ms(ptpClock->portDS.logMinPdelayReqInterval + 1)),
                   ptpClock->itimer);
        ptpClock->portDS.portState = PTP_PASSIVE;
        break;

    case PTP_UNCALIBRATED:
        DBG("→ PTP_UNCALIBRATED");
        timerStart(ANNOUNCE_RECEIPT_TIMER,
                   (UInteger32)(ptpClock->portDS.announceReceiptTimeout) *
                   (UInteger32)pow2ms(ptpClock->portDS.logAnnounceInterval),
                   ptpClock->itimer);
        timerStart(PDELAYREQ_INTERVAL_TIMER,
                   safeGetRand((UInteger32)pow2ms(ptpClock->portDS.logMinPdelayReqInterval + 1)),
                   ptpClock->itimer);
        ptpClock->portDS.portState = PTP_UNCALIBRATED;
        break;

    case PTP_SLAVE:
        DBG("→ PTP_SLAVE");
        ptpClock->portDS.portState = PTP_SLAVE;
        break;

    default:
        DBG("toState: unrecognized %d", state);
        break;
    }
}

/* ============================================================
 *  doInit: network + data initialization
 * ============================================================ */
static Boolean doInit(PtpClock *ptpClock)
{
    netShutdown(&ptpClock->netPath);

    if (!netInit(&ptpClock->netPath, ptpClock)) {
        ERROR("doInit: netInit failed");
        return FALSE;
    }

    initData(ptpClock);
    initTimer();
    initClock(ptpClock);
    m1(ptpClock);
    msgPackHeader(ptpClock, ptpClock->msgObuf);
    return TRUE;
}

/* ============================================================
 *  doState: main state machine dispatcher
 * ============================================================ */
void doState(PtpClock *ptpClock)
{
    ptpClock->messageActivity = FALSE;

    /* STATE_DECISION_EVENT: run BMCA */
    switch (ptpClock->portDS.portState) {
    case PTP_LISTENING:
    case PTP_UNCALIBRATED:
    case PTP_SLAVE:
    case PTP_PRE_MASTER:
    case PTP_MASTER:
    case PTP_PASSIVE:
        if (getFlag(ptpClock->events, STATE_DECISION_EVENT)) {
            clearFlag(ptpClock->events, STATE_DECISION_EVENT);
            ptpClock->recommendedState = bmc(ptpClock);

            /* slaveOnly or degraded clock class → stay listener */
            if (ptpClock->recommendedState == PTP_MASTER || ptpClock->recommendedState == PTP_PASSIVE) {
                if (ptpClock->defaultDS.slaveOnly ||
                    ptpClock->defaultDS.clockQuality.clockClass == 255) {
                    ptpClock->recommendedState = PTP_LISTENING;
                }
            }
        }
        break;
    default:
        break;
    }

    /* Act on recommendedState */
    switch (ptpClock->recommendedState) {
    case PTP_MASTER:
        switch (ptpClock->portDS.portState) {
        case PTP_PRE_MASTER:
            if (timerExpired(QUALIFICATION_TIMEOUT, ptpClock->itimer))
                toState(ptpClock, PTP_MASTER);
            break;
        case PTP_MASTER:
            break;
        default:
            toState(ptpClock, PTP_PRE_MASTER);
            break;
        }
        break;

    case PTP_PASSIVE:
        if (ptpClock->portDS.portState != PTP_PASSIVE)
            toState(ptpClock, PTP_PASSIVE);
        break;

    case PTP_SLAVE:
        switch (ptpClock->portDS.portState) {
        case PTP_UNCALIBRATED:
            if (getFlag(ptpClock->events, MASTER_CLOCK_SELECTED)) {
                clearFlag(ptpClock->events, MASTER_CLOCK_SELECTED);
                toState(ptpClock, PTP_SLAVE);
            }
            if (getFlag(ptpClock->events, MASTER_CLOCK_CHANGED)) {
                clearFlag(ptpClock->events, MASTER_CLOCK_CHANGED);
            }
            break;
        case PTP_SLAVE:
            if (getFlag(ptpClock->events, SYNCHRONIZATION_FAULT)) {
                clearFlag(ptpClock->events, SYNCHRONIZATION_FAULT);
                toState(ptpClock, PTP_UNCALIBRATED);
            }
            if (getFlag(ptpClock->events, MASTER_CLOCK_CHANGED)) {
                clearFlag(ptpClock->events, MASTER_CLOCK_CHANGED);
                toState(ptpClock, PTP_UNCALIBRATED);
            }
            break;
        default:
            toState(ptpClock, PTP_UNCALIBRATED);
            break;
        }
        break;

    case PTP_LISTENING:
        if (ptpClock->portDS.portState != PTP_LISTENING)
            toState(ptpClock, PTP_LISTENING);
        break;

    case PTP_INITIALIZING:
        break;

    default:
        DBGV("doState: unrecognized recommended=%d", ptpClock->recommendedState);
        break;
    }

    /* Per-state actions */
    switch (ptpClock->portDS.portState) {
    case PTP_INITIALIZING:
        if (doInit(ptpClock)) toState(ptpClock, PTP_LISTENING);
        else                  toState(ptpClock, PTP_FAULTY);
        break;

    case PTP_FAULTY:
        DBG("PTP_FAULTY: attempting re-init");
        toState(ptpClock, PTP_INITIALIZING);
        return;

    case PTP_DISABLED:
        handle(ptpClock);
        break;

    case PTP_LISTENING:
    case PTP_UNCALIBRATED:
    case PTP_SLAVE:
    case PTP_PASSIVE:
        if (timerExpired(ANNOUNCE_RECEIPT_TIMER, ptpClock->itimer)) {
            DBGV("ANNOUNCE_RECEIPT_TIMEOUT");
            ptpClock->foreignMasterDS.count = 0;
            ptpClock->foreignMasterDS.i     = 0;
            if (!(ptpClock->defaultDS.slaveOnly ||
                  ptpClock->defaultDS.clockQuality.clockClass == 255)) {
                m1(ptpClock);
                ptpClock->recommendedState = PTP_MASTER;
                toState(ptpClock, PTP_MASTER);
            } else if (ptpClock->portDS.portState != PTP_LISTENING) {
                toState(ptpClock, PTP_LISTENING);
            }
            break;
        }
        handle(ptpClock);
        issueDelayReqTimerExpired(ptpClock);
        break;

    case PTP_MASTER:
        if (timerExpired(SYNC_INTERVAL_TIMER, ptpClock->itimer)) {
            DBGV("SYNC_INTERVAL_TIMEOUT");
            issueSync(ptpClock);
        }
        if (timerExpired(ANNOUNCE_INTERVAL_TIMER, ptpClock->itimer)) {
            DBGV("ANNOUNCE_INTERVAL_TIMEOUT");
            issueAnnounce(ptpClock);
        }
        handle(ptpClock);
        issueDelayReqTimerExpired(ptpClock);
        break;

    default:
        DBGV("doState: unrecognized portState=%d", ptpClock->portDS.portState);
        break;
    }
}

/* ============================================================
 *  handle: receive and dispatch incoming messages
 * ============================================================ */
static void handle(PtpClock *ptpClock)
{
    int     ret;
    Boolean isFromSelf;
    TimeInternal time = {0, 0};

    if (!ptpClock->messageActivity) {
        ret = netSelect(&ptpClock->netPath, NULL);
        if (ret < 0) {
            ERROR("handle: netSelect error");
            toState(ptpClock, PTP_FAULTY);
            return;
        } else if (!ret) {
            return;
        }
    }

    ptpClock->msgIbufLength = netRecvEvent(&ptpClock->netPath, ptpClock->msgIbuf, &time);
    if (ptpClock->msgIbufLength < 0) {
        ERROR("handle: netRecvEvent error");
        toState(ptpClock, PTP_FAULTY);
        return;
    } else if (!ptpClock->msgIbufLength) {
        ptpClock->msgIbufLength = netRecvGeneral(&ptpClock->netPath, ptpClock->msgIbuf, &time);
        if (ptpClock->msgIbufLength < 0) {
            ERROR("handle: netRecvGeneral error");
            toState(ptpClock, PTP_FAULTY);
            return;
        } else if (!ptpClock->msgIbufLength) {
            return;
        }
    }

    ptpClock->messageActivity = TRUE;

    if (ptpClock->msgIbufLength < HEADER_LENGTH) {
        ERROR("handle: message shorter than header (%d bytes)", (int)ptpClock->msgIbufLength);
        return;
    }

    msgUnpackHeader(ptpClock->msgIbuf, &ptpClock->msgTmpHeader);

    if (ptpClock->msgTmpHeader.versionPTP != ptpClock->portDS.versionNumber) {
        DBGV("handle: wrong PTP version %d", (int)ptpClock->msgTmpHeader.versionPTP);
        return;
    }
    if (ptpClock->msgTmpHeader.domainNumber != ptpClock->defaultDS.domainNumber) {
        DBGV("handle: wrong domain %d", (int)ptpClock->msgTmpHeader.domainNumber);
        return;
    }

    isFromSelf = isSamePortIdentity(&ptpClock->portDS.portIdentity,
                                    &ptpClock->msgTmpHeader.sourcePortIdentity);

    /* Subtract inbound latency from event-message timestamps */
    if (!isFromSelf && time.seconds > 0)
        subTime(&time, &time, &ptpClock->inboundLatency);

    switch (ptpClock->msgTmpHeader.messageType) {
    case ANNOUNCE:               handleAnnounce(ptpClock, isFromSelf);          break;
    case SYNC:                   handleSync(ptpClock, &time, isFromSelf);        break;
    case FOLLOW_UP:              handleFollowUp(ptpClock, isFromSelf);           break;
    case DELAY_REQ:              handleDelayReq(ptpClock, &time, isFromSelf);    break;
    case PDELAY_REQ:             handlePDelayReq(ptpClock, &time, isFromSelf);   break;
    case DELAY_RESP:             handleDelayResp(ptpClock, isFromSelf);          break;
    case PDELAY_RESP:            handlePDelayResp(ptpClock, &time, isFromSelf);  break;
    case PDELAY_RESP_FOLLOW_UP:  handlePDelayRespFollowUp(ptpClock, isFromSelf); break;
    case MANAGEMENT:             handleManagement(ptpClock, isFromSelf);         break;
    case SIGNALING:              handleSignaling(ptpClock, isFromSelf);          break;
    default:
        DBGV("handle: unknown msgType=0x%x", ptpClock->msgTmpHeader.messageType);
        break;
    }
}

/* ============================================================
 *  Message handlers
 * ============================================================ */

static void handleAnnounce(PtpClock *ptpClock, Boolean isFromSelf)
{
    Boolean isFromCurrentParent = FALSE;
    DBGV("handleAnnounce");

    if (ptpClock->msgIbufLength < ANNOUNCE_LENGTH) {
        ERROR("handleAnnounce: short message");
        toState(ptpClock, PTP_FAULTY);
        return;
    }
    if (isFromSelf) { DBGV("handleAnnounce: from self, ignore"); return; }

    switch (ptpClock->portDS.portState) {
    case PTP_INITIALIZING:
    case PTP_FAULTY:
    case PTP_DISABLED:
        break;

    case PTP_UNCALIBRATED:
    case PTP_SLAVE:
        setFlag(ptpClock->events, STATE_DECISION_EVENT);
        isFromCurrentParent = isSamePortIdentity(&ptpClock->parentDS.parentPortIdentity,
                                                 &ptpClock->msgTmpHeader.sourcePortIdentity);
        msgUnpackAnnounce(ptpClock->msgIbuf, &ptpClock->msgTmp.announce);
        if (isFromCurrentParent) {
            s1(ptpClock, &ptpClock->msgTmpHeader, &ptpClock->msgTmp.announce);
            timerStart(ANNOUNCE_RECEIPT_TIMER,
                       (UInteger32)(ptpClock->portDS.announceReceiptTimeout) *
                       (UInteger32)pow2ms(ptpClock->portDS.logAnnounceInterval),
                       ptpClock->itimer);
        } else {
            addForeign(ptpClock, &ptpClock->msgTmpHeader, &ptpClock->msgTmp.announce);
        }
        break;

    case PTP_PASSIVE:
        timerStart(ANNOUNCE_RECEIPT_TIMER,
                   (UInteger32)(ptpClock->portDS.announceReceiptTimeout) *
                   (UInteger32)pow2ms(ptpClock->portDS.logAnnounceInterval),
                   ptpClock->itimer);
        /* FALLTHROUGH */
    case PTP_MASTER:
    case PTP_PRE_MASTER:
    case PTP_LISTENING:
    default:
        msgUnpackAnnounce(ptpClock->msgIbuf, &ptpClock->msgTmp.announce);
        setFlag(ptpClock->events, STATE_DECISION_EVENT);
        addForeign(ptpClock, &ptpClock->msgTmpHeader, &ptpClock->msgTmp.announce);
        break;
    }
}

static void handleSync(PtpClock *ptpClock, TimeInternal *time, Boolean isFromSelf)
{
    TimeInternal originTimestamp, correctionField;
    Boolean isFromCurrentParent = FALSE;
    DBGV("handleSync");

    if (ptpClock->msgIbufLength < SYNC_LENGTH) {
        ERROR("handleSync: short message"); toState(ptpClock, PTP_FAULTY); return;
    }

    switch (ptpClock->portDS.portState) {
    case PTP_INITIALIZING:
    case PTP_FAULTY:
    case PTP_DISABLED:
        break;

    case PTP_UNCALIBRATED:
    case PTP_SLAVE:
        if (isFromSelf) break;
        isFromCurrentParent = isSamePortIdentity(&ptpClock->parentDS.parentPortIdentity,
                                                 &ptpClock->msgTmpHeader.sourcePortIdentity);
        if (!isFromCurrentParent) break;

        ptpClock->timestamp_syncRecieve = *time;
        scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionfield, &correctionField);

        if (getFlag(ptpClock->msgTmpHeader.flagField[0], FLAG0_TWO_STEP)) {
            ptpClock->waitingForFollowUp   = TRUE;
            ptpClock->recvSyncSequenceId   = ptpClock->msgTmpHeader.sequenceId;
            ptpClock->correctionField_sync = correctionField;
            DBG_PRINTF("TWO_STEP: waiting for FollowUp");
        } else {
            /* ONE_STEP: originTimestamp in Sync body */
            msgUnpackSync(ptpClock->msgIbuf, &ptpClock->msgTmp.sync);
            ptpClock->waitingForFollowUp = FALSE;
            toInternalTime(&originTimestamp, &ptpClock->msgTmp.sync.originTimestamp);
            updateOffset(ptpClock, &ptpClock->timestamp_syncRecieve, &originTimestamp, &correctionField);
            updateClock(ptpClock);
        }
        break;

    case PTP_MASTER:
    case PTP_PASSIVE:
        issueDelayReqTimerExpired(ptpClock);
        break;

    default:
        break;
    }
}

static void handleFollowUp(PtpClock *ptpClock, Boolean isFromSelf)
{
    TimeInternal preciseOriginTimestamp, correctionField;
    Boolean isFromCurrentParent = FALSE;
    DBGV("handleFollowUp");

    if (ptpClock->msgIbufLength < FOLLOW_UP_LENGTH) {
        ERROR("handleFollowUp: short message"); toState(ptpClock, PTP_FAULTY); return;
    }
    if (isFromSelf) return;

    switch (ptpClock->portDS.portState) {
    case PTP_INITIALIZING:
    case PTP_FAULTY:
    case PTP_DISABLED:
    case PTP_LISTENING:
        break;

    case PTP_UNCALIBRATED:
    case PTP_SLAVE:
        isFromCurrentParent = isSamePortIdentity(&ptpClock->parentDS.parentPortIdentity,
                                                 &ptpClock->msgTmpHeader.sourcePortIdentity);
        if (!ptpClock->waitingForFollowUp) break;
        if (!isFromCurrentParent)          break;
        if (ptpClock->recvSyncSequenceId != ptpClock->msgTmpHeader.sequenceId) break;

        msgUnpackFollowUp(ptpClock->msgIbuf, &ptpClock->msgTmp.follow);
        ptpClock->waitingForFollowUp = FALSE;
        toInternalTime(&preciseOriginTimestamp, &ptpClock->msgTmp.follow.preciseOriginTimestamp);
        scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionfield, &correctionField);
        addTime(&correctionField, &correctionField, &ptpClock->correctionField_sync);
        updateOffset(ptpClock, &ptpClock->timestamp_syncRecieve, &preciseOriginTimestamp, &correctionField);
        updateClock(ptpClock);
        break;

    case PTP_MASTER:
        break;

    case PTP_PASSIVE:
        issueDelayReqTimerExpired(ptpClock);
        break;

    default:
        break;
    }
}

static void handleDelayReq(PtpClock *ptpClock, TimeInternal *time, Boolean isFromSelf)
{
    switch (ptpClock->portDS.delayMechanism) {
    case E2E:
        if (ptpClock->msgIbufLength < DELAY_REQ_LENGTH) {
            ERROR("handleDelayReq: short message"); toState(ptpClock, PTP_FAULTY); return;
        }
        if (ptpClock->portDS.portState == PTP_MASTER)
            issueDelayResp(ptpClock, time, &ptpClock->msgTmpHeader);
        break;
    case P2P:
        /* In 802.1AS (P2P) no E2E delay requests are used */
        break;
    default:
        break;
    }
}

static void handleDelayResp(PtpClock *ptpClock, Boolean isFromSelf)
{
    TimeInternal correctionField;
    Boolean isFromCurrentParent, isCurrentRequest;

    switch (ptpClock->portDS.delayMechanism) {
    case E2E:
        if (ptpClock->msgIbufLength < DELAY_RESP_LENGTH) {
            ERROR("handleDelayResp: short message"); toState(ptpClock, PTP_FAULTY); return;
        }
        switch (ptpClock->portDS.portState) {
        case PTP_UNCALIBRATED:
        case PTP_SLAVE:
            msgUnpackDelayResp(ptpClock->msgIbuf, &ptpClock->msgTmp.resp);
            isFromCurrentParent = isSamePortIdentity(&ptpClock->parentDS.parentPortIdentity,
                                                     &ptpClock->msgTmpHeader.sourcePortIdentity);
            isCurrentRequest    = isSamePortIdentity(&ptpClock->portDS.portIdentity,
                                                     &ptpClock->msgTmp.resp.requestingPortIdentity);
            if ((ptpClock->sentDelayReqSequenceId - 1) == ptpClock->msgTmpHeader.sequenceId
                && isCurrentRequest && isFromCurrentParent) {
                toInternalTime(&ptpClock->timestamp_delayReqRecieve,
                               &ptpClock->msgTmp.resp.receiveTimestamp);
                scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionfield, &correctionField);
                updateDelay(ptpClock,
                            &ptpClock->timestamp_delayReqSend,
                            &ptpClock->timestamp_delayReqRecieve,
                            &correctionField);
            }
            break;
        default:
            break;
        }
        break;
    case P2P:
        break; /* Not used in gPTP */
    default:
        break;
    }
}

static void handlePDelayReq(PtpClock *ptpClock, TimeInternal *time, Boolean isFromSelf)
{
    if (ptpClock->portDS.delayMechanism != P2P) {
        ERROR("handlePDelayReq: received in non-P2P mode"); return;
    }
    if (ptpClock->msgIbufLength < PDELAY_REQ_LENGTH) {
        ERROR("handlePDelayReq: short message"); toState(ptpClock, PTP_FAULTY); return;
    }

    switch (ptpClock->portDS.portState) {
    case PTP_INITIALIZING:
    case PTP_FAULTY:
    case PTP_DISABLED:
        return;

    case PTP_UNCALIBRATED:
    case PTP_LISTENING:
    case PTP_PASSIVE:
    case PTP_SLAVE:
    case PTP_MASTER:
        if (isFromSelf) break;

        /* Save T2 (requestReceiptTimestamp) */
        ptpClock->pdelay_t2 = *time;

        issuePDelayResp(ptpClock, time, &ptpClock->msgTmpHeader);

        /* TWO_STEP: send FollowUp with T3 */
        if (time->seconds != 0 && ptpClock->defaultDS.twoStepFlag)
            issuePDelayRespFollowUp(ptpClock, time, &ptpClock->msgTmpHeader);
        break;

    default:
        break;
    }
}

static void handlePDelayResp(PtpClock *ptpClock, TimeInternal *time, Boolean isFromSelf)
{
    TimeInternal requestReceiptTimestamp, correctionField;
    Boolean isCurrentRequest;

    if (ptpClock->portDS.delayMechanism != P2P) {
        ERROR("handlePDelayResp: received in non-P2P mode"); return;
    }
    if (ptpClock->msgIbufLength < PDELAY_RESP_LENGTH) {
        ERROR("handlePDelayResp: short message"); toState(ptpClock, PTP_FAULTY); return;
    }

    switch (ptpClock->portDS.portState) {
    case PTP_INITIALIZING:
    case PTP_FAULTY:
    case PTP_DISABLED:
        return;

    case PTP_UNCALIBRATED:
    case PTP_LISTENING:
    case PTP_PASSIVE:
    case PTP_MASTER:
    case PTP_SLAVE:
        if (isFromSelf) break;

        msgUnpackPDelayResp(ptpClock->msgIbuf, &ptpClock->msgTmp.presp);
        isCurrentRequest = isSamePortIdentity(&ptpClock->portDS.portIdentity,
                                              &ptpClock->msgTmp.presp.requestingPortIdentity);

        if ((ptpClock->sentPDelayReqSequenceId - 1) == ptpClock->msgTmpHeader.sequenceId
            && isCurrentRequest) {
            if (getFlag(ptpClock->msgTmpHeader.flagField[0], FLAG0_TWO_STEP)) {
                /* TWO_STEP: save T4 + T2, wait for FollowUp */
                ptpClock->waitingForPDelayRespFollowUp = TRUE;
                ptpClock->pdelay_t4 = *time;
                toInternalTime(&requestReceiptTimestamp,
                               &ptpClock->msgTmp.presp.requestReceiptTimestamp);
                ptpClock->pdelay_t2 = requestReceiptTimestamp;
                scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionfield, &correctionField);
                ptpClock->correctionField_pDelayResp = correctionField;
            } else {
                /* ONE_STEP: correctionField = (T3 - T2), compute delay directly */
                ptpClock->waitingForPDelayRespFollowUp = FALSE;
                ptpClock->pdelay_t4 = *time;
                scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionfield, &correctionField);
                updatePeerDelay(ptpClock, &correctionField, FALSE);
            }
        } else {
            DBGV("handlePDelayResp: seqId mismatch or wrong requester");
        }
        break;

    default:
        break;
    }
}

static void handlePDelayRespFollowUp(PtpClock *ptpClock, Boolean isFromSelf)
{
    TimeInternal responseOriginTimestamp, correctionField;

    if (ptpClock->portDS.delayMechanism != P2P) {
        ERROR("handlePDelayRespFollowUp: received in non-P2P mode"); return;
    }
    if (ptpClock->msgIbufLength < PDELAY_RESP_FOLLOW_UP_LENGTH) {
        ERROR("handlePDelayRespFollowUp: short message"); toState(ptpClock, PTP_FAULTY); return;
    }

    switch (ptpClock->portDS.portState) {
    case PTP_INITIALIZING:
    case PTP_FAULTY:
    case PTP_DISABLED:
        return;

    case PTP_UNCALIBRATED:
    case PTP_LISTENING:
    case PTP_PASSIVE:
    case PTP_SLAVE:
    case PTP_MASTER:
        if (!ptpClock->waitingForPDelayRespFollowUp) break;
        if (ptpClock->msgTmpHeader.sequenceId != ptpClock->sentPDelayReqSequenceId - 1) break;

        msgUnpackPDelayRespFollowUp(ptpClock->msgIbuf, &ptpClock->msgTmp.prespfollow);
        toInternalTime(&responseOriginTimestamp,
                       &ptpClock->msgTmp.prespfollow.responseOriginTimestamp);
        ptpClock->pdelay_t3 = responseOriginTimestamp;

        scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionfield, &correctionField);
        addTime(&correctionField, &correctionField, &ptpClock->correctionField_pDelayResp);
        updatePeerDelay(ptpClock, &correctionField, TRUE);
        ptpClock->waitingForPDelayRespFollowUp = FALSE;
        break;

    default:
        break;
    }
}

static void handleManagement(PtpClock *ptpClock, Boolean isFromSelf)
{
    (void)ptpClock; (void)isFromSelf;
}

static void handleSignaling(PtpClock *ptpClock, Boolean isFromSelf)
{
    (void)ptpClock; (void)isFromSelf;
}

/* ============================================================
 *  Issue functions: build and transmit PTP messages
 * ============================================================ */

static void issueDelayReqTimerExpired(PtpClock *ptpClock)
{
    /* In 802.1AS (P2P), PDelay runs in all active states */
    if (ptpClock->portDS.delayMechanism == P2P) {
        if (timerExpired(PDELAYREQ_INTERVAL_TIMER, ptpClock->itimer)) {
            timerStart(PDELAYREQ_INTERVAL_TIMER,
                       safeGetRand((UInteger32)pow2ms(ptpClock->portDS.logMinPdelayReqInterval + 1)),
                       ptpClock->itimer);
            DBGV("PDELAYREQ_INTERVAL_TIMEOUT");
            issuePDelayReq(ptpClock);
        }
    } else if (ptpClock->portDS.delayMechanism == E2E) {
        if (ptpClock->portDS.portState == PTP_SLAVE &&
            timerExpired(DELAYREQ_INTERVAL_TIMER, ptpClock->itimer)) {
            timerStart(DELAYREQ_INTERVAL_TIMER,
                       safeGetRand((UInteger32)pow2ms(ptpClock->portDS.logMinDelayReqInterval + 1)),
                       ptpClock->itimer);
            issueDelayReq(ptpClock);
        }
    }
}

static void issueAnnounce(PtpClock *ptpClock)
{
    msgPackAnnounce(ptpClock, ptpClock->msgObuf);
    if (!netSendGeneral(&ptpClock->netPath, ptpClock->msgObuf, ANNOUNCE_LENGTH)) {
        ERROR("issueAnnounce: send failed"); toState(ptpClock, PTP_FAULTY);
    } else {
        ptpClock->sentAnnounceSequenceId++;
        DBGV("issueAnnounce: seqId=%u", ptpClock->sentAnnounceSequenceId);
    }
}

static void issueSync(PtpClock *ptpClock)
{
    Timestamp    originTimestamp;
    TimeInternal internalTime = {0, 0};

    if (DEFAULT_TWO_STEP_FLAG) getTime(&internalTime);
    fromInternalTime(&internalTime, &originTimestamp);
    msgPackSync(ptpClock, ptpClock->msgObuf, &originTimestamp);

    if (!netSendEvent(&ptpClock->netPath, ptpClock->msgObuf, SYNC_LENGTH, &internalTime)) {
        ERROR("issueSync: send failed"); toState(ptpClock, PTP_FAULTY);
    } else {
        ptpClock->sentSyncSequenceId++;
        if (internalTime.seconds != 0 && ptpClock->defaultDS.twoStepFlag) {
            addTime(&internalTime, &internalTime, &ptpClock->outboundLatency);
            issueFollowup(ptpClock, &internalTime);
        }
    }
}

static void issueFollowup(PtpClock *ptpClock, const TimeInternal *time)
{
    Timestamp preciseOriginTimestamp;
    fromInternalTime(time, &preciseOriginTimestamp);
    msgPackFollowUp(ptpClock, ptpClock->msgObuf, &preciseOriginTimestamp);

    if (!netSendGeneral(&ptpClock->netPath, ptpClock->msgObuf, FOLLOW_UP_LENGTH)) {
        ERROR("issueFollowup: send failed"); toState(ptpClock, PTP_FAULTY);
    }
}

static void issueDelayReq(PtpClock *ptpClock)
{
    Timestamp    originTimestamp;
    TimeInternal internalTime = {0, 0};

    if (DEFAULT_TWO_STEP_FLAG) getTime(&internalTime);
    fromInternalTime(&internalTime, &originTimestamp);
    msgPackDelayReq(ptpClock, ptpClock->msgObuf, &originTimestamp);

    if (!netSendEvent(&ptpClock->netPath, ptpClock->msgObuf, DELAY_REQ_LENGTH, &internalTime)) {
        ERROR("issueDelayReq: send failed"); toState(ptpClock, PTP_FAULTY);
    } else {
        ptpClock->sentDelayReqSequenceId++;
        if (internalTime.seconds != 0) {
            addTime(&internalTime, &internalTime, &ptpClock->outboundLatency);
            ptpClock->timestamp_delayReqSend = internalTime;
        }
    }
}

static void issueDelayResp(PtpClock *ptpClock, const TimeInternal *time, const MsgHeader *header)
{
    Timestamp receiveTimestamp;
    fromInternalTime(time, &receiveTimestamp);
    msgPackDelayResp(ptpClock, ptpClock->msgObuf, header, &receiveTimestamp);

    if (!netSendGeneral(&ptpClock->netPath, ptpClock->msgObuf, DELAY_RESP_LENGTH)) {
        ERROR("issueDelayResp: send failed"); toState(ptpClock, PTP_FAULTY);
    }
}

static void issuePDelayReq(PtpClock *ptpClock)
{
    Timestamp    originTimestamp;
    TimeInternal internalTime = {0, 0};

    if (DEFAULT_TWO_STEP_FLAG) getTime(&internalTime);
    fromInternalTime(&internalTime, &originTimestamp);
    msgPackPDelayReq(ptpClock, ptpClock->msgObuf, &originTimestamp);

    if (!netSendPeerEvent(&ptpClock->netPath, ptpClock->msgObuf, PDELAY_REQ_LENGTH, &internalTime)) {
        ERROR("issuePDelayReq: send failed"); toState(ptpClock, PTP_FAULTY);
    } else {
        ptpClock->sentPDelayReqSequenceId++;
        if (internalTime.seconds != 0) {
            addTime(&internalTime, &internalTime, &ptpClock->outboundLatency);
            ptpClock->pdelay_t1 = internalTime;
        }
    }
}

static void issuePDelayResp(PtpClock *ptpClock, TimeInternal *time, const MsgHeader *pDelayReqHeader)
{
    Timestamp requestReceiptTimestamp;
    fromInternalTime(time, &requestReceiptTimestamp);
    msgPackPDelayResp(ptpClock->msgObuf, pDelayReqHeader, &requestReceiptTimestamp);

    if (!netSendPeerEvent(&ptpClock->netPath, ptpClock->msgObuf, PDELAY_RESP_LENGTH, time)) {
        ERROR("issuePDelayResp: send failed"); toState(ptpClock, PTP_FAULTY);
    } else {
        if (time->seconds != 0)
            addTime(time, time, &ptpClock->outboundLatency);
    }
}

static void issuePDelayRespFollowUp(PtpClock *ptpClock, const TimeInternal *time,
                                    const MsgHeader *pDelayReqHeader)
{
    Timestamp responseOriginTimestamp;
    fromInternalTime(time, &responseOriginTimestamp);
    msgPackPDelayRespFollowUp(ptpClock->msgObuf, pDelayReqHeader, &responseOriginTimestamp);

    if (!netSendPeerGeneral(&ptpClock->netPath, ptpClock->msgObuf, PDELAY_RESP_FOLLOW_UP_LENGTH)) {
        ERROR("issuePDelayRespFollowUp: send failed"); toState(ptpClock, PTP_FAULTY);
    }
}
