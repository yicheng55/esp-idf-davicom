/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * gPTP (IEEE 802.1AS) internal definitions for ESP-IDF.
 * Ported from ptpd-2.0.0 reference (constants.h, datatypes.h, dep/)
 * and adapted for ESP32 / L2TAP transport.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <arpa/inet.h>      /* htons / htonl */
#include "esp_log.h"

/* ============================================================
 *  Primitive types (platform-independent IEEE 1588 spec §5.2)
 * ============================================================ */
typedef bool             Boolean;
#define TRUE  true
#define FALSE false

typedef uint8_t          Enumeration4;
typedef uint8_t          Enumeration8;
typedef uint16_t         Enumeration16;
typedef uint8_t          UInteger4;
typedef int8_t           Integer8;
typedef uint8_t          UInteger8;
typedef int16_t          Integer16;
typedef uint16_t         UInteger16;
typedef int32_t          Integer32;
typedef uint32_t         UInteger32;
typedef int64_t          Integer64;
typedef uint8_t          Nibble;
typedef char             Octet;

/* 48-bit unsigned integer (seconds field in PTP timestamps) */
typedef struct { uint32_t lsb; uint16_t msb; } UInteger48;

/* ============================================================
 *  Platform-dependent constants (dep/constants_dep.h)
 * ============================================================ */
#define IF_NAMESIZE                 16
#define IFACE_NAME_LENGTH           IF_NAMESIZE
#define NET_ADDRESS_LENGTH          16

#define CLOCK_IDENTITY_LENGTH       8
#define PTP_UUID_LENGTH             6       /* EUI-48 MAC */
#define FLAG_FIELD_LENGTH           2

#define PACKET_SIZE                 300
#define PBUF_QUEUE_SIZE             16

#define PTP_ETHERTYPE               0x88F7U

/* ============================================================
 *  gPTP / IEEE 802.1AS constants (constants.h adapted)
 * ============================================================ */
#define DEFAULT_INBOUND_LATENCY            0
#define DEFAULT_OUTBOUND_LATENCY           0
#define DEFAULT_NO_RESET_CLOCK             FALSE
#define DEFAULT_DOMAIN_NUMBER              0
#define DEFAULT_DELAY_MECHANISM            P2P   /* 802.1AS: forced P2P */
#define DEFAULT_AP                         2
#define DEFAULT_AI                         16
#define DEFAULT_DELAY_S                    6
#define DEFAULT_OFFSET_S                   1
#define DEFAULT_UTC_OFFSET                 37
#define DEFAULT_UTC_VALID                  FALSE
#define DEFAULT_PDELAYREQ_INTERVAL         1
#define DEFAULT_DELAYREQ_INTERVAL          3
#define DEFAULT_SYNC_INTERVAL              0
#define DEFAULT_SYNC_RECEIPT_TIMEOUT       3
#define DEFAULT_ANNOUNCE_RECEIPT_TIMEOUT   6
#define DEFAULT_QUALIFICATION_TIMEOUT      (-9)
#define DEFAULT_FOREIGN_MASTER_TIME_WINDOW 4
#define DEFAULT_FOREIGN_MASTER_THRESHOLD   2
#define DEFAULT_CLOCK_CLASS                128
#define DEFAULT_CLOCK_CLASS_SLAVE_ONLY     255
#define DEFAULT_CLOCK_ACCURACY             0xFE
#define DEFAULT_PRIORITY1                  128
#define DEFAULT_PRIORITY2                  128
#define DEFAULT_CLOCK_VARIANCE             5000
#define DEFAULT_MAX_FOREIGN_RECORDS        5
#define DEFAULT_TWO_STEP_FLAG              FALSE   /* 802.1AS: ONE_STEP default */
#define DEFAULT_TIME_SOURCE                INTERNAL_OSCILLATOR
#define DEFAULT_TIME_TRACEABLE             FALSE
#define DEFAULT_FREQUENCY_TRACEABLE        FALSE
#define DEFAULT_TIMESCALE                  PTP_TIMESCALE
#define DEFAULT_PARENTS_STATS              FALSE
#define NUMBER_PORTS                       1
#define VERSION_PTP                        2
#define BOUNDARY_CLOCK                     FALSE
#define SLAVE_ONLY                         FALSE
#define NO_ADJUST                          FALSE

/* 802.1AS profile intervals (log2 seconds) */
#ifndef DEFAULT_ANNOUNCE_INTERVAL_8021AS
#define DEFAULT_ANNOUNCE_INTERVAL_8021AS         0    /* 2^0  = 1 s     */
#endif
#ifndef DEFAULT_SYNC_INTERVAL_8021AS
#define DEFAULT_SYNC_INTERVAL_8021AS             (-3) /* 2^-3 = 125 ms  */
#endif
#ifndef DEFAULT_PDELAYREQ_INTERVAL_8021AS
#define DEFAULT_PDELAYREQ_INTERVAL_8021AS        (-2) /* 2^-2 = 250 ms  */
#endif
#ifndef DEFAULT_ANNOUNCE_RECEIPT_TIMEOUT_8021AS
#define DEFAULT_ANNOUNCE_RECEIPT_TIMEOUT_8021AS  3
#endif

/* transportSpecific / SdoId */
#define DEFAULT_TRANSPORT_SPECIFIC_1588    0x0
#define DEFAULT_TRANSPORT_SPECIFIC_8021AS  0x1

/* Transport type IDs */
#define TRANSPORT_UDP_IPV4     1
#define TRANSPORT_UDP_IPV6     2
#define TRANSPORT_IEEE_802_3   3
#define TRANSPORT_IEEE_802_1AS 4
#define DEFAULT_NETWORK_TRANSPORT TRANSPORT_IEEE_802_1AS

/* Calibration thresholds */
#define DEFAULT_CALIBRATED_OFFSET_NS       10000
#define DEFAULT_UNCALIBRATED_OFFSET_NS     1000000
#define MAX_ADJ_OFFSET_NS                  100000000

/* Packet lengths */
#define HEADER_LENGTH                      34
#define ANNOUNCE_LENGTH                    64
#define SYNC_LENGTH                        44
#define FOLLOW_UP_LENGTH                   44
#define PDELAY_REQ_LENGTH                  54
#define DELAY_REQ_LENGTH                   44
#define DELAY_RESP_LENGTH                  54
#define PDELAY_RESP_LENGTH                 54
#define PDELAY_RESP_FOLLOW_UP_LENGTH       54
#define MANAGEMENT_LENGTH                  48

/* ADJ frequency max (ppb) */
#define ADJ_FREQ_MAX                       512000

/* ============================================================
 *  Enumerations
 * ============================================================ */
enum { DFLT_DOMAIN_NUMBER = 0, ALT1_DOMAIN_NUMBER, ALT2_DOMAIN_NUMBER, ALT3_DOMAIN_NUMBER };
enum { UDP_IPV4 = 1, UDP_IPV6, IEE_802_3, DeviceNet, ControlNet, PROFINET };
enum { ATOMIC_CLOCK = 0x10, GPS = 0x20, TERRESTRIAL_RADIO = 0x30,
       PTP = 0x40, NTP = 0x50, HAND_SET = 0x60, OTHER = 0x90, INTERNAL_OSCILLATOR = 0xA0 };

enum {
    PTP_INITIALIZING = 0, PTP_FAULTY, PTP_DISABLED, PTP_LISTENING,
    PTP_PRE_MASTER, PTP_MASTER, PTP_PASSIVE, PTP_UNCALIBRATED, PTP_SLAVE
};

enum { E2E = 1, P2P = 2, DELAY_DISABLED = 0xFE };

enum {
    PDELAYREQ_INTERVAL_TIMER = 0,
    DELAYREQ_INTERVAL_TIMER,
    SYNC_INTERVAL_TIMER,
    ANNOUNCE_RECEIPT_TIMER,
    ANNOUNCE_INTERVAL_TIMER,
    QUALIFICATION_TIMEOUT,
    TIMER_ARRAY_SIZE
};

enum {
    SYNC = 0x0, DELAY_REQ, PDELAY_REQ, PDELAY_RESP,
    FOLLOW_UP = 0x8, DELAY_RESP, PDELAY_RESP_FOLLOW_UP, ANNOUNCE,
    SIGNALING, MANAGEMENT
};

enum { CTRL_SYNC = 0x00, CTRL_DELAY_REQ, CTRL_FOLLOW_UP, CTRL_DELAY_RESP, CTRL_MANAGEMENT, CTRL_OTHER };

enum {
    FLAG0_ALTERNATE_MASTER       = 0x01,
    FLAG0_TWO_STEP               = 0x02,
    FLAG0_UNICAST                = 0x04,
    FLAG0_PTP_PROFILE_SPECIFIC_1 = 0x20,
    FLAG0_PTP_PROFILE_SPECIFIC_2 = 0x40,
    FLAG0_SECURITY               = 0x80,
};

enum {
    FLAG1_LEAP61              = 0x01,
    FLAG1_LEAP59              = 0x02,
    FLAG1_UTC_OFFSET_VALID    = 0x04,
    FLAG1_PTP_TIMESCALE       = 0x08,
    FLAG1_TIME_TRACEABLE      = 0x10,
    FLAG1_FREQUENCY_TRACEABLE = 0x20,
};

enum {
    POWERUP                          = 0x0001,
    INITIALIZE                       = 0x0002,
    DESIGNATED_ENABLED               = 0x0004,
    DESIGNATED_DISABLED              = 0x0008,
    FAULT_CLEARED                    = 0x0010,
    FAULT_DETECTED                   = 0x0020,
    STATE_DECISION_EVENT             = 0x0040,
    QUALIFICATION_TIMEOUT_EXPIRES    = 0x0080,
    ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES = 0x0100,
    SYNCHRONIZATION_FAULT            = 0x0200,
    MASTER_CLOCK_SELECTED            = 0x0400,
    MASTER_CLOCK_CHANGED             = 0x0800,
};

enum { ARB_TIMESCALE, PTP_TIMESCALE };

/* ============================================================
 *  Debug / log macros
 * ============================================================ */
#define GPTP_TAG "gptp"

#ifdef CONFIG_GPTP_LOG_VERBOSE
#define DBGVV(...) ESP_LOGV(GPTP_TAG, __VA_ARGS__)
#define DBGV(...)  ESP_LOGV(GPTP_TAG, __VA_ARGS__)
#define DBG(...)   ESP_LOGD(GPTP_TAG, __VA_ARGS__)
#else
#define DBGVV(...) do {} while(0)
#define DBGV(...)  do {} while(0)
#define DBG(...)   ESP_LOGD(GPTP_TAG, __VA_ARGS__)
#endif

#define DBG_PRINTF(...) ESP_LOGD(GPTP_TAG, __VA_ARGS__)
#define ERROR(...)      ESP_LOGE(GPTP_TAG, __VA_ARGS__)

/* ============================================================
 *  Bit manipulation macros
 * ============================================================ */
#define getFlag(flagField, mask)    ((Boolean)(((flagField) & (mask)) == (mask)))
#define setFlag(flagField, mask)    ((flagField) |= (mask))
#define clearFlag(flagField, mask)  ((flagField) &= ~(mask))

/* ============================================================
 *  Endianness / byte-swap (using standard htons/htonl)
 * ============================================================ */
#define flip16(x)  htons(x)
#define flip32(x)  htonl(x)

/* ============================================================
 *  Timer helper: convert log2-seconds to milliseconds
 * ============================================================ */
#define pow2ms(a)  (((a) > 0) ? (1000 << (a)) : (1000 >> (-(a))))

/* ============================================================
 *  Data types (datatypes.h)
 * ============================================================ */

typedef struct { Integer64 scaledNanoseconds; } TimeInterval;

typedef struct {
    UInteger48 secondsField;
    UInteger32 nanosecondsField;
} Timestamp;

typedef Octet ClockIdentity[CLOCK_IDENTITY_LENGTH];

typedef struct {
    ClockIdentity clockIdentity;
    UInteger16    portNumber;
} PortIdentity;

typedef struct {
    UInteger8    clockClass;
    Enumeration8 clockAccuracy;
    UInteger16   offsetScaledLogVariance;
} ClockQuality;

typedef struct {
    Nibble       transportSpecific;
    Enumeration4 messageType;
    UInteger4    versionPTP;
    UInteger16   messageLength;
    UInteger8    domainNumber;
    Octet        flagField[2];
    Integer64    correctionfield;
    PortIdentity sourcePortIdentity;
    UInteger16   sequenceId;
    UInteger8    controlField;
    Integer8     logMessageInterval;
} MsgHeader;

typedef struct {
    Timestamp originTimestamp;
    Integer16 currentUtcOffset;
    UInteger8 grandmasterPriority1;
    ClockQuality grandmasterClockQuality;
    UInteger8 grandmasterPriority2;
    ClockIdentity grandmasterIdentity;
    UInteger16 stepsRemoved;
    Enumeration8 timeSource;
} MsgAnnounce;

typedef struct { Timestamp originTimestamp; }          MsgSync;
typedef struct { Timestamp originTimestamp; }          MsgDelayReq;
typedef struct { Timestamp preciseOriginTimestamp; }   MsgFollowUp;
typedef struct { Timestamp originTimestamp; }          MsgPDelayReq;

typedef struct {
    Timestamp    receiveTimestamp;
    PortIdentity requestingPortIdentity;
} MsgDelayResp;

typedef struct {
    Timestamp    requestReceiptTimestamp;
    PortIdentity requestingPortIdentity;
} MsgPDelayResp;

typedef struct {
    Timestamp    responseOriginTimestamp;
    PortIdentity requestingPortIdentity;
} MsgPDelayRespFollowUp;

typedef struct {
    PortIdentity targetPortIdentity;
    char        *tlv;
} MsgSignaling;

typedef struct {
    PortIdentity targetPortIdentity;
    UInteger8    startingBoundaryHops;
    UInteger8    boundaryHops;
    Enumeration4 actionField;
    char        *tlv;
} MsgManagement;

typedef struct { Integer32 seconds; Integer32 nanoseconds; } TimeInternal;

typedef struct {
    Integer32  interval;
    Integer32  left;
    Boolean    expire;
} IntervalTimer;

/* Exponential smoothing filter */
typedef struct {
    Integer32 y_prev, y_sum;
    Integer16 s;
    Integer16 s_prev;
    Integer32 n;
} Filter;

/* ============================================================
 *  Network queue using static packet buffers (ESP-IDF: no pbuf)
 * ============================================================ */
#define GPTP_PKT_SIZE  PACKET_SIZE

typedef struct {
    uint8_t      buf[GPTP_PKT_SIZE];
    uint16_t     len;
    TimeInternal ts;
} GptpPkt;

typedef struct {
    GptpPkt  pkts[PBUF_QUEUE_SIZE];
    int32_t  get;
    int32_t  put;
    int32_t  count;
} BufQueue;

/* Network path state (adapted for L2TAP) */
typedef struct {
    int       fd;              /* L2TAP file descriptor    */
    uint8_t   hwaddr[6];       /* Our MAC address          */
    BufQueue  eventQ;          /* Event message queue      */
    BufQueue  generalQ;        /* General message queue    */
} NetPath;

/* Foreign master record */
typedef struct {
    PortIdentity foreignMasterPortIdentity;
    UInteger16   foreignMasterAnnounceMessages;
    MsgAnnounce  announce;
    MsgHeader    header;
} ForeignMasterRecord;

/* IEEE 1588 data sets */
typedef struct {
    Boolean      twoStepFlag;
    ClockIdentity clockIdentity;
    UInteger16   numberPorts;
    ClockQuality clockQuality;
    UInteger8    priority1;
    UInteger8    priority2;
    UInteger8    domainNumber;
    Boolean      slaveOnly;
} DefaultDS;

typedef struct {
    UInteger16   stepsRemoved;
    TimeInternal offsetFromMaster;
    TimeInternal meanPathDelay;
} CurrentDS;

typedef struct {
    PortIdentity parentPortIdentity;
    Boolean      parentStats;
    Integer16    observedParentOffsetScaledLogVariance;
    Integer32    observedParentClockPhaseChangeRate;
    ClockIdentity grandmasterIdentity;
    ClockQuality  grandmasterClockQuality;
    UInteger8     grandmasterPriority1;
    UInteger8     grandmasterPriority2;
} ParentDS;

typedef struct {
    Integer16    currentUtcOffset;
    Boolean      currentUtcOffsetValid;
    Boolean      leap59;
    Boolean      leap61;
    Boolean      timeTraceable;
    Boolean      frequencyTraceable;
    Boolean      ptpTimescale;
    Enumeration8 timeSource;
} TimePropertiesDS;

typedef struct {
    PortIdentity portIdentity;
    Enumeration8 portState;
    Integer8     logMinDelayReqInterval;
    TimeInternal peerMeanPathDelay;
    Integer8     logAnnounceInterval;
    UInteger8    announceReceiptTimeout;
    Integer8     logSyncInterval;
    Enumeration8 delayMechanism;
    Integer8     logMinPdelayReqInterval;
    UInteger4    versionNumber;
} PortDS;

typedef struct {
    ForeignMasterRecord *records;
    UInteger16  count;
    Integer16   capacity;
    Integer16   i;
    Integer16   best;
} ForeignMasterDS;

typedef struct {
    Boolean   noResetClock;
    Boolean   noAdjust;
    Integer16 ap, ai;
    Integer16 sDelay;
    Integer16 sOffset;
} Servo;

typedef struct {
    Integer8     announceInterval;
    Integer8     syncInterval;
    ClockQuality clockQuality;
    UInteger8    priority1;
    UInteger8    priority2;
    UInteger8    domainNumber;
    Boolean      slaveOnly;
    Integer16    currentUtcOffset;
    Octet        ifaceName[IFACE_NAME_LENGTH];
    Integer16    maxForeignRecords;
    Enumeration8 delayMechanism;
    Enumeration8 transportType;
    Servo        servo;
} RunTimeOpts;

/* Main PTP clock state */
typedef struct {
    DefaultDS          defaultDS;
    CurrentDS          currentDS;
    ParentDS           parentDS;
    TimePropertiesDS   timePropertiesDS;
    PortDS             portDS;
    ForeignMasterDS    foreignMasterDS;

    MsgHeader msgTmpHeader;

    union {
        MsgSync              sync;
        MsgFollowUp          follow;
        MsgDelayReq          req;
        MsgDelayResp         resp;
        MsgPDelayReq         preq;
        MsgPDelayResp        presp;
        MsgPDelayRespFollowUp prespfollow;
        MsgManagement        manage;
        MsgAnnounce          announce;
        MsgSignaling         signaling;
    } msgTmp;

    Octet  msgObuf[PACKET_SIZE];
    Octet  msgIbuf[PACKET_SIZE];
    ssize_t msgIbufLength;

    TimeInternal Tms;
    TimeInternal Tsm;

    TimeInternal pdelay_t1;
    TimeInternal pdelay_t2;
    TimeInternal pdelay_t3;
    TimeInternal pdelay_t4;

    TimeInternal timestamp_syncRecieve;
    TimeInternal timestamp_delayReqSend;
    TimeInternal timestamp_delayReqRecieve;

    TimeInternal correctionField_sync;
    TimeInternal correctionField_pDelayResp;

    UInteger16 sentPDelayReqSequenceId;
    UInteger16 sentDelayReqSequenceId;
    UInteger16 sentSyncSequenceId;
    UInteger16 sentAnnounceSequenceId;

    UInteger16 recvPDelayReqSequenceId;
    UInteger16 recvSyncSequenceId;

    Boolean waitingForFollowUp;
    Boolean waitingForPDelayRespFollowUp;

    Filter  ofm_filt;
    Filter  owd_filt;
    Filter  slv_filt;
    UInteger16 offsetHistory[2];
    Integer32  observedDrift;

    Boolean messageActivity;

    IntervalTimer itimer[TIMER_ARRAY_SIZE];

    NetPath netPath;

    Enumeration8 recommendedState;

    Octet portUuidField[PTP_UUID_LENGTH];

    TimeInternal inboundLatency;
    TimeInternal outboundLatency;

    Servo servo;

    Integer32 events;

    RunTimeOpts *rtOpts;
} PtpClock;
