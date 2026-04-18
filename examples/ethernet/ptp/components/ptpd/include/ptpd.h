/*
 * SPDX-FileCopyrightText: 2020-2024 The Apache Software Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPDX-FileContributor: 2024 Espressif Systems (Shanghai) CO LTD
 */

/****************************************************************************
 * apps/include/netutils/ptpd.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __APPS_INCLUDE_NETUTILS_PTPD_H
#define __APPS_INCLUDE_NETUTILS_PTPD_H

// ESP_PTP
#include <time.h>
#ifndef FAR
#define FAR
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Port state values for ptpd_status_s.port_state */

#define PTPD_PORT_STATE_INITIALIZING 0
#define PTPD_PORT_STATE_LISTENING    1
#define PTPD_PORT_STATE_MASTER       2
#define PTPD_PORT_STATE_SLAVE        3
#define PTPD_PORT_STATE_PASSIVE      4
#define PTPD_PORT_STATE_FAULT        5

/* Delay mechanism values for ptpd_status_s.delay_mechanism */

#define PTPD_DELAY_MECHANISM_E2E     0
#define PTPD_DELAY_MECHANISM_P2P     1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* PTPD status information structure */

struct ptpd_status_s
{
  /* Is there a valid remote clock source active? */

  bool clock_source_valid;

  /* Information about selected best clock source */

  struct
  {
    uint8_t id[8];     /* Clock identity */
    int utcoffset;     /* Offset between clock time and UTC time (seconds) */
    int priority1;     /* Main priority field */
    int clockclass;    /* Clock class (IEEE-1588, lower is better) */
    int accuracy;      /* Clock accuracy (IEEE-1588, lower is better) */
    int variance;      /* Clock variance (IEEE-1588, lower is better) */
    int priority2;     /* Secondary priority field */
    uint8_t gm_id[8];  /* Grandmaster clock identity */
    int stepsremoved;  /* How many steps from grandmaster clock */
    int timesource;    /* Type of time source (IEEE-1588) */
  } clock_source_info;

  /* When was clock last updated or adjusted (CLOCK_REALTIME).
   * Matches last_received_sync but in different clock.
   */

  struct timespec last_clock_update;

  /* Details of clock adjustment made at last_clock_update */

  int64_t last_delta_ns;     /* Latest measured clock error */
  int64_t last_adjtime_ns;   /* Previously applied adjtime() offset */

  /* Averaged clock drift estimate (parts per billion).
   * Positive means remote clock runs faster than local clock before
   * adjustment.
   */

  long drift_ppb;

  /* Averaged path delay (E2E mode) */

  long path_delay_ns;

  /* Averaged peer delay (P2P / gPTP mode). Zero when using E2E. */

  long peer_delay_ns;

  /* Active delay mechanism at build time */

  int delay_mechanism; /* PTPD_DELAY_MECHANISM_E2E or PTPD_DELAY_MECHANISM_P2P */

  /* Current port state */

  int port_state; /* PTPD_PORT_STATE_* */

  /* Latest offset from master computed at last Sync (nanoseconds) */

  int64_t offset_from_master_ns;

  /* Timestamps of latest received packets (CLOCK_MONOTONIC) */

  struct timespec last_received_multicast; /* Any multicast packet */
  struct timespec last_received_announce;  /* Announce from any server */
  struct timespec last_received_sync;      /* Sync from selected source */

  /* Timestamps of latest transmitted packets (CLOCK_MONOTONIC) */

  struct timespec last_transmitted_sync;
  struct timespec last_transmitted_announce;
  struct timespec last_transmitted_delayresp;
  struct timespec last_transmitted_delayreq;
  struct timespec last_transmitted_pdelay_req; /* P2P mode only */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ptpd_start
 *
 * Description:
 *   Start the PTP daemon and bind it to specified interface.
 *
 * Input Parameters:
 *   interface - Name of the network interface to bind to, e.g. "eth0"
 *
 * Returned Value:
 *   On success, the non-negative task ID of the PTP daemon is returned;
 *   On failure, a negated errno value is returned.
 *
 ****************************************************************************/

int ptpd_start(FAR const char *interface);

/****************************************************************************
 * Name: ptpd_status
 *
 * Description:
 *   Query status from a running PTP daemon.
 *
 * Input Parameters:
 *   pid     - Process ID previously returned by ptpd_start()
 *   status  - Pointer to storage for status information.
 *
 * Returned Value:
 *   On success, returns OK.
 *   On failure, a negated errno value is returned.
 *
 * Assumptions/Limitations:
 *   Multiple threads with priority less than CONFIG_NETUTILS_PTPD_SERVERPRIO
 *   can request status simultaneously. If higher priority threads request
 *   status simultaneously, some of the requests may timeout.
 *
 ****************************************************************************/

int ptpd_status(int pid, FAR struct ptpd_status_s *status);

/****************************************************************************
 * Name: ptpd_stop
 *
 * Description:
 *   Stop PTP daemon
 *
 * Input Parameters:
 *   pid     - Process ID previously returned by ptpd_start()
 *
 * Returned Value:
 *   On success, returns OK.
 *   On failure, a negated errno value is returned.
 *
 ****************************************************************************/

int ptpd_stop(int pid);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_NETUTILS_PTPD_H */
