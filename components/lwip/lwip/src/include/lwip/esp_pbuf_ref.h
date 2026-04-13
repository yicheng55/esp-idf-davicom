/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <time.h>
#include <stddef.h>

#include "lwip/pbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

struct esp_netif_obj;
typedef struct esp_netif_obj esp_netif_t;

struct pbuf *esp_pbuf_allocate(esp_netif_t *esp_netif, void *buffer, size_t len, void *l2_buff);
bool esp_pbuf_set_rx_timestamp(struct pbuf *p, const struct timespec *ts);
bool esp_pbuf_get_rx_timestamp(const struct pbuf *p, struct timespec *ts);

#ifdef __cplusplus
}
#endif