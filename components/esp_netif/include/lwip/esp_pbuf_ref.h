/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file esp_pbuf reference interface file
 */

#pragma once

#include <stdbool.h>
#include <time.h>
#include <stddef.h>
#include "lwip/pbuf.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Allocate custom pbuf containing pointer to a private l2-free function
 *
 * @note pbuf_free() will deallocate this custom pbuf and call the driver assigned free function
 */
struct pbuf* esp_pbuf_allocate(esp_netif_t *esp_netif, void *buffer, size_t len, void *l2_buff);

/**
 * @brief Attach RX timestamp metadata to an esp_netif custom pbuf
 *
 * @return true if metadata was stored; false if p is not an esp_netif custom pbuf
 */
bool esp_pbuf_set_rx_timestamp(struct pbuf *p, const struct timespec *ts);

/**
 * @brief Read RX timestamp metadata from an esp_netif custom pbuf
 *
 * @return true if metadata was present; false otherwise
 */
bool esp_pbuf_get_rx_timestamp(const struct pbuf *p, struct timespec *ts);

#ifdef __cplusplus
}
#endif
