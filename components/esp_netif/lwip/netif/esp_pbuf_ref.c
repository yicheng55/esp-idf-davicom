/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file esp_pbuf reference
 * This file handles lwip custom pbufs interfacing with esp_netif
 * and the L2 free function esp_netif_free_rx_buffer()
 */

#include "lwip/mem.h"
#include "lwip/esp_pbuf_ref.h"
#include "esp_netif_net_stack.h"

/**
 * @brief Specific pbuf structure for pbufs allocated by ESP netif
 * of PBUF_REF type
 */
typedef struct esp_custom_pbuf
{
    struct pbuf_custom p;
    esp_netif_t *esp_netif;
    void* l2_buf;
    bool rx_ts_valid;
    struct timespec rx_ts;
} esp_custom_pbuf_t;

static void esp_pbuf_free(struct pbuf *pbuf);

static esp_custom_pbuf_t *esp_pbuf_custom_from_pbuf(const struct pbuf *p)
{
    const struct pbuf_custom *custom = (const struct pbuf_custom *)p;

    if (p == NULL || (p->flags & PBUF_FLAG_IS_CUSTOM) == 0) {
        return NULL;
    }

    if (custom->custom_free_function != esp_pbuf_free) {
        return NULL;
    }

    return (esp_custom_pbuf_t *)custom;
}

/**
 * @brief Free custom pbuf containing the L2 layer buffer allocated in the driver
 *
 * @param pbuf Custom pbuf holding the packet passed to lwip input
 * @note This function called as a custom_free_function() upon pbuf_free()
 */
static void esp_pbuf_free(struct pbuf *pbuf)
{
    esp_custom_pbuf_t* esp_pbuf = (esp_custom_pbuf_t*)pbuf;
    esp_netif_free_rx_buffer(esp_pbuf->esp_netif, esp_pbuf->l2_buf);
    mem_free(pbuf);
}

/**
 * @brief Allocate custom pbuf for supplied sp_netif
 * @param esp_netif esp-netif handle
 * @param buffer Buffer to allocate
 * @param len Size of the buffer
 * @param l2_buff External l2 buffe
 * @return Custom pbuf pointer on success; NULL if no free heap
 */
struct pbuf* esp_pbuf_allocate(esp_netif_t *esp_netif, void *buffer, size_t len, void *l2_buff)
{
    struct pbuf *p;

    esp_custom_pbuf_t* esp_pbuf  = mem_malloc(sizeof(esp_custom_pbuf_t));
    if (esp_pbuf == NULL) {
        return NULL;
    }
    esp_pbuf->p.custom_free_function = esp_pbuf_free;
    esp_pbuf->esp_netif = esp_netif;
    esp_pbuf->l2_buf = l2_buff;
    esp_pbuf->rx_ts_valid = false;
    p = pbuf_alloced_custom(PBUF_RAW, len, PBUF_REF, &esp_pbuf->p, buffer, len);
    if (p == NULL) {
        mem_free(esp_pbuf);
        return NULL;
    }
    return p;
}

bool esp_pbuf_set_rx_timestamp(struct pbuf *p, const struct timespec *ts)
{
    esp_custom_pbuf_t *esp_pbuf = esp_pbuf_custom_from_pbuf(p);

    if (esp_pbuf == NULL || ts == NULL) {
        return false;
    }

    esp_pbuf->rx_ts = *ts;
    esp_pbuf->rx_ts_valid = true;
    return true;
}

bool esp_pbuf_get_rx_timestamp(const struct pbuf *p, struct timespec *ts)
{
    esp_custom_pbuf_t *esp_pbuf = esp_pbuf_custom_from_pbuf(p);

    if (esp_pbuf == NULL || ts == NULL || !esp_pbuf->rx_ts_valid) {
        return false;
    }

    *ts = esp_pbuf->rx_ts;
    return true;
}
