/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once

#include "esp_err.h"
#include "esp_eth.h"
#include "esp_eth_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SPI bus, install DM9058 MAC/PHY driver, attach the driver
 *        to a new netif and start Ethernet. When CONFIG_EXAMPLE_DM9058_ENABLE_PTP
 *        is enabled, the PTP clock is also initialized using the transport
 *        selected via Kconfig.
 *
 * Does not block on DHCP — caller is responsible for any IP-event handling.
 *
 * @return ESP_OK on success, otherwise an esp_err_t propagated from the
 *         underlying SPI / esp_eth / esp_netif APIs.
 */
esp_err_t dm9058_ethernet_connect(void);

/**
 * @brief Stop Ethernet, uninstall the driver and free the associated netif.
 */
esp_err_t dm9058_ethernet_disconnect(void);

/**
 * @brief Get the active DM9058 esp_eth handle (or NULL if not connected).
 *        Needed by PTP code (e.g. esp_eth_clock_init / ptpd).
 */
esp_eth_handle_t dm9058_get_eth_handle(void);

/**
 * @brief Get the netif created for the DM9058 interface (or NULL if not
 *        connected). Useful when callers want to register IP-event handlers.
 */
esp_netif_t *dm9058_get_netif(void);

#ifdef __cplusplus
}
#endif
