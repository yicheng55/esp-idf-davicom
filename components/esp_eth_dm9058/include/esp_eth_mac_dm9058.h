/*
 * SPDX-FileCopyrightText: 2019-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include "esp_eth_com.h"
#include "esp_eth_mac.h"
#include "esp_eth_mac_spi.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_ETH_SPI_ETHERNET_DM9058
/**
 * @brief DM9058 specific configuration
 *
 */
typedef struct {
    int int_gpio_num;                                   /*!< Interrupt GPIO number, set -1 to not use interrupt and to poll rx status periodically */
    uint32_t poll_period_ms;                            /*!< Period in ms to poll rx status when interrupt mode is not used */
    spi_host_device_t spi_host_id;                      /*!< SPI peripheral (this field is invalid when custom SPI driver is defined) */
    spi_device_interface_config_t *spi_devcfg;          /*!< SPI device configuration (this field is invalid when custom SPI driver is defined) */
    eth_spi_custom_driver_config_t custom_spi_driver;   /*!< Custom SPI driver definitions */
} eth_dm9058_config_t;

/**
 * @brief List of DM9058 specific commands for ioctl API
 */
typedef enum {
    ETH_MAC_DM9058_CMD_PTP_ENABLE = ETH_CMD_CUSTOM_MAC_CMDS_OFFSET, /*!< Enable IEEE1588 timestamping in DM9058 */
    ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS,                             /*!< Enable/disable automatic TX/RX PTP processing in MAC driver */
    ETH_MAC_DM9058_CMD_S_PTP_TIME,                                   /*!< Set PTP time in DM9058 */
    ETH_MAC_DM9058_CMD_G_PTP_TIME,                                   /*!< Get PTP time from DM9058 */
    ETH_MAC_DM9058_CMD_ADJ_PTP_FREQ,                                 /*!< Adjust PTP frequency by ppb value */
    ETH_MAC_DM9058_CMD_ADJ_PTP_TIME,                                 /*!< Adjust PTP time by signed offset */
    ETH_MAC_DM9058_CMD_G_PTP_TX_TIME,                                /*!< Get last TX timestamp from DM9058 */
    ETH_MAC_DM9058_CMD_G_PTP_RX_TIME,                                /*!< Get last RX timestamp from DM9058 */
    ETH_MAC_DM9058_CMD_S_TARGET_TIME,                                /*!< Set Target Time at which interrupt is invoked when PTP time exceeds this value*/
    ETH_MAC_DM9058_CMD_S_TARGET_CB                                   /*!< Set pointer to a callback function invoked when PTP time exceeds Target Time */
} eth_mac_dm9058_io_cmd_t;

/**
 * @brief PTP transport type for DM9058
 */
typedef enum {
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4    = 0, /*!< PTP over UDP/IPv4 (default) */
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV6,         /*!< PTP over UDP/IPv6 */
    ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3,        /*!< PTP over IEEE 802.3 Ethernet */
    ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_1AS,      /*!< PTP over IEEE 802.1AS (gPTP) */
} esp_eth_ptp_dm9058_transport_t;

/**
 * @brief Configuration passed to ETH_MAC_DM9058_CMD_PTP_ENABLE ioctl
 */
typedef struct {
    bool enable;                                /*!< Enable or disable PTP */
    esp_eth_ptp_dm9058_transport_t transport;   /*!< PTP transport type */
} esp_eth_ptp_dm9058_enable_config_t;


/**
 * @brief Default DM9058 specific configuration
 *
 */
#define ETH_DM9058_DEFAULT_CONFIG(spi_host, spi_devcfg_p) \
    {                                           \
        .int_gpio_num = 4,                      \
        .poll_period_ms = 0,                    \
        .spi_host_id = spi_host,                \
        .spi_devcfg = spi_devcfg_p,             \
        .custom_spi_driver = ETH_DEFAULT_SPI,   \
    }

/**
* @brief Create DM9058 Ethernet MAC instance
*
* @param dm9058_config: DM9058 specific configuration
* @param mac_config: Ethernet MAC configuration
*
* @return
*      - instance: create MAC instance successfully
*      - NULL: create MAC instance failed because some error occurred
*/
esp_eth_mac_t *esp_eth_mac_new_dm9058(const eth_dm9058_config_t *dm9058_config, const eth_mac_config_t *mac_config);
#endif // CONFIG_ETH_SPI_ETHERNET_DM9058

#ifdef __cplusplus
}
#endif
