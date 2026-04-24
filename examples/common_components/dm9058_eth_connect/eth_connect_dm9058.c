/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <string.h>
#include "sdkconfig.h"
#include "dm9058_eth_connect.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_eth_mac_dm9058.h"
#include "esp_eth_phy_dm9058.h"

#if CONFIG_EXAMPLE_DM9058_ENABLE_PTP
#include "esp_eth_time.h"
#endif

static const char *TAG = "dm9058_eth";

static esp_eth_handle_t            s_eth_handle = NULL;
static esp_eth_mac_t              *s_mac        = NULL;
static esp_eth_phy_t              *s_phy        = NULL;
static esp_eth_netif_glue_handle_t s_glue       = NULL;
static esp_netif_t                *s_netif      = NULL;
static bool                        s_spi_bus_inited = false;

esp_err_t dm9058_ethernet_connect(void)
{
    if (s_eth_handle != NULL) {
        ESP_LOGW(TAG, "DM9058 already connected");
        return ESP_OK;
    }

    esp_err_t ret = ESP_OK;

    /* 1. Create netif with default Ethernet config */
    esp_netif_inherent_config_t base_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();
    base_cfg.if_desc    = "dm9058";
    base_cfg.route_prio = 64;
    esp_netif_config_t netif_cfg = {
        .base  = &base_cfg,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };
    s_netif = esp_netif_new(&netif_cfg);
    ESP_GOTO_ON_FALSE(s_netif != NULL, ESP_FAIL, err, TAG, "esp_netif_new failed");

    /* 2. SPI bus + per-device config */
    gpio_install_isr_service(0); /* OK if already installed; ESP_ERR_INVALID_STATE is benign */

    spi_bus_config_t buscfg = {
        .miso_io_num   = CONFIG_EXAMPLE_DM9058_SPI_MISO_GPIO,
        .mosi_io_num   = CONFIG_EXAMPLE_DM9058_SPI_MOSI_GPIO,
        .sclk_io_num   = CONFIG_EXAMPLE_DM9058_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_GOTO_ON_ERROR(spi_bus_initialize(CONFIG_EXAMPLE_DM9058_SPI_HOST,
                                         &buscfg, SPI_DMA_CH_AUTO),
                      err, TAG, "spi_bus_initialize failed");
    s_spi_bus_inited = true;

    spi_device_interface_config_t devcfg = {
        .mode           = 0,
        .clock_speed_hz = CONFIG_EXAMPLE_DM9058_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num   = CONFIG_EXAMPLE_DM9058_SPI_CS_GPIO,
        .queue_size     = 20,
    };

    /* 3. MAC + PHY (mirrors dm9058_basic ethernet_init.c) */
    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr       = CONFIG_EXAMPLE_DM9058_PHY_ADDR;
    phy_cfg.reset_gpio_num = CONFIG_EXAMPLE_DM9058_PHY_RST_GPIO;

    eth_dm9058_config_t dm_cfg = ETH_DM9058_DEFAULT_CONFIG(
        CONFIG_EXAMPLE_DM9058_SPI_HOST, &devcfg);
    dm_cfg.int_gpio_num = CONFIG_EXAMPLE_DM9058_SPI_INT_GPIO;
#if CONFIG_EXAMPLE_DM9058_SPI_INT_GPIO < 0
    dm_cfg.poll_period_ms = CONFIG_EXAMPLE_DM9058_SPI_POLL_MS;
#endif
    s_mac = esp_eth_mac_new_dm9058(&dm_cfg, &mac_cfg);
    ESP_GOTO_ON_FALSE(s_mac != NULL, ESP_FAIL, err, TAG, "esp_eth_mac_new_dm9058 failed");
    s_phy = esp_eth_phy_new_dm9058(&phy_cfg);
    ESP_GOTO_ON_FALSE(s_phy != NULL, ESP_FAIL, err, TAG, "esp_eth_phy_new_dm9058 failed");

    /* 4. Driver install + custom MAC address (DM9058 has no factory-burned MAC) */
    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(s_mac, s_phy);
    ESP_GOTO_ON_ERROR(esp_eth_driver_install(&eth_cfg, &s_eth_handle),
                      err, TAG, "esp_eth_driver_install failed");

    uint8_t mac_addr[6] = {0};
    ESP_GOTO_ON_ERROR(esp_read_mac(mac_addr, ESP_MAC_ETH),
                      err, TAG, "esp_read_mac failed");
    ESP_GOTO_ON_ERROR(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr),
                      err, TAG, "set MAC address failed");

    /* 5. Attach to netif and start */
    s_glue = esp_eth_new_netif_glue(s_eth_handle);
    ESP_GOTO_ON_FALSE(s_glue != NULL, ESP_FAIL, err, TAG, "esp_eth_new_netif_glue failed");
    ESP_GOTO_ON_ERROR(esp_netif_attach(s_netif, s_glue),
                      err, TAG, "esp_netif_attach failed");
    ESP_GOTO_ON_ERROR(esp_eth_start(s_eth_handle),
                      err, TAG, "esp_eth_start failed");

    /* 6. (Optional) PTP clock init — mirrors dm9058_ptp/main/ptp_main.c */
#if CONFIG_EXAMPLE_DM9058_ENABLE_PTP
    esp_eth_clock_cfg_t clk_cfg = {
        .eth_hndl = s_eth_handle,
#if CONFIG_EXAMPLE_DM9058_PTP_TRANSPORT_802_1AS
        .transport = ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_1AS,
#elif CONFIG_EXAMPLE_DM9058_PTP_TRANSPORT_UDP_V4
        .transport = ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4,
#elif CONFIG_EXAMPLE_DM9058_PTP_TRANSPORT_UDP_V6
        .transport = ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV6,
#else
        .transport = ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3,
#endif
    };
    ESP_GOTO_ON_ERROR(esp_eth_clock_init(CLOCK_PTP_SYSTEM, &clk_cfg),
                      err, TAG, "esp_eth_clock_init failed");
    ESP_LOGI(TAG, "PTP clock initialised (transport=%d)", clk_cfg.transport);
#endif

    ESP_LOGI(TAG, "DM9058 Ethernet connected");
    return ESP_OK;

err:
    dm9058_ethernet_disconnect();
    return ret;
}

esp_err_t dm9058_ethernet_disconnect(void)
{
    if (s_eth_handle != NULL) {
        esp_eth_stop(s_eth_handle);
    }
    if (s_glue != NULL) {
        esp_eth_del_netif_glue(s_glue);
        s_glue = NULL;
    }
    if (s_eth_handle != NULL) {
        esp_eth_driver_uninstall(s_eth_handle);
        s_eth_handle = NULL;
    }
    if (s_phy != NULL) {
        s_phy->del(s_phy);
        s_phy = NULL;
    }
    if (s_mac != NULL) {
        s_mac->del(s_mac);
        s_mac = NULL;
    }
    if (s_netif != NULL) {
        esp_netif_destroy(s_netif);
        s_netif = NULL;
    }
    if (s_spi_bus_inited) {
        spi_bus_free(CONFIG_EXAMPLE_DM9058_SPI_HOST);
        s_spi_bus_inited = false;
    }
    return ESP_OK;
}

esp_eth_handle_t dm9058_get_eth_handle(void)
{
    return s_eth_handle;
}

esp_netif_t *dm9058_get_netif(void)
{
    return s_netif;
}
