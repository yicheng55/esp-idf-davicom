#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t seconds;
    uint32_t nanoseconds;
} esp_eth_ptp_dm9058_time_t;

typedef struct {
    esp_err_t (*reg_read)(void *io_ctx, uint8_t reg, uint8_t *value);
    esp_err_t (*reg_write)(void *io_ctx, uint8_t reg, uint8_t value);
    esp_err_t (*reg_burst_read)(void *io_ctx, uint8_t reg, uint8_t *buffer, size_t len);
    esp_err_t (*reg_burst_write)(void *io_ctx, uint8_t reg, const uint8_t *buffer, size_t len);
    void (*delay_us)(uint32_t us);
    void (*delay_ms)(uint32_t ms);
    bool (*lock)(void *io_ctx);
    void (*unlock)(void *io_ctx);
} esp_eth_ptp_dm9058_ops_t;

typedef struct {
    void *io_ctx;
    esp_eth_ptp_dm9058_ops_t ops;
    bool initialized;
    bool enabled;
    int64_t last_rate;
} esp_eth_ptp_dm9058_t;

typedef enum {
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4 = 0,
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV6,
    ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3,
} esp_eth_ptp_dm9058_transport_t;

esp_err_t esp_eth_ptp_dm9058_init(esp_eth_ptp_dm9058_t *ptp, void *io_ctx, const esp_eth_ptp_dm9058_ops_t *ops);
esp_err_t esp_eth_ptp_dm9058_enable(esp_eth_ptp_dm9058_t *ptp, bool enable, esp_eth_ptp_dm9058_transport_t transport);
esp_err_t esp_eth_ptp_dm9058_get_time(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time);
esp_err_t esp_eth_ptp_dm9058_set_time(esp_eth_ptp_dm9058_t *ptp, const esp_eth_ptp_dm9058_time_t *time);
esp_err_t esp_eth_ptp_dm9058_adj_time(esp_eth_ptp_dm9058_t *ptp, const esp_eth_ptp_dm9058_time_t *offset);
esp_err_t esp_eth_ptp_dm9058_adj_freq(esp_eth_ptp_dm9058_t *ptp, int32_t adj_ppb);
esp_err_t esp_eth_ptp_dm9058_get_tx_timestamp(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time);
esp_err_t esp_eth_ptp_dm9058_tx_timestamp(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time);
esp_err_t esp_eth_ptp_dm9058_rx_timestamp(const uint8_t *rx_ts_buffer, size_t rx_ts_len, esp_eth_ptp_dm9058_time_t *time);

#ifdef __cplusplus
}
#endif
