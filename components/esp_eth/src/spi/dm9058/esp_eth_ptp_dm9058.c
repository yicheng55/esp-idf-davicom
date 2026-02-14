#include <string.h>
#include "esp_check.h"
#include "esp_eth_ptp_dm9058.h"

/* DM9058 PTP registers */
#define DM9058_PTPCR  (0x60)
#define DM9058_PTPCW  (0x61)
#define DM9058_PTPTSM (0x62)
#define DM9058_PTPTX  (0x63)
#define DM9058_PTPMMP (0x64)
#define DM9058_PTPTSO (0x65)
#define DM9058_PTPCSO (0x66)
#define DM9058_PTPTS  (0x68)

#define DM9058_PTP_TCR_ENABLE             (0x01)
#define DM9058_PTP_TCR_RESET_INDEX        (0x80)
#define DM9058_PTP_TCR_READ_CLOCK         (0x84)
#define DM9058_PTP_TCR_APPLY_SET_TIME     (0x09)
#define DM9058_PTP_TCR_APPLY_OFFSET       (0x10)
#define DM9058_PTP_TCR_APPLY_ADJUST_FAST  (0x20)
#define DM9058_PTP_TCR_APPLY_ADJUST_SLOW  (0x60)
#define DM9058_PTP_MAX_ADJUSTMENT         (0xEFFFFFFFU)

/* 2^32 * 40 / 1e9 in Q16 format */
#define DM9058_PTP_FREQ_BASE_ADDEND_Q16   (11259106)

static esp_err_t dm9058_ptp_try_lock(esp_eth_ptp_dm9058_t *ptp, bool *locked)
{
    *locked = false;
    if (ptp->ops.lock != NULL) {
        if (!ptp->ops.lock(ptp->io_ctx)) {
            return ESP_ERR_TIMEOUT;
        }
        *locked = true;
    }
    return ESP_OK;
}

static void dm9058_ptp_unlock_if_needed(esp_eth_ptp_dm9058_t *ptp, bool locked)
{
    if (locked && ptp->ops.unlock != NULL) {
        ptp->ops.unlock(ptp->io_ctx);
    }
}

static inline void dm9058_ptp_delay_us(esp_eth_ptp_dm9058_t *ptp, uint32_t us)
{
    if (ptp->ops.delay_us) {
        ptp->ops.delay_us(us);
    }
}

static inline void dm9058_ptp_delay_ms(esp_eth_ptp_dm9058_t *ptp, uint32_t ms)
{
    if (ptp->ops.delay_ms) {
        ptp->ops.delay_ms(ms);
    } else {
        dm9058_ptp_delay_us(ptp, ms * 1000U);
    }
}

static esp_err_t dm9058_ptp_read_bytes(esp_eth_ptp_dm9058_t *ptp, uint8_t reg, uint8_t *buffer, size_t len)
{
    if (ptp->ops.reg_burst_read) {
        return ptp->ops.reg_burst_read(ptp->io_ctx, reg, buffer, len);
    }
    for (size_t i = 0; i < len; i++) {
        esp_err_t ret = ptp->ops.reg_read(ptp->io_ctx, reg, &buffer[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

static esp_err_t dm9058_ptp_write_bytes(esp_eth_ptp_dm9058_t *ptp, uint8_t reg, const uint8_t *buffer, size_t len)
{
    if (ptp->ops.reg_burst_write) {
        return ptp->ops.reg_burst_write(ptp->io_ctx, reg, buffer, len);
    }
    for (size_t i = 0; i < len; i++) {
        esp_err_t ret = ptp->ops.reg_write(ptp->io_ctx, reg, buffer[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

static inline void dm9058_ptp_encode_time(const esp_eth_ptp_dm9058_time_t *time, uint8_t out[8])
{
    out[0] = (uint8_t)(time->nanoseconds);
    out[1] = (uint8_t)(time->nanoseconds >> 8);
    out[2] = (uint8_t)(time->nanoseconds >> 16);
    out[3] = (uint8_t)(time->nanoseconds >> 24);
    out[4] = (uint8_t)(time->seconds);
    out[5] = (uint8_t)(time->seconds >> 8);
    out[6] = (uint8_t)(time->seconds >> 16);
    out[7] = (uint8_t)(time->seconds >> 24);
}

static inline void dm9058_ptp_decode_time(const uint8_t in[8], esp_eth_ptp_dm9058_time_t *time)
{
    time->nanoseconds = (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
    time->seconds = (uint32_t)in[4] | ((uint32_t)in[5] << 8) | ((uint32_t)in[6] << 16) | ((uint32_t)in[7] << 24);
}

esp_err_t esp_eth_ptp_dm9058_init(esp_eth_ptp_dm9058_t *ptp, void *io_ctx, const esp_eth_ptp_dm9058_ops_t *ops)
{
    ESP_RETURN_ON_FALSE(ptp != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "null ptp handle");
    ESP_RETURN_ON_FALSE(ops != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "null ptp ops");
    ESP_RETURN_ON_FALSE(ops->reg_read && ops->reg_write, ESP_ERR_INVALID_ARG, "dm9058.ptp", "missing reg access callbacks");

    memset(ptp, 0, sizeof(*ptp));
    ptp->io_ctx = io_ctx;
    ptp->ops = *ops;
    ptp->initialized = true;
    return ESP_OK;
}

esp_err_t esp_eth_ptp_dm9058_enable(esp_eth_ptp_dm9058_t *ptp, bool enable, esp_eth_ptp_dm9058_transport_t transport)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && ptp->initialized, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not initialized");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t ts_offset = 0x4E;
    uint8_t checksum_offset = 0x3C;

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");

    if (!enable) {
        ret = ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, 0x00);
        ESP_GOTO_ON_ERROR(ret, err, "dm9058.ptp", "disable ptp failed");
        ptp->enabled = false;
        goto err;
    }

    switch (transport) {
    case ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV6:
        ts_offset = 0x62;
        checksum_offset = 0x50;
        break;
    case ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3:
        ts_offset = 0x32;
        checksum_offset = 0x20;
        break;
    case ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4:
    default:
        break;
    }

    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCR, 0x01), err, "dm9058.ptp", "ptp reset assert failed");
    dm9058_ptp_delay_ms(ptp, 1);
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCR, 0x00), err, "dm9058.ptp", "ptp reset deassert failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, DM9058_PTP_TCR_ENABLE), err, "dm9058.ptp", "ptp enable failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, 0x02, 0x00), err, "dm9058.ptp", "clear tx control failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPMMP, 0x12), err, "dm9058.ptp", "set ptp mode failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPTX, 0x00), err, "dm9058.ptp", "disable one-step failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPTSO, ts_offset), err, "dm9058.ptp", "set ts offset failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCSO, checksum_offset), err, "dm9058.ptp", "set checksum offset failed");
    ptp->enabled = true;
    ptp->last_rate = 0;

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_get_time(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && time != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t raw[8] = {0};

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, DM9058_PTP_TCR_READ_CLOCK), err, "dm9058.ptp", "prepare get time failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_read_bytes(ptp, DM9058_PTPTS, raw, sizeof(raw)), err, "dm9058.ptp", "read time failed");
    dm9058_ptp_decode_time(raw, time);

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_set_time(esp_eth_ptp_dm9058_t *ptp, const esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && time != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t raw[8];
    dm9058_ptp_encode_time(time, raw);

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCR, 0x01), err, "dm9058.ptp", "ptp reset assert failed");
    dm9058_ptp_delay_us(ptp, 2);
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCR, 0x00), err, "dm9058.ptp", "ptp reset deassert failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, DM9058_PTP_TCR_RESET_INDEX), err, "dm9058.ptp", "reset index failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_write_bytes(ptp, DM9058_PTPTS, raw, sizeof(raw)), err, "dm9058.ptp", "write time failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, DM9058_PTP_TCR_APPLY_SET_TIME), err, "dm9058.ptp", "apply set time failed");
    ptp->last_rate = 0;

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_adj_time(esp_eth_ptp_dm9058_t *ptp, const esp_eth_ptp_dm9058_time_t *offset)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && offset != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");

    esp_eth_ptp_dm9058_time_t current = {0};
    ESP_RETURN_ON_ERROR(esp_eth_ptp_dm9058_get_time(ptp, &current), "dm9058.ptp", "get current time failed");

    int64_t sec = (int64_t)current.seconds + (int32_t)offset->seconds;
    int64_t nsec = (int64_t)current.nanoseconds + (int32_t)offset->nanoseconds;
    while (nsec >= 1000000000LL) {
        sec++;
        nsec -= 1000000000LL;
    }
    while (nsec < 0) {
        sec--;
        nsec += 1000000000LL;
    }
    if (sec < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_eth_ptp_dm9058_time_t updated = {
        .seconds = (uint32_t)sec,
        .nanoseconds = (uint32_t)nsec,
    };
    return esp_eth_ptp_dm9058_set_time(ptp, &updated);
}

esp_err_t esp_eth_ptp_dm9058_adj_freq(esp_eth_ptp_dm9058_t *ptp, int32_t adj_ppb)
{
    ESP_RETURN_ON_FALSE(ptp != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t raw[4];
    int64_t signed_addend = ((int64_t)adj_ppb * (int64_t)DM9058_PTP_FREQ_BASE_ADDEND_Q16) >> 16;
    int64_t delta = signed_addend - ptp->last_rate;
    uint32_t adjustment;
    uint8_t control_value;

    if (delta < 0) {
        adjustment = (uint32_t)(-delta);
        control_value = DM9058_PTP_TCR_APPLY_ADJUST_SLOW;
    } else {
        adjustment = (uint32_t)delta;
        control_value = DM9058_PTP_TCR_APPLY_ADJUST_FAST;
    }
    if (adjustment > DM9058_PTP_MAX_ADJUSTMENT) {
        adjustment = DM9058_PTP_MAX_ADJUSTMENT;
    }

    raw[0] = (uint8_t)adjustment;
    raw[1] = (uint8_t)(adjustment >> 8);
    raw[2] = (uint8_t)(adjustment >> 16);
    raw[3] = (uint8_t)(adjustment >> 24);

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, DM9058_PTP_TCR_RESET_INDEX), err, "dm9058.ptp", "reset index failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_write_bytes(ptp, DM9058_PTPTS, raw, sizeof(raw)), err, "dm9058.ptp", "write freq adjust failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, control_value), err, "dm9058.ptp", "apply freq adjust failed");
    ptp->last_rate = signed_addend;

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_get_tx_timestamp(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(ptp != NULL && time != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(ptp->initialized && ptp->enabled, ESP_ERR_INVALID_STATE, "dm9058.ptp", "ptp not enabled");

    esp_err_t ret = ESP_OK;
    bool locked = false;
    uint8_t raw[8] = {0};

    ESP_GOTO_ON_ERROR(dm9058_ptp_try_lock(ptp, &locked), err, "dm9058.ptp", "lock timeout");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPCW, DM9058_PTP_TCR_RESET_INDEX), err, "dm9058.ptp", "reset index failed");
    ESP_GOTO_ON_ERROR(ptp->ops.reg_write(ptp->io_ctx, DM9058_PTPTSM, 0x01), err, "dm9058.ptp", "set tx timestamp mode failed");
    ESP_GOTO_ON_ERROR(dm9058_ptp_read_bytes(ptp, DM9058_PTPTS, raw, sizeof(raw)), err, "dm9058.ptp", "read tx timestamp failed");
    dm9058_ptp_decode_time(raw, time);

err:
    dm9058_ptp_unlock_if_needed(ptp, locked);
    return ret;
}

esp_err_t esp_eth_ptp_dm9058_tx_timestamp(esp_eth_ptp_dm9058_t *ptp, esp_eth_ptp_dm9058_time_t *time)
{
    return esp_eth_ptp_dm9058_get_tx_timestamp(ptp, time);
}

esp_err_t esp_eth_ptp_dm9058_rx_timestamp(const uint8_t *rx_ts_buffer, size_t rx_ts_len, esp_eth_ptp_dm9058_time_t *time)
{
    ESP_RETURN_ON_FALSE(rx_ts_buffer != NULL && time != NULL, ESP_ERR_INVALID_ARG, "dm9058.ptp", "invalid args");
    ESP_RETURN_ON_FALSE(rx_ts_len == 4 || rx_ts_len == 8, ESP_ERR_INVALID_ARG, "dm9058.ptp", "rx timestamp length must be 4 or 8");

    memset(time, 0, sizeof(*time));
    time->nanoseconds = (uint32_t)rx_ts_buffer[0] |
                        ((uint32_t)rx_ts_buffer[1] << 8) |
                        ((uint32_t)rx_ts_buffer[2] << 16) |
                        ((uint32_t)rx_ts_buffer[3] << 24);
    if (rx_ts_len == 8) {
        time->seconds = (uint32_t)rx_ts_buffer[4] |
                        ((uint32_t)rx_ts_buffer[5] << 8) |
                        ((uint32_t)rx_ts_buffer[6] << 16) |
                        ((uint32_t)rx_ts_buffer[7] << 24);
    }
    return ESP_OK;
}
