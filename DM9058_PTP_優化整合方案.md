# DM9058 PTP 功能優化整合方案

## 優化目標

將 PTP 功能無縫整合到 MAC 層的 `transmit` 和 `receive` 函數中，實現：
1. **自動 PTP 封包處理** - 發送時自動識別並配置 PTP 硬體
2. **透明時戳提取** - 接收時自動提取並存儲 PTP 時戳
3. **簡化應用層使用** - 應用層無需手動調用 PTP 函數

## 方案一：全自動模式（推薦）

### 1. 擴展 MAC 層結構

在 `esp_eth_mac_dm9058.c` 中的 `esp32_DM9058_t` 結構添加 PTP 配置：

```c
typedef struct {
    esp_eth_mac_t parent;
    esp_eth_mediator_t *eth;
    eth_spi_custom_driver_t spi;
    TaskHandle_t rx_task_hdl;
    SemaphoreHandle_t multi_reg_axs_mutex;
    uint32_t sw_reset_timeout_ms;
    int int_gpio_num;
    esp_timer_handle_t poll_timer;
    uint32_t poll_period_ms;
    uint8_t addr[ETH_ADDR_LEN];
    bool packets_remain;
    bool flow_ctrl_enabled;
    uint8_t *rx_buffer;
    uint8_t hash_filter_cnt[DM9058_HASH_FILTER_TABLE_SIZE];
    esp_eth_ptp_dm9058_t ptp;

    /* === 新增 PTP 配置欄位 === */
    bool ptp_auto_process;                    // 是否自動處理 PTP 封包
    bool ptp_two_step_mode;                   // PTP 模式：true=雙步，false=單步
    esp_eth_ptp_dm9058_time_t last_rx_timestamp;  // 最後接收的時戳
    bool rx_timestamp_valid;                  // RX 時戳是否有效
    esp_eth_ptp_dm9058_time_t last_tx_timestamp;  // 最後傳輸的時戳
    bool tx_timestamp_valid;                  // TX 時戳是否有效
} esp32_DM9058_t;
```

### 2. 優化 Transmit 函數

**位置**: `esp_eth_mac_dm9058.c` 的 `esp32_DM9058_transmit()` 函數（第 797 行）

**優化前**（當前實作）：
```c
static esp_err_t esp32_DM9058_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);

    ESP_GOTO_ON_FALSE(length <= ETH_MAX_PACKET_SIZE, ESP_ERR_INVALID_ARG, err,
                      TAG, "frame size is too big (actual %" PRIu32 ", maximum %d)", length, ETH_MAX_PACKET_SIZE);

    uint8_t reg_nsr = 0;
    int64_t wait_time =  esp_timer_get_time();
    do {
        ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_NSR, &reg_nsr), err, TAG, "read NSR failed");
        reg_nsr &= (NSR_TX2END | NSR_TX1END);
    } while((!reg_nsr) && ((esp_timer_get_time() - wait_time) < 100));

    if (!reg_nsr) {
        ESP_LOGE(TAG, "last transmit still in progress, cannot send.");
        return ESP_ERR_INVALID_STATE;
    }

    if(reg_nsr == (NSR_TX2END | NSR_TX1END)) {
        ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_MPTRCR, MPTRCR_RST_TX), err, TAG, "write MPTRCR failed");
    }

    /* set tx length */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_TXPLL, length & 0xFF), err, TAG, "write TXPLL failed");
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_TXPLH, (length >> 8) & 0xFF), err, TAG, "write TXPLH failed");
    /* copy data to tx memory */
    ESP_GOTO_ON_ERROR(DM9058_memory_write(emac, buf, length), err, TAG, "write memory failed");
    return ESP_OK;
err:
    return ret;
}
```

**優化後**（整合 PTP 自動處理）：
```c
static esp_err_t esp32_DM9058_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);

    ESP_GOTO_ON_FALSE(length <= ETH_MAX_PACKET_SIZE, ESP_ERR_INVALID_ARG, err,
                      TAG, "frame size is too big (actual %" PRIu32 ", maximum %d)", length, ETH_MAX_PACKET_SIZE);

    /* === 新增：PTP 自動處理 === */
    if (emac->ptp_auto_process && emac->ptp.enabled) {
        ret = esp_eth_ptp_dm9058_prepare_tx(&emac->ptp, buf, length, emac->ptp_two_step_mode);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "PTP prepare tx failed: %d, continue anyway", ret);
            // 不返回錯誤，繼續發送非 PTP 封包
        }
    }
    /* === PTP 處理結束 === */

    uint8_t reg_nsr = 0;
    int64_t wait_time =  esp_timer_get_time();
    do {
        ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_NSR, &reg_nsr), err, TAG, "read NSR failed");
        reg_nsr &= (NSR_TX2END | NSR_TX1END);
    } while((!reg_nsr) && ((esp_timer_get_time() - wait_time) < 100));

    if (!reg_nsr) {
        ESP_LOGE(TAG, "last transmit still in progress, cannot send.");
        return ESP_ERR_INVALID_STATE;
    }

    if(reg_nsr == (NSR_TX2END | NSR_TX1END)) {
        ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_MPTRCR, MPTRCR_RST_TX), err, TAG, "write MPTRCR failed");
    }

    /* set tx length */
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_TXPLL, length & 0xFF), err, TAG, "write TXPLL failed");
    ESP_GOTO_ON_ERROR(DM9058_register_write(emac, DM9058_TXPLH, (length >> 8) & 0xFF), err, TAG, "write TXPLH failed");
    /* copy data to tx memory */
    ESP_GOTO_ON_ERROR(DM9058_memory_write(emac, buf, length), err, TAG, "write memory failed");

    /* === 新增：發送後獲取 TX 時戳 === */
    if (emac->ptp_auto_process && emac->ptp.enabled) {
        // 標記 TX 時戳為無效，等待實際讀取
        emac->tx_timestamp_valid = false;
        // 注意：實際的 TX 時戳需要在發送完成後讀取
        // 可以在後續的中斷處理或輪詢中讀取
    }
    /* === TX 時戳處理結束 === */

    return ESP_OK;
err:
    return ret;
}
```

### 3. 優化 Receive 相關函數

**位置**: `esp_eth_mac_dm9058.c` 的 `DM9058_frame_to_rx_buffer()` 函數（第 862 行）

**優化前**（當前實作）：
```c
static esp_err_t DM9058_frame_to_rx_buffer(esp32_DM9058_t *emac, uint16_t *size)
{
    esp_err_t ret = ESP_OK;
    uint8_t rxbyte = 0;
    __attribute__((aligned(4))) DM9058_rx_header_t header;
    bool try_again = false;

    do {
        *size = 0;
        uint8_t reg_nsr = 0;
        ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_NSR, &reg_nsr), err, TAG, "read NSR failed");
        if (reg_nsr & NSR_RXRDY) {
            /* dummy read, get the most updated data */
            ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_MRCMDX, &rxbyte), err, TAG, "read MRCMDX failed");
            ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_MRCMDX, &rxbyte), err, TAG, "read MRCMDX failed");
            if (0x01 != rxbyte) {
                ESP_GOTO_ON_ERROR(DM9058_flush_recv_queue(emac), err, TAG, "flush rx queue failed");
                ESP_GOTO_ON_FALSE(false, ESP_FAIL, err, TAG, "unexpected rx flag (0x%" PRIx8 "), reset rx fifo pointer", rxbyte);
            }
            ESP_GOTO_ON_ERROR(DM9058_memory_read(emac, (uint8_t *)&header, sizeof(header)), err, TAG, "read rx header failed");
            uint16_t rx_len = header.length_low + (header.length_high << 8);
            /* store the whole frame to preallocated memory */
            if (rx_len <= ETH_MAX_PACKET_SIZE) {
                ESP_GOTO_ON_ERROR(DM9058_memory_read(emac, emac->rx_buffer, rx_len), err, TAG, "read rx data failed");
            } else {
                /* we are out of sync or data is corrupted, there is no way how to fix position in rx fifo => flush all */
                ESP_GOTO_ON_ERROR(DM9058_flush_recv_queue(emac), err, TAG, "flush rx queue failed");
                ESP_GOTO_ON_FALSE(false, ESP_FAIL, err, TAG, "invalid frame length, reset rx fifo pointer");
            }
            if (header.status & (RSR_RF | RSR_RWTO | RSR_CE | RSR_FOE)) {
                /* erroneous frames should not be forwarded by DM9058, however, if it happens, just skip it */
                DM9058_skip_recv_frame(emac, rx_len);
                ESP_LOGE(TAG, "receive status error: %" PRIx8 "H", header.status);
                /* try again to check if other frame is waiting */
                try_again = true;
            } else {
                *size = rx_len;
            }
        }
    } while (try_again);
err:
    return ret;
}
```

**優化後**（整合 PTP 時戳提取）：
```c
static esp_err_t DM9058_frame_to_rx_buffer(esp32_DM9058_t *emac, uint16_t *size)
{
    esp_err_t ret = ESP_OK;
    uint8_t rxbyte = 0;
    __attribute__((aligned(4))) DM9058_rx_header_t header;
    bool try_again = false;

    do {
        *size = 0;
        uint8_t reg_nsr = 0;
        ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_NSR, &reg_nsr), err, TAG, "read NSR failed");
        if (reg_nsr & NSR_RXRDY) {
            /* dummy read, get the most updated data */
            ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_MRCMDX, &rxbyte), err, TAG, "read MRCMDX failed");
            ESP_GOTO_ON_ERROR(DM9058_register_read(emac, DM9058_MRCMDX, &rxbyte), err, TAG, "read MRCMDX failed");
            if (0x01 != rxbyte) {
                ESP_GOTO_ON_ERROR(DM9058_flush_recv_queue(emac), err, TAG, "flush rx queue failed");
                ESP_GOTO_ON_FALSE(false, ESP_FAIL, err, TAG, "unexpected rx flag (0x%" PRIx8 "), reset rx fifo pointer", rxbyte);
            }
            ESP_GOTO_ON_ERROR(DM9058_memory_read(emac, (uint8_t *)&header, sizeof(header)), err, TAG, "read rx header failed");
            uint16_t rx_len = header.length_low + (header.length_high << 8);

            /* === 新增：PTP 時戳預處理 === */
            esp_eth_ptp_dm9058_rx_info_t rx_info = {0};
            uint8_t rx_header_bytes[4] = {header.flag, header.status, header.length_low, header.length_high};
            uint8_t ts_buffer[8] = {0};
            size_t ts_len = 0;

            // 檢查是否有 PTP 時戳
            if (emac->ptp_auto_process && emac->ptp.enabled) {
                ret = esp_eth_ptp_dm9058_parse_rx_header(rx_header_bytes, sizeof(rx_header_bytes),
                                                          ETH_MAX_PACKET_SIZE, &rx_info);
                if (ret == ESP_OK && rx_info.timestamp_available) {
                    ts_len = rx_info.timestamp_len;
                }
            }
            /* === PTP 預處理結束 === */

            /* store the whole frame to preallocated memory */
            if (rx_len <= ETH_MAX_PACKET_SIZE) {
                ESP_GOTO_ON_ERROR(DM9058_memory_read(emac, emac->rx_buffer, rx_len), err, TAG, "read rx data failed");

                /* === 新增：讀取 PTP 時戳 === */
                if (ts_len > 0) {
                    // 時戳位於封包資料之後
                    ESP_GOTO_ON_ERROR(DM9058_memory_read(emac, ts_buffer, ts_len), err, TAG, "read rx timestamp failed");

                    // 解析並存儲時戳
                    ret = esp_eth_ptp_dm9058_rx_timestamp(ts_buffer, ts_len, &emac->last_rx_timestamp);
                    if (ret == ESP_OK) {
                        emac->rx_timestamp_valid = true;
                        ESP_LOGD(TAG, "RX timestamp: %lu.%09lu",
                                 emac->last_rx_timestamp.seconds,
                                 emac->last_rx_timestamp.nanoseconds);
                    } else {
                        emac->rx_timestamp_valid = false;
                        ESP_LOGW(TAG, "Failed to parse RX timestamp");
                    }
                }
                /* === PTP 時戳處理結束 === */
            } else {
                /* we are out of sync or data is corrupted, there is no way how to fix position in rx fifo => flush all */
                ESP_GOTO_ON_ERROR(DM9058_flush_recv_queue(emac), err, TAG, "flush rx queue failed");
                ESP_GOTO_ON_FALSE(false, ESP_FAIL, err, TAG, "invalid frame length, reset rx fifo pointer");
            }
            if (header.status & (RSR_RF | RSR_RWTO | RSR_CE | RSR_FOE)) {
                /* erroneous frames should not be forwarded by DM9058, however, if it happens, just skip it */
                DM9058_skip_recv_frame(emac, rx_len);
                ESP_LOGE(TAG, "receive status error: %" PRIx8 "H", header.status);
                /* try again to check if other frame is waiting */
                try_again = true;
            } else {
                *size = rx_len;
            }
        }
    } while (try_again);
err:
    return ret;
}
```

### 4. 擴展 IOCTL 命令

**位置**: `esp_eth_mac_spi.h` 中添加新命令：

```c
typedef enum {
    ETH_MAC_DM9058_CMD_PTP_ENABLE = ETH_CMD_CUSTOM_MAC_CMDS_OFFSET,
    ETH_MAC_DM9058_CMD_S_PTP_TIME,
    ETH_MAC_DM9058_CMD_G_PTP_TIME,
    ETH_MAC_DM9058_CMD_ADJ_PTP_FREQ,
    ETH_MAC_DM9058_CMD_ADJ_PTP_TIME,
    ETH_MAC_DM9058_CMD_G_PTP_TX_TIME,
    ETH_MAC_DM9058_CMD_S_TARGET_TIME,
    ETH_MAC_DM9058_CMD_S_TARGET_CB,

    /* === 新增命令 === */
    ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS,     // 啟用/停用自動 PTP 處理
    ETH_MAC_DM9058_CMD_PTP_SET_MODE,         // 設定 PTP 模式（單步/雙步）
    ETH_MAC_DM9058_CMD_G_PTP_RX_TIME,        // 獲取最後的 RX 時戳
} eth_mac_dm9058_io_cmd_t;
```

**位置**: `esp_eth_mac_dm9058.c` 的 `esp32_DM9058_custom_ioctl()` 函數（第 750 行）添加處理：

```c
static esp_err_t esp32_DM9058_custom_ioctl(esp_eth_mac_t *mac, int cmd, void *data)
{
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    esp_eth_ptp_dm9058_time_t ptp_time = {0};
    eth_mac_time_t *time = (eth_mac_time_t *)data;

    switch (cmd) {
    case ETH_MAC_DM9058_CMD_PTP_ENABLE:
        ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "PTP_ENABLE expects bool*");
        return esp_eth_ptp_dm9058_enable(&emac->ptp, *(bool *)data, ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4);

    /* ... 其他現有命令 ... */

    /* === 新增命令處理 === */
    case ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS:
        ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "PTP_AUTO_PROCESS expects bool*");
        emac->ptp_auto_process = *(bool *)data;
        ESP_LOGI(TAG, "PTP auto process %s", emac->ptp_auto_process ? "enabled" : "disabled");
        return ESP_OK;

    case ETH_MAC_DM9058_CMD_PTP_SET_MODE:
        ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "PTP_SET_MODE expects bool*");
        emac->ptp_two_step_mode = *(bool *)data;
        ESP_LOGI(TAG, "PTP mode set to %s", emac->ptp_two_step_mode ? "two-step" : "one-step");
        return ESP_OK;

    case ETH_MAC_DM9058_CMD_G_PTP_RX_TIME:
        ESP_RETURN_ON_FALSE(time != NULL, ESP_ERR_INVALID_ARG, TAG, "G_PTP_RX_TIME expects eth_mac_time_t*");
        if (!emac->rx_timestamp_valid) {
            return ESP_ERR_NOT_FOUND;
        }
        time->seconds = emac->last_rx_timestamp.seconds;
        time->nanoseconds = emac->last_rx_timestamp.nanoseconds;
        return ESP_OK;
    /* === 新增命令結束 === */

    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}
```

### 5. 初始化時設定預設值

**位置**: `esp_eth_mac_dm9058.c` 的 `esp_eth_mac_new_dm9058()` 函數（第 1046 行）：

```c
esp_eth_mac_t *esp_eth_mac_new_dm9058(const eth_dm9058_config_t *DM9058_config, const eth_mac_config_t *mac_config)
{
    // ... 現有程式碼 ...

    /* === 新增：初始化 PTP 配置 === */
    emac->ptp_auto_process = false;        // 預設關閉自動處理
    emac->ptp_two_step_mode = true;        // 預設使用雙步模式
    emac->rx_timestamp_valid = false;
    emac->tx_timestamp_valid = false;
    memset(&emac->last_rx_timestamp, 0, sizeof(emac->last_rx_timestamp));
    memset(&emac->last_tx_timestamp, 0, sizeof(emac->last_tx_timestamp));
    /* === 初始化結束 === */

    // ... 其餘程式碼 ...
}
```

## 應用層使用範例

### 範例 1：啟用全自動 PTP 處理

```c
#include "esp_eth.h"
#include "esp_eth_mac_spi.h"

void setup_auto_ptp(esp_eth_handle_t eth_handle)
{
    // 1. 啟用 PTP 硬體
    bool enable = true;
    esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_ENABLE, &enable);

    // 2. 啟用自動 PTP 處理
    bool auto_process = true;
    esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS, &auto_process);

    // 3. 設定 PTP 模式（可選，預設為雙步模式）
    bool two_step = false;  // 使用單步模式
    esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_SET_MODE, &two_step);

    // 4. 設定初始時間
    eth_mac_time_t init_time = {
        .seconds = 0,
        .nanoseconds = 0
    };
    esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_S_PTP_TIME, &init_time);

    ESP_LOGI("PTP", "Auto PTP processing enabled");
}

void ptp_receive_handler(void *buffer, size_t len)
{
    // 封包已經由 MAC 層接收
    // 如果是 PTP 封包，時戳已自動提取

    // 可以讀取最後的 RX 時戳
    esp_eth_handle_t eth_handle = /* 你的 eth handle */;
    eth_mac_time_t rx_time;

    if (esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_RX_TIME, &rx_time) == ESP_OK) {
        ESP_LOGI("PTP", "RX timestamp: %lu.%09lu", rx_time.seconds, rx_time.nanoseconds);
    }
}

void ptp_transmit_example(esp_eth_handle_t eth_handle, uint8_t *ptp_packet, size_t len)
{
    // 直接發送，PTP 硬體配置自動完成
    esp_eth_transmit(eth_handle, ptp_packet, len);

    // 等待一小段時間讓硬體完成發送
    vTaskDelay(pdMS_TO_TICKS(10));

    // 讀取 TX 時戳
    eth_mac_time_t tx_time;
    if (esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TX_TIME, &tx_time) == ESP_OK) {
        ESP_LOGI("PTP", "TX timestamp: %lu.%09lu", tx_time.seconds, tx_time.nanoseconds);
    }
}
```

### 範例 2：混合模式（部分自動）

如果你想保留對某些 PTP 封包的手動控制：

```c
void setup_semi_auto_ptp(esp_eth_handle_t eth_handle)
{
    // 1. 啟用 PTP 硬體
    bool enable = true;
    esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_ENABLE, &enable);

    // 2. 不啟用全自動處理（預設就是關閉）
    bool auto_process = false;
    esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS, &auto_process);
}

void ptp_transmit_with_manual_control(esp_eth_handle_t eth_handle,
                                      esp_eth_ptp_dm9058_t *ptp,
                                      uint8_t *packet, size_t len)
{
    // 手動準備 TX
    bool two_step_mode = true;
    esp_eth_ptp_dm9058_prepare_tx(ptp, packet, len, two_step_mode);

    // 發送
    esp_eth_transmit(eth_handle, packet, len);

    // 讀取時戳
    esp_eth_ptp_dm9058_time_t tx_time;
    esp_eth_ptp_dm9058_get_tx_timestamp(ptp, &tx_time);

    ESP_LOGI("PTP", "Manual TX: %lu.%09lu", tx_time.seconds, tx_time.nanoseconds);
}
```

## 方案二：異步時戳讀取（更高效）

對於高性能需求，可以在中斷或獨立任務中異步讀取時戳：

### 1. 在接收任務中處理時戳

**位置**: `esp_eth_mac_dm9058.c` 的 `esp32_DM9058_task()` 函數（第 978 行）：

```c
static void esp32_DM9058_task(void *arg)
{
    esp32_DM9058_t *emac = (esp32_DM9058_t *)arg;
    uint8_t status = 0;
    while (1) {
        // ... 現有的通知等待邏輯 ...

        /* clear interrupt status */
        DM9058_register_read(emac, DM9058_ISR, &status);
        DM9058_register_write(emac, DM9058_ISR, status);

        /* === 新增：處理 TX 完成後讀取時戳 === */
        if ((status & ISR_PT) && emac->ptp_auto_process && emac->ptp.enabled) {
            // TX 完成，讀取時戳
            if (esp_eth_ptp_dm9058_get_tx_timestamp(&emac->ptp, &emac->last_tx_timestamp) == ESP_OK) {
                emac->tx_timestamp_valid = true;
                ESP_LOGD(TAG, "TX timestamp captured: %lu.%09lu",
                         emac->last_tx_timestamp.seconds,
                         emac->last_tx_timestamp.nanoseconds);
            }
        }
        /* === TX 時戳處理結束 === */

        /* packet received */
        if (status & ISR_PR) {
            do {
                uint32_t buf_len;
                if (emac->parent.receive(&emac->parent, emac->rx_buffer, &buf_len) == ESP_OK) {
                    /* if there is waiting frame */
                    if (buf_len > 0) {
                        uint8_t *buffer = malloc(buf_len);
                        if (buffer == NULL) {
                            ESP_LOGE(TAG, "no mem for receive buffer");
                        } else {
                            memcpy(buffer, emac->rx_buffer, buf_len);
                            ESP_LOGD(TAG, "receive len=%" PRIu32, buf_len);

                            /* === 新增：如果有 RX 時戳，附加到封包資訊 === */
                            // 這裡可以擴展 stack_input 來傳遞時戳資訊
                            // 或者使用應用層回調來通知時戳
                            /* === RX 時戳通知結束 === */

                            /* pass the buffer to stack (e.g. TCP/IP layer) */
                            emac->eth->stack_input(emac->eth, buffer, buf_len);
                        }
                    }
                } else {
                    ESP_LOGE(TAG, "frame read from module failed");
                }
            } while (emac->packets_remain);
        }
    }
    vTaskDelete(NULL);
}
```

## 性能考量

### 1. 自動處理的開銷

在每次 `transmit` 調用時：
- **封包解析**: ~50-100 μs（取決於封包大小）
- **暫存器配置**: ~10-20 μs（2-3 次 SPI 傳輸）
- **總開銷**: ~60-120 μs

對於大多數應用（封包速率 < 10000 pps），這個開銷可以忽略不計。

### 2. 優化建議

**方案 A**：快速路徑優化
```c
static esp_err_t esp32_DM9058_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);

    // 快速路徑：如果 PTP 未啟用，直接跳過
    if (emac->ptp_auto_process && emac->ptp.enabled) {
        // 進一步優化：快速檢查 EtherType
        uint16_t ethertype = ((uint16_t)buf[12] << 8) | buf[13];
        if (ethertype == 0x88F7 || ethertype == 0x0800) {
            // 可能是 PTP，執行完整檢查
            esp_eth_ptp_dm9058_prepare_tx(&emac->ptp, buf, length, emac->ptp_two_step_mode);
        }
    }

    // ... 其餘發送邏輯 ...
}
```

**方案 B**：針對特定埠號的優化
```c
typedef struct {
    // ... 現有欄位 ...
    bool ptp_filter_ports;              // 是否過濾特定 UDP 埠
    uint16_t ptp_event_port;            // PTP event 埠（預設 319）
    uint16_t ptp_general_port;          // PTP general 埠（預設 320）
} esp32_DM9058_t;

// 在 transmit 中使用：
if (emac->ptp_filter_ports) {
    // 只檢查目標埠為 319 或 320 的 UDP 封包
    // 進一步減少非 PTP 封包的處理開銷
}
```

## 除錯和日誌

### 添加條件編譯的除錯日誌

在 `esp_eth_mac_dm9058.c` 頂部添加：

```c
#define DM9058_PTP_DEBUG 1

#if DM9058_PTP_DEBUG
#define PTP_LOGD(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
#define PTP_LOGI(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#else
#define PTP_LOGD(tag, format, ...)
#define PTP_LOGI(tag, format, ...)
#endif
```

使用範例：
```c
if (emac->ptp_auto_process && emac->ptp.enabled) {
    ret = esp_eth_ptp_dm9058_prepare_tx(&emac->ptp, buf, length, emac->ptp_two_step_mode);
    PTP_LOGD(TAG, "PTP prepare_tx result: %d", ret);
}
```

## 總結

### 優勢

1. **無縫整合**: PTP 功能透明地整合到 MAC 層
2. **自動化**: 應用層無需手動調用 PTP 函數
3. **靈活性**: 支援全自動和半自動模式
4. **向後相容**: 不影響現有非 PTP 應用

### 使用建議

| 場景 | 推薦配置 | 說明 |
|------|----------|------|
| PTP 從時鐘 | 全自動 + 雙步模式 | 自動處理所有 PTP 封包 |
| PTP 主時鐘 | 全自動 + 單步模式 | 硬體自動插入 SYNC 時戳 |
| 混合應用 | 半自動模式 | 手動控制關鍵封包 |
| 高性能需求 | 異步時戳讀取 | 在中斷中處理時戳 |
| 非 PTP 應用 | 關閉自動處理 | 零開銷 |

### 實作順序建議

1. **第一階段**: 實作基本的自動處理（transmit 中調用 prepare_tx）
2. **第二階段**: 添加 RX 時戳提取和存儲
3. **第三階段**: 實作新的 IOCTL 命令
4. **第四階段**: 添加性能優化（快速路徑、埠過濾）
5. **第五階段**: 實作異步時戳讀取

每個階段都可以獨立測試和部署。
