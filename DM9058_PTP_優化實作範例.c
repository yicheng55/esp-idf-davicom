/*
 * DM9058 PTP 優化整合 - 實際程式碼修改
 * 這個檔案展示需要修改的關鍵部分
 */

/* ============================================================================
 * 修改 1: 在 esp32_DM9058_t 結構中添加 PTP 配置欄位
 * 位置: esp_eth_mac_dm9058.c 約第 65-82 行
 * ============================================================================ */

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

    // === 新增 PTP 優化欄位 ===
    bool ptp_auto_process;                         // 啟用自動 PTP 處理
    bool ptp_two_step_mode;                        // PTP 模式選擇
    esp_eth_ptp_dm9058_time_t last_rx_timestamp;   // 最後 RX 時戳
    bool rx_timestamp_valid;                       // RX 時戳有效旗標
} esp32_DM9058_t;


/* ============================================================================
 * 修改 2: 優化 transmit 函數，整合自動 PTP 處理
 * 位置: esp_eth_mac_dm9058.c 的 esp32_DM9058_transmit() 函數
 * ============================================================================ */

static esp_err_t esp32_DM9058_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);

    ESP_GOTO_ON_FALSE(length <= ETH_MAX_PACKET_SIZE, ESP_ERR_INVALID_ARG, err,
                      TAG, "frame size is too big (actual %" PRIu32 ", maximum %d)", length, ETH_MAX_PACKET_SIZE);

    // === 新增：PTP 自動處理（僅在啟用時執行）===
    if (emac->ptp_auto_process && emac->ptp.enabled) {
        // 快速路徑：只檢查可能的 PTP 封包
        if (length >= 14) {
            uint16_t ethertype = ((uint16_t)buf[12] << 8) | buf[13];
            // 檢查是否為 PTP (0x88F7) 或 IPv4 (0x0800，可能包含 UDP PTP)
            if (ethertype == 0x88F7 || ethertype == 0x0800) {
                ret = esp_eth_ptp_dm9058_prepare_tx(&emac->ptp, buf, length, emac->ptp_two_step_mode);
                if (ret != ESP_OK) {
                    ESP_LOGD(TAG, "PTP prepare tx returned %d", ret);
                }
            }
        }
    }
    // === PTP 自動處理結束 ===

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


/* ============================================================================
 * 修改 3: 優化 receive 流程，整合 PTP 時戳提取
 * 位置: esp_eth_mac_dm9058.c 的 DM9058_frame_to_rx_buffer() 函數
 *
 * 方案 A：使用低層次 API（推薦，更高效）
 * ============================================================================ */

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

            // === 新增：使用 parse_rx_header API 解析 PTP 資訊 ===
            esp_eth_ptp_dm9058_rx_info_t rx_info = {0};
            uint8_t rx_header_bytes[4] = {header.flag, header.status, header.length_low, header.length_high};

            if (emac->ptp_auto_process && emac->ptp.enabled) {
                // 使用 PTP API 解析 header，這比手動檢查位元更清晰和安全
                ret = esp_eth_ptp_dm9058_parse_rx_header(rx_header_bytes, sizeof(rx_header_bytes),
                                                          ETH_MAX_PACKET_SIZE, &rx_info);
                if (ret != ESP_OK) {
                    ESP_LOGD(TAG, "PTP parse rx header failed: %d", ret);
                    // 繼續處理非 PTP 封包
                    rx_info.timestamp_available = false;
                }
            }
            // === PTP header 解析結束 ===

            /* store the whole frame to preallocated memory */
            if (rx_len <= ETH_MAX_PACKET_SIZE) {
                ESP_GOTO_ON_ERROR(DM9058_memory_read(emac, emac->rx_buffer, rx_len), err, TAG, "read rx data failed");

                // === 新增：讀取 PTP 時戳（如果有） ===
                if (rx_info.timestamp_available && rx_info.timestamp_len > 0) {
                    uint8_t ts_buffer[8] = {0};
                    ret = DM9058_memory_read(emac, ts_buffer, rx_info.timestamp_len);
                    if (ret == ESP_OK) {
                        // 使用 PTP API 解析時戳
                        ret = esp_eth_ptp_dm9058_rx_timestamp(ts_buffer, rx_info.timestamp_len,
                                                               &emac->last_rx_timestamp);
                        if (ret == ESP_OK) {
                            emac->rx_timestamp_valid = true;
                            ESP_LOGD(TAG, "RX PTP timestamp: %lu.%09lu",
                                     emac->last_rx_timestamp.seconds,
                                     emac->last_rx_timestamp.nanoseconds);
                        }
                    }
                    if (ret != ESP_OK) {
                        ESP_LOGW(TAG, "Failed to read/parse RX timestamp");
                        emac->rx_timestamp_valid = false;
                    }
                }
                // === PTP 時戳處理結束 ===
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


/* ============================================================================
 * 方案 B：使用高層次 parse_rx_packet API（適用於特定場景）
 *
 * 注意：此方案需要先讀取所有資料到 buffer，然後一次性解析
 * 適用於需要在讀取後再決定如何處理的場景
 * ============================================================================ */

static esp_err_t DM9058_frame_to_rx_buffer_with_parse_rx_packet(esp32_DM9058_t *emac, uint16_t *size)
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

            // 讀取 header
            ESP_GOTO_ON_ERROR(DM9058_memory_read(emac, (uint8_t *)&header, sizeof(header)), err, TAG, "read rx header failed");
            uint16_t rx_len = header.length_low + (header.length_high << 8);

            /* store the whole frame to preallocated memory */
            if (rx_len <= ETH_MAX_PACKET_SIZE) {
                ESP_GOTO_ON_ERROR(DM9058_memory_read(emac, emac->rx_buffer, rx_len), err, TAG, "read rx data failed");

                // === 使用 parse_rx_packet 整合 API ===
                if (emac->ptp_auto_process && emac->ptp.enabled) {
                    uint8_t rx_header_bytes[4] = {header.flag, header.status, header.length_low, header.length_high};
                    uint8_t ts_buffer[8] = {0};
                    esp_eth_ptp_dm9058_rx_info_t rx_info = {0};

                    // 預先讀取可能的時戳資料（最多 8 bytes）
                    // 注意：這裡需要根據實際的 FIFO 讀取順序來決定是否預讀時戳
                    size_t ts_buffer_len = sizeof(ts_buffer);

                    // 一次性解析所有 RX 資訊（包含 ready check, header parse, timestamp decode）
                    ret = esp_eth_ptp_dm9058_parse_rx_packet(
                        &emac->ptp,
                        rx_header_bytes, sizeof(rx_header_bytes),
                        ts_buffer, ts_buffer_len,
                        ETH_MAX_PACKET_SIZE,
                        &rx_info,
                        &emac->last_rx_timestamp
                    );

                    if (ret == ESP_OK && rx_info.timestamp_available) {
                        emac->rx_timestamp_valid = true;
                        ESP_LOGD(TAG, "RX PTP timestamp: %lu.%09lu",
                                 emac->last_rx_timestamp.seconds,
                                 emac->last_rx_timestamp.nanoseconds);
                    } else {
                        emac->rx_timestamp_valid = false;
                        if (ret != ESP_OK) {
                            ESP_LOGD(TAG, "PTP parse rx packet returned: %d", ret);
                        }
                    }
                }
                // === parse_rx_packet 整合結束 ===
            } else {
                ESP_GOTO_ON_ERROR(DM9058_flush_recv_queue(emac), err, TAG, "flush rx queue failed");
                ESP_GOTO_ON_FALSE(false, ESP_FAIL, err, TAG, "invalid frame length, reset rx fifo pointer");
            }

            if (header.status & (RSR_RF | RSR_RWTO | RSR_CE | RSR_FOE)) {
                DM9058_skip_recv_frame(emac, rx_len);
                ESP_LOGE(TAG, "receive status error: %" PRIx8 "H", header.status);
            }
        }
    } while (try_again);
err:
    return ret;
}


/* ============================================================================
 * 修改 4: 擴展 custom_ioctl 函數
 * 位置: esp_eth_mac_dm9058.c 的 esp32_DM9058_custom_ioctl() 函數
 * ============================================================================ */

static esp_err_t esp32_DM9058_custom_ioctl(esp_eth_mac_t *mac, int cmd, void *data)
{
    esp32_DM9058_t *emac = __containerof(mac, esp32_DM9058_t, parent);
    esp_eth_ptp_dm9058_time_t ptp_time = {0};
    eth_mac_time_t *time = (eth_mac_time_t *)data;

    switch (cmd) {
    case ETH_MAC_DM9058_CMD_PTP_ENABLE:
        ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "PTP_ENABLE expects bool*");
        return esp_eth_ptp_dm9058_enable(&emac->ptp, *(bool *)data, ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4);

    case ETH_MAC_DM9058_CMD_S_PTP_TIME:
        ESP_RETURN_ON_FALSE(time != NULL, ESP_ERR_INVALID_ARG, TAG, "S_PTP_TIME expects eth_mac_time_t*");
        ptp_time.seconds = time->seconds;
        ptp_time.nanoseconds = time->nanoseconds;
        return esp_eth_ptp_dm9058_set_time(&emac->ptp, &ptp_time);

    case ETH_MAC_DM9058_CMD_G_PTP_TIME:
        ESP_RETURN_ON_FALSE(time != NULL, ESP_ERR_INVALID_ARG, TAG, "G_PTP_TIME expects eth_mac_time_t*");
        ESP_RETURN_ON_ERROR(esp_eth_ptp_dm9058_get_time(&emac->ptp, &ptp_time), TAG, "get ptp time failed");
        time->seconds = ptp_time.seconds;
        time->nanoseconds = ptp_time.nanoseconds;
        return ESP_OK;

    case ETH_MAC_DM9058_CMD_ADJ_PTP_FREQ:
        ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "ADJ_PTP_FREQ expects int32_t*");
        return esp_eth_ptp_dm9058_adj_freq(&emac->ptp, *(int32_t *)data);

    case ETH_MAC_DM9058_CMD_ADJ_PTP_TIME:
        ESP_RETURN_ON_FALSE(time != NULL, ESP_ERR_INVALID_ARG, TAG, "ADJ_PTP_TIME expects eth_mac_time_t*");
        ptp_time.seconds = (uint32_t)((int32_t)time->seconds);
        ptp_time.nanoseconds = (uint32_t)((int32_t)time->nanoseconds);
        return esp_eth_ptp_dm9058_adj_time(&emac->ptp, &ptp_time);

    case ETH_MAC_DM9058_CMD_G_PTP_TX_TIME:
        ESP_RETURN_ON_FALSE(time != NULL, ESP_ERR_INVALID_ARG, TAG, "G_PTP_TX_TIME expects eth_mac_time_t*");
        ESP_RETURN_ON_ERROR(esp_eth_ptp_dm9058_get_tx_timestamp(&emac->ptp, &ptp_time), TAG, "get tx timestamp failed");
        time->seconds = ptp_time.seconds;
        time->nanoseconds = ptp_time.nanoseconds;
        return ESP_OK;

    // === 新增命令 ===
    case ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS:
        ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "PTP_AUTO_PROCESS expects bool*");
        emac->ptp_auto_process = *(bool *)data;
        ESP_LOGI(TAG, "PTP auto-process %s", emac->ptp_auto_process ? "enabled" : "disabled");
        return ESP_OK;

    case ETH_MAC_DM9058_CMD_PTP_SET_MODE:
        ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "PTP_SET_MODE expects bool*");
        emac->ptp_two_step_mode = *(bool *)data;
        ESP_LOGI(TAG, "PTP mode: %s", emac->ptp_two_step_mode ? "two-step" : "one-step");
        return ESP_OK;

    case ETH_MAC_DM9058_CMD_G_PTP_RX_TIME:
        ESP_RETURN_ON_FALSE(time != NULL, ESP_ERR_INVALID_ARG, TAG, "G_PTP_RX_TIME expects eth_mac_time_t*");
        if (!emac->rx_timestamp_valid) {
            return ESP_ERR_NOT_FOUND;
        }
        time->seconds = emac->last_rx_timestamp.seconds;
        time->nanoseconds = emac->last_rx_timestamp.nanoseconds;
        emac->rx_timestamp_valid = false;  // 讀取後清除旗標
        return ESP_OK;
    // === 新增命令結束 ===

    case ETH_MAC_DM9058_CMD_S_TARGET_TIME:
        ESP_LOGW(TAG, "Target time feature not yet implemented for DM9058");
        return ESP_ERR_NOT_SUPPORTED;

    case ETH_MAC_DM9058_CMD_S_TARGET_CB:
        ESP_LOGW(TAG, "Target callback feature not yet implemented for DM9058");
        return ESP_ERR_NOT_SUPPORTED;

    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}


/* ============================================================================
 * 修改 5: 在初始化函數中設定預設值
 * 位置: esp_eth_mac_dm9058.c 的 esp_eth_mac_new_dm9058() 函數
 * ============================================================================ */

esp_eth_mac_t *esp_eth_mac_new_dm9058(const eth_dm9058_config_t *DM9058_config, const eth_mac_config_t *mac_config)
{
    esp_eth_mac_t *ret = NULL;
    esp32_DM9058_t *emac = NULL;
    ESP_GOTO_ON_FALSE(DM9058_config, NULL, err, TAG, "can't set DM9058 specific config to null");
    ESP_GOTO_ON_FALSE(mac_config, NULL, err, TAG, "can't set mac config to null");
    ESP_GOTO_ON_FALSE((DM9058_config->int_gpio_num >= 0) != (DM9058_config->poll_period_ms > 0), NULL, err, TAG, "invalid configuration argument combination");
    emac = calloc(1, sizeof(esp32_DM9058_t));
    ESP_GOTO_ON_FALSE(emac, NULL, err, TAG, "calloc emac failed");

    /* bind methods and attributes */
    emac->sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
    emac->int_gpio_num = DM9058_config->int_gpio_num;
    emac->poll_period_ms = DM9058_config->poll_period_ms;
    emac->parent.set_mediator = esp32_DM9058_set_mediator;
    emac->parent.init = esp32_DM9058_init;
    emac->parent.deinit = esp32_DM9058_deinit;
    emac->parent.start = esp32_DM9058_start;
    emac->parent.stop = esp32_DM9058_stop;
    emac->parent.del = esp32_DM9058_del;
    emac->parent.write_phy_reg = esp32_DM9058_write_phy_reg;
    emac->parent.read_phy_reg = esp32_DM9058_read_phy_reg;
    emac->parent.set_addr = esp32_DM9058_set_addr;
    emac->parent.get_addr = esp32_DM9058_get_addr;
    emac->parent.set_speed = esp32_DM9058_set_speed;
    emac->parent.set_duplex = esp32_DM9058_set_duplex;
    emac->parent.set_link = esp32_DM9058_set_link;
    emac->parent.set_promiscuous = esp32_DM9058_set_promiscuous;
    emac->parent.set_all_multicast = esp32_DM9058_set_all_multicast;
    emac->parent.set_peer_pause_ability = esp32_DM9058_set_peer_pause_ability;
    emac->parent.enable_flow_ctrl = esp32_DM9058_enable_flow_ctrl;
    emac->parent.transmit = esp32_DM9058_transmit;
    emac->parent.receive = esp32_DM9058_receive;
    emac->parent.add_mac_filter = esp32_DM9058_add_mac_filter;
    emac->parent.rm_mac_filter = esp32_DM9058_rm_mac_filter;
    emac->parent.custom_ioctl = esp32_DM9058_custom_ioctl;

    // === 新增：初始化 PTP 優化欄位 ===
    emac->ptp_auto_process = false;        // 預設關閉自動處理
    emac->ptp_two_step_mode = true;        // 預設使用雙步模式（更通用）
    emac->rx_timestamp_valid = false;
    memset(&emac->last_rx_timestamp, 0, sizeof(emac->last_rx_timestamp));
    // === 初始化結束 ===

    // ... 其餘初始化程式碼 ...

    if (DM9058_config->custom_spi_driver.init != NULL && DM9058_config->custom_spi_driver.deinit != NULL
            && DM9058_config->custom_spi_driver.read != NULL && DM9058_config->custom_spi_driver.write != NULL) {
        ESP_LOGD(TAG, "Using user's custom SPI Driver");
        emac->spi.init = DM9058_config->custom_spi_driver.init;
        emac->spi.deinit = DM9058_config->custom_spi_driver.deinit;
        emac->spi.read = DM9058_config->custom_spi_driver.read;
        emac->spi.write = DM9058_config->custom_spi_driver.write;
        ESP_GOTO_ON_FALSE((emac->spi.ctx = emac->spi.init(DM9058_config->custom_spi_driver.config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    } else {
        ESP_LOGD(TAG, "Using default SPI Driver");
        emac->spi.init = DM9058_spi_init;
        emac->spi.deinit = DM9058_spi_deinit;
        emac->spi.read = DM9058_spi_read;
        emac->spi.write = DM9058_spi_write;
        ESP_GOTO_ON_FALSE((emac->spi.ctx = emac->spi.init(DM9058_config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    }

    emac->multi_reg_axs_mutex = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(emac->multi_reg_axs_mutex, NULL, err, TAG, "create multi registers access mutex failed");

    const esp_eth_ptp_dm9058_ops_t ptp_ops = {
        .reg_read = DM9058_ptp_reg_read,
        .reg_write = DM9058_ptp_reg_write,
        .reg_burst_read = DM9058_ptp_reg_burst_read,
        .reg_burst_write = DM9058_ptp_reg_burst_write,
        .delay_us = DM9058_ptp_delay_us,
        .delay_ms = DM9058_ptp_delay_ms,
        .lock = DM9058_ptp_lock,
        .unlock = DM9058_ptp_unlock,
    };
    ESP_GOTO_ON_FALSE(esp_eth_ptp_dm9058_init(&emac->ptp, emac, &ptp_ops) == ESP_OK, NULL, err, TAG, "init dm9058 ptp context failed");

    BaseType_t core_num = tskNO_AFFINITY;
    if (mac_config->flags & ETH_MAC_FLAG_PIN_TO_CORE) {
        core_num = esp_cpu_get_core_id();
    }
    BaseType_t xReturned = xTaskCreatePinnedToCore(esp32_DM9058_task, "DM9058_tsk", mac_config->rx_task_stack_size, emac,
                                                   mac_config->rx_task_prio, &emac->rx_task_hdl, core_num);
    ESP_GOTO_ON_FALSE(xReturned == pdPASS, NULL, err, TAG, "create DM9058 task failed");

    emac->rx_buffer = heap_caps_malloc(ETH_MAX_PACKET_SIZE, MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(emac->rx_buffer, NULL, err, TAG, "RX buffer allocation failed");

    if (emac->int_gpio_num < 0) {
        const esp_timer_create_args_t poll_timer_args = {
            .callback = DM9058_poll_timer,
            .name = "esp32_spi_poll_timer",
            .arg = emac,
            .skip_unhandled_events = true
        };
        ESP_GOTO_ON_FALSE(esp_timer_create(&poll_timer_args, &emac->poll_timer) == ESP_OK, NULL, err, TAG, "create poll timer failed");
    }

    return &(emac->parent);

err:
    if (emac) {
        if (emac->poll_timer) {
            esp_timer_delete(emac->poll_timer);
        }
        if (emac->rx_task_hdl) {
            vTaskDelete(emac->rx_task_hdl);
        }
        if (emac->spi.ctx) {
            emac->spi.deinit(emac->spi.ctx);
        }
        if (emac->multi_reg_axs_mutex) {
            vSemaphoreDelete(emac->multi_reg_axs_mutex);
        }
        heap_caps_free(emac->rx_buffer);
        free(emac);
    }
    return ret;
}


/* ============================================================================
 * 修改 6: 在 esp_eth_mac_spi.h 中添加新的 IOCTL 命令定義
 * 位置: components/esp_eth/include/esp_eth_mac_spi.h
 * ============================================================================ */

typedef enum {
    ETH_MAC_DM9058_CMD_PTP_ENABLE = ETH_CMD_CUSTOM_MAC_CMDS_OFFSET,
    ETH_MAC_DM9058_CMD_S_PTP_TIME,
    ETH_MAC_DM9058_CMD_G_PTP_TIME,
    ETH_MAC_DM9058_CMD_ADJ_PTP_FREQ,
    ETH_MAC_DM9058_CMD_ADJ_PTP_TIME,
    ETH_MAC_DM9058_CMD_G_PTP_TX_TIME,
    ETH_MAC_DM9058_CMD_S_TARGET_TIME,
    ETH_MAC_DM9058_CMD_S_TARGET_CB,

    // === 新增命令 ===
    ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS,  /*!< Enable/disable auto PTP packet processing, expects bool* */
    ETH_MAC_DM9058_CMD_PTP_SET_MODE,      /*!< Set PTP mode (true=two-step, false=one-step), expects bool* */
    ETH_MAC_DM9058_CMD_G_PTP_RX_TIME,     /*!< Get last RX timestamp from cached value, expects eth_mac_time_t* */
} eth_mac_dm9058_io_cmd_t;


/* ============================================================================
 * 方案 C：創建一個簡化的輔助函數（最佳實踐）
 *
 * 將 PTP RX 處理封裝成一個輔助函數，讓主接收流程更清晰
 * ============================================================================ */

/**
 * @brief 處理 RX PTP 時戳（如果有）
 *
 * @param emac MAC 層上下文
 * @param header RX header 結構
 * @return ESP_OK 成功，其他錯誤碼表示失敗（但不影響正常封包接收）
 */
static esp_err_t DM9058_handle_rx_ptp_timestamp(esp32_DM9058_t *emac, const DM9058_rx_header_t *header)
{
    if (!emac->ptp_auto_process || !emac->ptp.enabled) {
        return ESP_OK;  // PTP 未啟用，直接返回
    }

    esp_err_t ret = ESP_OK;
    uint8_t rx_header_bytes[4] = {header->flag, header->status, header->length_low, header->length_high};
    esp_eth_ptp_dm9058_rx_info_t rx_info = {0};

    // 解析 header 確定是否有時戳
    ret = esp_eth_ptp_dm9058_parse_rx_header(rx_header_bytes, sizeof(rx_header_bytes),
                                              ETH_MAX_PACKET_SIZE, &rx_info);
    if (ret != ESP_OK || !rx_info.timestamp_available) {
        emac->rx_timestamp_valid = false;
        return ESP_OK;  // 沒有時戳不算錯誤
    }

    // 讀取時戳資料
    uint8_t ts_buffer[8] = {0};
    ret = DM9058_memory_read(emac, ts_buffer, rx_info.timestamp_len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read RX timestamp from FIFO");
        emac->rx_timestamp_valid = false;
        return ret;
    }

    // 解析時戳
    ret = esp_eth_ptp_dm9058_rx_timestamp(ts_buffer, rx_info.timestamp_len,
                                           &emac->last_rx_timestamp);
    if (ret == ESP_OK) {
        emac->rx_timestamp_valid = true;
        ESP_LOGD(TAG, "RX PTP TS: %lu.%09lu",
                 emac->last_rx_timestamp.seconds,
                 emac->last_rx_timestamp.nanoseconds);
    } else {
        emac->rx_timestamp_valid = false;
        ESP_LOGW(TAG, "Failed to decode RX timestamp");
    }

    return ret;
}

/**
 * @brief 簡化版的 frame_to_rx_buffer，使用輔助函數
 */
static esp_err_t DM9058_frame_to_rx_buffer_simplified(esp32_DM9058_t *emac, uint16_t *size)
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

                // === 使用輔助函數處理 PTP 時戳（一行搞定！） ===
                DM9058_handle_rx_ptp_timestamp(emac, &header);
                // 注意：即使時戳處理失敗，也不影響封包接收

            } else {
                ESP_GOTO_ON_ERROR(DM9058_flush_recv_queue(emac), err, TAG, "flush rx queue failed");
                ESP_GOTO_ON_FALSE(false, ESP_FAIL, err, TAG, "invalid frame length, reset rx fifo pointer");
            }
            if (header.status & (RSR_RF | RSR_RWTO | RSR_CE | RSR_FOE)) {
                DM9058_skip_recv_frame(emac, rx_len);
                ESP_LOGE(TAG, "receive status error: %" PRIx8 "H", header.status);
                try_again = true;
            } else {
                *size = rx_len;
            }
        }
    } while (try_again);
err:
    return ret;
}


/* ============================================================================
 * 應用層使用範例
 * ============================================================================ */

#include "esp_eth.h"
#include "esp_eth_mac_spi.h"
#include "esp_log.h"

static const char *TAG = "ptp_example";

void ptp_auto_mode_example(esp_eth_handle_t eth_handle)
{
    ESP_LOGI(TAG, "Setting up auto PTP mode");

    // 1. 啟用 PTP 硬體
    bool enable = true;
    esp_err_t ret = esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_ENABLE, &enable);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable PTP: %d", ret);
        return;
    }

    // 2. 啟用自動 PTP 處理
    bool auto_process = true;
    ret = esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS, &auto_process);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable auto PTP: %d", ret);
        return;
    }

    // 3. 設定 PTP 模式（可選，預設為雙步）
    bool two_step = false;  // 使用單步模式
    ret = esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_SET_MODE, &two_step);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PTP mode: %d", ret);
        return;
    }

    // 4. 設定初始時間
    eth_mac_time_t init_time = {
        .seconds = 0,
        .nanoseconds = 0
    };
    ret = esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_S_PTP_TIME, &init_time);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PTP time: %d", ret);
        return;
    }

    ESP_LOGI(TAG, "Auto PTP mode configured successfully");
}

void ptp_transmit_example(esp_eth_handle_t eth_handle, uint8_t *ptp_packet, size_t len)
{
    ESP_LOGI(TAG, "Sending PTP packet, len=%d", len);

    // 直接發送，PTP 硬體會自動配置（如果啟用了 auto_process）
    esp_err_t ret = esp_eth_transmit(eth_handle, ptp_packet, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transmit: %d", ret);
        return;
    }

    // 等待一小段時間讓硬體完成發送
    vTaskDelay(pdMS_TO_TICKS(10));

    // 讀取 TX 時戳
    eth_mac_time_t tx_time;
    ret = esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TX_TIME, &tx_time);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "TX timestamp: %lu.%09lu", tx_time.seconds, tx_time.nanoseconds);
    } else {
        ESP_LOGW(TAG, "Failed to get TX timestamp: %d", ret);
    }
}

void ptp_receive_check(esp_eth_handle_t eth_handle)
{
    // 在接收到封包後，可以檢查是否有 RX 時戳
    eth_mac_time_t rx_time;
    esp_err_t ret = esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_RX_TIME, &rx_time);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RX timestamp: %lu.%09lu", rx_time.seconds, rx_time.nanoseconds);
    } else if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGD(TAG, "No RX timestamp available");
    } else {
        ESP_LOGW(TAG, "Failed to get RX timestamp: %d", ret);
    }
}

void ptp_periodic_sync_task(void *arg)
{
    esp_eth_handle_t eth_handle = (esp_eth_handle_t)arg;

    while (1) {
        // 定期讀取 PTP 時間
        eth_mac_time_t current_time;
        esp_err_t ret = esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TIME, &current_time);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Current PTP time: %lu.%09lu",
                     current_time.seconds, current_time.nanoseconds);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
