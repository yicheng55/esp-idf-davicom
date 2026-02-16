# esp_eth_ptp_dm9058_parse_rx_packet 整合使用指南

## 函數概述

`esp_eth_ptp_dm9058_parse_rx_packet` 是一個高層次的 API，它整合了多個 RX 相關的功能：

```c
esp_err_t esp_eth_ptp_dm9058_parse_rx_packet(
    esp_eth_ptp_dm9058_t *ptp,
    const uint8_t *rx_header,           // RX 4-byte header
    size_t rx_header_len,               // = 4
    const uint8_t *rx_ts_buffer,        // 時戳 buffer (4 or 8 bytes)
    size_t rx_ts_buffer_len,            // 時戳 buffer 長度
    uint16_t max_packet_len,            // 最大封包長度
    esp_eth_ptp_dm9058_rx_info_t *info, // 輸出：RX 資訊
    esp_eth_ptp_dm9058_time_t *time     // 輸出：時戳
);
```

### 內部執行的操作

1. **檢查 RX 就緒**: 調用 `esp_eth_ptp_dm9058_rx_ready()`
2. **解析 header**: 調用 `esp_eth_ptp_dm9058_parse_rx_header()`
3. **解析時戳**: 調用 `esp_eth_ptp_dm9058_rx_timestamp()`

## 整合策略比較

### 方案 A：使用低層次 API（推薦）✅

**優點**:
- 更高效：避免重複的 ready check
- 更靈活：可以根據需要調整流程
- 更清晰：每個步驟都是明確的

**適用場景**:
- DM9058 的標準接收流程（逐步讀取 FIFO）
- 需要細粒度控制的場景

**實作**（見 `DM9058_PTP_優化實作範例.c`）:
```c
// 1. 讀取並解析 header
uint8_t rx_header_bytes[4] = {...};
esp_eth_ptp_dm9058_rx_info_t rx_info;
esp_eth_ptp_dm9058_parse_rx_header(rx_header_bytes, 4, max_len, &rx_info);

// 2. 讀取封包資料
DM9058_memory_read(emac, emac->rx_buffer, rx_len);

// 3. 如果有時戳，讀取並解析
if (rx_info.timestamp_available) {
    uint8_t ts_buffer[8];
    DM9058_memory_read(emac, ts_buffer, rx_info.timestamp_len);
    esp_eth_ptp_dm9058_rx_timestamp(ts_buffer, rx_info.timestamp_len, &time);
}
```

### 方案 B：使用高層次 API

**優點**:
- 代碼簡潔：一次調用完成所有操作
- 標準化：使用統一的 API

**缺點**:
- 可能產生不必要的 ready check（因為已經知道有資料）
- 需要預先準備時戳 buffer（可能不知道長度）

**適用場景**:
- 已經有完整的 RX buffer（包含 header + timestamp）
- 從外部讀取完整的 RX 資料後再解析

**實作**:
```c
// 假設已經讀取了 header 和可能的時戳
uint8_t rx_header[4];
uint8_t ts_buffer[8];
esp_eth_ptp_dm9058_rx_info_t rx_info;
esp_eth_ptp_dm9058_time_t time;

// 一次性解析
esp_eth_ptp_dm9058_parse_rx_packet(
    &emac->ptp,
    rx_header, 4,
    ts_buffer, 8,
    max_len,
    &rx_info,
    &time
);
```

### 方案 C：創建輔助函數（最佳實踐）✅✅

**優點**:
- 封裝良好：主流程清晰
- 易於維護：PTP 邏輯集中在一處
- 容錯性好：時戳處理失敗不影響封包接收

**實作**（見 `DM9058_PTP_優化實作範例.c`）:
```c
static esp_err_t DM9058_handle_rx_ptp_timestamp(esp32_DM9058_t *emac,
                                                 const DM9058_rx_header_t *header)
{
    // 封裝所有 PTP 時戳處理邏輯
    // 使用低層次 API 實現
}

// 在主接收流程中：
DM9058_memory_read(emac, emac->rx_buffer, rx_len);
DM9058_handle_rx_ptp_timestamp(emac, &header);  // 一行搞定！
```

## 推薦實作流程

### 步驟 1：創建輔助函數

```c
/**
 * @brief 處理 RX PTP 時戳（如果有）
 *
 * 這個函數封裝了完整的 RX 時戳處理流程：
 * 1. 檢查是否啟用 PTP
 * 2. 解析 header 確定是否有時戳
 * 3. 從 FIFO 讀取時戳資料
 * 4. 解析並存儲時戳
 *
 * @param emac MAC 層上下文
 * @param header RX header 結構
 * @return ESP_OK 成功，其他錯誤碼表示失敗
 * @note 即使失敗也不影響正常封包接收
 */
static esp_err_t DM9058_handle_rx_ptp_timestamp(esp32_DM9058_t *emac,
                                                 const DM9058_rx_header_t *header)
{
    // 快速路徑：PTP 未啟用則直接返回
    if (!emac->ptp_auto_process || !emac->ptp.enabled) {
        return ESP_OK;
    }

    // 準備 header bytes
    uint8_t rx_header_bytes[4] = {
        header->flag,
        header->status,
        header->length_low,
        header->length_high
    };

    // 解析 header 獲取 PTP 資訊
    esp_eth_ptp_dm9058_rx_info_t rx_info = {0};
    esp_err_t ret = esp_eth_ptp_dm9058_parse_rx_header(
        rx_header_bytes,
        sizeof(rx_header_bytes),
        ETH_MAX_PACKET_SIZE,
        &rx_info
    );

    // 沒有時戳或解析失敗，不算錯誤
    if (ret != ESP_OK || !rx_info.timestamp_available) {
        emac->rx_timestamp_valid = false;
        return ESP_OK;
    }

    // 讀取時戳資料
    uint8_t ts_buffer[8] = {0};
    ret = DM9058_memory_read(emac, ts_buffer, rx_info.timestamp_len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read RX timestamp");
        emac->rx_timestamp_valid = false;
        return ret;
    }

    // 解析時戳
    ret = esp_eth_ptp_dm9058_rx_timestamp(
        ts_buffer,
        rx_info.timestamp_len,
        &emac->last_rx_timestamp
    );

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
```

### 步驟 2：在接收流程中使用

```c
static esp_err_t DM9058_frame_to_rx_buffer(esp32_DM9058_t *emac, uint16_t *size)
{
    // ... 前面的程式碼 ...

    // 讀取封包資料
    ESP_GOTO_ON_ERROR(DM9058_memory_read(emac, emac->rx_buffer, rx_len),
                      err, TAG, "read rx data failed");

    // 處理 PTP 時戳（一行搞定！）
    DM9058_handle_rx_ptp_timestamp(emac, &header);
    // 注意：即使時戳處理失敗，也不影響封包接收

    // ... 後面的程式碼 ...
}
```

## API 對比表

| API | 層次 | 用途 | 何時使用 |
|-----|------|------|----------|
| `esp_eth_ptp_dm9058_rx_ready` | 低 | 檢查 RX 是否就緒 | 需要手動檢查 FIFO 狀態 |
| `esp_eth_ptp_dm9058_parse_rx_header` | 低 | 解析 RX header | 已讀取 header，需要知道時戳資訊 |
| `esp_eth_ptp_dm9058_rx_timestamp` | 低 | 解析時戳 buffer | 已讀取時戳資料，需要轉換為時間結構 |
| `esp_eth_ptp_dm9058_parse_rx_packet` | 高 | 一次完成所有操作 | 有完整資料需要一次性解析 |
| `DM9058_handle_rx_ptp_timestamp` | 輔助 | 封裝的 PTP 時戳處理 | **推薦用於 DM9058** |

## 完整使用範例

### 應用層讀取 RX 時戳

```c
void ethernet_rx_callback(void *buffer, size_t len)
{
    // 封包已經由 MAC 層接收並處理（包括 PTP 時戳）

    // 檢查是否有 PTP 時戳可用
    esp_eth_handle_t eth_handle = get_eth_handle();
    eth_mac_time_t rx_time;

    esp_err_t ret = esp_eth_ioctl(eth_handle,
                                   ETH_MAC_DM9058_CMD_G_PTP_RX_TIME,
                                   &rx_time);

    if (ret == ESP_OK) {
        // 有 RX 時戳可用
        ESP_LOGI(TAG, "Packet received with timestamp: %lu.%09lu",
                 rx_time.seconds, rx_time.nanoseconds);

        // 處理 PTP 封包...
        process_ptp_packet(buffer, len, &rx_time);
    } else if (ret == ESP_ERR_NOT_FOUND) {
        // 沒有時戳（非 PTP 封包或時戳未啟用）
        ESP_LOGD(TAG, "Packet received without timestamp");
    } else {
        // 錯誤
        ESP_LOGW(TAG, "Failed to get RX timestamp: %d", ret);
    }
}
```

### 完整的 PTP 接收處理流程

```c
void ptp_sync_task(void *arg)
{
    esp_eth_handle_t eth_handle = (esp_eth_handle_t)arg;

    // 1. 啟用 PTP
    bool enable = true;
    esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_ENABLE, &enable);

    // 2. 啟用自動處理
    bool auto_process = true;
    esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS, &auto_process);

    // 3. 在接收回調中
    // MAC 層會自動處理 RX 時戳
    // 應用層只需要在需要時讀取即可

    while (1) {
        // 定期檢查 PTP 狀態
        eth_mac_time_t current;
        esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TIME, &current);

        // 如果收到了 PTP 封包，可以讀取最後的 RX 時戳
        eth_mac_time_t rx_ts;
        if (esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_RX_TIME, &rx_ts) == ESP_OK) {
            // 處理時戳
            ESP_LOGI(TAG, "Last RX TS: %lu.%09lu", rx_ts.seconds, rx_ts.nanoseconds);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

## 性能分析

### 各方案的性能對比

| 方案 | CPU 開銷 | 記憶體 | 代碼複雜度 | 推薦度 |
|------|---------|--------|-----------|--------|
| 方案 A（低層次 API） | 低 | 最少 | 中 | ⭐⭐⭐⭐ |
| 方案 B（高層次 API） | 中 | 中 | 低 | ⭐⭐⭐ |
| 方案 C（輔助函數） | 低 | 少 | 低 | ⭐⭐⭐⭐⭐ |

### 時序分析（每個封包）

```
非 PTP 封包（auto_process 開啟）:
├─ 快速檢查: < 1 μs
└─ 總開銷: < 1 μs

PTP 封包無時戳:
├─ parse_rx_header: ~15 μs
└─ 總開銷: ~15 μs

PTP 封包有時戳:
├─ parse_rx_header: ~15 μs
├─ memory_read (4-8 bytes): ~20 μs
├─ rx_timestamp: ~10 μs
└─ 總開銷: ~45 μs
```

## 故障排除

### 常見問題

**Q1: RX 時戳總是返回 ESP_ERR_NOT_FOUND**
```c
// 檢查清單
1. PTP 是否已啟用？
   esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_ENABLE, &enable);

2. 自動處理是否已啟用？
   esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS, &auto_process);

3. 是否真的收到了 PTP 封包？
   // 檢查封包的 EtherType 是否為 0x88F7 或 UDP port 319/320
```

**Q2: 時戳值看起來不正確**
```c
// 確認：
1. PTP 時鐘是否已正確初始化？
   eth_mac_time_t init_time = {0, 0};
   esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_S_PTP_TIME, &init_time);

2. 傳輸層協議是否匹配？
   // 確保 enable 時使用的 transport 與實際封包一致
   esp_eth_ptp_dm9058_enable(&ptp, true, ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4);
```

**Q3: 時戳讀取影響效能**
```c
// 優化方法：
1. 使用 auto_process，只在需要時讀取
2. 快取機制已內建（rx_timestamp_valid 旗標）
3. 考慮使用更高的任務優先級處理 PTP
```

## 總結

### 推薦實作方式

對於 DM9058，**推薦使用方案 C**（輔助函數封裝）：

1. ✅ **清晰**: 主接收流程簡潔
2. ✅ **高效**: 使用低層次 API，避免不必要的操作
3. ✅ **可靠**: 錯誤處理完善
4. ✅ **易維護**: PTP 邏輯集中管理

### 關鍵要點

- `esp_eth_ptp_dm9058_parse_rx_packet` 是高層次 API，適合完整資料解析
- DM9058 的 FIFO 讀取特性更適合使用低層次 API
- 創建輔助函數封裝 PTP 處理是最佳實踐
- 自動處理模式讓應用層使用更簡單

### 下一步

1. 根據 `DM9058_PTP_優化實作範例.c` 中的程式碼修改你的專案
2. 測試 PTP 封包的接收和時戳提取
3. 監控效能和日誌，確認運作正常
