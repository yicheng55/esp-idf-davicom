# DM9058 PTP 功能整合指南

這份文件說明如何使用 `esp_eth_mac_dm9058.c` 和 `esp_eth_ptp_dm9058.c` 兩個檔案來實現 IEEE 1588 PTP (Precision Time Protocol) 功能。

## 架構概述

### 檔案關係

```
esp_eth_mac_dm9058.c (MAC 層驅動)
    ├── 包含 esp_eth_ptp_dm9058.h
    ├── 內部維護一個 esp_eth_ptp_dm9058_t 結構
    └── 透過 custom_ioctl 介面暴露 PTP 功能

esp_eth_ptp_dm9058.c (PTP 功能模組)
    ├── 提供 PTP 時間管理 API
    ├── 提供時戳擷取和處理 API
    └── 透過回調函數存取 MAC 層硬體
```

### 關鍵資料結構

```c
// MAC 層結構 (esp_eth_mac_dm9058.c)
typedef struct {
    esp_eth_mac_t parent;
    // ... 其他成員
    esp_eth_ptp_dm9058_t ptp;  // PTP 功能子系統
} esp32_DM9058_t;

// PTP 子系統結構 (esp_eth_ptp_dm9058.h)
typedef struct {
    void *io_ctx;                      // 指向 MAC 層的 esp32_DM9058_t
    esp_eth_ptp_dm9058_ops_t ops;      // 硬體存取回調函數
    bool initialized;
    bool enabled;
    int64_t last_rate;                 // 頻率調整追蹤
} esp_eth_ptp_dm9058_t;
```

## 初始化流程

### 1. MAC 層初始化時自動初始化 PTP

在 `esp_eth_mac_new_dm9058()` 函數中(第 1102-1110 行):

```c
// 設定 PTP 操作回調函數
const esp_eth_ptp_dm9058_ops_t ptp_ops = {
    .reg_read = DM9058_ptp_reg_read,          // 讀取暫存器
    .reg_write = DM9058_ptp_reg_write,        // 寫入暫存器
    .reg_burst_read = DM9058_ptp_reg_burst_read,   // 連續讀取
    .reg_burst_write = DM9058_ptp_reg_burst_write, // 連續寫入
    .delay_us = DM9058_ptp_delay_us,          // 微秒延遲
    .delay_ms = DM9058_ptp_delay_ms,          // 毫秒延遲
    .lock = DM9058_ptp_lock,                  // 互斥鎖
    .unlock = DM9058_ptp_unlock,              // 解鎖
};

// 初始化 PTP 子系統
ESP_GOTO_ON_FALSE(esp_eth_ptp_dm9058_init(&emac->ptp, emac, &ptp_ops) == ESP_OK,
                  NULL, err, TAG, "init dm9058 ptp context failed");
```

### 2. 回調函數實作 (第 205-246 行)

這些函數將 PTP 層的請求轉發給 MAC 層:

```c
static esp_err_t DM9058_ptp_reg_write(void *io_ctx, uint8_t reg, uint8_t value)
{
    return DM9058_register_write((esp32_DM9058_t *)io_ctx, reg, value);
}

static esp_err_t DM9058_ptp_reg_read(void *io_ctx, uint8_t reg, uint8_t *value)
{
    return DM9058_register_read((esp32_DM9058_t *)io_ctx, reg, value);
}

static bool DM9058_ptp_lock(void *io_ctx)
{
    return DM9058_mutex_lock((esp32_DM9058_t *)io_ctx);
}

static void DM9058_ptp_unlock(void *io_ctx)
{
    DM9058_mutex_unlock((esp32_DM9058_t *)io_ctx);
}
```

## 使用方式

### 1. 啟用 PTP 功能

透過 `custom_ioctl` 介面 (第 756-758 行):

```c
// 在應用程式中
esp_eth_handle_t eth_handle;

// 啟用 PTP
bool enable = true;
esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_ENABLE, &enable);
```

內部執行 (esp_eth_ptp_dm9058.c 第 158-205 行):
- 重置 PTP 控制器
- 設定傳輸層協議 (UDP IPv4/IPv6 或 IEEE 802.3)
- 配置時戳偏移和校驗和偏移
- 啟用 PTP 硬體

### 2. 設定和讀取 PTP 時間

```c
#include "esp_eth_mac_spi.h"

// 設定 PTP 時間
eth_mac_time_t set_time = {
    .seconds = 1234567890,
    .nanoseconds = 123456789
};
esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_S_PTP_TIME, &set_time);

// 讀取 PTP 時間
eth_mac_time_t get_time;
esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TIME, &get_time);
printf("PTP Time: %lu.%09lu\n", get_time.seconds, get_time.nanoseconds);
```

### 3. 調整 PTP 頻率 (頻率校正)

```c
// 調整頻率，單位為 ppb (parts per billion)
// 正值表示加快，負值表示減慢
int32_t adj_ppb = 100;  // 加快 100 ppb
esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_ADJ_PTP_FREQ, &adj_ppb);
```

內部實作 (esp_eth_ptp_dm9058.c 第 278-317 行):
- 計算頻率調整增量
- 決定加快或減慢模式
- 寫入調整值到硬體

### 4. 調整 PTP 時間 (時間偏移)

```c
// 調整時間偏移 (加上或減去一個時間差)
eth_mac_time_t offset = {
    .seconds = 0,          // 可以是負值 (int32_t)
    .nanoseconds = 1000000 // 增加 1 毫秒
};
esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_ADJ_PTP_TIME, &offset);
```

內部實作 (esp_eth_ptp_dm9058.c 第 254-276 行):
- 讀取當前時間
- 加上偏移值
- 處理進位和借位
- 設定新時間

### 5. 獲取傳輸時戳 (TX Timestamp)

```c
// 在發送 PTP 封包後獲取時戳
eth_mac_time_t tx_time;
esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TX_TIME, &tx_time);
printf("TX Timestamp: %lu.%09lu\n", tx_time.seconds, tx_time.nanoseconds);
```

## PTP 封包處理

### 傳送端處理流程

PTP 模組會自動分析傳送的封包並配置硬體:

1. **封包解析** (esp_eth_ptp_dm9058.c 第 460-516 行)
   - `esp_eth_ptp_dm9058_parse_tx_packet()` 分析封包類型
   - 檢查是否為 PTP 封包 (Layer 2 或 UDP)
   - 識別訊息類型 (SYNC, DELAY_REQ, PDELAY_REQ 等)

2. **模式決策** (第 476-513 行)
   ```c
   switch (msg_type) {
   case ESP_ETH_PTP_DM9058_MSG_SYNC:
       if (!two_step_mode) {
           // One-step: 硬體自動插入時戳
           config->enable_onestep_insert = true;
       } else {
           // Two-step: 擷取時戳供 Follow_Up 使用
           config->enable_timestamp_capture = true;
       }
       break;

   case ESP_ETH_PTP_DM9058_MSG_DELAY_REQ:
   case ESP_ETH_PTP_DM9058_MSG_PDELAY_REQ:
   case ESP_ETH_PTP_DM9058_MSG_PDELAY_RESP:
       // 永遠擷取時戳
       config->enable_timestamp_capture = true;
       break;
   }
   ```

3. **硬體配置** (第 519-557 行)
   - `esp_eth_ptp_dm9058_prepare_tx()` 配置 TCR 暫存器
   - 設定 bit 7 (TSEN_CAP): 啟用時戳擷取
   - 設定 bit 6 (TS1STEP_EMIT): 啟用單步插入

### 接收端處理流程

1. **檢查接收就緒** (第 560-595 行)
   ```c
   bool ready = false;
   esp_eth_ptp_dm9058_rx_ready(&emac->ptp, &ready);
   ```

2. **解析接收標頭** (第 599-620 行)
   - `esp_eth_ptp_dm9058_parse_rx_header()` 解析 4 位元組標頭
   - 檢查是否有時戳可用 (RSR_RXTS_EN 位元)
   - 確定時戳長度 (4 或 8 位元組)

3. **提取時戳** (第 622-664 行)
   ```c
   esp_eth_ptp_dm9058_rx_info_t rx_info;
   esp_eth_ptp_dm9058_time_t rx_time;

   esp_eth_ptp_dm9058_parse_rx_packet(
       &emac->ptp,
       rx_header, sizeof(rx_header),
       rx_ts_buffer, ts_len,
       max_packet_len,
       &rx_info,
       &rx_time
   );

   if (rx_info.timestamp_available) {
       printf("RX Timestamp: %lu.%09lu\n",
              rx_time.seconds, rx_time.nanoseconds);
   }
   ```

## PTP 訊息類型

```c
typedef enum {
    ESP_ETH_PTP_DM9058_MSG_SYNC         = 0x0,  // 同步訊息
    ESP_ETH_PTP_DM9058_MSG_DELAY_REQ    = 0x1,  // 延遲請求
    ESP_ETH_PTP_DM9058_MSG_PDELAY_REQ   = 0x2,  // 點對點延遲請求
    ESP_ETH_PTP_DM9058_MSG_PDELAY_RESP  = 0x3,  // 點對點延遲回應
    ESP_ETH_PTP_DM9058_MSG_FOLLOW_UP    = 0x8,  // 跟隨訊息
    ESP_ETH_PTP_DM9058_MSG_DELAY_RESP   = 0x9,  // 延遲回應
    ESP_ETH_PTP_DM9058_MSG_ANNOUNCE     = 0xB,  // 公告訊息
} esp_eth_ptp_dm9058_msg_type_t;
```

## PTP 傳輸模式

### One-Step (單步) 模式

硬體自動在 SYNC 訊息中插入時戳:

```c
esp_eth_ptp_dm9058_set_tx_mode(&emac->ptp, ESP_ETH_PTP_DM9058_TX_MODE_ONE_STEP);
```

- 優點: 不需要發送 Follow_Up 訊息，效率高
- 缺點: 需要硬體支援，彈性較低

### Two-Step (雙步) 模式

軟體在 Follow_Up 訊息中插入時戳:

```c
esp_eth_ptp_dm9058_set_tx_mode(&emac->ptp, ESP_ETH_PTP_DM9058_TX_MODE_TWO_STEP);
```

1. 發送 SYNC 訊息
2. 讀取傳輸時戳
3. 在 Follow_Up 訊息中報告時戳

## PTP 傳輸層支援

```c
typedef enum {
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4 = 0,  // UDP over IPv4
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV6,      // UDP over IPv6
    ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3,    // Layer 2 Ethernet
} esp_eth_ptp_dm9058_transport_t;

// 在啟用 PTP 時指定傳輸層
esp_eth_ptp_dm9058_enable(&emac->ptp, true,
                          ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4);
```

## 實際應用範例

### 完整的 PTP 客戶端範例

```c
#include "esp_eth.h"
#include "esp_eth_mac_spi.h"

void ptp_client_example(esp_eth_handle_t eth_handle)
{
    // 1. 啟用 PTP
    bool enable = true;
    esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_ENABLE, &enable);

    // 2. 設定初始時間
    eth_mac_time_t init_time = {
        .seconds = 0,
        .nanoseconds = 0
    };
    esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_S_PTP_TIME, &init_time);

    // 3. 設定為雙步模式 (如果需要)
    // esp_eth_ptp_dm9058_set_tx_mode(&emac->ptp, ESP_ETH_PTP_DM9058_TX_MODE_TWO_STEP);

    // 4. 啟用 TX 時戳擷取
    // esp_eth_ptp_dm9058_enable_tx_timestamp(&emac->ptp, true);

    // 5. 定期讀取時間
    while (1) {
        eth_mac_time_t current_time;
        esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TIME, &current_time);

        printf("Current PTP Time: %lu.%09lu\n",
               current_time.seconds, current_time.nanoseconds);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### PTP 同步範例 (簡化版)

```c
void ptp_sync_example(esp_eth_handle_t eth_handle)
{
    // 假設已從主時鐘收到時間資訊
    eth_mac_time_t master_time = {
        .seconds = 1234567890,
        .nanoseconds = 500000000
    };

    // 讀取本地時間
    eth_mac_time_t local_time;
    esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TIME, &local_time);

    // 計算時間差
    int64_t diff_sec = (int64_t)master_time.seconds - (int64_t)local_time.seconds;
    int64_t diff_nsec = (int64_t)master_time.nanoseconds - (int64_t)local_time.nanoseconds;

    // 如果差異很大，直接設定時間
    if (llabs(diff_sec) > 1 || llabs(diff_nsec) > 100000000) {
        esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_S_PTP_TIME, &master_time);
        printf("Large offset detected, time set directly\n");
    } else {
        // 如果差異小，用調整方式
        eth_mac_time_t offset = {
            .seconds = (uint32_t)diff_sec,
            .nanoseconds = (uint32_t)diff_nsec
        };
        esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_ADJ_PTP_TIME, &offset);
        printf("Small offset adjusted: %lld.%09lld s\n", diff_sec, diff_nsec);
    }
}
```

## 硬體暫存器對應

### PTP 相關暫存器 (esp_eth_ptp_dm9058.c 第 8-26 行)

| 暫存器 | 位址 | 功能 |
|--------|------|------|
| PTPCR  | 0x60 | PTP 控制暫存器 (重置) |
| PTPCW  | 0x61 | PTP 命令寫入 |
| PTPTSM | 0x62 | PTP 時戳模式 |
| PTPTX  | 0x63 | PTP 傳輸模式 (單步/雙步) |
| PTPMMP | 0x64 | PTP 多播模式 |
| PTPTSO | 0x65 | PTP 時戳偏移 |
| PTPCSO | 0x66 | PTP 校驗和偏移 |
| PTPTS  | 0x68 | PTP 時戳資料 |
| TCR    | 0x02 | 傳輸控制暫存器 |

### TCR 暫存器位元 (第 18-21 行)

| 位元 | 功能 |
|------|------|
| Bit 7 (TSEN_CAP) | 啟用 TX 時戳擷取 |
| Bit 6 (TS1STEP_EMIT) | 啟用單步時戳插入 |

## 注意事項

### 1. 執行緒安全
- PTP 模組內部使用 `lock`/`unlock` 回調保證執行緒安全
- 多個任務可以安全地調用 PTP API

### 2. 時戳精度
- 硬體提供奈秒級精度
- 實際精度取決於晶振品質和系統延遲

### 3. 頻率調整範圍
```c
#define DM9058_PTP_MAX_ADJUSTMENT  (0xEFFFFFFFU)
```
- 調整值超過此範圍會被限制

### 4. 傳輸延遲
- 接收和傳輸路徑都有固定延遲
- PTPTSO (時戳偏移) 用於補償這些延遲

### 5. 封包過濾
- PTP 模組會自動識別 PTP 封包
- 支援 Layer 2 (EtherType 0x88F7) 和 Layer 3/4 (UDP port 319/320)

## 除錯技巧

### 啟用除錯日誌

在 `esp_eth_ptp_dm9058.c` 中使用:
```c
ESP_LOGD("dm9058.ptp", "PTP time: %lu.%09lu", time->seconds, time->nanoseconds);
```

### 檢查 PTP 狀態

```c
// 檢查是否已初始化和啟用
if (!ptp->initialized) {
    ESP_LOGE(TAG, "PTP not initialized");
}
if (!ptp->enabled) {
    ESP_LOGE(TAG, "PTP not enabled");
}
```

### 驗證時戳

```c
// 讀取時間兩次，確認時鐘在運行
eth_mac_time_t time1, time2;
esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TIME, &time1);
vTaskDelay(pdMS_TO_TICKS(100));
esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TIME, &time2);

if (time2.seconds == time1.seconds && time2.nanoseconds == time1.nanoseconds) {
    ESP_LOGW(TAG, "PTP clock not running!");
}
```

## 相關檔案

- `esp_eth_mac_dm9058.c` - MAC 層驅動實作
- `esp_eth_ptp_dm9058.c` - PTP 功能實作
- `esp_eth_ptp_dm9058.h` - PTP API 定義
- `esp_eth_mac_spi.h` - SPI 乙太網路介面定義 (包含 ioctl 命令)
- `dm9058.h` - DM9058 暫存器定義

## 總結

DM9058 的 PTP 功能整合採用了模組化設計:

1. **MAC 層** (`esp_eth_mac_dm9058.c`) 負責:
   - 硬體初始化和存取
   - 提供 PTP 功能介面
   - 處理封包傳輸和接收

2. **PTP 層** (`esp_eth_ptp_dm9058.c`) 負責:
   - PTP 協議處理
   - 時間管理和調整
   - 封包分析和時戳處理

3. **應用層** 透過標準的 `esp_eth_ioctl()` API 來:
   - 啟用/停用 PTP
   - 設定和讀取時間
   - 調整頻率和時間偏移
   - 讀取傳輸時戳

這種設計使得 PTP 功能可以輕鬆整合到現有的 ESP-IDF 乙太網路驅動架構中，並提供清晰的 API 供應用程式使用。
