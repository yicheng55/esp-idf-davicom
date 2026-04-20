# DM9058 PTP 功能整合指南

這份文件依照目前專案中的實作，整理 DM9058 在 ESP-IDF 內的 IEEE 1588 PTP 整合方式。內容以實際程式碼為準，涵蓋 MAC 驅動、PTP 子模組、ioctl 介面，以及 RX/TX 時戳在系統中的傳遞路徑。

目前對應的主要檔案如下：

- `components/esp_eth/src/spi/dm9058/esp_eth_mac_dm9058.c`
- `components/esp_eth/src/spi/dm9058/esp_eth_ptp_dm9058.c`
- `components/esp_eth/src/spi/dm9058/esp_eth_ptp_dm9058.h`
- `components/esp_eth/include/esp_eth_mac_spi.h`
- `examples/ethernet/ptp/components/esp_eth_time/esp_eth_time.c`

## 專案現況摘要

先列出目前實作中最容易和舊版文件搞混的幾個重點：

1. `ETH_MAC_DM9058_CMD_PTP_ENABLE` 目前不是傳 `bool *`，而是傳 `esp_eth_ptp_dm9058_enable_config_t *`。
2. `ETH_MAC_DM9058_CMD_G_PTP_RX_TIME` 在 DM9058 驅動中已明確回傳 `ESP_ERR_NOT_SUPPORTED`，RX 時戳不再用 ioctl 輪詢。
3. RX 硬體時戳是在 MAC RX task 內解碼後，透過 `stack_input_info()` 和封包一起往上層傳。
4. 目前驅動只會把 `SYNC` 和 `DELAY_REQ` 這兩類 PTP 封包的 RX 時戳往上轉交；其他 PTP 訊息即使有時戳，也會在 MAC 層被過濾掉。
5. DM9058 的 Target Time / Target Callback ioctl case 已保留，但目前實作仍回傳 `ESP_ERR_NOT_SUPPORTED`。
6. two-step / one-step 的預設模式由 `CONFIG_ETH_DM9058_PTP_TWO_STEP_MODE` 控制，MAC 自動處理路徑會使用這個設定。

## 架構概述

### 檔案關係

```text
esp_eth_mac_dm9058.c
    ├── 建立 DM9058 MAC 實例
    ├── 內含 esp_eth_ptp_dm9058_t ptp 子系統
    ├── 透過 custom_ioctl 對外暴露 PTP 控制
    ├── 在 TX 路徑中自動分析 PTP 封包並配置硬體
    └── 在 RX task 中解析 DM9058 FIFO 內嵌時戳並透過 stack_input_info() 往上轉交

esp_eth_ptp_dm9058.c
    ├── 提供 PTP enable / get / set / adjust API
    ├── 提供 TX 封包分析與 TCR 配置 helper
    ├── 提供 RX header 與 RX timestamp 解碼 helper
    └── 透過 callback 存取 DM9058 暫存器與延遲函式

esp_eth_mac_spi.h
    ├── 定義 DM9058 的 ioctl 命令
    ├── 定義 PTP transport 列舉
    └── 定義 esp_eth_ptp_dm9058_enable_config_t
```

### 關鍵資料結構

```c
typedef struct {
    esp_eth_mac_t parent;
    esp_eth_mediator_t *eth;
    // ... 其他成員
    esp_eth_ptp_dm9058_t ptp;
    bool ptp_auto_process;
    bool ptp_two_step_mode;
} esp32_DM9058_t;

typedef struct {
    void *io_ctx;
    esp_eth_ptp_dm9058_ops_t ops;
    bool initialized;
    bool enabled;
    int64_t last_rate;
} esp_eth_ptp_dm9058_t;

typedef struct {
    bool enable;
    esp_eth_ptp_dm9058_transport_t transport;
} esp_eth_ptp_dm9058_enable_config_t;
```

`ptp_auto_process` 和 `ptp_two_step_mode` 都在 MAC 層結構內維護：

- `ptp_auto_process` 預設為 `true`，代表 MAC 會自動處理 TX/RX PTP 封包。
- `ptp_two_step_mode` 的預設值由 `CONFIG_ETH_DM9058_PTP_TWO_STEP_MODE` 決定。

## 初始化流程

### 1. 建立 MAC 實例時初始化 PTP 子系統

在 `esp_eth_mac_new_dm9058()` 中，驅動會先建立一組 PTP callback，然後呼叫 `esp_eth_ptp_dm9058_init()`：

```c
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

ESP_GOTO_ON_FALSE(esp_eth_ptp_dm9058_init(&emac->ptp, emac, &ptp_ops) == ESP_OK,
                  NULL, err, TAG, "init dm9058 ptp context failed");
```

這個初始化只建立 context，不會自動 enable PTP 硬體。

### 2. 啟用時才真正配置 PTP 硬體

真正啟用是在應用程式呼叫 ioctl 後，進到 `esp_eth_ptp_dm9058_enable()`。啟用時會做以下事情：

- reset PTP controller
- 根據 transport 設定 `PTPTSO` 和 `PTPCSO`
- 啟用 `PTPCW`
- 清掉 `TCR`
- 設定 `PTPMMP`
- 關閉 `PTPTX` 的 one-step 預設位元
- 將 `ptp->enabled` 標為 `true`

## 對外控制介面

DM9058 PTP 是透過 `esp_eth_ioctl()` 和 `ETH_MAC_DM9058_CMD_*` 命令來控制。

### 啟用 PTP

正確寫法如下：

```c
#include "esp_eth.h"
#include "esp_eth_mac_spi.h"

esp_eth_ptp_dm9058_enable_config_t ptp_cfg = {
    .enable = true,
    .transport = ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4,
};

ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_ENABLE, &ptp_cfg));
```

若只傳 `bool enable`，會和目前驅動的 `custom_ioctl` 參數型別不一致。

### 啟用或停用 MAC 自動 PTP 處理

```c
bool auto_process = true;
ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS, &auto_process));
```

`PTP_AUTO_PROCESS` 會影響兩條路徑：

- TX 時是否自動呼叫 `esp_eth_ptp_dm9058_prepare_tx_locked()` 解析封包並配置 TCR
- RX 時是否自動從 RX FIFO 讀出內嵌時戳並轉交到上層

### 設定與讀取 PTP 時間

```c
eth_mac_time_t set_time = {
    .seconds = 1234567890,
    .nanoseconds = 123456789,
};
ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_S_PTP_TIME, &set_time));

eth_mac_time_t now = {0};
ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TIME, &now));
printf("PTP Time: %lu.%09lu\n", now.seconds, now.nanoseconds);
```

### 調整頻率

```c
int32_t adj_ppb = 100;
ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_ADJ_PTP_FREQ, &adj_ppb));
```

內部做法是把新的 `adj_ppb` 轉成 addend，再和 `last_rate` 比較，最後用 fast 或 slow control code 寫入硬體。

### 調整時間偏移

```c
eth_mac_time_t offset = {
    .seconds = 0,
    .nanoseconds = 1000000,
};
ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_ADJ_PTP_TIME, &offset));
```

驅動會先讀目前時間，再以 signed 方式套用偏移值，最後回寫新時間。

### 讀取 TX timestamp

```c
eth_mac_time_t tx_time = {0};
ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TX_TIME, &tx_time));
printf("TX Timestamp: %lu.%09lu\n", tx_time.seconds, tx_time.nanoseconds);
```

### RX timestamp 的現況

雖然 `esp_eth_mac_spi.h` 還保留 `ETH_MAC_DM9058_CMD_G_PTP_RX_TIME` 列舉，但 DM9058 實際驅動中該命令會直接回傳 `ESP_ERR_NOT_SUPPORTED`：

```c
case ETH_MAC_DM9058_CMD_G_PTP_RX_TIME:
    return ESP_ERR_NOT_SUPPORTED;
```

也就是說，DM9058 的 RX 硬體時戳已改成「跟著接收到的 frame metadata 一起往上傳」，不是由應用層事後用 ioctl 拉取。

### 尚未支援的功能

以下命令在 enum 中存在，但目前 DM9058 驅動仍未實作：

- `ETH_MAC_DM9058_CMD_S_TARGET_TIME`
- `ETH_MAC_DM9058_CMD_S_TARGET_CB`

目前這兩個 case 會記錄 warning，並回傳 `ESP_ERR_NOT_SUPPORTED`。

## PTP 傳輸模式

### 支援的 transport 列舉

目前 `esp_eth_mac_spi.h` 定義如下：

```c
typedef enum {
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4 = 0,
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV6,
    ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3,
    ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_1AS,
} esp_eth_ptp_dm9058_transport_t;
```

其中 `esp_eth_ptp_dm9058_enable()` 的 offset 對應為：

- `UDP_IPV4`: `PTPTSO = 0x4E`, `PTPCSO = 0x3C`
- `UDP_IPV6`: `PTPTSO = 0x62`, `PTPCSO = 0x50`
- `IEEE_802_3` 與 `IEEE_802_1AS`: `PTPTSO = 0x32`, `PTPCSO = 0x20`

### one-step / two-step 模式

DM9058 MAC 自動處理 TX 封包時，會把 `emac->ptp_two_step_mode` 傳給 `esp_eth_ptp_dm9058_prepare_tx_locked()`。這個模式值不是透過專用 ioctl 動態切換，而是主要由 Kconfig 決定：

```text
CONFIG_ETH_DM9058_PTP_TWO_STEP_MODE=y
```

Kconfig 說明如下：

- 啟用時: 硬體擷取 TX timestamp，由軟體在後續 `FOLLOW_UP` 訊息中帶出，屬於 two-step
- 停用時: 硬體直接在 `SYNC` 封包送出時插入 timestamp，屬於 one-step

這個設定必須和 PTP daemon 的 two-step 設定一致。

## TX 封包處理流程

當 `ptp_auto_process == true` 且 PTP 已 enable 時，TX 路徑大致如下：

1. 等待 TX pointer 可用
2. 進入 MAC multi-register lock
3. 呼叫 `esp_eth_ptp_dm9058_prepare_tx_locked()` 分析封包
4. 根據分析結果調整 `TCR` 的 PTP 相關位元
5. 寫入封包到 TX memory
6. 觸發 `TCR_TXREQ`
7. 若有開啟 PTP auto process，等待 TX complete

### TX 封包分析規則

`esp_eth_ptp_dm9058_parse_tx_packet()` 目前只辨識：

- Layer 2 PTP: EtherType `0x88F7`
- PTP over IPv4 UDP: UDP port `319` 或 `320`

目前這個 helper 不處理 IPv6 封包解析，它只根據 enable 時的 transport 決定 offset，並不是用來做完整的 IPv6 PTP payload 檢查。

訊息類型對應行為如下：

- `SYNC`
  - one-step 模式: 開啟 `TS1STEP_EMIT`
  - two-step 模式: 開啟 `TSEN_CAP`
- `DELAY_REQ`
  - 目前同時開啟 `TSEN_CAP`
  - 也會開啟 `TS1STEP_EMIT`
- `PDELAY_REQ` / `PDELAY_RESP`
  - 開啟 `TSEN_CAP`
- 其他訊息類型
  - 不做時戳處理

### 透過 `esp_eth_transmit_ctrl_vargs()` 取得 TX 時戳

DM9058 MAC 已實作 `transmit_ctrl_vargs()`。若呼叫端有提供 `ctrl`，且 PTP auto process 開啟，驅動會在送出完成後把 TX timestamp 寫回 `eth_mac_time_t`。

L2TAP 的 TX timestamp 也是透過這條路徑取得。

## RX 封包與 RX timestamp 流程

### RX 路徑的實際作法

目前 RX 時戳流程不再是「先收包，再由應用層呼叫 ioctl 取最後一筆 RX 時戳」，而是：

1. RX task 讀出 DM9058 RX header
2. 由 `DM9058_handle_rx_ptp_timestamp()` 判斷該 frame 是否附帶 PTP timestamp
3. 若有 timestamp，先從 RX FIFO 把 4 或 8 byte timestamp 讀出
4. 用 `esp_eth_ptp_dm9058_rx_timestamp()` 解碼成 `esp_eth_ptp_dm9058_time_t`
5. 之後把 frame 與可選的 `rx_info` 一起交給 `emac->eth->stack_input_info()`

### RX header 解析

`esp_eth_ptp_dm9058_parse_rx_header()` 會解析 4-byte DM9058 RX header，主要產出：

- `packet_len`
- `rx_status`
- `timestamp_available`
- `timestamp_len`

`timestamp_len` 目前只可能是：

- `0`
- `4`
- `8`

### RX timestamp 解碼

目前可使用的 helper 是：

```c
esp_err_t esp_eth_ptp_dm9058_rx_timestamp(const uint8_t *rx_ts_buffer,
                                          size_t rx_ts_len,
                                          esp_eth_ptp_dm9058_time_t *time);
```

舊文件中提到的 `esp_eth_ptp_dm9058_parse_rx_packet()` 在目前檔案裡是被 `#if 0` 包住的停用程式碼，不應視為現行可用 API。

### 目前哪些 RX PTP 封包會把時戳往上層傳

在 `esp32_DM9058_task()` 裡，驅動還會再看封包型別。只有在以下條件成立時，才會把 `rx_ts` 指標傳給 `stack_input_info()`：

- `rx_ts_valid == true`
- 該封包成功被辨識為 PTP
- 訊息型別是 `SYNC` 或 `DELAY_REQ`

其他 PTP 封包即使 RX FIFO 內有時戳，目前也只會在 MAC 層記錄 debug log，不會往上層傳遞。

這點對 L2TAP、ptpd 或任何要做 packet-bound timestamp 的上層邏輯都很重要。

## 範例程式對應方式

`examples/ethernet/ptp/components/esp_eth_time/esp_eth_time.c` 已經使用正確的 enable 設定結構：

```c
esp_eth_ptp_dm9058_enable_config_t ptp_cfg = {
    .enable = true,
    .transport = cfg->transport,
};

esp_eth_ioctl(cfg->eth_hndl, ETH_MAC_ESP_CMD_PTP_ENABLE, &ptp_cfg);
```

需要注意的是：

- `esp_eth_clock_gettime()` / `settime()` / `adjtime()` 對 DM9058 是可用的
- `esp_eth_clock_get_rx_time()` 目前對 DM9058 不能再當成有效的 RX timestamp 來源，因為底層 ioctl 已不支援

PTP example 中保留的 `G_PTP_RX_TIME` 呼叫片段已被 `#if 0` 包住，這正反映出目前路徑已改為依賴 L2TAP / `stack_input_info()` 提供的封包綁定時間資訊。

## 硬體暫存器對應

### PTP 相關暫存器

| 暫存器 | 位址 | 功能 |
|--------|------|------|
| PTPCR  | 0x60 | PTP 控制暫存器，用於 reset |
| PTPCW  | 0x61 | PTP command / control write |
| PTPTSM | 0x62 | PTP timestamp mode |
| PTPTX  | 0x63 | one-step / two-step 模式控制 |
| PTPMMP | 0x64 | PTP multicast mode |
| PTPTSO | 0x65 | timestamp offset |
| PTPCSO | 0x66 | checksum offset |
| PTPTS  | 0x68 | PTP timestamp / time data |
| TCR    | 0x02 | TX control register |

### TCR 內目前使用的 PTP 位元

| 位元 | 功能 |
|------|------|
| Bit 7 | `TSEN_CAP`，啟用 TX timestamp capture |
| Bit 6 | `TS1STEP_EMIT`，啟用 one-step timestamp insertion |

## 注意事項

### 1. `PTP_ENABLE` 的參數型別必須正確

這是目前文件與實作差異最大的地方。若傳 `bool *`，會直接和驅動預期的結構不符。

### 2. RX timestamp 必須走 packet-bound 路徑

若上層仍然依賴「收到封包之後再單獨呼叫 ioctl 取 RX 時戳」，在 DM9058 上不再成立。要保留 DM9058 的每封包 RX timestamp 綁定，必須讓資料沿著 `stack_input_info()` 往上走。

### 3. 目前 RX timestamp 不是所有 PTP 訊息都會往上傳

現在 MAC 層只轉送 `SYNC` 和 `DELAY_REQ` 的 RX timestamp。若後續需要 `PDELAY_REQ`、`PDELAY_RESP` 或其他訊息類型的 packet-bound RX timestamp，必須再調整 MAC 層過濾條件。

### 4. `parse_tx_packet()` 的 transport 解析能力有限

雖然 enable 時支援 `UDP_IPV6` transport，但 `esp_eth_ptp_dm9058_parse_tx_packet()` 本身目前只實作 Layer 2 與 IPv4 UDP 的封包檢查邏輯。若 TX 自動處理需要完整支援 IPv6 PTP payload 分析，這裡還要再補。

### 5. Target Time 相關功能目前未完成

`ETH_MAC_DM9058_CMD_S_TARGET_TIME` 和 `ETH_MAC_DM9058_CMD_S_TARGET_CB` 的骨架與註解仍在，但實際路徑被停用，現階段不能當成可用功能。

## 除錯建議

### 檢查 PTP 是否真的 enable

```c
if (!ptp->initialized) {
    ESP_LOGE(TAG, "PTP not initialized");
}
if (!ptp->enabled) {
    ESP_LOGE(TAG, "PTP not enabled");
}
```

### 檢查 RX timestamp 是否被 MAC 層過濾

若上層一直收不到 RX timestamp，要先確認兩件事：

- RX FIFO 內是否真的有 timestamp
- 封包型別是否屬於目前 MAC 允許往上轉交的 `SYNC` 或 `DELAY_REQ`

### 檢查 two-step 設定是否和 PTP daemon 一致

`CONFIG_ETH_DM9058_PTP_TWO_STEP_MODE` 與使用中的 ptpd two-step 設定若不一致，PTP 同步行為會出錯。

## 相關檔案

- `components/esp_eth/src/spi/dm9058/esp_eth_mac_dm9058.c`
- `components/esp_eth/src/spi/dm9058/esp_eth_ptp_dm9058.c`
- `components/esp_eth/src/spi/dm9058/esp_eth_ptp_dm9058.h`
- `components/esp_eth/include/esp_eth_mac_spi.h`
- `components/esp_eth/Kconfig`
- `examples/ethernet/ptp/components/esp_eth_time/esp_eth_time.c`
- `examples/ethernet/ptp/components/ptpd/ptpd.c`

## 總結

目前專案中的 DM9058 PTP 整合有三個核心特性：

1. PTP 時鐘控制是透過 `esp_eth_ioctl()` 暴露，啟用時必須傳 `esp_eth_ptp_dm9058_enable_config_t`。
2. TX 自動時戳與 one-step / two-step 行為由 MAC 驅動配合 `CONFIG_ETH_DM9058_PTP_TWO_STEP_MODE` 完成。
3. RX timestamp 已改為沿著 `stack_input_info()` 傳遞封包綁定資訊，不再支援用 `G_PTP_RX_TIME` 事後輪詢。

如果後續要再擴充 DM9058 的 PTP 能力，最值得優先處理的通常是：

- 補齊 TX 自動解析對 IPv6 PTP 的支援
- 放寬 RX timestamp 上送的訊息型別過濾條件
- 完成 Target Time / callback 的正式實作