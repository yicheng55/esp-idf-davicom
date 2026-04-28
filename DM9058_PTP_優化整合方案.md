# DM9058 PTP 功能優化整合方案

這份文件依據目前專案實作，整理 DM9058 PTP 已完成的 MAC 層整合方式，以及後續仍值得優化的項目。舊版內容曾以「尚未整合」的角度描述要修改 `transmit()`、`receive()`、ioctl enum 與 RX timestamp 快取；目前這些核心路徑已經重構完成，因此本文件改以現況、限制與下一步優化為主。

## 目前結論

目前 DM9058 PTP 整合已具備以下能力：

1. PTP 硬體啟用透過 `esp_eth_ptp_dm9058_enable_config_t` 指定 enable 狀態與 transport。
2. MAC TX 路徑會在 `ptp_auto_process` 與 `ptp.enabled` 同時成立時，自動解析 PTP frame 並配置 `TCR`。
3. MAC RX task 會解析 DM9058 RX FIFO 內嵌 timestamp，並透過 `stack_input_info()` 把 timestamp metadata 隨 frame 傳給上層。
4. L2TAP / ptpd 透過 extended buffer info record 取得封包綁定的 RX/TX timestamp。
5. RX timestamp 不再透過「最後一筆 RX timestamp」ioctl 輪詢取得。
6. one-step / two-step 模式由 `CONFIG_ETH_DM9058_PTP_TWO_STEP_MODE` 決定，沒有現行的動態切換 ioctl。

## 現行架構

```text
Application / ptpd
    ├── esp_eth_clock_init()
    │   └── ETH_MAC_ESP_CMD_PTP_ENABLE
    ├── L2TAP read/write extended buffer
    │   └── L2TAP_IREC_TIME_STAMP
    └── esp_eth_clock_gettime/settime/adjtime()

esp_eth MAC mediator
    ├── stack_input_info(frame, len, optional_rx_timestamp)
    └── transmit_ctrl_vargs(optional_tx_timestamp)

DM9058 MAC driver
    ├── esp32_DM9058_custom_ioctl()
    ├── esp32_DM9058_transmit()
    ├── DM9058_task_receive()
    └── DM9058_handle_rx_ptp_timestamp()

DM9058 PTP helper
    ├── esp_eth_ptp_dm9058_enable()
    ├── esp_eth_ptp_dm9058_prepare_tx_locked()
    ├── esp_eth_ptp_dm9058_parse_tx_packet()
    ├── esp_eth_ptp_dm9058_parse_rx_header()
    └── esp_eth_ptp_dm9058_rx_timestamp()
```

## MAC 層資料結構

目前 `esp32_DM9058_t` 已經內含 PTP context 與自動處理設定：

```c
typedef struct {
    esp_eth_mac_t parent;
    esp_eth_mediator_t *eth;
    eth_spi_custom_driver_t spi;
    TaskHandle_t rx_task_hdl;
    SemaphoreHandle_t multi_reg_axs_mutex;
    /* ... */
    esp_eth_ptp_dm9058_t ptp;
    bool ptp_auto_process;
    bool ptp_two_step_mode;
} esp32_DM9058_t;
```

注意目前沒有用 `last_rx_timestamp` / `rx_timestamp_valid` 在 MAC 物件上快取最後一筆 RX timestamp。RX timestamp 以區域變數保留在 RX task 流程中，確認封包類型後直接透過 `stack_input_info()` 往上傳，這比全域快取更能維持 timestamp 與 frame 的對應關係。

## PTP 啟用流程

### 正確的 enable 參數

目前啟用 PTP 必須傳入設定結構：

```c
esp_eth_ptp_dm9058_enable_config_t ptp_cfg = {
    .enable = true,
    .transport = ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3,
};

ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_ENABLE, &ptp_cfg));
```

`transport` 會決定 DM9058 timestamp offset 與 checksum offset：

| Transport | PTPTSO | PTPCSO | 用途 |
| --- | --- | --- | --- |
| `ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4` | `0x4E` | `0x3C` | PTP over UDP/IPv4 |
| `ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV6` | `0x62` | `0x50` | PTP over UDP/IPv6 |
| `ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3` | `0x32` | `0x20` | Layer 2 IEEE 1588 |
| `ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_1AS` | `0x32` | `0x20` | gPTP / 802.1AS |

### PTP example 的 transport 選擇

`examples/ethernet/ptp/main/ptp_main.c` 目前會依 `CONFIG_NETUTILS_PTPD_IEEE_802_1AS` 選擇 `IEEE_802_1AS` 或 `IEEE_802_3`：

```c
esp_eth_clock_cfg_t clock_cfg = {
    .eth_hndl  = s_eth_handles[0],
#if CONFIG_NETUTILS_PTPD_IEEE_802_1AS
    .transport = ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_1AS,
#else
    .transport = ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3,
#endif
};
esp_eth_clock_init(CLOCK_PTP_SYSTEM, &clock_cfg);
```

## TX 自動處理

目前 `esp32_DM9058_transmit()` 的主要流程是：

1. 等待 TX pointer ready。
2. 進入 `multi_reg_axs_mutex` 保護區。
3. 若 `ptp_auto_process && ptp.enabled`，呼叫 `esp_eth_ptp_dm9058_prepare_tx_locked()`。
4. 寫入 TX 長度與 frame data。
5. 觸發 `TCR_TXREQ`。
6. PTP 啟用時等待 TX complete。

目前實作重點如下：

```c
if (emac->ptp_auto_process && emac->ptp.enabled) {
    esp_err_t ptp_ret = esp_eth_ptp_dm9058_prepare_tx_locked(&emac->ptp, buf, length, emac->ptp_two_step_mode);
    if (ptp_ret != ESP_OK) {
        ESP_LOGW(TAG, "prepare tx ptp failed: %s", esp_err_to_name(ptp_ret));
    }
}
```

`prepare_tx_locked()` 會使用 `esp_eth_ptp_dm9058_parse_tx_packet()` 判斷封包類型，然後設定 `TCR` 內的 `TSEN_CAP` 與 `TS1STEP_EMIT`。

### TX timestamp 回傳

有兩種常見方式：

- 一般 ioctl：使用 `ETH_MAC_DM9058_CMD_G_PTP_TX_TIME` 讀取 DM9058 最近一次 TX timestamp。
- L2TAP / extended path：由 `esp32_DM9058_transmit_ctrl_vargs()` 在送出後把 timestamp 寫入 `eth_mac_time_t` 控制資料，L2TAP 再以 `L2TAP_IREC_TIME_STAMP` 回傳給 ptpd。

## RX timestamp 自動處理

目前 RX timestamp 不使用 MAC 物件上的最後一筆快取，而是在 RX task 中保持 frame-bound 流程：

1. `DM9058_task_receive()` 呼叫 `DM9058_frame_to_rx_buffer()`。
2. `DM9058_handle_rx_ptp_timestamp()` 解析 RX header。
3. 若 RX FIFO 內含 timestamp，讀出 4 或 8 byte timestamp。
4. `esp_eth_ptp_dm9058_rx_timestamp()` 解碼成 `esp_eth_ptp_dm9058_time_t`。
5. `esp32_DM9058_task()` 再檢查 frame 是否為允許上送 timestamp 的 PTP event 訊息。
6. 呼叫 `stack_input_info()`，把 frame 與可選的 `eth_mac_time_t` 一起交給上層。

目前會上送 RX timestamp 的訊息型別：

| 訊息型別 | 是否上送 RX timestamp |
| --- | --- |
| `SYNC` | 是 |
| `DELAY_REQ` | 是 |
| `PDELAY_REQ` | 是 |
| `PDELAY_RESP` | 是 |
| `FOLLOW_UP` | 否 |
| `DELAY_RESP` | 否 |
| `ANNOUNCE` | 否 |
| 其他 general message | 否 |

這個策略符合 PTP event message 需要精準 timestamp 的需求，也避免把 general message 的 timestamp 誤交給上層。

## L2TAP / ptpd 對接

`examples/ethernet/ptp/components/ptpd/ptpd.c` 的 ESP PTP 路徑使用 `/dev/net/tap` 收發 EtherType `0x88F7` frame。收發時都配置 `L2TAP_IREC_TIME_STAMP`：

```c
l2tap_irec_hdr_t *ts_info = L2TAP_IREC_FIRST(&ptp_msg_ext_buff);
ts_info->len = L2TAP_IREC_LEN(sizeof(struct timespec));
ts_info->type = L2TAP_IREC_TIME_STAMP;
```

TX 時，`write()` 完成後從 info record 取回 TX timestamp。RX 時，`read()` 完成後從 info record 取回 frame-bound RX timestamp。這條路徑是目前 DM9058 PTP example 的主要 timestamp 來源。

## 已不採用的舊設計

以下舊提案不再適用於目前專案：

- PTP enable 只傳 `bool`。目前必須傳 `esp_eth_ptp_dm9058_enable_config_t`。
- 新增執行期 set-mode ioctl。one-step / two-step 目前由 `CONFIG_ETH_DM9058_PTP_TWO_STEP_MODE` 決定。
- 在 `esp32_DM9058_t` 上快取 `last_rx_timestamp`，再由應用層事後讀取。這會破壞 RX timestamp 與封包的精準綁定。
- 應用層收到封包後再輪詢 RX timestamp。DM9058 driver 目前明確不支援這條路徑。
- 使用 `esp_eth_ptp_dm9058_parse_rx_packet()` 作為現行 API。該函式目前在 `#if 0` 區塊中，不是可用的正式路徑。
- 以 driver-local debug macro 取代 ESP-IDF 既有 logging pattern。現行程式仍使用 `ESP_LOGx()`。

## 後續優化項目

### 1. 補齊 TX IPv6 PTP 封包解析

目前 enable 階段支援 `UDP_IPV6` transport offset，但 `esp_eth_ptp_dm9058_parse_tx_packet()` 的封包解析主要覆蓋 Layer 2 PTP 與 UDP/IPv4 PTP。若要完整支援 PTP over UDP/IPv6，應在 TX parser 中加入 IPv6 header、extension header 與 UDP port 319/320 的解析。

建議範圍：

- 在 `esp_eth_ptp_dm9058_parse_tx_packet()` 新增 IPv6 path。
- 保留現有 L2 與 IPv4 path 行為。
- 增加針對 `SYNC`、`DELAY_REQ`、`PDELAY_REQ`、`PDELAY_RESP` 的單元測試或最小封包測試資料。

### 2. 評估 Target Time / callback 正式化

`ETH_MAC_DM9058_CMD_S_TARGET_TIME` 與 `ETH_MAC_DM9058_CMD_S_TARGET_CB` 目前 enum 與部分註解骨架仍存在，但實際 case 回傳 `ESP_ERR_NOT_SUPPORTED`。若 example 仍需要 GPIO pulse target time，應決定要走正式 driver 功能，或改在 example 層清楚標示該功能尚未可用。

建議範圍：

- 釐清 DM9058 是否能以硬體中斷或 timer 可靠觸發 target time。
- 若使用軟體 timer，需要明確標示精度限制。
- 補齊 callback lifetime、ISR 安全性與 deinit 清理。

### 3. 明確化 RX event message policy

目前 MAC 層已上送四種 event message 的 RX timestamp。若後續 profile 或 daemon 行為需要更精細控制，可考慮把 filter policy 變成明確 helper，例如：

```c
static bool dm9058_should_forward_rx_timestamp(uint8_t message_type)
{
    switch (message_type) {
    case ESP_ETH_PTP_DM9058_MSG_SYNC:
    case ESP_ETH_PTP_DM9058_MSG_DELAY_REQ:
    case ESP_ETH_PTP_DM9058_MSG_PDELAY_REQ:
    case ESP_ETH_PTP_DM9058_MSG_PDELAY_RESP:
        return true;
    default:
        return false;
    }
}
```

這類重構不改變行為，但能降低後續修改時漏改條件的風險。

### 4. 清理 RX helper API 邊界

目前 `esp_eth_ptp_dm9058_parse_rx_header()` 和 `esp_eth_ptp_dm9058_rx_timestamp()` 是現行可用 helper；`parse_rx_packet()` 停用。後續可二選一：

- 移除停用函式，避免誤導使用者。
- 或正式恢復並調整成符合目前 FIFO 讀取順序的 API。

若恢復，必須避免 helper 內自行做 RX ready dummy read，否則可能和 MAC RX task 的 FIFO 操作順序互相干擾。

### 5. 增加文件化測試步驟

建議在 PTP example README 或本文件補充最小驗證流程：

- `CONFIG_ETH_DM9058_PTP_TWO_STEP_MODE` 與 `CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC` 是否一致。
- `CONFIG_NETUTILS_PTPD_IEEE_802_1AS` 是否符合 `ptp_main.c` transport。
- L2TAP 是否成功開啟 timestamp。
- ptpd log 是否能看到 RX timestamp。
- master / slave 是否能穩定進入 selected clock source 狀態。

## 建議優先順序

1. 文件與範例清理：移除舊 API 與 RX polling 誤導。
2. RX event message policy helper：低風險、可讀性提升。
3. TX IPv6 parser：中等風險，需要封包測試。
4. Target Time / callback：高風險，需要硬體行為與精度驗證。
5. RX helper API 整理：中等風險，需要確認 FIFO 操作順序。

## 總結

目前 DM9058 PTP 的主要整合目標已經完成：TX 由 MAC 自動配置硬體 timestamp 行為，RX 由 MAC task 以 packet-bound metadata 往上傳遞，PTP example 透過 L2TAP extended buffer 使用 timestamp。後續優化應集中在補齊未完成能力、清理停用 API、增加測試與文件一致性，而不是回到舊版的「應用層輪詢最後一筆 RX timestamp」設計。
