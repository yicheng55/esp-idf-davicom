# L2TAP + DM9058 PTP Daemon 運作原理與參數使用手冊

> **適用範例**：`esp-idf/examples/ethernet/ptp`
> **目標硬體**：ESP32-S3 / ESP32-P4 / ESP32-C6 + Davicom **DM9058** SPI Ethernet PHY
> **PTP 規格**：IEEE 1588-2008 (PTPv2)，**Annex F** 的 L2（raw Ethernet）傳輸
> **當前分支設定**：`transport = ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3`、雙模 CLIENT + SERVER、Two-step、Delay Request 開啟

本手冊聚焦「當前專案在 **L2TAP 模式**下如何收發 PTP 封包、如何取得硬體時間戳、各 Kconfig 參數實際影響哪段程式」。不重複 `README.md` 已涵蓋的編譯與燒錄流程，也不取代 `components/ptpd/ref/doc/gptp_porting_guide.md` 的完整 gPTP 移植指南。

---

## 目錄

1. [概觀](#1-概觀)
2. [為何選用 L2TAP + `ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3`](#2-為何選用-l2tap--esp_eth_ptp_dm9058_transport_ieee_802_3)
3. [L2TAP 管道運作原理](#3-l2tap-管道運作原理)
4. [時鐘初始化順序（非常重要）](#4-時鐘初始化順序非常重要)
5. [Kconfig 與程式碼參數對照表](#5-kconfig-與程式碼參數對照表)
6. [Two-step vs One-step](#6-two-step-vs-one-step)
7. [同步迴圈與 PI 控制器](#7-同步迴圈與-pi-控制器)
8. [Delay Mechanism：E2E vs P2P 詳細說明](#8-delay-mechanisme2e-vs-p2p-詳細說明)
9. [常見調校與疑難排解](#9-常見調校與疑難排解)
10. [附錄 A：DM9058 PTP 硬體暫存器](#10-附錄-adm9058-ptp-硬體暫存器)
11. [附錄 B：完整封包流程時序](#11-附錄-b完整封包流程時序)
12. [附錄 C：參考檔案索引](#12-附錄-c參考檔案索引)

---

## 1. 概觀

### 1.1 PTPv2 與本範例的關係

IEEE 1588-2008（PTPv2）定義「在封包網路上達到次微秒級時間同步」的協定。協定本身與傳輸層解耦，規範書列出三種常見 mapping：

| Mapping | EtherType / Port | Annex |
|---|---|---|
| UDP over IPv4 | UDP port 319 (event) / 320 (general) | Annex D |
| UDP over IPv6 | UDP port 319 / 320 | Annex E |
| **Raw Ethernet (L2)** | **EtherType `0x88F7`** | **Annex F** |

本範例採用 **Annex F 的 L2 mapping**，原因會在第 2 節詳述。

### 1.2 系統架構

```
┌───────────────────────────────────────────────────────────────┐
│                   ESP32 應用層（ptp_main.c）                   │
│  - init_ethernet_and_netif()                                  │
│  - esp_eth_clock_init(..., ESP_ETH_PTP_DM9058_TRANSPORT_...) │
│  - ptpd_start("ETH_0")                                        │
│  - PTP 同步狀態監看與驗證                                      │
└───────────────┬────────────────────────────┬──────────────────┘
                │                            │
┌───────────────▼────────────┐  ┌────────────▼──────────────────┐
│  ptp daemon（ptpd.c）       │  │  esp_eth_time 元件             │
│  - ptp_daemon() task        │  │  - esp_eth_clock_gettime       │
│  - 狀態機 / BMCA            │  │  - esp_eth_clock_settime       │
│  - PI 頻率調整              │  │  - esp_eth_clock_adjtime       │
│  - 收發 PTP 封包            │  │  - esp_eth_clock_set_target_time│
└───────────────┬────────────┘  └────────────┬──────────────────┘
                │                            │
┌───────────────▼────────────────────────────▼──────────────────┐
│      L2TAP VFS（esp_vfs_l2tap）                                │
│  /dev/net/tap + ioctl + read/write + L2TAP_IREC_TIME_STAMP    │
└───────────────┬───────────────────────────────────────────────┘
                │
┌───────────────▼───────────────────────────────────────────────┐
│      esp_eth 驅動（DM9058 SPI MAC driver）                     │
│  - SPI bus → DM9058 PTP 硬體時間戳引擎                          │
└───────────────┬───────────────────────────────────────────────┘
                │ SPI
┌───────────────▼───────────────────────────────────────────────┐
│      DM9058 晶片：MAC + PHY + PTP HW timestamp engine         │
└───────────────────────────────────────────────────────────────┘
```

### 1.3 Master / Slave 角色與 BMCA

- 本範例同時啟用 `CONFIG_NETUTILS_PTPD_CLIENT=y` 與 `CONFIG_NETUTILS_PTPD_SERVER=y`
- 兩端執行 **Best Master Clock Algorithm (BMCA)**：比較 `priority1 → clockClass → accuracy → variance → priority2 → identity`，決定哪一方成為 master
- 勝出的一方開始週期性發送 `Announce` / `Sync` / `Follow_Up`；落敗的一方切換為 slave，進行時鐘同步

---

## 2. 為何選用 L2TAP + `ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3`

### 2.1 UDP 模式與 L2 模式的差異

| 面向 | UDP over IPv4 | **L2 raw Ethernet（本範例）** |
|---|---|---|
| 封裝層數 | MAC + IP + UDP + PTP | **MAC + PTP** |
| 固定 overhead | 14 + 20 + 8 = 42 bytes | **14 bytes** |
| TCP/IP stack 參與 | ✓（lwIP 處理 IP/UDP） | ✗（直接 L2 VFS） |
| 時間戳延遲 | 封包穿過 lwIP 再到 MAC | **ioctl 直接存取 MAC** |
| Broadcast 範圍 | 跨 router（可路由） | 僅在同 L2 domain |
| DM9058 分類器 | 依 UDP port 319/320 | **依 EtherType 0x88F7** |
| DM9058 TS 偏移 | 0x4E (78) | **0x32 (50)** |
| 典型精度 | µs 等級 | **~100 ns** 量級 |

### 2.2 `esp_eth_ptp_dm9058_transport_t` 枚舉

於 `components/esp_eth_time/esp_eth_time.h:40`：

```c
typedef struct {
    esp_eth_handle_t eth_hndl;
    esp_eth_ptp_dm9058_transport_t transport;
    /*!< PTP transport type (default: ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4 = 0) */
} esp_eth_clock_cfg_t;
```

兩個值代表：

| 值 | 意義 | 對 DM9058 的實質影響 |
|---|---|---|
| `ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4` (0) | UDP/IPv4 模式（預設） | 寫入 `TS offset = 0x4E`、`CRC offset = 0x3C` |
| `ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3` | L2 raw Ethernet（本範例） | 寫入 `TS offset = 0x32`、`CRC offset = 0x20` |

> DM9058 必須知道 PTP 欄位在乙太封包中的位置，硬體才能在封包流過 MAC 時自動擷取 / 回填時間戳。**選錯 transport 會導致硬體到錯誤位置讀寫，時間戳無效。**

### 2.3 實際設定位置

於 `main/ptp_main.c:159`：

```c
esp_eth_clock_cfg_t clock_cfg = {
    .eth_hndl  = s_eth_handles[0],
    .transport = ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3,
};
esp_eth_clock_init(CLOCK_PTP_SYSTEM, &clock_cfg);
```

→ 呼叫此函式時，DM9058 驅動會完成：
1. 啟動 PTP 引擎 (`Reg 0x61 = 0x01`)
2. 設定 TWO_STEP / ONE_STEP 模式（`Reg 0x63`）
3. **按 transport 寫入 `0x65` / `0x66` 偏移暫存器**
4. 註冊 RX/TX 時間戳回呼供 L2TAP 取用

---

## 3. L2TAP 管道運作原理

### 3.1 什麼是 L2TAP

L2TAP（Layer-2 TAP）是 ESP-IDF 提供的 VFS 介面，讓使用者空間透過 `open("/dev/net/tap", 0)` 取得一個 raw Ethernet socket，可直接收發 L2 幀且略過 lwIP 的 IP/UDP 處理。對 PTP 來說最重要的是：L2TAP 能把驅動擷取到的 **硬體時間戳** 透過 *Info Records* 機制與封包一起回傳給 app。

註冊時機：`main/ptp_main.c:85`

```c
ret = esp_vfs_l2tap_intf_register(NULL);
```

### 3.2 PTP daemon 的 L2TAP 初始化流程

位置：`components/ptpd/ptpd.c:670-715` 的 `ptp_initialize_state()`。逐項說明：

```c
/* (a) 建立 L2TAP VFS socket */
state->ptp_socket = open("/dev/net/tap", 0);

/* (b) 綁定網卡（依 netif key "ETH_0"） */
ioctl(state->ptp_socket, L2TAP_S_INTF_DEVICE, interface);

/* (c) 設定 RX EtherType 過濾 */
uint16_t eth_type_filter = ETH_TYPE_PTP;          // 0x88F7
ioctl(state->ptp_socket, L2TAP_S_RCV_FILTER, &eth_type_filter);

/* (d) 取得底層 esp_eth_handle_t（供 clock 或後續 MAC filter 使用） */
ioctl(state->ptp_socket, L2TAP_G_DEVICE_DRV_HNDL, &state->eth_handle);

/* (e) 啟用硬體時間戳透傳 */
ioctl(state->ptp_socket, L2TAP_S_TIMESTAMP_EN);
```

| ioctl | 作用 |
|---|---|
| `L2TAP_S_INTF_DEVICE` | 指定這個 socket 掛到哪張 netif；接收 "ETH_0" 之類的字串 key |
| `L2TAP_S_RCV_FILTER` | 設定 EtherType 過濾；非 `0x88F7` 的封包直接被 L2TAP 丟棄，不會進入 PTP daemon |
| `L2TAP_G_DEVICE_DRV_HNDL` | 反向取得 `esp_eth_handle_t`，後續用來加 multicast MAC filter、控制 DM9058 PTP 硬體 |
| `L2TAP_S_TIMESTAMP_EN` | 開啟 Info Records 通道，每個 TX/RX 封包都會附帶 `L2TAP_IREC_TIME_STAMP` 記錄 |

### 3.3 Info Records：把時間戳夾帶進 read/write

Info Records（IREC）是 L2TAP 為了攜帶 metadata 而設計的機制。相關巨集定義於 `esp_vfs_l2tap.h`：

| 巨集 / 型別 | 用途 |
|---|---|
| `l2tap_extended_buff_t` | 同時包含 payload buffer 與 IREC buffer 的延伸結構 |
| `L2TAP_IREC_SPACE(size)` | IREC 所需總空間（含對齊） |
| `L2TAP_IREC_LEN(size)` | 單筆 IREC 的長度 |
| `L2TAP_IREC_FIRST(buf)` | 取得 IREC buffer 的第一筆指標 |
| `L2TAP_IREC_TIME_STAMP` | IREC type：表示這筆記錄是 `struct timespec` 型態的時間戳 |
| `l2tap_irec_hdr_t` | 單筆 IREC header（type + len + data） |

**TX 範例**（`ptpd.c:324-356` 的 `ptp_net_send()`）：

```c
union {
    uint8_t info_recs_buff[L2TAP_IREC_SPACE(sizeof(struct timespec))];
    l2tap_irec_hdr_t align;  // 對齊用（雙字寬存取）
} u;

l2tap_extended_buff_t ptp_msg_ext_buff = {
    .info_recs_len  = sizeof(u.info_recs_buff),
    .info_recs_buff = u.info_recs_buff,
    .buff           = eth_frame,
    .buff_len       = sizeof(eth_frame),
};

l2tap_irec_hdr_t *ts_info = L2TAP_IREC_FIRST(&ptp_msg_ext_buff);
ts_info->len  = L2TAP_IREC_LEN(sizeof(struct timespec));
ts_info->type = L2TAP_IREC_TIME_STAMP;   // 請求 L2TAP 回填 TX 時間戳

int ret = write(state->ptp_socket, &ptp_msg_ext_buff, 0);

if (ret > 0 && ts && ts_info->type == L2TAP_IREC_TIME_STAMP) {
    *ts = *(struct timespec *)ts_info->data;  // 取回 TX 完成時的硬體時間戳
}
```

**RX 路徑**（`ptpd.c:370-404` 的 `ptp_net_recv()`）同結構，差別是 `read()` 後 `ts_info->data` 帶回的是 RX 時間戳。

### 3.4 EtherType 與 Multicast MAC

**EtherType**：

```c
// 於 L2TAP RX filter 與 ptp_create_eth_frame() 組封包時同時使用
#define ETH_TYPE_PTP  0x88F7
```

**PTP multicast MAC 位址**（`ptpd.c:720-725`）：

```c
uint8_t dest_addr[ETH_ADDR_LEN];
SET_MAC_ADDR(dest_addr, 0x01, 0x1B, 0x19, 0x00, 0x00, 0x00);  // PTPv2 非 peer-delay
esp_eth_ioctl(state->eth_handle, ETH_CMD_ADD_MAC_FILTER, dest_addr);
SET_MAC_ADDR(dest_addr, 0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E);  // IEEE 802.1AS peer-delay
esp_eth_ioctl(state->eth_handle, ETH_CMD_ADD_MAC_FILTER, dest_addr);
```

當前 daemon 發送時固定用 `01:1B:19:00:00:00`（`ptpd.c:315`），peer-delay 相關封包尚未實作。

---

## 4. 時鐘初始化順序（非常重要）

### 4.1 為何順序必須正確

DM9058 的硬體時間戳電路是透過 `esp_eth_clock_init()` 開啟的。若 L2TAP 先呼叫 `L2TAP_S_TIMESTAMP_EN`，然後才呼叫 `esp_eth_clock_init()`，L2TAP 會開啟 Info Records 通道但驅動尚未啟動 PTP 引擎，導致取到的 `timespec` 是 `{0, 0}` 或舊資料。

### 4.2 本範例的實際順序

```
app_main()                       ─ ptp_main.c:149
├── init_ethernet_and_netif()
│   └── esp_vfs_l2tap_intf_register(NULL)    ─ ptp_main.c:85
├── esp_eth_clock_init(CLOCK_PTP_SYSTEM, &cfg)  ← 先啟動 PTP HW  ─ ptp_main.c:163
├── esp_eth_clock_register_target_cb(...)
└── ptpd_start("ETH_0")
    └── ptp_daemon()
        └── ptp_initialize_state()           ─ ptpd.c:670
            ├── open("/dev/net/tap")
            ├── ioctl L2TAP_S_INTF_DEVICE
            ├── ioctl L2TAP_S_RCV_FILTER
            ├── ioctl L2TAP_G_DEVICE_DRV_HNDL
            └── ioctl L2TAP_S_TIMESTAMP_EN    ← 再啟用 L2TAP 時間戳通道
```

### 4.3 歷史演進

`ptpd.c:701-708` 有一段被註解掉的舊程式，原本是在 daemon 內部呼叫 `esp_eth_clock_init()`。近期調整為在 `app_main()` 層呼叫，理由是讓 app 能在 daemon 啟動前就能 `esp_eth_clock_gettime()` 查詢時間、設定 target callback。當前建議流程即：**app 層先 init clock、再 start daemon**。

```c
// ptpd.c:701-708（已註解掉的舊寫法，僅供對照）
// esp_eth_clock_cfg_t clk_cfg = {
//   .eth_hndl  = state->eth_handle,
//   .transport = ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3,
// };
// esp_eth_clock_init(CLOCK_PTP_SYSTEM, &clk_cfg);
```

---

## 5. Kconfig 與程式碼參數對照表

所有 PTP daemon 相關 Kconfig 集中於 `components/ptpd/Kconfig.projbuild`。以下只列出本範例會用到者，並標出當前預設值（對照 `sdkconfig.defaults`）。

### 5.1 通用選項

| Kconfig | 預設 | 程式碼位置 | 意義 |
|---|---|---|---|
| `CONFIG_NETUTILS_PTPD` | y | — | 啟用整個 daemon |
| `CONFIG_NETUTILS_PTPD_CLIENT` | y | `ptpd.c` 到處都有 `#ifdef CLIENT` | 支援 slave 模式 |
| `CONFIG_NETUTILS_PTPD_SERVER` | y | 同上 | 支援 master 模式（兩者皆開 → BMCA 決定角色） |
| `CONFIG_NETUTILS_PTPD_DOMAIN` | 0 | `ptpd.c:735` | PTP domain 編號，跨 domain 不互通 |
| `CONFIG_NETUTILS_PTPD_STACKSIZE` | 4096 | `ptpd.c:2180` | daemon FreeRTOS task 堆疊 |
| `CONFIG_NETUTILS_PTPD_SERVERPRIO` | 100 | — | daemon task 優先權 |
| `CONFIG_NETUTILS_PTPD_DEBUG` | n | — | 額外除錯訊息（需 DEBUG_INFO） |

### 5.2 Transport（非 Kconfig，位於程式碼）

| 設定 | 預設 | 位置 |
|---|---|---|
| `esp_eth_clock_cfg_t.transport` | `ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3` | `ptp_main.c:161` |
| `ETH_TYPE_PTP` | `0x88F7` | `esp_eth.h` 或 `lwIP` |
| 發送用 dst MAC | `01:1B:19:00:00:00` | `ptpd.c:315` |

### 5.3 Master（SERVER）選項

| Kconfig | 預設 | 意義 |
|---|---|---|
| `CONFIG_NETUTILS_PTPD_PRIORITY1` | 128 | BMCA 主優先權（小=高優先） |
| `CONFIG_NETUTILS_PTPD_PRIORITY2` | 128 | BMCA 次優先權（tie-break） |
| `CONFIG_NETUTILS_PTPD_CLASS` | 248 | clockClass（248 = unknown / slave-only 以外） |
| `CONFIG_NETUTILS_PTPD_ACCURACY` | 254 | clockAccuracy（254 = unknown；32=±25ns…） |
| `CONFIG_NETUTILS_PTPD_CLOCKSOURCE` | 160 | timeSource（32=GPS, 64=PTP, 80=NTP, 144=other, 160=internal osc） |
| `CONFIG_NETUTILS_PTPD_SYNC_INTERVAL_MSEC` | 1000 | Sync 封包送出週期 |
| `CONFIG_NETUTILS_PTPD_ANNOUNCE_INTERVAL_MSEC` | 10000 | Announce 封包送出週期 |
| `CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC` | 跟 `ETH_DM9058_PTP_TWO_STEP_MODE` 一致 | Sync + Follow_Up（見第 6 節） |
| `CONFIG_NETUTILS_PTPD_DELAYRESP_INTERVAL` | 4 | DelayResp header 裡告知 slave 的建議 delay req 週期 = 2^N 秒 |

### 5.4 Slave（CLIENT）選項

| Kconfig | 預設 | 意義 |
|---|---|---|
| `CONFIG_NETUTILS_PTPD_TIMEOUT_MS` | 60000 | 超過此時間未收到選定 master 的封包就換源 |
| `CONFIG_NETUTILS_PTPD_SETTIME_THRESHOLD_MS` | 1000 | 誤差大於 1 s 時直接 step（`settime`）；小於則 slew（頻率微調） |
| `CONFIG_NETUTILS_PTPD_SEND_DELAYREQ` | **y**（本例） | 啟用 DelayReq/DelayResp 量測 path delay |
| `CONFIG_NETUTILS_PTPD_MAX_PATH_DELAY_NS` | 100000 | 量到超過此值的 path delay 視為異常並丟棄 |
| `CONFIG_NETUTILS_PTPD_DELAYREQ_AVGCOUNT` | 100 | path delay 移動平均樣本數 |
| `CONFIG_NETUTILS_PTPD_PATH_DELAY_STABILITY_NS` | 500 | 本地時鐘穩定度門檻（ns），穩定後才開始發送 DelayReq |

### 5.5 範例專屬 Kconfig（`main/Kconfig.projbuild`）

| Kconfig | 預設 | 意義 |
|---|---|---|
| `CONFIG_EXAMPLE_PTP_PULSE_GPIO` | 20 | 範例保留的脈衝輸出 GPIO 設定（目前手冊不展開） |
| `CONFIG_EXAMPLE_PTP_PULSE_WIDTH_NS` | 500_000_000（500 ms） | 範例保留的脈衝寬度設定（目前手冊不展開） |

### 5.6 硬體依賴 Kconfig

| Kconfig | 預設（本例） | 意義 |
|---|---|---|
| `CONFIG_ETH_SPI_ETHERNET_DM9058` | y | 啟用 DM9058 SPI MAC driver |
| `CONFIG_EXAMPLE_USE_DM9058` | y | 範例層選用 DM9058 |
| `CONFIG_ESP_NETIF_L2_TAP` | y | **啟用 L2TAP VFS（必備）** |
| `ETH_DM9058_PTP_TWO_STEP_MODE` | y | DM9058 硬體開啟 Two-step（與 `CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC` 必須對齊） |

---

## 6. Two-step vs One-step

### 6.1 兩種模式差異

**Two-step**（當前設定）：
```
Master → Slave :  Sync        (硬體在送出時 latch TX_ts，但封包已離開，內含時間是預估值)
Master → Slave :  Follow_Up   (帶實際 TX_ts，由軟體讀取後填入)
```

**One-step**：
```
Master → Slave :  Sync  (硬體在封包流過 MAC 時即時改寫 correctionField，填入真實 TX_ts)
```

One-step 封包數較少但對硬體要求高（必須能在 MAC 內線上改寫 correctionField，並重算 CRC）。DM9058 兩者都支援，透過 `Reg 0x63` 切換。

### 6.2 程式碼中的 Two-step 旗標

在 `ptp_send_sync()` 中（`ptpd.c:1087-1089`）：

```c
#ifdef CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC
  msg.header.flags[0] = PTP_FLAGS0_TWOSTEP;    // 告訴對端「後續會有 Follow_Up」
#endif
```

發完 Sync 之後（`ptpd.c:1119-1143`）：

```c
#ifdef CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC
  timespec_to_ptp_format(&ts, msg.origintimestamp);   // ts 來自 Sync 的 TX_ts
  msg.header.messagetype = PTP_MSGTYPE_FOLLOW_UP;
  msg.header.flags[0] = 0;
  ret = ptp_net_send(state, &msg, sizeof(msg), NULL);  // 送 Follow_Up
#endif
```

重點是 `ts` 是透過 `ptp_net_send()` 的第 3 個參數回填的 **硬體 TX 時間戳**（見第 3.3 節 Info Records 機制），準確度比軟體時間戳高一到兩個數量級。

### 6.3 Kconfig 預設邏輯

```kconfig
config NETUTILS_PTPD_TWOSTEP_SYNC
    bool "PTP server sends two-step synchronization packets"
    default ETH_DM9058_PTP_TWO_STEP_MODE if ETH_SPI_ETHERNET_DM9058
    default y
```

→ 只要選了 DM9058 driver，`CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC` 就自動跟著 `ETH_DM9058_PTP_TWO_STEP_MODE`，避免協定與硬體不同調。

> **⚠️ 兩端必須一致**：Master 若設 Two-step、Slave 期待 One-step，Slave 會一直等 Follow_Up；反之則收到空的 correctionField。調整時請兩端的 `CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC` 與 `ETH_DM9058_PTP_TWO_STEP_MODE` 都改。

---

## 7. 同步迴圈與 PI 控制器

### 7.1 主要事件迴圈

`ptp_daemon()` → `poll()` L2TAP socket（`ptpd.c:2054-2076`）：

```c
while (!state->stop) {
    state->can_send_delayreq = false;
    ret = poll(pollfds, 1, PTPD_POLL_INTERVAL);

    if (pollfds[0].revents) {
        ret = ptp_net_recv(state, &state->rxbuf, sizeof(state->rxbuf), &state->rxtime);
        if (ret > 0)
            ptp_process_rx_packet(state, ret);   // 依 msgtype 分派
    }
    // 週期性動作：送 Announce / Sync / DelayReq、超時檢查...
}
```

### 7.2 Slave 收到 Sync/Follow_Up 後的動作

1. `t1` = Follow_Up 帶回的 master TX 時間
2. `t2` = 本地 RX 時間（L2TAP IREC 回傳）
3. 計算 `offset = (t2 − t1) − path_delay`（若啟用 DelayReq 才有 path_delay）
4. 若 `|offset| > SETTIME_THRESHOLD_MS` → `esp_eth_clock_settime()` 直接跳轉
5. 否則 → 把 `offset` 餵給 PI 控制器 → `esp_eth_clock_adjtime(ETH_CLK_ADJ_FREQ_SCALE)`

### 7.3 PI 控制器

初始化於 `ptpd.c:730-732`：

```c
state->offset_pi.kp = 5;
state->offset_pi.ki = 50;
state->offset_pi.drift_acc = 0;
```

同檔註解清楚說明：

- `kp = 1`（dead-beat）會在 2-cycle limit cycle 之間震盪 → 故選 kp = 5 取約 20% 步階校正 → 指數收斂
- `ki = 50` 作為慢速積分器，允許 `drift_acc` 慢慢收斂到真實頻率誤差（~490 ppm，對應晶體老化 / 溫漂），同時避免 windup

簡化的控制律：

```
adj_ns      = kp * offset_ns + drift_acc
drift_acc  += offset_ns / ki              // 積分器緩慢累積
freq_scale  = 1 + adj_ns / SYNC_INTERVAL
esp_eth_clock_adjtime(ETH_CLK_ADJ_FREQ_SCALE, &{mode=FREQ_SCALE, freq_scale})
```

`freq_scale` 的意義：`> 1` 加快、`< 1` 減慢。內部由 `esp_eth_clock_adjtime()` 轉換為 DM9058 的 addend 寄存器調整量（次納秒 accumulator）。

### 7.4 Path delay 公式（`SEND_DELAYREQ=y` 時）

四個時間戳：

```
t1 : master 送出 Sync 的本地時間 (Follow_Up 帶回)
t2 : slave 收到 Sync 的本地時間
t3 : slave 送出 Delay_Req 的本地時間
t4 : master 收到 Delay_Req 的本地時間 (Delay_Resp 帶回)
```

公式：

```
offset     = ((t2 − t1) − (t4 − t3)) / 2
path_delay = ((t2 − t1) + (t4 − t3)) / 2
```

daemon 會對 `path_delay` 做 N 點移動平均（`DELAYREQ_AVGCOUNT`，本範例 = 100），並丟棄超過 `MAX_PATH_DELAY_NS`（本範例 = 100 µs）的異常值。

### 7.5 狀態查詢 API

`ptpd_status(pid, &s)` 回填 `struct ptpd_status_s`（`ptpd.h:52-108`）：

| 欄位 | 意義 |
|---|---|
| `clock_source_valid` | 是否找到可同步的 master |
| `clock_source_info.*` | 被選中的 master 資訊（identity、priority、class…） |
| `last_delta_ns` | 最近一次 offset（ns） |
| `drift_ppb` | 平均漂移速率（正 = master 比 local 快） |
| `path_delay_ns` | 平均 path delay |
| `last_received_sync` / `last_received_announce` | 最近一次收到的時戳（`CLOCK_MONOTONIC`） |

app 端（`ptp_main.c:195-231`）用 `clock_source_valid_cnt > 2` 來判定同步狀態是否已穩定。

---

## 8. Delay Mechanism：E2E vs P2P 詳細說明

Path delay 的量測方式是 PTP 協定的核心之一。IEEE 1588-2008 定義兩種 **delayMechanism**：**End-to-End (E2E)** 與 **Peer-to-Peer (P2P / Pdelay)**。兩者使用不同的封包類型與 multicast MAC 位址，對網路拓樸與 switch 能力的假設也不同。本節詳細說明兩者差異，以及**本專案目前的實作現況**。

### 8.1 概念比較

| 面向 | E2E (End-to-End) | P2P (Peer-to-Peer) |
|---|---|---|
| 又稱 | Delay Request-Response Mechanism | Peer Delay Mechanism |
| 使用封包 | `Delay_Req` / `Delay_Resp` | `Pdelay_Req` / `Pdelay_Resp` / `Pdelay_Resp_Follow_Up` |
| `messageType` 值 | 1 / 9 | 2 / 3 / 10 |
| Multicast MAC | `01:1B:19:00:00:00` | **`01:80:C2:00:00:0E`**（IEEE 802.1 保留） |
| 量測對象 | Slave ↔ Grandmaster（**端到端**整條路徑） | 相鄰兩 port（**逐段 hop-by-hop**） |
| 跨 switch | ✓ 可以；但 switch transit time 會污染量測 | ✗ link-local，標準 bridge 不轉發 |
| Switch 需求 | 最好搭配 Transparent Clock (TC) | 必須所有 switch 都是 P2P-capable（P2P TC / BC） |
| 典型應用 | 傳統 PTPv2、電信級同步 | IEEE 802.1AS (gPTP)、TSN、工業乙太 |
| IEEE 1588 profile | Default E2E profile | P2P profile / Annex J |
| IEEE 802.1AS (gPTP) | ✗ 不允許 | **✓ 強制使用** |

### 8.2 E2E 時序（本範例當前採用）

```
           SLAVE                                    MASTER
             |                                        |
             |    Sync         (seq N)       <------  |  t1 = TX ts
         t2 = RX ts                                   |
             |    Follow_Up    (seq N, t1)  <------   |
             |                                        |
         t3 = TX ts                                   |
             |    Delay_Req    (seq M)       ------>  |  t4 = RX ts
             |                                        |
             |    Delay_Resp   (seq M, t4)  <------   |
             |                                        |

     path_delay = ((t2 − t1) + (t4 − t3)) / 2
     offset     = ((t2 − t1) − (t4 − t3)) / 2
```

**特性**：

- Slave 獨自發起 `Delay_Req`，Master 以 `Delay_Resp` 回覆 master 端的 RX 時間 `t4`
- 量測結果是 slave 到 master **整條路徑**的單向延遲（假設對稱）
- 跨 switch 時，若 switch 不是 Transparent Clock，switch 內的 queuing / transit time 會直接汙染 delay 量測並造成不對稱誤差

### 8.3 P2P (Pdelay) 時序

```
           PORT A                                   PORT B
             |                                        |
         t1 = TX ts                                   |
             |   Pdelay_Req         (seq N)   ----->  |   t2 = RX ts
             |                                        |
             |                                        |   t3 = TX ts
             |   Pdelay_Resp        (seq N, t2) <---  |
             |   Pdelay_Resp_FU     (seq N, t3) <---  |
         t4 = RX ts                                   |
             |                                        |

     peer_delay = ((t4 − t1) − (t3 − t2)) / 2   （需逐段延遲量測與同步）
```

**特性**：

- **每個 port 獨立**與相鄰 port 量測 peer delay，不關心 Grandmaster 位置
- `01:80:C2:00:00:0E` 是 IEEE 802.1 link-local multicast，**標準 bridge 依規範不轉發**，天然保證「只量到下一跳」
- Master 的 `Sync` / `Follow_Up` 透過 `correctionField` **累加每一跳的 residence time**，Slave 直接套用 master 帶過來的修正量
- 與 P2P Transparent Clock / Boundary Clock 完美協作

### 8.4 本專案的實作現況（重要）

**當前 ESP-IDF 版 ptpd 只實作 E2E**，尚未實作 P2P。證據對照：

| 檢查點 | 現況 | 程式碼位置 |
|---|---|---|
| 訊息型別定義 | 只有 `SYNC=0 / DELAY_REQ=1 / FOLLOW_UP=8 / DELAY_RESP=9 / ANNOUNCE=11` | `ptpv2.h:55-60` |
| `PTP_MSGTYPE_PDELAY_*` | **無定義**（2、3、10 皆缺） | — |
| TX 目的 MAC | 固定 `01:1B:19:00:00:00`（所有封包共用） | `ptpd.c:315` |
| peer-delay MAC 使用 | 註解標註 `TODO only for Pdelay_Req...`，尚未啟用 | `ptpd.c:314` |
| Kconfig `delayMechanism` 選項 | 無（硬編碼 E2E） | `components/ptpd/Kconfig.projbuild` |
| peer-delay multicast filter | 已加進 DM9058 MAC filter（預備） | `ptpd.c:724` |
| Daemon 對 Pdelay 封包處理 | **無** msgtype 分支；即使收到也會被忽略 | `ptp_process_rx_packet()` |

→ 換言之：若對端是 gPTP (802.1AS) 主時鐘，或透過 P2P TC switch 網路，本範例目前**無法正確同步**。對端必須使用 E2E profile 才能互通。

> **備註**：`components/ptpd/ref/ptpd-2.0.0/` 下保有完整版 ptpd-2.0.0 原始碼（支援 E2E + P2P），但僅作移植參考，**並未**編進本範例。

### 8.5 Path delay 量測本身的啟用開關

即使協定層固定 E2E，delay 量測是否執行仍可由 Kconfig 切換：

```
CONFIG_NETUTILS_PTPD_SEND_DELAYREQ=y    # 本範例預設開啟
```

- **開啟（y）**：Slave 週期性送 `Delay_Req`、接收 `Delay_Resp`，以平均後的 `path_delay` 修正 offset
- **關閉（n）**：Slave 假設 `path_delay = 0`，`offset = t2 − t1`。僅在直連且不計算上下游延遲時可接受；任何網路中繼都會累積系統性誤差

相關參數（第 5 節 §5.4 亦有列出，此處聚焦作用）：

| Kconfig | 預設 | 作用於 E2E 量測 |
|---|---|---|
| `CONFIG_NETUTILS_PTPD_MAX_PATH_DELAY_NS` | 100_000 | 超過此值的單次量測視為異常丟棄；在達到此精度前**不發送** `Delay_Req`（避免早期不穩定汙染估計） |
| `CONFIG_NETUTILS_PTPD_DELAYREQ_AVGCOUNT` | 100 | `path_delay` 移動平均樣本數 |
| `CONFIG_NETUTILS_PTPD_PATH_DELAY_STABILITY_NS` | 500 | 本地時鐘連續多次 offset 抖動小於此值才允許送 `Delay_Req` |
| `CONFIG_NETUTILS_PTPD_DELAYRESP_INTERVAL`（Master 端） | 4 | `Delay_Resp` header `logMessageInterval` 欄位，告知 Slave 建議下一次發送間隔 = 2^N 秒 |

### 8.6 何時需要 P2P？（拓樸判斷表）

| 網路拓樸 | 建議 delayMechanism | 理由 |
|---|---|---|
| Master ↔ Slave 直連（單一 link） | E2E 或 P2P 皆可 | 單 hop，兩者結果等價 |
| 透過普通（非 PTP-aware）switch | E2E（精度劣化） | switch transit time 無法修正，且對稱性差 |
| 透過 Transparent Clock（E2E TC） | **E2E** | TC 會更新 Sync/Follow_Up 的 `correctionField` |
| 透過 P2P TC / Boundary Clock | **P2P** | 逐段量測才能正確與 TC/BC 的 residence time 累加 |
| **IEEE 802.1AS (gPTP) / TSN** | **P2P（強制）** | 規範只允許 P2P；E2E 違反 profile |
| 跨 VLAN / 跨 bridge | E2E | P2P multicast 不被 bridge 轉發 |
| 工廠 TSN、車載乙太、AVB | **P2P** | 生態系皆為 gPTP |

### 8.7 擴充至 P2P 的實作路線（供未來參考）

若要在本 daemon 加上 P2P 支援，關鍵步驟：

1. **訊息型別**：於 `ptpv2.h` 新增
   ```c
   #define PTP_MSGTYPE_PDELAY_REQ            2
   #define PTP_MSGTYPE_PDELAY_RESP           3
   #define PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP 10
   ```
2. **資料結構**：參考 `components/ptpd/ref/ptpd-2.0.0/src/datatypes.h` 的 `MsgPDelayReq / MsgPDelayResp / MsgPDelayRespFollowUp`
3. **發送路徑**：`ptp_create_eth_frame()` 對 peer-delay msgType 改用 `01:80:C2:00:00:0E`（`ptpd.c:314` 已預留 TODO）
4. **接收路徑**：於 `ptp_process_rx_packet()` 加入 PDELAY msgtype 分支；參考 `ref/ptpd-2.0.0/src/protocol.c` 的 `handlePDelayReq / handlePDelayResp / handlePDelayRespFollowUp`
5. **狀態**：新增 `peer_delay_ns` 欄位與 Pdelay 專屬 timer（gPTP 預設 `logMinPdelayReqInterval = -2`，即 250 ms）
6. **Kconfig**：新增 `CONFIG_NETUTILS_PTPD_DELAY_MECHANISM`（choice：E2E / P2P）
7. **DM9058 端**：確認 EtherType 分類器對 `Pdelay_Req/Resp` 也能正確鎖存時間戳（見附錄 A 與 `gptp_porting_guide.md` §9）
8. **MAC filter**：peer-delay 目的 MAC 已於 `ptpd.c:724` 加入驅動 filter，**此前置作業已完成**

---

## 9. 常見調校與疑難排解

### 9.1 診斷流程圖

```
沒同步                 ──► 檢查 BMCA（兩端 domain、priority 是否一致）
 │
有 Announce 但沒 Sync ──► 檢查 EtherType filter、multicast MAC filter
 │
有 Sync 但沒 Follow_Up──► Two-step 設定不一致（ETH_DM9058_PTP_TWO_STEP_MODE）
 │
Time 一直跳變         ──► SETTIME_THRESHOLD_MS 太小；或 path delay 沒量測
 │
有同步但 drift 大      ──► kp/ki 需調整；或 transport 選錯（TS offset 不對）
 │
```

### 9.2 常見錯誤

| 現象 | 原因 | 解法 |
|---|---|---|
| `esp_eth_clock_gettime()` 始終回 `-1` | `esp_eth_clock_init()` 還沒呼叫或失敗 | 檢查 `ptp_main.c:163` 回傳值；確認 DM9058 driver 已選 |
| L2TAP 讀到時間戳都是 `0` | `L2TAP_S_TIMESTAMP_EN` 早於 `clock_init` | 確保第 4 節順序；或改在 daemon 外先 init |
| 只有 Master 送 Sync，Slave 收不到 | multicast MAC 沒加進 DM9058 filter | `ptpd.c:720-725` 是否執行；檢查 driver 是否支援 `ETH_CMD_ADD_MAC_FILTER` |
| Sync 收得到，Follow_Up 超時 | Two-step 設定不一致 | 兩端同步 `CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC` 與 `ETH_DM9058_PTP_TWO_STEP_MODE` |
| path delay 極大（>100 µs） | 被丟棄，不影響 offset 計算，但同步變差 | 調整 `MAX_PATH_DELAY_NS`；或檢查網路是否過 switch |
| 兩端 domain 不同 | 封包互相忽略 | 對齊 `CONFIG_NETUTILS_PTPD_DOMAIN` |
| Offset 收斂慢 | PI 常數太保守 | 短期可將 `kp` 調至 8–10；`ki` 視漂移量調整 |
| 跨 switch 精度差 | Transparent clock 可補償，一般 switch 則會引入不對稱 delay | 換 PTP-aware switch，或用直連 |

### 9.3 觀察 log 的重點

啟用 daemon log 後會看到：

```
ptpd: Got sync packet, seq N
ptpd: Got follow-up packet, seq N
ptpd: Local time: X.X, remote time Y.Y
ptpd: offset_ns X, adj Y, drift_acc Z
ptpd: Sent delay req, seq M
ptpd: Got delay-resp, seq M
ptpd: Path delay: X ns (avg: Y ns)
```

判斷同步品質：
- `offset_ns` 收斂到個位~兩位數 ns → 已鎖定
- `drift_acc` 應趨近一個穩定值（對應晶體 ppm 誤差 × SYNC_INTERVAL）
- `Path delay (avg)` 應小於 `MAX_PATH_DELAY_NS` 且方差低

---

## 10. 附錄 A：DM9058 PTP 硬體暫存器

節錄自 `components/ptpd/ref/doc/gptp_porting_guide.md` 第 9 節，此處僅列出對 L2TAP 模式有直接影響的暫存器：

| 暫存器 | 位址 | 值（L2 模式） | 說明 |
|---|---|---|---|
| PTP 重啟 | `0x60` | `0x01` | 觸發 PTP 引擎軟體 reset |
| PTP 致能 | `0x61` | `0x01` | 啟用 PTP 時間戳電路 |
| TX 時間戳控制 | `0x02` | `0x00` | TX 擷取設定 |
| Master/Slave 模式 | `0x64` | `0x12` | 運作模式選擇 |
| ONE_STEP 控制 | `0x63` | `0x00` | 0 = Two-step |
| **TS 偏移位址** | **`0x65`** | **`0x32` (50)** | PTP message 在封包中的位元組偏移（L2 = 50、UDP = 78） |
| **CRC 偏移位址** | **`0x66`** | **`0x20` (32)** | checksum 欄位偏移（L2 = 32、UDP = 60） |

**L2 模式 offset 推導**：

```
Ethernet (14B) + PTP header (34B) + flags (2B) = 50 → 0x32
                                                 ^ 指到 correctionField 起點
```

UDP 模式會多 `IP (20B) + UDP (8B) = 28B`，所以 50 + 28 = 78 → `0x4E`。

→ 選 `ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3` 時，DM9058 driver 會寫 `0x65=0x32`、`0x66=0x20`，封裝層級一致。

---

## 11. 附錄 B：完整封包流程時序

### 11.1 Two-step + DelayReq（本範例設定）

```
      MASTER                                       SLAVE
         |                                            |
         |  (1) Announce  --------------------------> | BMCA 比較
         |                                            |
         |  (2) Sync     [seq=N, TWO_STEP=1]  -----> | t2 = RX ts (L2TAP IREC)
         |       ↑ t1' = 軟體估算的 TX 時間           |
         |       ↑ 硬體實際 latch TX_ts = t1          |
         |                                            |
         |  (3) Follow_Up [seq=N, originTs=t1] ----> | 對齊 t1 與 t2
         |                                            |   → offset 候選值 = t2 − t1
         |                                            |
         |                              (4) Delay_Req | t3 = TX ts
         | t4 = RX ts (L2TAP IREC)  <---- [seq=M] -- |
         |                                            |
         |  (5) Delay_Resp [seq=M, rxTs=t4] -------> | path_delay = ((t2−t1)+(t4−t3))/2
         |                                            | offset     = ((t2−t1)−(t4−t3))/2
         |                                            |
         |                                            | PI(offset) → adjtime/settime
         |                                            |
         | (每 SYNC_INTERVAL_MSEC 重複 2~5)            |
         | (每 10 × SYNC 或 ANNOUNCE_INTERVAL 重複 1)  |
```

### 11.2 BMCA 狀態轉換（簡化）

```
┌──────────┐   收到較優 Announce    ┌──────┐
│ LISTENING│ ─────────────────────► │SLAVE │
└────┬─────┘                        └──┬───┘
     │ 一直沒收到 Announce             │ 連續 TIMEOUT_MS 沒收到 master
     ▼                                 ▼
┌──────────┐   自己 priority 最高   ┌──────────┐
│PRE_MASTER│ ─────────────────────► │  MASTER  │
└──────────┘                        └──────────┘
```

`CONFIG_NETUTILS_PTPD_TIMEOUT_MS`（預設 60 000 ms）控制 SLAVE → LISTENING → MASTER 的切換時間。

### 11.3 時間戳擷取位置

| 路徑 | 時間戳來源 | 精度 |
|---|---|---|
| TX 軟體時間（舊 NuttX 實作） | `ptp_gettime()` 呼叫當下的 `CLOCK_REALTIME` | µs~ms |
| TX 硬體時間（本範例） | DM9058 MAC 內部 `latch on MII` → L2TAP IREC | ~100 ns |
| RX 軟體時間 | 從 ISR 到 task 的時間 | 不定 |
| RX 硬體時間（本範例） | DM9058 接收完成時自動 latch → L2TAP IREC | ~100 ns |

---

## 12. 附錄 C：參考檔案索引

| 檔案 | 本手冊引用段落 | 內容 |
|---|---|---|
| `main/ptp_main.c` | §1.2、§4、§7 | App 入口、clock init、同步狀態控制 |
| `main/Kconfig.projbuild` | §5.5 | 範例層 GPIO 與脈衝相關設定 |
| `components/ptpd/ptpd.c` | §3.2–§3.4、§6.2、§7、§8.4 | daemon 主體、L2TAP init、PI 控制、收發流程 |
| `components/ptpd/include/ptpd.h` | §7.5 | `ptpd_start/status/stop`、`struct ptpd_status_s` |
| `components/ptpd/Kconfig.projbuild` | §5、§8.5 | 所有 PTP Kconfig 定義 |
| `components/ptpd/ptpv2.h` | §8.4、§12 | PTP v2 訊息格式、`PTP_FLAGS0_*`、`PTP_MSGTYPE_*` |
| `components/esp_eth_time/esp_eth_time.h` | §2.2、§7.3 | `esp_eth_clock_*` API、transport enum、`clockid_t CLOCK_PTP_SYSTEM` |
| `components/esp_eth_time/esp_eth_time.c` | §4、§11 | 實際操作 DM9058 暫存器的實作 |
| `components/ptpd/ref/doc/gptp_porting_guide.md` | §8.7、§11 | DM9058 暫存器映射、gPTP / P2P 移植細節 |
| `components/ptpd/ref/ptpd-2.0.0/` | §8.4、§8.7 | 原版 ptpd-2.0.0 原始碼（E2E + P2P 完整實作，僅供參考） |
| `sdkconfig.defaults` | §5 | 本範例預設 Kconfig 值 |
| `README.md` | 全文 | 範例編譯 / 燒錄 / log 範例 |

### 常用函式快速索引

| 函式 | 位置 | 作用 |
|---|---|---|
| `esp_eth_clock_init()` | `esp_eth_time.h:155` | 開啟 DM9058 PTP 引擎、寫 transport 偏移 |
| `esp_eth_clock_gettime()` | `esp_eth_time.h:106` | 讀 DM9058 當前時間 |
| `esp_eth_clock_settime()` | `esp_eth_time.h:94` | 跳轉 DM9058 時間 |
| `esp_eth_clock_adjtime()` | `esp_eth_time.h:82` | 頻率微調（`freq_scale`） |
| `esp_eth_clock_set_target_time()` | `esp_eth_time.h:131` | 設下一個時間戳 compare 點 |
| `esp_eth_clock_register_target_cb()` | `esp_eth_time.h:142` | 註冊 target-exceeded ISR |
| `ptpd_start()` | `ptpd.h:141` | 啟動 daemon task |
| `ptpd_status()` | `ptpd.h:164` | 查詢同步狀態 |
| `ptp_net_send()` / `ptp_net_recv()` | `ptpd.c:324 / 370` | L2TAP 收發 + 時間戳擷取 |
| `ptp_initialize_state()` | `ptpd.c:670` | L2TAP socket + filter 設定 |

---

*本手冊對應範例分支：`L2TAP_gPTP_v004`（`HEAD b5de4fba`）。若程式碼重構，請以 `path:line` 搜尋對應段落。*
