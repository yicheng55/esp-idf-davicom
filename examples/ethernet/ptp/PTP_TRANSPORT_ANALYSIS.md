# PTP 傳輸模式代碼流程分析報告

## `CONFIG_EXAMPLE_PTP_TRANSPORT_UDP_IPV4` 開/關 行為完整對比

---

## 目錄

1. [概述](#概述)
2. [Kconfig 設定](#kconfig-設定)
3. [Macro 定義差異](#macro-定義差異)
4. [初始化流程](#初始化流程)
5. [Socket 初始化與 Filter 設定](#socket-初始化與-filter-設定)
6. [TX 封包構建流程](#tx-封包構建流程)
7. [RX 封包接收與解析流程](#rx-封包接收與解析流程)
8. [硬體層封包分類 (DM9058)](#硬體層封包分類-dm9058)
9. [封包格式對比圖](#封包格式對比圖)
10. [完整比較總表](#完整比較總表)

---

## 概述

PTP 範例支援兩種傳輸模式，透過 `CONFIG_EXAMPLE_PTP_TRANSPORT_UDP_IPV4` 在編譯時選擇：

| 模式 | Define 狀態 | 傳輸封裝 | EtherType |
|------|------------|---------|-----------|
| **UDP/IPv4** | `= 1` (ON) | Ethernet → IPv4 → UDP → PTP | `0x0800` |
| **IEEE 802.3 L2** | `= 0` (OFF) | Ethernet → PTP | `0x88F7` |

---

## Kconfig 設定

**檔案：** `examples/ethernet/ptp/main/Kconfig.projbuild`

```kconfig
choice EXAMPLE_PTP_TRANSPORT          # 行 5
    prompt "PTP transport type"
    default EXAMPLE_PTP_TRANSPORT_IEEE_802_3   # 預設為 L2 模式

    config EXAMPLE_PTP_TRANSPORT_IEEE_802_3    # 行 13
        bool "IEEE 802.3 (Layer 2 Ethernet)"

    config EXAMPLE_PTP_TRANSPORT_UDP_IPV4      # 行 16
        bool "UDP IPv4"
endchoice

if EXAMPLE_PTP_TRANSPORT_UDP_IPV4             # 行 20 — 僅 UDP/IPv4 模式出現以下選項

    config EXAMPLE_ETH_STATIC_IP_ADDR         # 預設 "192.168.249.50"
    config EXAMPLE_ETH_STATIC_GW_ADDR         # 預設 "192.168.249.1"
    config EXAMPLE_ETH_STATIC_NETMASK_ADDR    # 預設 "255.255.255.0"

endif
```

> **重點：** UDP/IPv4 模式需要靜態 IP；L2 模式不需要任何 IP 配置。

---

## Macro 定義差異

**檔案：** `examples/ethernet/ptp/components/ptpd/ptpd.c`，行 85–98

```c
#define ETH_TYPE_PTP  0x88F7
#define ETH_TYPE_IPV4 0x0800

#ifdef CONFIG_EXAMPLE_PTP_TRANSPORT_UDP_IPV4
// ─── UDP/IPv4 模式 (ON) ────────────────────────────
#define PTP_ETH_FILTER_TYPE  ETH_TYPE_IPV4   // L2TAP 過濾 0x0800
#define PTP_EVENT_PORT       319              // PTP 時間關鍵訊息
#define PTP_GENERAL_PORT     320              // PTP 管理訊息
#define IP_PROTO_UDP         17
#define IP_HDR_LEN           20
#define UDP_HDR_LEN          8
#define UDP_IPV4_EXTRA_HDR   (IP_HDR_LEN + UDP_HDR_LEN)  // = 28
#else
// ─── IEEE 802.3 L2 模式 (OFF) ─────────────────────
#define PTP_ETH_FILTER_TYPE  ETH_TYPE_PTP    // L2TAP 過濾 0x88F7
#endif
```

---

## 初始化流程

### 1. 靜態 IP 配置（僅 UDP/IPv4 模式）

**檔案：** `examples/ethernet/ptp/main/ptp_main.c`

```c
// 行 31–47：函數定義（UDP/IPv4 模式才編譯）
#if CONFIG_EXAMPLE_PTP_TRANSPORT_UDP_IPV4
static void configure_static_ip(esp_netif_t *eth_netif)
{
    esp_netif_ip_info_t ip_info = {0};
    ESP_ERROR_CHECK(esp_netif_str_to_ip4(CONFIG_EXAMPLE_ETH_STATIC_IP_ADDR, &ip_info.ip));
    ESP_ERROR_CHECK(esp_netif_str_to_ip4(CONFIG_EXAMPLE_ETH_STATIC_GW_ADDR, &ip_info.gw));
    ESP_ERROR_CHECK(esp_netif_str_to_ip4(CONFIG_EXAMPLE_ETH_STATIC_NETMASK_ADDR, &ip_info.netmask));
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netif));    // 停止 DHCP
    ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif, &ip_info)); // 設定靜態 IP
}
#endif

// 行 130–134：在 netif 建立後立即套用（僅 ETH_0）
#if CONFIG_EXAMPLE_PTP_TRANSPORT_UDP_IPV4
if (i == 0) {
    configure_static_ip(eth_netif);
}
#endif
```

### 2. Transport 類型傳入 Clock 初始化

**檔案：** `examples/ethernet/ptp/main/ptp_main.c`，行 183–191

```c
esp_eth_clock_cfg_t clock_cfg = {
    .eth_hndl  = s_eth_handles[0],
#if CONFIG_EXAMPLE_PTP_TRANSPORT_UDP_IPV4
    .transport = ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4,   // → 傳給 DM9058 驅動
#else
    .transport = ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3, // → 傳給 DM9058 驅動
#endif
};
esp_eth_clock_init(CLOCK_PTP_SYSTEM, &clock_cfg);
```

### 3. Transport 枚舉定義

**檔案：** `components/esp_eth/include/esp_eth_mac_spi.h`

```c
typedef enum {
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4   = 0,  // PTP over UDP/IPv4 (預設)
    ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV6,         // PTP over UDP/IPv6
    ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3,        // PTP over IEEE 802.3 Ethernet
    ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_1AS,      // PTP over IEEE 802.1AS (gPTP)
} esp_eth_ptp_dm9058_transport_t;
```

---

## Socket 初始化與 Filter 設定

**檔案：** `examples/ethernet/ptp/components/ptpd/ptpd.c`，行 775–864  
**函數：** `ptp_initialize_state()`

### 步驟流程

```
open("/dev/net/tap", 0)                    ← 建立 L2TAP socket
    │
ioctl(L2TAP_S_INTF_DEVICE, "ETH_0")       ← 綁定以太網介面
    │
ioctl(L2TAP_S_RCV_FILTER, &eth_type_filter)
    │
    ├── [UDP/IPv4 ON]  eth_type_filter = 0x0800  ← 只收 IPv4 封包
    └── [UDP/IPv4 OFF] eth_type_filter = 0x88F7  ← 只收 PTP L2 封包
    │
ioctl(L2TAP_G_DEVICE_DRV_HNDL, &eth_handle)  ← 取得 eth driver handle
    │
ioctl(L2TAP_S_TIMESTAMP_EN)               ← 啟用硬體時間戳記
    │
esp_eth_ioctl(ETH_CMD_G_MAC_ADDR, ...)    ← 取得本機 MAC 地址
    │
[UDP/IPv4 ON only]
    │   esp_netif_get_handle_from_ifkey("ETH_0")
    │   esp_netif_get_ip_info(netif, &ip_info)
    │   state->intf_ip4_addr = ip_info.ip.addr  ← 儲存本機 IP 供 TX 使用
    │
新增 Multicast MAC Filter：
    ├── [UDP/IPv4 ON]
    │       ETH_CMD_ADD_MAC_FILTER: 01:00:5E:00:01:81  (IP multicast for 224.0.1.129)
    └── [UDP/IPv4 OFF]
            ETH_CMD_ADD_MAC_FILTER: 01:1B:19:00:00:00  (PTP 通用 multicast)
            ETH_CMD_ADD_MAC_FILTER: 01:80:C2:00:00:0E  (PTP P2P delay multicast)
```

---

## TX 封包構建流程

**檔案：** `examples/ethernet/ptp/components/ptpd/ptpd.c`  
**函數：** `ptp_net_send()`，行 400–441

### UDP/IPv4 模式 (ON)

```c
// 行 402–407
uint8_t msg_type_nibble = ((struct ptp_header_s *)ptp_msg)->messagetype & 0x0F;
uint16_t udp_port = (msg_type_nibble < 8) ? PTP_EVENT_PORT   // 319 — Sync, Delay_Req 等時間敏感訊息
                                           : PTP_GENERAL_PORT; // 320 — Announce, Follow_Up 等
uint16_t frame_len = 14 + 28 + ptp_msg_len;  // Eth(14) + IP(20)+UDP(8) + PTP
ptp_create_udp_ipv4_frame(state, eth_frame, ptp_msg, ptp_msg_len, udp_port);
```

**`ptp_create_udp_ipv4_frame()`** 行 355–397：

```
┌─ Ethernet Header (14 bytes) ─────────────────────────────────┐
│  Dst MAC : 01:00:5E:00:01:81  (IPv4 multicast for 224.0.1.129)│
│  Src MAC : state->intf_hw_addr                                │
│  EtherType: 0x08 0x00  (IPv4)                                 │
└───────────────────────────────────────────────────────────────┘
┌─ IPv4 Header (20 bytes) ─────────────────────────────────────┐
│  Version/IHL : 0x45                                          │
│  DSCP/ECN    : 0x00                                          │
│  Total Length: ip_len (big-endian)                           │
│  TTL         : 1  (multicast, 不跨 router)                   │
│  Protocol    : 17 (UDP)                                      │
│  Checksum    : ptp_ipv4_header_checksum() 計算               │
│  Src IP      : state->intf_ip4_addr  (本機靜態 IP)           │
│  Dst IP      : 224.0.1.129  (PTP multicast group)            │
└───────────────────────────────────────────────────────────────┘
┌─ UDP Header (8 bytes) ───────────────────────────────────────┐
│  Src Port : udp_dst_port  (319 或 320)                        │
│  Dst Port : udp_dst_port  (319 或 320)                        │
│  Length   : udp_len                                          │
│  Checksum : 0x0000  (disabled)                               │
└───────────────────────────────────────────────────────────────┘
┌─ PTP Payload ────────────────────────────────────────────────┐
│  ptp_msg (原始 PTP 訊息)                                      │
└───────────────────────────────────────────────────────────────┘
```

### IEEE 802.3 L2 模式 (OFF)

```c
// 行 408–412
uint16_t frame_len = ETH_HEADER_LEN + ptp_msg_len;  // Eth(14) + PTP
ptp_create_eth_frame(state, eth_frame, ptp_msg, ptp_msg_len);
```

**`ptp_create_eth_frame()`** 行 326–337：

```
┌─ Ethernet Header (14 bytes) ─────────────────────────────────┐
│  Dst MAC  : 01:1B:19:00:00:00  (PTP L2 通用 multicast)        │
│  Src MAC  : state->intf_hw_addr                               │
│  EtherType: 0x88 0xF7  (PTP)                                  │
└───────────────────────────────────────────────────────────────┘
┌─ PTP Payload ────────────────────────────────────────────────┐
│  ptp_msg (原始 PTP 訊息)                                      │
└───────────────────────────────────────────────────────────────┘
```

### UDP 端口選擇邏輯

```
PTP Message Type (bits 0-3)
    │
    ├── < 8  →  Event Messages  →  Port 319
    │     Sync (0x0), Delay_Req (0x1), Pdelay_Req (0x2), Pdelay_Resp (0x3)
    │
    └── >= 8  →  General Messages  →  Port 320
          Follow_Up (0x8), Delay_Resp (0x9), Pdelay_Resp_Follow_Up (0xA), Announce (0xB)
```

---

## RX 封包接收與解析流程

**檔案：** `examples/ethernet/ptp/components/ptpd/ptpd.c`  
**函數：** `ptp_net_recv()`，行 455–511

### Buffer 大小計算

```c
// 行 457–461
#ifdef CONFIG_EXAMPLE_PTP_TRANSPORT_UDP_IPV4
uint16_t frame_buf_len = 14 + 28 + ptp_msg_len;  // Eth+IPv4+UDP+PTP
#else
uint16_t frame_buf_len = ETH_HEADER_LEN + ptp_msg_len;  // Eth+PTP
#endif
```

### 接收與解析流程

```
read(state->ptp_socket, &ptp_msg_ext_buff, 0)
    │
    ├── 取得硬體時間戳記 (L2TAP_IREC_TIME_STAMP)
    │
    ├─ [UDP/IPv4 ON] ─────────────────────────────────────────────
    │   ├── ret <= 0 ?  →  return ret  (錯誤/無資料)
    │   │
    │   ├── 驗證 EtherType (eth_frame[12:13])
    │   │       ≠ 0x0800 ?  →  return 0  (丟棄)
    │   │
    │   ├── 驗證 IP Protocol (eth_frame[14+9])
    │   │       ≠ 17 (UDP) ?  →  return 0  (丟棄)
    │   │
    │   ├── 驗證 UDP Dst Port (eth_frame[14+20+2:3])
    │   │       ≠ 319 且 ≠ 320 ?  →  return 0  (丟棄)
    │   │
    │   ├── ptp_offset = 14 + 20 + 8 = 42
    │   ├── ptp_len    = ret - 28
    │   └── memcpy(ptp_msg, eth_frame + 42, ptp_len)  → 萃取 PTP payload
    │
    └─ [UDP/IPv4 OFF] ────────────────────────────────────────────
        └── memcpy(ptp_msg, eth_frame + 14, ret)  → 直接複製 PTP payload
```

---

## 硬體層封包分類 (DM9058)

**檔案：** `components/esp_eth/src/spi/dm9058/esp_eth_ptp_dm9058.c`，行 402–463

DM9058 驅動層**同時支援兩種格式**，不依賴編譯時選項，而是在 runtime 動態判斷。

### `is_valid_ptp_packet()` 行 402–427

```c
uint16_t ethertype = ((uint16_t)packet[12] << 8) | packet[13];

// 分支一：Layer 2 PTP
if (ethertype == ETH_TYPE_PTP) {         // 0x88F7
    return len >= ETH_HLEN + 34;         // 最小 PTP header = 34 bytes
}

// 分支二：Layer 3/4 PTP over UDP/IPv4
if (ethertype == ETH_TYPE_IPV4) {        // 0x0800
    uint8_t protocol = packet[14 + 9];   // IPv4 Protocol field
    if (protocol == IP_PROTO_UDP) {       // = 17
        uint16_t dest_port = (packet[14+20+2] << 8) | packet[14+20+3];
        return (dest_port == 319 || dest_port == 320);
    }
}
return false;
```

### `get_ptp_info()` 行 432–463

```c
const uint8_t *ptp_hdr;

if (ethertype == ETH_TYPE_PTP) {
    ptp_hdr = packet + 14;           // L2: PTP 在 Ethernet 之後
} else if (ethertype == ETH_TYPE_IPV4) {
    ptp_hdr = packet + 14 + 20 + 8; // L3/L4: PTP 在 Eth+IP+UDP 之後 (offset 42)
}

*msg_type      = ptp_hdr[0] & 0x0F;                         // 訊息類型
uint16_t flags = ((uint16_t)ptp_hdr[6] << 8) | ptp_hdr[7]; // flagField
*two_step_flag = (flags & PTP_FLAG_TWO_STEP) != 0;          // bit 9
```

---

## 封包格式對比圖

### UDP/IPv4 模式 (`CONFIG_EXAMPLE_PTP_TRANSPORT_UDP_IPV4 = 1`)

```
Byte offset:
 0        6        12  14   34      54  62
 |--------|--------|---|----|---------|----|---------|
 | Dst MAC| Src MAC|ETH| IPv4 Header | UDP| PTP Msg |
 |01:00:5E|  本機  |08 |Dst:224.0.1.129|319| Sync...  |
 |:00:01:81|  MAC  |00 |Src: 靜態IP  |or |         |
 |--------|--------|---|----|---------|----|---------|
    6B       6B     2B      20B         8B   可變長度

總 header overhead = 14 + 20 + 8 = 42 bytes
```

### IEEE 802.3 L2 模式 (`CONFIG_EXAMPLE_PTP_TRANSPORT_UDP_IPV4 = 0`)

```
Byte offset:
 0        6        12  14
 |--------|--------|---|---------|
 | Dst MAC| Src MAC|ETH| PTP Msg |
 |01:1B:19|  本機  |88 | Sync... |
 |:00:00:00|  MAC  |F7 |         |
 |--------|--------|---|---------|
    6B       6B     2B   可變長度

總 header overhead = 14 bytes
```

---

## 完整比較總表

| 面向 | UDP/IPv4 (ON) | IEEE 802.3 L2 (OFF) |
|------|--------------|---------------------|
| **EtherType** | `0x0800` (IPv4) | `0x88F7` (PTP) |
| **封包格式** | Eth + IPv4 + UDP + PTP | Eth + PTP |
| **Header overhead** | 42 bytes (14+20+8) | 14 bytes |
| **Dst MAC** | `01:00:5E:00:01:81` | `01:1B:19:00:00:00` |
| **Dst IP** | `224.0.1.129` (multicast) | N/A |
| **TTL** | 1 (不跨 router) | N/A |
| **UDP Port** | 319 (Event) / 320 (General) | N/A |
| **UDP Checksum** | 0x0000 (disabled) | N/A |
| **L2TAP Ethertype Filter** | `0x0800` | `0x88F7` |
| **Multicast MAC Filter** | `01:00:5E:00:01:81` (×1) | `01:1B:19:00:00:00` + `01:80:C2:00:00:0E` (×2) |
| **需要靜態 IP** | 是 (Kconfig 設定) | 否 |
| **IP 地址取得** | `esp_netif_get_ip_info()` | 不需要 |
| **RX 驗證層數** | 3 層 (EtherType→Protocol→Port) | 1 層 (EtherType by filter) |
| **PTP payload offset** | 42 (TX) / 42 (RX) | 14 (TX) / 14 (RX) |
| **DM9058 transport 設定** | `ESP_ETH_PTP_DM9058_TRANSPORT_UDP_IPV4` | `ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3` |
| **硬體層識別** | `is_valid_ptp_packet()` 動態判斷 | `is_valid_ptp_packet()` 動態判斷 |
| **PTP header 位置** | `packet + 42` | `packet + 14` |

---

## 關鍵檔案索引

```
examples/ethernet/ptp/
├── main/
│   ├── Kconfig.projbuild          — 選項定義、靜態 IP 參數（行 5-40）
│   └── ptp_main.c                 — 應用入口，transport 選擇（行 31-47, 130-134, 183-191）
└── components/
    ├── ptpd/
    │   └── ptpd.c                 — 核心協議層
    │       ├── Macro 定義          行 85-98
    │       ├── ptp_create_eth_frame()      行 326-337  [L2 TX]
    │       ├── ptp_ipv4_header_checksum()  行 340-353  [UDP/IPv4 only]
    │       ├── ptp_create_udp_ipv4_frame() 行 355-397  [UDP/IPv4 TX]
    │       ├── ptp_net_send()              行 400-441  [TX dispatch]
    │       ├── ptp_net_recv()              行 455-511  [RX dispatch]
    │       └── ptp_initialize_state()      行 775-864  [socket init]
    └── esp_eth_time/
        └── esp_eth_time.c         — Clock 初始化，transport 傳給 DM9058 驅動

components/esp_eth/
├── include/
│   └── esp_eth_mac_spi.h          — esp_eth_ptp_dm9058_transport_t 枚舉（行 188-193）
└── src/spi/dm9058/
    └── esp_eth_ptp_dm9058.c       — 硬體層封包分類
        ├── Protocol constants      行 37-44
        ├── is_valid_ptp_packet()   行 402-427
        └── get_ptp_info()          行 432-463
```

---

*報告生成日期：2026-04-10*  
*分析基於 ESP-IDF branch: UDP_IPV4_one_step_v004*
