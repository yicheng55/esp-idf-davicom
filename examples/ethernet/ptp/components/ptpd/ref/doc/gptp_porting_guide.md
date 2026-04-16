# gPTP (IEEE 802.1AS) 技術分析與移植指南

> **適用專案**：`at32f403a_dm9058_ptp_daemon`（AT32F403A MCU + DM9058 SPI 乙太網 + ptpd-2.0.0）
> **目標讀者**：需要將 gPTP 功能移植至其他嵌入式平台（如 ESP-IDF / ESP32）的工程師
> **文件版本**：基於 `esp32_master_one_step_v001` 分支原始碼分析

---

## 目錄

1. [概述與背景](#1-概述與背景)
2. [專案硬體架構](#2-專案硬體架構)
3. [核心流程分析：L2 封包交換邏輯](#3-核心流程分析l2-封包交換邏輯)
   - 3.1 [L2 收發底層實作](#31-l2-收發底層實作)
   - 3.2 [PDelay 路徑延遲測量機制](#32-pdelay-路徑延遲測量機制)
   - 3.3 [Sync / Follow\_Up 處理流程](#33-sync--follow_up-處理流程)
4. [狀態機解析](#4-狀態機解析)
   - 4.1 [PTP 狀態機整體圖](#41-ptp-狀態機整體圖)
   - 4.2 [訊息分派機制（handle）](#42-訊息分派機制handle)
   - 4.3 [BMCA 在 802.1AS 下的簡化實作](#43-bmca-在-8021as-下的簡化實作)
   - 4.4 [計時器驅動機制](#44-計時器驅動機制)
5. [關鍵資料結構](#5-關鍵資料結構)
6. [移植關鍵點（Porting Guide）](#6-移植關鍵點porting-guide)
   - 6.1 [HAL：MAC 層 Timestamping 介面](#61-halmac-層-timestamping-介面)
   - 6.2 [網路層：L2 Raw Ethernet 繞過 L3](#62-網路層l2-raw-ethernet-繞過-l3)
   - 6.3 [定時器與系統時鐘](#63-定時器與系統時鐘)
   - 6.4 [必須開啟的編譯巨集](#64-必須開啟的編譯巨集)
   - 6.5 [ESP-IDF 移植具體建議](#65-esp-idf-移植具體建議)
7. [附錄 A：訊息流時序圖](#7-附錄-a訊息流時序圖)
8. [附錄 B：關鍵函式速查表](#8-附錄-b關鍵函式速查表)
9. [附錄 C：DM9058 PTP 寄存器映射](#9-附錄-cdm9058-ptp-寄存器映射)

---

## 1. 概述與背景

### 1.1 gPTP 與標準 PTP（IEEE 1588）的差異

| 特性 | 標準 PTP（IEEE 1588-2008） | gPTP（IEEE 802.1AS） |
|------|--------------------------|---------------------|
| 傳輸層 | UDP/IPv4、UDP/IPv6、L2 Ethernet | **僅限 L2 Ethernet** |
| 延遲測量 | 可選 E2E 或 P2P | **強制 P2P**（Peer-to-Peer） |
| 多播 MAC | `01:1B:19:00:00:00` | **`01:80:C2:00:00:0E`** |
| EtherType | 0x88F7 | 0x88F7（相同） |
| `transportSpecific` | 0x0 | **0x1**（SdoId = 1） |
| Sync 間隔 | 可變（典型 1s） | 預設 **125ms**（log = −3） |
| PDelayReq 間隔 | 可選 | 預設 **250ms**（log = −2） |
| BMCA | 完整版（含 E2E 路徑比較） | **簡化版**（只做 P2P） |
| VLAN 標籤 | 可選 | 支援 802.1Q VLAN（本專案已實作） |

### 1.2 gPTP 在本專案的啟用方式

本專案透過以下單一巨集控制傳輸層模式：

```c
// middlewares/3rd_party/ptpd-2.0.0/src/constants.h:111
#define PTP_NETWORK_TRANSPORT 4  /* TRANSPORT_IEEE_802_1AS */
```

當此值為 4 時，整個協議棧從 L2 收發、MAC 多播位址選擇，到 DM9058 硬體時間戳偏移量，均自動切換為 gPTP 模式。

---

## 2. 專案硬體架構

```
┌─────────────────────────────────────────────────────────┐
│                  AT32F403A MCU (200 MHz)                 │
│                                                         │
│  ┌─────────────┐    ┌──────────────────────────────┐   │
│  │ ptpd-2.0.0  │    │       應用程式主迴圈           │   │
│  │ 協議引擎    │◄──►│  ptpd_Periodic_Handle()      │   │
│  │             │    │  (by TMR6 interrupt / main)  │   │
│  └──────┬──────┘    └──────────────────────────────┘   │
│         │                                               │
│  ┌──────▼──────┐    ┌──────────────────────────────┐   │
│  │ dep/net.c   │    │ dep/sys_time.c               │   │
│  │ (L2 收發)   │    │ getTime/setTime/adjFreq       │   │
│  └──────┬──────┘    └──────────┬───────────────────┘   │
│         │                      │                        │
│  ┌──────▼──────────────────────▼───────────────────┐   │
│  │              lwIP 2.1.2（NO_SYS=1 輪詢模式）     │   │
│  │  netconf_register_ptp_callback()                │   │
│  └──────────────────────────┬────────────────────┘    │
│                             │ SPI1 (PA5/6/7, CS=PA15)  │
└─────────────────────────────┼───────────────────────────┘
                              │
              ┌───────────────▼──────────────────┐
              │    DM9058 SPI 乙太網控制器        │
              │                                  │
              │  ┌────────────────────────────┐  │
              │  │  硬體 PTP 時間戳暫存器      │  │
              │  │  Reg 0x60: PTP 控制        │  │
              │  │  Reg 0x61: PTP 致能        │  │
              │  │  Reg 0x64: Master/Slave    │  │
              │  │  Reg 0x65: TS 偏移位址     │  │
              │  │  Reg 0x66: CRC 偏移位址    │  │
              │  └────────────────────────────┘  │
              └──────────────────────────────────┘
                              │
                         乙太網實體層（PHY）
                              │
                          網路（gPTP）
```

### 軟體層次依賴

```
constants.h ─────────────── 所有 PTP 常數（含 gPTP profile 預設值）
datatypes.h ─────────────── 核心 C 結構體定義
protocol.c  ─────────────── PTP 狀態機（doState / handle）
bmc.c       ─────────────── BMCA 演算法
dep/net.c   ─────────────── L2/UDP 網路收發
dep/servo.c ─────────────── PI 伺服控制器（updateOffset / updateClock）
dep/timer.c ─────────────── 軟體計時器（catchAlarm / timerUpdate）
dep/sys_time.c ──────────── 系統時鐘 HAL（getTime / setTime / adjFreq）
dm9058_ptp.c ────────────── DM9058 硬體 PTP 時間戳驅動
```

---

## 3. 核心流程分析：L2 封包交換邏輯

### 3.1 L2 收發底層實作

#### 3.1.1 EtherType 與 MAC 多播位址

```c
// dep/net.c:22-23
static const uint8_t PTP_MAC_IEEE_802_3[6]   = {0x01, 0x1B, 0x19, 0x00, 0x00, 0x00};
static const uint8_t PTP_MAC_IEEE_802_1AS[6] = {0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E};

// EtherType（來自 lwIP 的 ETHTYPE_PTP 定義）
#define PTP_ETHERTYPE  0x88F7U
```

gPTP 使用 `01:80:C2:00:00:0E`（IEEE 802.1 橋接保留多播位址），此位址**不跨越橋接器**，確保 PDelay 測量的 peer-to-peer 特性。

#### 3.1.2 L2 接收：`netRecvL2Callback()`

此函式由 lwIP 的 MAC 驅動回調（透過 `netconf_register_ptp_callback()` 注冊）：

```c
// dep/net.c:39-102
static int netRecvL2Callback(struct pbuf *p)
{
    uint16_t ethType;
    uint16_t hdrLen = ETH_HDR_LEN;  // 14 bytes

    // 讀取 EtherType（big-endian，位於 Ethernet frame 偏移 12-13）
    ethType = ((uint16_t)b[12] << 8) | (uint16_t)b[13];

    // 處理 802.1Q VLAN 標籤（EtherType == 0x8100）
    if (ethType == ETHTYPE_VLAN) {
        ethType = ((uint16_t)b[16] << 8) | (uint16_t)b[17];
        hdrLen  = VLAN_ETH_HDR_LEN;  // 18 bytes
    }

    // 過濾：只接受 PTP EtherType（0x88F7）
    if (ethType != PTP_ETHERTYPE)
        return 0;

    // 剝除 Ethernet 標頭
    pbuf_remove_header(p, hdrLen);

    // 根據訊息類型分發到 eventQ 或 generalQ
    messageType = ((const uint8_t *)p->payload)[0] & 0x0F;
    targetQ = isEventMessageType(messageType) ? &g_ptp_netpath_l2->eventQ
                                              : &g_ptp_netpath_l2->generalQ;
    netQPut(targetQ, p);
    return 1;
}
```

**Event 訊息**（需硬體時間戳）：`SYNC(0x0)`、`DELAY_REQ(0x1)`、`PDELAY_REQ(0x2)`、`PDELAY_RESP(0x3)`

**General 訊息**（不需時間戳）：`FOLLOW_UP(0x8)`、`DELAY_RESP(0x9)`、`PDELAY_RESP_FOLLOW_UP(0xA)`、`ANNOUNCE(0xB)`

#### 3.1.3 L2 發送：`netSendL2()`

```c
// dep/net.c:133-193
static ssize_t netSendL2(const Octet *buf, UInteger16 length, TimeInternal *time)
{
    struct netif *iface = netif_default;

    // 根據傳輸模式選擇目標 MAC
    dstMac = (g_ptp_transport_l2 == TRANSPORT_IEEE_802_1AS)
             ? PTP_MAC_IEEE_802_1AS    // 01:80:C2:00:00:0E
             : PTP_MAC_IEEE_802_3;     // 01:1B:19:00:00:00

    // 分配 pbuf（14 bytes Ethernet 標頭 + PTP payload）
    p = pbuf_alloc(PBUF_RAW, ETH_HDR_LEN + length, PBUF_RAM);

    // 組裝 Ethernet Frame
    memcpy(out + 0, dstMac, 6);           // 目標 MAC（6 bytes）
    memcpy(out + 6, iface->hwaddr, 6);    // 來源 MAC（6 bytes）
    out[12] = (PTP_ETHERTYPE >> 8);       // EtherType 高字節 = 0x88
    out[13] = (PTP_ETHERTYPE & 0xFF);     // EtherType 低字節 = 0xF7
    memcpy(out + ETH_HDR_LEN, buf, length); // PTP 訊息本體

    // 透過 lwIP netif 的 linkoutput 發送
    iface->linkoutput(iface, p);

    // 讀取硬體時間戳（TX timestamp）
    if (time != NULL) {
#if LWIP_PTP
        time->seconds     = p->time_sec;
        time->nanoseconds = p->time_nsec;
        if (time->seconds == 0 && time->nanoseconds == 0)
            getTime(time);  // 降級為軟體時間戳
#else
        getTime(time);
#endif
    }
}
```

#### 3.1.4 發送選擇器（四個入口函式）

```c
// dep/net.c（示意）
// L2 模式判斷：eventPcb == NULL && generalPcb == NULL

// ① Sync / PDelayReq / PDelayResp（需要 TX 時間戳）
ssize_t netSendEvent(NetPath*, const Octet*, UInteger16, TimeInternal*time)
    → netSendL2(buf, length, time)  // gPTP 模式

// ② Follow_Up / Announce（不需 TX 時間戳）
ssize_t netSendGeneral(NetPath*, const Octet*, UInteger16)
    → netSendL2(buf, length, NULL)

// ③ PDelayReq / PDelayResp（P2P event，對 peer 多播位址）
ssize_t netSendPeerEvent(NetPath*, const Octet*, UInteger16, TimeInternal*)
    → netSendL2(buf, length, time)

// ④ PDelayRespFollowUp（P2P general）
ssize_t netSendPeerGeneral(NetPath*, const Octet*, UInteger16)
    → netSendL2(buf, length, NULL)
```

> **注意**：在 gPTP（L2）模式下，`netPath->eventPcb` 和 `netPath->generalPcb` 均為 `NULL`，四個函式都直接呼叫 `netSendL2()`，差異只在於是否傳入時間戳指標。

---

### 3.2 PDelay 路徑延遲測量機制

PDelay（Peer Delay）是 gPTP 強制使用的 P2P 延遲測量方法，用於測量相鄰兩節點之間單向連路延遲（`peerMeanPathDelay`）。

#### 3.2.1 四個時間戳的定義

```
[Requester]                          [Responder]
    │                                    │
    │──── PDelayReq ─────────────────►  │
    │  T1: TX 時間戳（Requester 時鐘）    │  T2: RX 時間戳（Responder 時鐘）
    │                                    │  T3: TX 時間戳（Responder 時鐘）
    │  ◄──── PDelayResp ──────────────  │  （ONE_STEP 模式含在 Resp 中）
    │  ◄── PDelayRespFollowUp ────────  │  （TWO_STEP 模式含在 FollowUp 中）
    │  T4: RX 時間戳（Requester 時鐘）   │
```

在 `PtpClock` 結構體中對應的欄位：

```c
// datatypes.h:456-459
TimeInternal pdelay_t1;  // Requester 發送 PDelayReq 的 TX 時間戳
TimeInternal pdelay_t2;  // Responder 接收 PDelayReq 的 RX 時間戳（從 Resp 封包取得）
TimeInternal pdelay_t3;  // Responder 發送 PDelayResp 的 TX 時間戳（從 FollowUp 取得）
TimeInternal pdelay_t4;  // Requester 接收 PDelayResp 的 RX 時間戳
```

#### 3.2.2 計算公式

**TWO_STEP 模式**（`DEFAULT_TWO_STEP_FLAG = TRUE`）：

```
peerMeanPathDelay = ( (t2 - t1) + (t4 - t3) ) / 2  -  CorrectionField

其中：
  (t2 - t1) = Responder 視角的往程時間（含連路延遲 + 時鐘偏差）
  (t4 - t3) = Requester 視角的回程時間（含連路延遲 - 時鐘偏差）
  兩者相加再除以 2，時鐘偏差項抵消，得到單向連路延遲
```

**ONE_STEP 模式**（`DEFAULT_TWO_STEP_FLAG = FALSE`，本專案預設）：

```
peerMeanPathDelay = (t4 - t1) / 2  -  CorrectionField

其中：
  t2 和 t3 由 Responder 計算差值 (t3 - t2) 後放入 Resp 的 correctionField
  Requester 使用 (t4 - t1) 後扣除 correctionField 計算延遲
```

#### 3.2.3 Requester 端訊息流程

**Step 1：發送 PDelayReq**（`protocol.c: issuePDelayReq()`）

```c
// 觸發時機：PDELAYREQ_INTERVAL_TIMER 計時器超期（預設 250ms）
static void issuePDelayReq(PtpClock *ptpClock)
{
    Timestamp originTimestamp;
    TimeInternal internalTime;

    getTime(&internalTime);
    fromInternalTime(&internalTime, &originTimestamp);

    // 打包 PDelayReq 訊息（含 originTimestamp 欄位）
    msgPackPDelayReq(ptpClock, ptpClock->msgObuf, &originTimestamp);

    // 發送：netSendPeerEvent() → netSendL2()，同時獲取硬體 TX 時間戳
    netSendPeerEvent(&ptpClock->netPath, ptpClock->msgObuf,
                     PDELAY_REQ_LENGTH, &internalTime);

    ptpClock->pdelay_t1 = internalTime;  // 保存 T1
    ptpClock->sentPDelayReqSequenceId++;
}
```

**Step 2：接收 PDelayResp** → 取得 T4 和 T2，等待 FollowUp

**Step 3：接收 PDelayRespFollowUp** → 取得 T3，計算延遲

```c
// protocol.c: handlePDelayRespFollowUp()
if (ptpClock->waitingForPDelayRespFollowUp) {
    // 驗證 sequenceId 匹配
    if (ptpClock->msgTmpHeader.sequenceId == ptpClock->sentPDelayReqSequenceId - 1) {
        // 從 FollowUp 取得 T3（responseOriginTimestamp）
        toInternalTime(&ptpClock->pdelay_t3, &resp.responseOriginTimestamp);

        // 計算並更新 peerMeanPathDelay
        updatePeerDelay(ptpClock, &correctionField, TRUE);  // TRUE = TWO_STEP

        ptpClock->waitingForPDelayRespFollowUp = FALSE;
    }
}
```

#### 3.2.4 Responder 端訊息流程

**接收 PDelayReq，發送 PDelayResp**（`protocol.c: handlePDelayReq()`）：

```c
static void handlePDelayReq(PtpClock *ptpClock, TimeInternal *time, ...)
{
    // 在任何非 INITIALIZING / FAULTY / DISABLED 狀態均響應
    if (ptpClock->portDS.portState in [UNCALIBRATED, LISTENING, PASSIVE, SLAVE, MASTER])
    {
        // 保存 RX 時間戳作為 T2
        ptpClock->pdelay_t2 = *time;

        // 發送 PDelayResp（含 requestReceiptTimestamp = T2）
        issuePDelayResp(ptpClock, time, &ptpClock->msgTmpHeader);

        // TWO_STEP 模式：另外發送 FollowUp（含精準 T3）
        if (ptpClock->defaultDS.twoStepFlag) {
            issuePDelayRespFollowUp(ptpClock, &tx_time, &ptpClock->msgTmpHeader);
        }
        // ONE_STEP 模式：T3 已由硬體嵌入 PDelayResp 的 correctionField
    }
}
```

**`updatePeerDelay()` 計算核心**（`dep/servo.c`）：

```c
void updatePeerDelay(PtpClock *ptpClock, const TimeInternal *correctionField, Boolean twoStep)
{
    if (twoStep) {
        // Tab = t2 - t1  （Responder 時鐘視角的往程）
        subTime(&Tab, &ptpClock->pdelay_t2, &ptpClock->pdelay_t1);
        // Tba = t4 - t3  （Requester 時鐘視角的回程）
        subTime(&Tba, &ptpClock->pdelay_t4, &ptpClock->pdelay_t3);
        // peerMeanPathDelay = (Tab + Tba) / 2
        addTime(&ptpClock->portDS.peerMeanPathDelay, &Tab, &Tba);
    } else {
        // ONE_STEP：peerMeanPathDelay = t4 - t1
        subTime(&ptpClock->portDS.peerMeanPathDelay,
                &ptpClock->pdelay_t4, &ptpClock->pdelay_t1);
    }

    // 扣除 correctionField
    subTime(&ptpClock->portDS.peerMeanPathDelay,
            &ptpClock->portDS.peerMeanPathDelay, correctionField);

    // 除以 2 得到單向延遲
    div2Time(&ptpClock->portDS.peerMeanPathDelay);

    // 指數平滑濾波（alpha = 1/2^s，由 DEFAULT_DELAY_S=6 控制）
    filter(&ptpClock->portDS.peerMeanPathDelay.nanoseconds, &ptpClock->owd_filt);
}
```

---

### 3.3 Sync / Follow\_Up 處理流程

#### 3.3.1 Master 端發送流程

**發送 Sync**（`protocol.c: issueSync()`），由 `SYNC_INTERVAL_TIMER` 觸發（預設每 125ms）：

```c
static void issueSync(PtpClock *ptpClock)
{
    Timestamp originTimestamp;
    TimeInternal internalTime;

    if (DEFAULT_TWO_STEP_FLAG == TRUE) {
        getTime(&internalTime);  // 預估發送時間（軟體時間戳）
    } else {
        // ONE_STEP：originTimestamp 為 0，硬體在發送時自動填入
        internalTime = {0, 0};
    }

    fromInternalTime(&internalTime, &originTimestamp);
    msgPackSync(ptpClock, ptpClock->msgObuf, &originTimestamp);

    // 發送 Sync 到 event 通道，同時取得硬體 TX 時間戳
    netSendEvent(&ptpClock->netPath, ptpClock->msgObuf, SYNC_LENGTH, &internalTime);
    ptpClock->sentSyncSequenceId++;
    // 若 TWO_STEP：internalTime 現在包含精準的硬體 TX 時間戳
}
```

**發送 Follow\_Up**（`protocol.c: issueFollowup()`），在 TWO_STEP 模式緊接 Sync 之後：

```c
static void issueFollowup(PtpClock *ptpClock, const TimeInternal *time)
{
    Timestamp preciseOriginTimestamp;
    fromInternalTime(time, &preciseOriginTimestamp);

    // 打包 Follow_Up，含精準的 TX 時間戳（preciseOriginTimestamp = T1）
    msgPackFollowUp(ptpClock, ptpClock->msgObuf, &preciseOriginTimestamp);

    // 透過 general 通道發送（不需硬體時間戳）
    netSendGeneral(&ptpClock->netPath, ptpClock->msgObuf, FOLLOW_UP_LENGTH);
}
```

#### 3.3.2 Slave 端接收流程

**接收 Sync**（`protocol.c: handleSync()`）：

```c
static void handleSync(PtpClock *ptpClock, TimeInternal *time, Boolean isFromSelf)
{
    if (ptpClock->portDS.portState in [UNCALIBRATED, SLAVE])
    {
        // 驗證來自當前 Master
        if (!isSamePortIdentity(&ptpClock->parentDS.parentPortIdentity,
                                &ptpClock->msgTmpHeader.sourcePortIdentity))
            return;

        ptpClock->timestamp_syncRecieve = *time;  // T2：Sync RX 時間戳

        // 取得 Sync 的 correctionField（可能含傳播延遲補償）
        scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionfield,
                                       &correctionField);

        if (getFlag(ptpClock->msgTmpHeader.flagField[0], FLAG0_TWO_STEP)) {
            // TWO_STEP：等待 Follow_Up 取得精準 T1
            ptpClock->waitingForFollowUp = TRUE;
            ptpClock->recvSyncSequenceId = ptpClock->msgTmpHeader.sequenceId;
            ptpClock->correctionField_sync = correctionField;
        } else {
            // ONE_STEP：直接取得 T1（在 originTimestamp 欄位）
            toInternalTime(&originTimestamp, &ptpClock->msgTmp.sync.originTimestamp);
            updateOffset(ptpClock, &ptpClock->timestamp_syncRecieve,
                        &originTimestamp, &correctionField);
            updateClock(ptpClock);
        }
    }
}
```

**接收 Follow\_Up**（`protocol.c: handleFollowUp()`）：

```c
static void handleFollowUp(PtpClock *ptpClock, Boolean isFromSelf)
{
    if (!ptpClock->waitingForFollowUp)  return;

    // 驗證 sequenceId 匹配
    if (ptpClock->recvSyncSequenceId != ptpClock->msgTmpHeader.sequenceId)
        return;

    // 取得精準 T1（preciseOriginTimestamp）
    toInternalTime(&preciseOriginTimestamp,
                  &ptpClock->msgTmp.follow.preciseOriginTimestamp);

    // 合併 Sync 與 Follow_Up 的 correctionField
    scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionfield,
                                   &correctionField_followup);
    addTime(&correctionField, &correctionField_followup,
           &ptpClock->correctionField_sync);

    // 計算偏移並更新時鐘
    updateOffset(ptpClock, &ptpClock->timestamp_syncRecieve,
                &preciseOriginTimestamp, &correctionField);
    updateClock(ptpClock);

    ptpClock->waitingForFollowUp = FALSE;
}
```

#### 3.3.3 時間偏移計算

**`updateOffset()`**（`dep/servo.c:129`）：

```c
void updateOffset(PtpClock *ptpClock,
                  const TimeInternal *syncEventIngressTimestamp,  // T2
                  const TimeInternal *preciseOriginTimestamp,     // T1
                  const TimeInternal *correctionField)            // CF_sync + CF_followup
{
    // offsetFromMaster = T2 - T1 - correctionField
    subTime(&ptpClock->Tms, syncEventIngressTimestamp, preciseOriginTimestamp);
    subTime(&ptpClock->Tms, &ptpClock->Tms, correctionField);

    ptpClock->currentDS.offsetFromMaster = ptpClock->Tms;

    // P2P 模式：再扣除 peerMeanPathDelay
    switch (ptpClock->portDS.delayMechanism) {
        case P2P:
            subTime(&ptpClock->currentDS.offsetFromMaster,
                    &ptpClock->currentDS.offsetFromMaster,
                    &ptpClock->portDS.peerMeanPathDelay);
            break;
        case E2E:
            subTime(&ptpClock->currentDS.offsetFromMaster,
                    &ptpClock->currentDS.offsetFromMaster,
                    &ptpClock->currentDS.meanPathDelay);
            break;
    }

    // 指數平滑濾波（alpha = 1/2^1，由 DEFAULT_OFFSET_S=1 控制）
    filter(&ptpClock->currentDS.offsetFromMaster.nanoseconds, &ptpClock->ofm_filt);

    // 校準狀態判斷
    if (abs(offset.ns) < DEFAULT_CALIBRATED_OFFSET_NS)   // 10,000 ns = 10 µs
        setFlag(MASTER_CLOCK_SELECTED);
    else if (abs(offset.ns) > DEFAULT_UNCALIBRATED_OFFSET_NS)  // 1,000,000 ns = 1 ms
        setFlag(SYNCHRONIZATION_FAULT);
}
```

**最終公式總結**：

```
offsetFromMaster = (T2 - T1)  -  (CF_sync + CF_followup)  -  peerMeanPathDelay

其中：
  T1 = Sync 精準發送時間（Master 時鐘）
  T2 = Sync 接收時間（Slave 時鐘）
  CF = Sync 和 Follow_Up 的 correctionField 總和
  peerMeanPathDelay = 相鄰連路的單向傳播延遲（由 PDelay 機制測得）
```

---

## 4. 狀態機解析

### 4.1 PTP 狀態機整體圖

```
                    ┌─────────────────┐
                    │ PTP_INITIALIZING│ (state=0)
                    └────────┬────────┘
                             │ doInit() 成功
                             ▼
                    ┌─────────────────┐
               ┌───│  PTP_LISTENING  │◄──────────────────┐
               │   └────────┬────────┘ (state=3)          │
               │            │                             │
               │       STATE_DECISION_EVENT               │
               │       (收到 Announce / ANNOUNCE_RECEIPT  │
               │        TIMEOUT_EXPIRES)                  │
               │            │                             │
               │     ┌──────┴──────┐                     │
               │     │    BMCA     │                     │
               │     └──────┬──────┘                     │
               │            │                             │
     ┌─────────┤    ┌───────┴────────┐   ┌───────────────┤
     │         │    │  本機時鐘最優？  │   │               │
     │         │    └───────┬────────┘   │               │
     │         │            │            │               │
     │    ┌────▼───┐  ┌─────▼──────┐  ┌─▼────────────┐  │
     │    │PASSIVE │  │PRE_MASTER  │  │ UNCALIBRATED │  │
     │    │(state=6)  │ (state=4)  │  │  (state=7)   │  │
     │    └────┬───┘  └─────┬──────┘  └──────┬───────┘  │
     │         │            │                │           │
     │   再次   │  QUALIFICATION_TIMEOUT      │           │
     │   BMCA  │  (等待 announce 超時)        │  MASTER_  │
     │         │            │            CLOCK_SELECTED   │
     │         │      ┌─────▼──────┐        │           │
     │         │      │PTP_MASTER  │    ┌───▼─────┐     │
     │         │      │ (state=5)  │    │PTP_SLAVE│     │
     │         │      └────────────┘    │(state=8)│     │
     │         │                        └───┬─────┘     │
     │         │                           │            │
     │         └──────── SYNCHRONIZATION_FAULT ─────────┘
     │                    或 MASTER_CLOCK_CHANGED
     │
     └──── FAULT_DETECTED ──► PTP_FAULTY (state=1)
                                    │
                              FAULT_CLEARED
                                    ▼
                            PTP_INITIALIZING
```

**各狀態行為摘要**：

| 狀態 | 值 | 主要行為 |
|------|----|---------|
| `PTP_INITIALIZING` | 0 | 初始化網路、計時器、資料集 |
| `PTP_FAULTY` | 1 | 錯誤恢復；等待 FAULT_CLEARED |
| `PTP_DISABLED` | 2 | 關閉所有 PTP 功能 |
| `PTP_LISTENING` | 3 | 等待 Announce；執行 BMCA |
| `PTP_PRE_MASTER` | 4 | 等待 QUALIFICATION_TIMEOUT 後成為 Master |
| `PTP_MASTER` | 5 | 定期發送 Sync + Announce；響應 PDelayReq |
| `PTP_PASSIVE` | 6 | 接收 Announce 但不同步；仍參與 PDelay |
| `PTP_UNCALIBRATED` | 7 | 接收 Sync/FollowUp 但尚未達到校準閾值 |
| `PTP_SLAVE` | 8 | 完全同步到 Master；定期進行 PDelay 測量 |

### 4.2 訊息分派機制（handle）

`handle()` 函式（`protocol.c`）在每個主迴圈中被呼叫，根據 `netSelect()` 的結果決定接收哪種訊息：

```c
void handle(PtpClock *ptpClock)
{
    // 檢查是否有待處理訊息（非阻塞）
    if (!netSelect(&ptpClock->netPath, &timeout))
        return;

    // 優先處理 Event 訊息（含時間戳，較時間敏感）
    if (netRecvEvent(&ptpClock->netPath, ptpClock->msgIbuf, &time)) {
        msgUnpackHeader(ptpClock->msgIbuf, &ptpClock->msgTmpHeader);

        switch (ptpClock->msgTmpHeader.messageType) {
            case SYNC:           handleSync(ptpClock, &time, FALSE);       break;
            case DELAY_REQ:      handleDelayReq(ptpClock, &time, FALSE);   break;
            case PDELAY_REQ:     handlePDelayReq(ptpClock, &time, FALSE);  break;
            case PDELAY_RESP:    handlePDelayResp(ptpClock, &time, FALSE); break;
        }
    }

    // 處理 General 訊息
    if (netRecvGeneral(&ptpClock->netPath, ptpClock->msgIbuf, &time)) {
        switch (ptpClock->msgTmpHeader.messageType) {
            case FOLLOW_UP:              handleFollowUp(ptpClock, FALSE);            break;
            case DELAY_RESP:             handleDelayResp(ptpClock, FALSE);           break;
            case PDELAY_RESP_FOLLOW_UP:  handlePDelayRespFollowUp(ptpClock, FALSE); break;
            case ANNOUNCE:               handleAnnounce(ptpClock, FALSE);           break;
            case MANAGEMENT:             handleManagement(ptpClock, FALSE);         break;
        }
    }
}
```

### 4.3 BMCA 在 802.1AS 下的簡化實作

#### 4.3.1 演算法入口：`bmc()`

```c
// bmc.c:425
UInteger8 bmc(PtpClock *ptpClock)
{
    // 從所有已知的 Foreign Master 記錄中選出最優者
    for (i = 1, best = 0; i < ptpClock->foreignMasterDS.count; i++) {
        if (bmcDataSetComparison(...records[i]..., ...records[best]...) < 0)
            best = i;
    }
    ptpClock->foreignMasterDS.best = best;

    // 根據最優 Foreign Master 決定本機狀態
    return bmcStateDecision(
        &records[best].header, &records[best].announce, ptpClock);
}
```

#### 4.3.2 數據集比較優先序（IEEE 1588-2008 Figure 27-28）

`bmcDataSetComparison()` 依序比較以下欄位（值越小越優先）：

```
優先序  欄位                              說明
  1.    grandmasterPriority1              手動設定的主要優先級（本專案預設 128）
  2.    grandmasterClockQuality.clockClass 時鐘等級（128 = 可為 Master）
  3.    grandmasterClockQuality.clockAccuracy 時鐘精度等級
  4.    grandmasterClockQuality.offsetScaledLogVariance 穩定度指標
  5.    grandmasterPriority2              次要優先級（本專案預設 128）
  6.    grandmasterIdentity               8 字節 EUI-64 時鐘身份（二進位比較）

若 Grandmaster 相同（同一個 Grandmaster 透過多路徑傳播）：
  7.    stepsRemoved                      拓撲距離（越近越優）
  8.    sourcePortIdentity                Port Identity 比較（決定拓撲關係）
```

#### 4.3.3 狀態決定：`bmcStateDecision()`

```c
UInteger8 bmcStateDecision(MsgHeader *header, MsgAnnounce *announce, PtpClock *ptpClock)
{
    // 比較本機（D0）與最優 Foreign Master
    int comp = bmcDataSetComparison(self_header, self_announce, header, announce, ptpClock);

    if (ptpClock->defaultDS.clockQuality.clockClass < 128) {
        // 高品質時鐘（GPS 等）：只能是 Master 或 Passive
        return (comp < 0) ? PTP_MASTER : PTP_PASSIVE;  // M1 or P1
    } else {
        // 一般時鐘：可以是 Master 或 Slave
        return (comp < 0) ? PTP_MASTER : PTP_SLAVE;    // M2 or S1
    }
}
```

#### 4.3.4 gPTP 中 BMCA 的簡化特性

IEEE 802.1AS 中 BMCA 的特殊之處：

1. **P2P 強制**：不需要考慮 E2E 拓撲中的 `stepsRemoved` 路由問題
2. **Announce 訊息**：在 gPTP 中仍然存在，但其角色相對 IEEE 1588 更簡單
3. **單一埠**：本專案 `NUMBER_PORTS = 1`，無需複雜的埠間 BMCA 協調
4. **快速收斂**：`ANNOUNCE_RECEIPT_TIMEOUT = 3`（3 個 Announce 週期），共約 3 秒

### 4.4 計時器驅動機制

#### 4.4.1 六個 PTP 計時器

```c
// constants.h:216-225
enum {
    PDELAYREQ_INTERVAL_TIMER = 0,  // P2P 模式：觸發 issuePDelayReq()
    DELAYREQ_INTERVAL_TIMER,       // E2E 模式：觸發 issueDelayReq()
    SYNC_INTERVAL_TIMER,           // Master：觸發 issueSync()
    ANNOUNCE_RECEIPT_TIMER,        // Slave：超時則清空 Foreign Masters
    ANNOUNCE_INTERVAL_TIMER,       // Master：觸發 issueAnnounce()
    QUALIFICATION_TIMEOUT,         // PRE_MASTER → MASTER 的等待計時器
    TIMER_ARRAY_SIZE = 6
};
```

| 計時器 | 觸發動作 | gPTP 預設間隔 |
|--------|---------|--------------|
| `PDELAYREQ_INTERVAL_TIMER` | `issuePDelayReq()` | 250ms（log=−2） |
| `DELAYREQ_INTERVAL_TIMER` | `issueDelayReq()`（E2E 模式用） | 不適用 |
| `SYNC_INTERVAL_TIMER` | `issueSync()` + `issueFollowup()` | 125ms（log=−3） |
| `ANNOUNCE_RECEIPT_TIMER` | Master 選取或故障恢復 | 3s（3 × 1s） |
| `ANNOUNCE_INTERVAL_TIMER` | `issueAnnounce()` | 1s（log=0） |
| `QUALIFICATION_TIMEOUT` | PRE_MASTER → MASTER 狀態轉換 | 約 0.5s |

#### 4.4.2 計時器工作原理

```
硬體計時器 ISR
    │
    ▼
catchAlarm(time_ms)          // timer.c:7
    │ elapsed_ms += time_ms  // 累積經過時間（毫秒）
    │
    ▼
timerExpired(index, itimer)  // 在 doState() 或 handle() 中呼叫
    │
    ▼
timerUpdate(itimer)          // timer.c:21
    │ delta = elapsed_ms; elapsed_ms = 0;
    │ for each timer:
    │   itimer[i].left -= delta
    │   if (left <= 0): left = interval; expire = TRUE
    │
    ▼
if (itimer[index].expire):   // expire 旗標由 timerExpired() 清除
    expire = FALSE
    return TRUE              // 觸發對應動作
```

```c
// timer.c 完整實作
unsigned int elapsed_ms;  // 全域累積器（由 ISR 中的 catchAlarm() 寫入）

void catchAlarm(UInteger32 time_ms) {
    elapsed_ms += time_ms;  // 在硬體計時器 ISR 中被呼叫
}

void timerUpdate(IntervalTimer *itimer) {
    int delta = elapsed_ms;
    elapsed_ms = 0;

    for (int i = 0; i < TIMER_ARRAY_SIZE; i++) {
        if (itimer[i].interval > 0 && (itimer[i].left -= delta) <= 0) {
            itimer[i].left = itimer[i].interval;  // 自動重載
            itimer[i].expire = TRUE;
        }
    }
}

Boolean timerExpired(UInteger16 index, IntervalTimer *itimer) {
    timerUpdate(itimer);       // 每次查詢都先更新
    if (!itimer[index].expire) return FALSE;
    itimer[index].expire = FALSE;
    return TRUE;
}
```

---

## 5. 關鍵資料結構

### 5.1 `TimeInternal`（內部時間表示）

```c
// datatypes.h:239-243
typedef struct {
    Integer32 seconds;      // 整數秒（有符號，可表示負偏移）
    Integer32 nanoseconds;  // 納秒部分（0~999,999,999 或負值表示負數時間）
} TimeInternal;
```

> 與 `Timestamp` 的差異：`TimeInternal` 用於內部計算（有符號），`Timestamp` 是 PTP 封包中的欄位（無符號 48 bits 秒）。

### 5.2 `Timestamp`（PTP 封包時間戳）

```c
// datatypes.h:27-31
typedef struct {
    UInteger48 secondsField;    // 48 位元無符號整數秒（IEEE 1588 epoch）
    UInteger32 nanosecondsField; // 32 位元納秒
} Timestamp;
```

轉換函式：
```c
void fromInternalTime(const TimeInternal *internal, Timestamp *external);  // 內部 → 封包
void toInternalTime(TimeInternal *internal, const Timestamp *external);    // 封包 → 內部
```

### 5.3 `MsgHeader`（PTP 訊息通用標頭）

```c
// datatypes.h:111-124
typedef struct {
    Nibble      transportSpecific;  // 高 4 bits：gPTP 固定為 0x1（SdoId）
    Enumeration4 messageType;       // 低 4 bits：訊息類型（0x0~0xD）
    UInteger4   versionPTP;         // PTP 版本（固定 2）
    UInteger16  messageLength;      // 整個 PTP 訊息長度（bytes）
    UInteger8   domainNumber;       // PTP 域號（預設 0）
    Octet       flagField[2];       // 標誌位元組
                                    //   [0]: TWO_STEP(bit1), UNICAST(bit2)...
                                    //   [1]: LEAP61, PTP_TIMESCALE...
    Integer64   correctionfield;    // Correction Field（scaled nanoseconds = ns × 2^16）
    PortIdentity sourcePortIdentity; // 來源埠身份（Clock Identity 8B + Port Number 2B）
    UInteger16  sequenceId;         // 訊息序列號（每種訊息類型獨立計數）
    UInteger8   controlField;       // 控制欄位（舊版相容，PTPv2 不常用）
    Integer8    logMessageInterval; // 訊息間隔（log2 秒）
} MsgHeader;
```

### 5.4 `PtpClock`（主程式狀態結構體）

```c
// datatypes.h:422-507（精選關鍵欄位）
typedef struct {
    /* === 五大 IEEE 1588 Data Sets === */
    DefaultDS        defaultDS;       // 本機時鐘特性（clockClass, priority1, priority2...）
    CurrentDS        currentDS;       // 即時狀態（offsetFromMaster, stepsRemoved）
    ParentDS         parentDS;        // 當前 Master 資訊
    TimePropertiesDS timePropertiesDS;// 時間屬性（UTC offset, timescale...）
    PortDS           portDS;          // 埠狀態（portState, peerMeanPathDelay, delayMechanism）
    ForeignMasterDS  foreignMasterDS; // 已知 Foreign Master 列表（BMCA 用）

    /* === 訊息緩衝區 === */
    MsgHeader  msgTmpHeader;          // 接收訊息的 Header（解包後存於此）
    union { MsgSync sync; MsgFollowUp follow; MsgPDelayReq preq;
            MsgPDelayResp presp; MsgPDelayRespFollowUp prespfollow;
            MsgAnnounce announce; ... } msgTmp;  // 接收訊息的 Body
    Octet      msgObuf[PACKET_SIZE];  // 發送緩衝區（300 bytes）
    Octet      msgIbuf[PACKET_SIZE];  // 接收緩衝區
    ssize_t    msgIbufLength;         // 接收訊息長度

    /* === 時間戳與狀態 === */
    TimeInternal Tms;                 // T2 - T1（Master→Slave 方向時間差）
    TimeInternal Tsm;                 // T4 - T3（Slave→Master 方向時間差）
    TimeInternal pdelay_t1;           // P2P T1：PDelayReq TX 時間
    TimeInternal pdelay_t2;           // P2P T2：PDelayReq RX 時間（Responder 端）
    TimeInternal pdelay_t3;           // P2P T3：PDelayResp TX 時間（Responder 端）
    TimeInternal pdelay_t4;           // P2P T4：PDelayResp RX 時間（Requester 端）
    TimeInternal timestamp_syncRecieve;      // Sync 接收時間（= T2 in Sync flow）
    TimeInternal timestamp_delayReqSend;     // E2E DelayReq TX 時間（= T3 in E2E）
    TimeInternal timestamp_delayReqRecieve;  // E2E DelayReq RX 時間
    TimeInternal correctionField_sync;       // 已接收 Sync 的 CF（等待 FollowUp 時暫存）
    TimeInternal correctionField_pDelayResp; // PDelayResp 的 CF（等待 FollowUp 時暫存）

    /* === 序列號計數器 === */
    UInteger16 sentPDelayReqSequenceId;  // 已送出的 PDelayReq 計數
    UInteger16 sentDelayReqSequenceId;
    UInteger16 sentSyncSequenceId;
    UInteger16 sentAnnounceSequenceId;
    UInteger16 recvPDelayReqSequenceId;  // 已接收的 PDelayReq sequenceId（Responder 端）
    UInteger16 recvSyncSequenceId;       // 最後接收到的 Sync sequenceId

    /* === 等待旗標 === */
    Boolean waitingForFollowUp;          // 已收到 Sync（TWO_STEP），等待 FollowUp
    Boolean waitingForPDelayRespFollowUp;// 已收到 PDelayResp（TWO_STEP），等待 FollowUp

    /* === 濾波器 === */
    Filter  ofm_filt;   // Offset From Master 濾波器（alpha = 1/2^s，s=DEFAULT_OFFSET_S=1）
    Filter  owd_filt;   // One Way Delay 濾波器（s=DEFAULT_DELAY_S=6）
    Filter  slv_filt;   // Scaled Log Variance 濾波器

    /* === 計時器 === */
    IntervalTimer itimer[TIMER_ARRAY_SIZE];  // 六個軟體計時器

    /* === 其他 === */
    NetPath  netPath;        // 網路路徑（含收發隊列）
    Servo    servo;          // PI 控制器參數
    Integer32 events;        // 待處理事件旗標（POWERUP, STATE_DECISION_EVENT...）
    RunTimeOpts *rtOpts;     // 指向執行時配置
} PtpClock;
```

### 5.5 `RunTimeOpts`（執行時配置）

```c
// datatypes.h:396-414
typedef struct {
    Integer8      announceInterval;   // log2 秒（gPTP 建議 0 = 1s）
    Integer8      syncInterval;       // log2 秒（gPTP 建議 -3 = 125ms）
    ClockQuality  clockQuality;       // 本機時鐘品質（clockClass, clockAccuracy, variance）
    UInteger8     priority1;          // BMCA 主要優先級（0~255，越小越優，本專案預設 128）
    UInteger8     priority2;          // BMCA 次要優先級（本專案預設 128）
    UInteger8     domainNumber;       // PTP 域號（預設 0）
    Boolean       slaveOnly;          // TRUE = 只能作 Slave；FALSE = 可以競爭 Master
    Integer16     currentUtcOffset;   // UTC 偏移量（TAI - UTC，目前約 37 秒）
    Octet         ifaceName[IFACE_NAME_LENGTH]; // 網路介面名稱
    TimeInternal  inboundLatency, outboundLatency; // 收發延遲補償
    Integer16     maxForeignRecords;  // Foreign Master 最大記錄數（預設 5）
    Enumeration8  delayMechanism;     // E2E=1 或 P2P=2（gPTP 強制 P2P）
    Enumeration8  transportType;      // 傳輸類型（1=UDP/IPv4, 3=802.3, 4=802.1AS）
    Servo         servo;              // PI 控制器參數（ap, ai, sDelay, sOffset）
} RunTimeOpts;
```

### 5.6 `NetPath`（網路路徑）

```c
// datatypes_dep.h（示意）
typedef struct {
    Integer32     multicastAddr;      // 一般 PTP 多播 IP（UDP 模式用）
    Integer32     peerMulticastAddr;  // P2P PTP 多播 IP（UDP 模式用）
    Integer32     unicastAddr;        // 單播 IP（可選）
    struct udp_pcb *eventPcb;         // UDP Event 埠 PCB（L2 模式為 NULL）
    struct udp_pcb *generalPcb;       // UDP General 埠 PCB（L2 模式為 NULL）
    BufQueue      eventQ;             // Event 訊息環形隊列（SYNC, PDELAY_REQ...）
    BufQueue      generalQ;           // General 訊息環形隊列（ANNOUNCE, FOLLOWUP...）
} NetPath;
```

> 判斷是否為 L2 模式：`netPath->eventPcb == NULL && netPath->generalPcb == NULL`

### 5.7 `IntervalTimer`（軟體計時器）

```c
// datatypes.h:249-254
typedef struct {
    Integer32 interval;  // 計時週期（毫秒）；0 表示計時器已停止
    Integer32 left;      // 剩餘時間（毫秒）；倒計時
    Boolean   expire;    // 超期旗標（TRUE = 已到期，讀取後自動清除）
} IntervalTimer;
```

### 5.8 `Servo`（PI 伺服控制器參數）

```c
// datatypes.h:382-389
typedef struct {
    Boolean noResetClock;  // TRUE = 首次 SLAVE 時不強制跳變時鐘
    Boolean noAdjust;      // TRUE = 只計算，不實際調整時鐘（除錯用）
    Integer16 ap, ai;      // 比例係數 ap（預設 2）、積分係數 ai（預設 16）
    Integer16 sDelay;      // 路徑延遲濾波器階數（預設 6，alpha=1/64）
    Integer16 sOffset;     // 偏移濾波器階數（預設 1，alpha=1/2）
} Servo;
```

### 5.9 `Filter`（指數平滑濾波器）

```c
// datatypes_dep.h
typedef struct {
    Integer32 y_prev;   // 上次濾波輸出
    Integer32 y_sum;    // 累積和（內部狀態）
    Integer16 s;        // 當前階數（alpha = 1/2^s）
    Integer16 s_prev;   // 上次階數（用於調整 y_sum）
    Integer32 n;        // 已處理樣本數（用於初始暖機）
} Filter;
```

### 5.10 `BufQueue`（環形封包隊列）

```c
// datatypes_dep.h
typedef struct {
    void    *pbuf[PBUF_QUEUE_SIZE]; // 指標陣列（PBUF_QUEUE_SIZE=16）
    Integer32 get;                  // 出隊指標
    Integer32 put;                  // 入隊指標
    Integer32 count;                // 當前隊列中的元素數
} BufQueue;
```

### 5.11 `ptptime_t`（DM9058 HAL 時間結構）

```c
// dm9058_ptp.h:54-57
struct ptptime_t {
    s32_t tv_sec;   // 秒（有符號 32 位元）
    s32_t tv_nsec;  // 納秒（有符號 32 位元）
};
```

---

## 6. 移植關鍵點（Porting Guide）

### 6.1 HAL：MAC 層 Timestamping 介面

移植 gPTP 的核心挑戰在於**硬體時間戳**。必須實作以下四個函式：

```c
// dep/sys_time.c 中需要實作的介面
void getTime(TimeInternal *time);          // 讀取目前硬體 PTP 時鐘值
void setTime(const TimeInternal *time);    // 跳變設定 PTP 時鐘（粗調）
void updateTime(const TimeInternal *time); // 偏移更新（細調，time 為負值偏移量）
Boolean adjFreq(Integer32 adj);            // 頻率調整（單位：ppb，±ADJ_FREQ_MAX=512000）
```

#### 本專案（DM9058）的實作方式

```c
// dep/sys_time.c（本專案實作）
void getTime(TimeInternal *time) {
#if (EDRIVER_ADDING_PTP && LWIP_PTP)
    struct ptptime_t timestamp;
    dm9058_ptptime_gettime(&timestamp);  // 讀取 DM9058 PTP 時間暫存器
    time->seconds     = timestamp.tv_sec;
    time->nanoseconds = timestamp.tv_nsec;
#endif
}

void setTime(const TimeInternal *time) {
#if (EDRIVER_ADDING_PTP && LWIP_PTP)
    struct ptptime_t ts = { time->seconds, time->nanoseconds };
    dm9058_ptptime_settime(&ts);
#endif
}

void updateTime(const TimeInternal *time) {
#if (EDRIVER_ADDING_PTP && LWIP_PTP)
    // 注意：updateoffset 接受的是偏移量的「負值」
    struct ptptime_t offset = { -time->seconds, -time->nanoseconds };
    dm9058_ptptime_updateoffset(&offset);
#endif
}

Boolean adjFreq(Integer32 adj) {
    if (adj > ADJ_FREQ_MAX)  adj = ADJ_FREQ_MAX;
    if (adj < -ADJ_FREQ_MAX) adj = -ADJ_FREQ_MAX;
#if (EDRIVER_ADDING_PTP && LWIP_PTP)
    dm9058_ptptime_adjfreq(adj);  // adj 單位：ppb（parts per billion）
#endif
    return TRUE;
}
```

#### 硬體時間戳嵌入位置

ptpd 期望在呼叫 `netSendL2()` 後，能從 `pbuf` 結構中讀到精確的 TX 時間戳：

```c
// dep/net.c:178-185（TX 時間戳讀取）
#if LWIP_PTP
    time->seconds     = p->time_sec;   // lwIP pbuf 擴展欄位
    time->nanoseconds = p->time_nsec;
    if (time->seconds == 0 && time->nanoseconds == 0)
        getTime(time);  // 硬體未提供時降級為軟體時間戳
#else
    getTime(time);
#endif
```

**接收端 RX 時間戳**由 `netRecvL2Callback()` 在 pbuf 送達時同步讀取，同樣儲存在 `pbuf->time_sec` / `pbuf->time_nsec`。

#### ESP32 移植建議（Timestamping）

ESP32 的 EMAC 支援硬體 TX/RX 時間戳（ESP-IDF v5.x）：

```c
// ESP-IDF 硬體時間戳讀取（示意）
esp_eth_ioctl(eth_handle, ETH_CMD_PTP_GET_TIME, &ptp_time);
esp_eth_ioctl(eth_handle, ETH_CMD_PTP_SET_TIME, &ptp_time);
esp_eth_ioctl(eth_handle, ETH_CMD_PTP_ADJ_FREQ, &adj_ppb);
```

SPI 乙太網控制器依晶片而異，分兩種情況：

- **DM9058（本專案）**：**支援硬體時間戳**，透過 SPI 讀取晶片內部的 PTP 時間戳暫存器（Reg 0x65/0x66 配置偏移，TX/RX 時間戳由晶片在媒體存取層自動捕獲，完成後透過 SPI 讀回）。
- **W5500 / ENC28J60 等簡易 SPI MAC**：**不支援硬體時間戳**，只能在 SPI 傳輸完成後立即讀取 MCU 系統計數器（`esp_timer_get_time()`）作為軟體時間戳，精度約 10~100 µs（不適合亞微秒應用）。

因此「SPI 介面」本身不是決定硬體時間戳能力的因素；**關鍵在於 MAC 晶片本身是否內建 PTP 時間戳引擎**。DM9058 屬於具備完整 PTP 硬體支援的 SPI MAC，可在 ESP32 等平台上透過 SPI 匯流排存取其時間戳暫存器。

---

### 6.2 網路層：L2 Raw Ethernet 繞過 L3

#### 6.2.1 L2 接收：注冊 EtherType 過濾回調

```c
// 本專案的做法（lwIP netconf 層）：
netconf_register_ptp_callback(netRecvL2Callback);
```

`netconf_register_ptp_callback()` 是本專案自定義的函式，在 lwIP 的 `ethernetif_input()` 中，當 EtherType 為 0x88F7 時呼叫此回調，**而不是**丟給 IP 層處理。

**ESP-IDF 對應實作**：

```c
// ESP-IDF 方式 1：使用 esp_eth 的 raw frame 接收 hook
static esp_err_t ptp_eth_recv_hook(esp_eth_handle_t hdl, esp_eth_mediator_t *eth,
                                    uint8_t *buffer, uint32_t length)
{
    // 讀取 EtherType（位於 offset 12-13）
    uint16_t eth_type = (buffer[12] << 8) | buffer[13];
    if (eth_type == 0x88F7) {
        // 複製到 PTP 隊列
        ptp_enqueue(buffer + 14, length - 14, rx_timestamp);
        return ESP_OK;  // 已消費，不繼續傳給 IP 層
    }
    return ESP_ERR_NOT_FOUND;  // 繼續交給 IP 層
}

// 或使用 esp_netif 的 packet filter
esp_netif_receive_ptp_frame(netif, frame_hook);
```

**ESP-IDF 方式 2（FreeRTOS Queue）**：
```c
// 創建接收隊列
QueueHandle_t g_ptp_rx_queue = xQueueCreate(16, sizeof(ptp_frame_t));

// 在 ETH RX callback 中入隊
void eth_event_handler(void *arg, esp_event_base_t event_base,
                       int32_t event_id, void *event_data) {
    if (event_base == ETH_EVENT && event_id == ETHERNET_EVENT_RECEIVE) {
        ptp_frame_t frame;
        memcpy(frame.data, data, len);
        frame.timestamp = get_ptp_time();
        xQueueSendFromISR(g_ptp_rx_queue, &frame, NULL);
    }
}
```

#### 6.2.2 L2 發送：構建 Ethernet Frame

```c
// 直接構建 14 bytes Ethernet Header + PTP Payload
uint8_t frame[14 + PACKET_SIZE];

// 目標 MAC（gPTP 多播）
memcpy(frame + 0, "\x01\x80\xC2\x00\x00\x0E", 6);

// 來源 MAC（本機 MAC）
esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, frame + 6);

// EtherType = 0x88F7
frame[12] = 0x88;
frame[13] = 0xF7;

// PTP 訊息本體
memcpy(frame + 14, ptp_payload, ptp_length);

// 發送（以下是 ESP-IDF 方式）
esp_eth_transmit(eth_handle, frame, 14 + ptp_length);

// 發送完成後讀取 TX 時間戳
esp_eth_ioctl(eth_handle, ETH_CMD_PTP_GET_TX_TIMESTAMP, &tx_ts);
```

#### 6.2.3 L2 模式初始化

```c
// 本專案的 netInitL2()（dep/net.c:104-131）關鍵步驟：
netPath->eventPcb   = NULL;  // 標記為 L2 模式
netPath->generalPcb = NULL;
netQInit(&netPath->eventQ);  // 初始化接收隊列
netQInit(&netPath->generalQ);
g_ptp_netpath_l2 = netPath;
netconf_register_ptp_callback(netRecvL2Callback);
```

**移植要點**：將 `netconf_register_ptp_callback()` 替換為目標平台的 EtherType 截取機制。

---

### 6.3 定時器與系統時鐘

#### 6.3.1 軟體計時器接線

本專案的計時器需要外部 ISR 定期呼叫 `catchAlarm()`：

```c
// 需要在硬體計時器 ISR 中插入：
void TMR6_IRQHandler(void) {  // AT32F403A 定時器中斷
    catchAlarm(1);  // 每次 ISR 代表 1ms 已過去
    // 或：catchAlarm(TICK_MS);  // 若 tick 非 1ms
}
```

**ESP-IDF 對應（FreeRTOS 計時器）**：

```c
// 方式 1：使用 esp_timer（高精度）
static esp_timer_handle_t g_ptp_timer;

void ptp_timer_callback(void *arg) {
    catchAlarm(1);  // 每 1ms 呼叫一次
}

esp_timer_create_args_t timer_args = {
    .callback        = ptp_timer_callback,
    .name            = "ptp_tick",
    .dispatch_method = ESP_TIMER_TASK,
};
esp_timer_create(&timer_args, &g_ptp_timer);
esp_timer_start_periodic(g_ptp_timer, 1000);  // 1000 µs = 1ms
```

```c
// 方式 2：使用 FreeRTOS 軟體計時器
TimerHandle_t ptp_sw_timer = xTimerCreate(
    "ptp_tick",
    pdMS_TO_TICKS(1),      // 1ms period
    pdTRUE,                // auto-reload
    NULL,
    (TimerCallbackFunction_t)ptp_timer_callback
);
xTimerStart(ptp_sw_timer, 0);
```

#### 6.3.2 主迴圈結構

本專案（無 RTOS，`NO_SYS=1`）的主迴圈：

```c
// 本專案的輪詢模式
void ptpd_Periodic_Handle(__IO UInteger32 localtime) {
    doState(&ptpClock);   // 執行狀態機（含計時器觸發）
    handle(&ptpClock);    // 接收並處理訊息
}
```

**ESP-IDF 移植（FreeRTOS 任務）**：

```c
void ptp_task(void *pvParameters) {
    PtpClock ptpClock;
    RunTimeOpts rtOpts;

    // 初始化
    ptpd_init(&ptpClock, &rtOpts);

    while (1) {
        doState(&ptpClock);   // 狀態機
        handle(&ptpClock);    // 訊息處理

        // 適度讓出（避免 watchdog）
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// 建立任務
xTaskCreate(ptp_task, "ptp", 8192, NULL, 10, NULL);
```

> **注意**：若使用 FreeRTOS，`NO_SYS` 應改為 0，相關 lwIP 互斥鎖需要正確配置。

---

### 6.4 必須開啟的編譯巨集

#### 6.4.1 ptpd 協議層巨集

```c
// constants.h
#define PTP_NETWORK_TRANSPORT  TRANSPORT_IEEE_802_1AS  // = 4，強制 gPTP L2 模式
// DEFAULT_DELAY_MECHANISM 會在 ptpd.c 初始化時被強制設為 P2P

// gPTP 時序參數（可透過編譯器 -D 覆蓋）
#define DEFAULT_SYNC_INTERVAL_8021AS        -3   // 2^-3 = 125ms
#define DEFAULT_PDELAYREQ_INTERVAL_8021AS   -2   // 2^-2 = 250ms
#define DEFAULT_ANNOUNCE_INTERVAL_8021AS     0   // 2^0  = 1s
#define DEFAULT_ANNOUNCE_RECEIPT_TIMEOUT_8021AS  3  // 3 個 Announce 週期超時

// ONE_STEP 或 TWO_STEP（視 MAC 硬體能力）
#define DEFAULT_TWO_STEP_FLAG  FALSE   // ONE_STEP（MAC 自動填入 TX 時間戳）
// #define DEFAULT_TWO_STEP_FLAG  TRUE  // TWO_STEP（軟體讀取 TX 時間戳後發 FollowUp）

// 時鐘品質（影響 BMCA 競選結果）
#define DEFAULT_CLOCK_CLASS     128    // 允許 Master 模式（< 128 = 高品質，不允許 Slave）
#define DEFAULT_PRIORITY1       128    // 0=最高優先，255=最低
#define DEFAULT_PRIORITY2       128
#define SLAVE_ONLY              FALSE  // FALSE = 可以競爭 Master
```

#### 6.4.2 平台 HAL 層巨集

```c
// lwipopts.h 或平台配置
#define EDRIVER_ADDING_PTP  1   // 啟用硬體 PTP 時間戳驅動（控制 sys_time.c 中的條件編譯）
#define LWIP_PTP            1   // 啟用 lwIP pbuf 中的 time_sec / time_nsec 擴展欄位
#define NO_SYS              1   // 無 RTOS 輪詢模式（若移植到 FreeRTOS 需改為 0）
```

#### 6.4.3 DM9058 傳輸模式自動配置（`dm9058_ptp.c`）

```c
// dm9058_ptp.c 中的傳輸模式切換
#if (PTP_NETWORK_TRANSPORT == TRANSPORT_IEEE_802_1AS)
    ptp_offset_addr   = 0x32;  // gPTP/L2 模式下 PTP 訊息在 Ethernet frame 中的偏移
    ptp_checksum_addr = 0x20;  // Checksum 欄位偏移
#elif (PTP_NETWORK_TRANSPORT == TRANSPORT_UDP_IPV4)
    ptp_offset_addr   = 0x4E;  // UDP/IP 模式（含 IP + UDP 標頭）
    ptp_checksum_addr = 0x3C;
#endif

// 寫入 DM9058 PTP 時間戳偏移暫存器
HAL_write_reg(0x65, ptp_offset_addr);
HAL_write_reg(0x66, ptp_checksum_addr);
```

---

### 6.5 ESP-IDF 移植具體建議

#### 6.5.1 硬體能力評估

| 功能 | ESP32 EMAC（有線） | SPI MAC + PTP 晶片（如 DM9058） | 簡易 SPI MAC（W5500 等） | 備註 |
|------|------------------|--------------------------------|------------------------|------|
| 硬體 TX 時間戳 | ✓（ESP-IDF v5.x） | ✓（SPI 讀取晶片暫存器） | ✗ | 決定精度上限 |
| 硬體 RX 時間戳 | ✓ | ✓（SPI 讀取晶片暫存器） | ✗ | 需在 RX 完成後立即讀取 |
| 頻率調整 | ✓（EMAC PTPCLK） | ✓（DM9058 Addend 暫存器） | ✗ | 替代方案：調整系統計數器 |
| 精度上限 | ~100 ns | ~100 ns ~ 1 µs（含 SPI 延遲）| ~10-50 µs（軟體時間戳）| 視 PHY 和 cable |

> **DM9058 說明**：DM9058 透過 SPI 匯流排將 PTP 時間戳引擎的能力暴露給 MCU，TX 時間戳在 MAC 層媒體存取完成後自動鎖存於晶片暫存器，RX 時間戳在幀接收完成時同步捕獲。SPI 讀取操作本身引入約 1~5 µs 的額外延遲，仍遠優於純軟體時間戳方案。

#### 6.5.2 最小移植工作清單

```
移植工作               對應本專案的原始碼              ESP-IDF 替代方案
─────────────────────────────────────────────────────────────────────
1. getTime()          dep/sys_time.c               esp_eth_ioctl(GET_TIME)
                      → dm9058_ptptime_gettime()    或 esp_timer_get_time()

2. setTime()          dep/sys_time.c               esp_eth_ioctl(SET_TIME)
                      → dm9058_ptptime_settime()

3. updateTime()       dep/sys_time.c               esp_eth_ioctl(ADJ_OFFSET)
                      → dm9058_ptptime_updateoffset()

4. adjFreq()          dep/sys_time.c               esp_eth_ioctl(ADJ_FREQ)
                      → dm9058_ptptime_adjfreq()

5. L2 TX              dep/net.c: netSendL2()        esp_eth_transmit()
                      → iface->linkoutput()

6. L2 RX callback     dep/net.c: netRecvL2Callback  esp_eth RX hook 或
                      → netconf_register_ptp_callback  esp_event ETH_EVENT_RECEIVE

7. 計時器 ISR         dep/timer.c: catchAlarm()     esp_timer 或 FreeRTOS timer

8. TX 時間戳讀取      dep/net.c: pbuf->time_sec     【DM9058】dm9058_ptptime_gettime()
                      （lwIP pbuf 擴展欄位）         透過 SPI 讀取 DM9058 TX TS 暫存器
                                                    【ESP32 EMAC】esp_eth_ioctl(GET_TX_TS)
                                                    【無硬體 TS】esp_timer_get_time()（降級）

9. RX 時間戳讀取      netRecvL2Callback 中          【DM9058】在 SPI RX 完成後立即讀取
                      pbuf->time_sec                dm9058 RX TS 暫存器（dm9058_rx_ptp()）
                                                    【ESP32 EMAC】esp_eth_ioctl(GET_RX_TS)
```

#### 6.5.3 DM9058 移植至 ESP32 的時間戳接線方式

DM9058 透過 SPI 暴露完整的 PTP 時間戳引擎，移植至 ESP32 時，`dep/sys_time.c` 的四個函式均可對應至 DM9058 的 SPI 暫存器操作：

```c
// dep/sys_time.c（ESP32 + DM9058 SPI 移植版）
void getTime(TimeInternal *time) {
    struct ptptime_t ts;
    dm9058_ptptime_gettime(&ts);   // 透過 SPI 讀取 DM9058 PTP 時間暫存器
    time->seconds     = ts.tv_sec;
    time->nanoseconds = ts.tv_nsec;
}

void setTime(const TimeInternal *time) {
    struct ptptime_t ts = { time->seconds, time->nanoseconds };
    dm9058_ptptime_settime(&ts);   // 透過 SPI 寫入 DM9058 PTP 時間暫存器（跳變）
}

void updateTime(const TimeInternal *time) {
    // 注意：傳入的 time 為需補償的偏移量，updateoffset 接受其負值
    struct ptptime_t offset = { -time->seconds, -time->nanoseconds };
    dm9058_ptptime_updateoffset(&offset);  // 細粒度偏移調整（加減法方式）
}

Boolean adjFreq(Integer32 adj) {
    if (adj >  ADJ_FREQ_MAX) adj =  ADJ_FREQ_MAX;
    if (adj < -ADJ_FREQ_MAX) adj = -ADJ_FREQ_MAX;
    dm9058_ptptime_adjfreq(adj);   // 透過 SPI 調整 DM9058 Addend 暫存器（ppb 單位）
    return TRUE;
}
```

`dm9058_ptptime_gettime()` 等函式的底層實作（`dm9058_ptp.c`）透過 `HAL_read_reg()` / `HAL_write_reg()` 操作 SPI，移植至 ESP32 時只需將 `HAL_read_reg` / `HAL_write_reg` 對應至 ESP-IDF 的 `spi_device_transmit()` 即可，其餘邏輯不需修改。

#### 6.5.4 無 PTP 硬體支援的純軟體降級策略

若目標 SPI MAC（如 W5500、ENC28J60）**不含 PTP 時間戳引擎**，只能以 MCU 系統計數器降級：

```c
// 軟體時間戳（精度有限，僅供參考）
void getTime(TimeInternal *time) {
    // ESP-IDF：esp_timer_get_time() 返回微秒（自啟動後累積）
    int64_t us = esp_timer_get_time();
    time->seconds     = (int32_t)(us / 1000000LL);
    time->nanoseconds = (int32_t)((us % 1000000LL) * 1000);
}

void adjFreq(Integer32 adj) {
    // 軟體頻率調整：無精確硬體支援，可以 SNTP 定期校正
    // 或記錄漂移率後在 getTime() 中做線性補償（準確度有限）
    (void)adj;
}
```

> **精度說明**：純軟體時間戳的 gPTP 精度通常在 10~50 µs 範圍，無法達到 IEEE 802.1AS 要求的亞微秒等級。**若使用 DM9058（本專案方案），則無此限制**，硬體時間戳精度可達 100 ns ~ 1 µs（主要受 SPI 讀取延遲影響）。

#### 6.5.4 RTOS 整合注意事項

1. **任務優先級**：PTP 任務應設為較高優先級，避免時間戳讀取延遲
2. **互斥鎖**：若多個任務共用 `netPath`，需保護 `eventQ` / `generalQ` 的讀寫
3. **`NO_SYS` 設定**：移植到 FreeRTOS 時，lwIP 的 `NO_SYS` 應設為 0，並啟用 `LWIP_TCPIP_CORE_LOCKING`
4. **watchdog**：若 PTP 任務在計算密集時不讓出，需注意 watchdog 超時

---

## 7. 附錄 A：訊息流時序圖

### A.1 PDelay 交換（TWO_STEP 模式）

```
[Node A / Requester]                     [Node B / Responder]
        │                                         │
        │ t1 = HW_TX_timestamp                    │
        │──────── PDelayReq ─────────────────────►│
        │                                         │ t2 = HW_RX_timestamp
        │                                         │ t3 = HW_TX_timestamp
        │◄─────── PDelayResp(t2) ─────────────── │
        │         (requestReceiptTimestamp=t2)    │
        │ t4 = HW_RX_timestamp                    │
        │◄─────── PDelayRespFollowUp(t3) ──────── │
        │         (responseOriginTimestamp=t3)    │
        │                                         │
        │ 計算：                                   │
        │   peerMeanPathDelay                     │
        │   = ( (t2-t1) + (t4-t3) ) / 2 - CF     │
```

### A.2 Sync 同步（TWO_STEP 模式）

```
[Master]                                 [Slave]
   │                                        │
   │ T1 = HW_TX_timestamp                  │
   │──────────── Sync ──────────────────►  │
   │             (originTimestamp=0 或預估) │ T2 = HW_RX_timestamp
   │──────────── FollowUp ───────────────► │
   │             (preciseOriginTimestamp=T1)│
   │                                        │
   │                               計算：   │
   │                    offsetFromMaster    │
   │                    = (T2 - T1)         │
   │                      - (CF_sync        │
   │                         + CF_followup) │
   │                      - peerMeanPathDelay│
   │                                        │
   │                      updateClock()     │
   │                      （PI 控制器調整）  │
```

### A.3 Sync 同步（ONE_STEP 模式）

```
[Master]                                 [Slave]
   │                                        │
   │ T1 由 MAC 硬體自動嵌入 Sync 封包       │
   │──────────── Sync ──────────────────►  │
   │       (originTimestamp=T1，MAC填入)    │ T2 = HW_RX_timestamp
   │       無需 FollowUp                   │
   │                                        │
   │                               計算：   │
   │                    offsetFromMaster    │
   │                    = (T2 - T1) - CF   │
   │                      - peerMeanPathDelay│
```

### A.4 BMCA 完整流程

```
節點啟動
    │
    ▼
PTP_LISTENING
    │
    │ 接收 Announce 訊息（若有）
    │ → addForeign() 更新 Foreign Master 列表
    │
    │ STATE_DECISION_EVENT（ANNOUNCE_RECEIPT_TIMER 週期觸發）
    │        ↓
    │   bmc()
    │        ├─ bmcDataSetComparison()（比較本機 D0 vs 最優 Foreign Master）
    │        │   優先序：priority1 > clockClass > clockAccuracy > variance
    │        │            > priority2 > clockIdentity
    │        └─ bmcStateDecision()
    │                ├─ 本機更優 → PTP_PRE_MASTER → PTP_MASTER（m1()）
    │                ├─ Foreign 更優 → PTP_UNCALIBRATED → PTP_SLAVE（s1()）
    │                └─ 拓撲被動 → PTP_PASSIVE（p1()）
```

---

## 8. 附錄 B：關鍵函式速查表

| 函式名稱 | 所在檔案 | 功能說明 |
|---------|---------|---------|
| `doState()` | protocol.c | 主狀態機驅動（每次主迴圈呼叫） |
| `handle()` | protocol.c | 網路訊息接收與分派 |
| `toState()` | protocol.c | 狀態轉換（含清理/初始化工作） |
| `issueSync()` | protocol.c | Master：發送 Sync 訊息 |
| `issueFollowup()` | protocol.c | Master：發送 Follow_Up 訊息 |
| `issueAnnounce()` | protocol.c | Master：發送 Announce 訊息 |
| `handleSync()` | protocol.c | Slave：接收並處理 Sync |
| `handleFollowUp()` | protocol.c | Slave：接收並處理 Follow_Up |
| `handleAnnounce()` | protocol.c | 接收 Announce，更新 Foreign Master 列表 |
| `issuePDelayReq()` | protocol.c | Requester：發送 PDelay Request |
| `handlePDelayReq()` | protocol.c | Responder：接收 PDelay Request，發送 Response |
| `handlePDelayResp()` | protocol.c | Requester：接收 PDelay Response |
| `handlePDelayRespFollowUp()` | protocol.c | Requester：接收 PDelay FollowUp，計算延遲 |
| `issuePDelayResp()` | protocol.c | Responder：發送 PDelay Response（含 T2） |
| `issuePDelayRespFollowUp()` | protocol.c | Responder：發送 PDelay FollowUp（含 T3） |
| `updateOffset()` | dep/servo.c | 計算 `offsetFromMaster`（含濾波） |
| `updatePeerDelay()` | dep/servo.c | 計算 `peerMeanPathDelay`（含濾波） |
| `updateClock()` | dep/servo.c | PI 控制器：調用 `adjFreq()` / `setTime()` |
| `bmc()` | bmc.c | BMCA 主函式（選出最優 Master） |
| `bmcDataSetComparison()` | bmc.c | 比較兩個時鐘候選（返回負=A優，正=B優） |
| `bmcStateDecision()` | bmc.c | 根據 BMCA 結果決定本機狀態 |
| `m1()` / `s1()` / `p1()` | bmc.c | 進入 Master / Slave / Passive 時的初始化 |
| `msgPackSync()` | dep/msg.c | 打包 Sync 訊息到輸出緩衝區 |
| `msgUnpackHeader()` | dep/msg.c | 解包任何 PTP 訊息的通用標頭 |
| `netSendL2()` | dep/net.c | 組裝並發送 L2 Ethernet Frame |
| `netRecvL2Callback()` | dep/net.c | lwIP 的 L2 接收回調（EtherType 0x88F7 過濾） |
| `netInitL2()` | dep/net.c | 初始化 L2 模式網路（注冊回調，初始化隊列） |
| `netSelect()` | dep/net.c | 非阻塞檢查是否有待處理訊息 |
| `getTime()` | dep/sys_time.c | **【移植點】** 讀取硬體 PTP 時間 |
| `setTime()` | dep/sys_time.c | **【移植點】** 設定硬體 PTP 時間（粗調） |
| `updateTime()` | dep/sys_time.c | **【移植點】** 時間偏移更新（細調） |
| `adjFreq()` | dep/sys_time.c | **【移植點】** 頻率調整（ppb 單位） |
| `catchAlarm()` | dep/timer.c | **【移植點】** 在硬體計時器 ISR 中呼叫 |
| `timerStart()` | dep/timer.c | 啟動指定計時器（毫秒） |
| `timerExpired()` | dep/timer.c | 查詢計時器是否超期（非阻塞） |

---

## 9. 附錄 C：DM9058 PTP 寄存器映射

以下為本專案 DM9058 PTP 功能相關的暫存器配置，供參考其他 MAC 晶片移植時的對應設計。

| 暫存器 | 位址 | 寫入值 | 功能說明 |
|--------|------|--------|---------|
| PTP 重啟 | 0x60 | 0x01 | 觸發 PTP 功能軟體重置 |
| PTP 致能 | 0x61 | 0x01 | 啟用 PTP 時間戳功能 |
| TX 時間戳控制 | 0x02 | 0x00 | TX 時間戳捕獲設定 |
| Master/Slave 模式 | 0x64 | 0x12 | 設定工作模式 |
| ONE_STEP 控制 | 0x63 | 0x00 | 禁用 ONE_STEP（使用 TWO_STEP） |
| **TS 偏移位址（gPTP）** | **0x65** | **0x32** | PTP 訊息在 L2 frame 中的偏移（gPTP = 0x32，UDP/IP = 0x4E） |
| **CRC 偏移位址（gPTP）**| **0x66** | **0x20** | Checksum 欄位偏移（gPTP = 0x20，UDP/IP = 0x3C） |

**偏移量說明**：

```
L2 Ethernet Frame（gPTP 模式，ptp_offset_addr = 0x32 = 50 dec）：
┌──────────────────────────────────────────────────────────┐
│ Dst MAC (6B) │ Src MAC (6B) │ EtherType (2B) │ PTP Payload │
│   0~5        │   6~11       │   12~13        │  14~...     │
└──────────────────────────────────────────────────────────┘

PTP Payload 內部（從 offset 14 開始）：
│ Header (34B)   │ Body │
│  14~47         │ 48+  │
  ↑
  PTP 訊息從 Ethernet frame 的 offset 14 開始
  DM9058 的 0x32（50）= 14（Ethernet Header）+ 34（PTP Header）+ 2（flags）
  → 指向 PTP 訊息中 correctionField 的位置（供硬體自動更新）

UDP/IPv4 模式（ptp_offset_addr = 0x4E = 78 dec）：
  UDP 模式多了 IP Header(20B) + UDP Header(8B) = 額外 28 bytes
  78 = 50 + 28
```

---

*本文件由原始碼自動分析生成，最後更新：2026-04-15*
*原始碼分支：`esp32_master_one_step_v001`*
*如有疑問，請參照對應原始碼檔案中的行號註解。*
