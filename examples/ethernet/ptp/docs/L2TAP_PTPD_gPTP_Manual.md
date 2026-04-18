# L2TAP PTPD gPTP / IEEE 802.1AS 實作與移植說明書

> 適用範例：esp-idf/examples/ethernet/ptp
> 目標平台：ESP-IDF + L2TAP + DM9058
> 文件目的：將目前分支的 PTPD 精簡版實作，整理成可直接用於 gPTP / IEEE 802.1AS 導入、重構與移植的完整說明
> 文件定位：同時描述「目前已實作內容」、「完整 802.1AS 目標架構」與「兩者之間的補齊路線」

本文件不再只把目前分支視為一組 P2P 補丁，而是從完整 gPTP 實作的角度，重新整理整個系統的控制流程、時間戳流向、狀態機責任、移植介面與驗證策略。

若你需要更偏原始碼分析風格、以傳統 ptpd-2.0.0 為核心的背景文件，請搭配 [doc/gptp_porting_guide.md](../doc/gptp_porting_guide.md) 一起閱讀。本文件則聚焦在 ESP-IDF / L2TAP / DM9058 這條實作路徑，回答下列問題：

- 現在這個分支已經具備哪些 gPTP 必要能力。
- 哪些地方仍然只是「接近 802.1AS」，還不算完整實作。
- 如果要把它重構成完整 gPTP 移植版本，應該怎麼分階段做。

## 1. 概述

### 1.1 gPTP 與一般 PTP 的差異

| 面向 | 一般 PTP（IEEE 1588） | gPTP（IEEE 802.1AS） |
|------|------------------------|----------------------|
| 傳輸層 | UDP 或 L2 | 強制 L2 Ethernet |
| 延遲機制 | E2E 或 P2P | 強制 P2P |
| 多播 MAC | 常見為 `01:1B:19:00:00:00` | 802.1AS 為 `01:80:C2:00:00:0E` |
| Sync 模式 | one-step 或 two-step | 兩者皆可，視 MAC 能力 |
| profile 欄位 | 一般 PTP header | 需符合 802.1AS profile 約束 |
| BMCA | 完整 1588 比較流程 | 需符合 802.1AS 的資料集與拓撲假設 |

這代表如果目前實作仍保留 E2E path delay、混用一般 PTP multicast、或缺少 profile-specific header 行為，就只能說是「具備部分 gPTP 路徑」，還不能說是完整 802.1AS 實作。

### 1.2 本分支目前的位置

目前分支已經把最關鍵的 P2P path delay 量測能力補上，並且可在 ESP-IDF + L2TAP + DM9058 上運作；但從完整 gPTP 角度來看，它仍屬於「可用的第一階段實作」，不是終態。

| 項目 | 目前分支 | 完整 802.1AS 目標 |
|------|----------|--------------------|
| L2TAP raw Ethernet | 已具備 | 保留 |
| DM9058 硬體時間戳 | 已具備 | 保留 |
| Sync / Follow_Up | 已具備 | 保留，需檢查 profile 欄位 |
| Pdelay requester/responder | 已具備 two-step | 保留，並補 profile 一致性 |
| `transportSpecific = 0x1` | 未完整保證 | 必須補齊 |
| 802.1AS multicast 一致性 | 僅 Pdelay 類確定使用 | 所有應屬 gPTP 的訊息需一致檢視 |
| BMCA / announce profile | 沿用精簡版 ptpd | 需評估並補 profile 約束 |
| 與 Linux `ptp4l` 互通 | 未保證 | 應列為驗證目標 |

## 2. 系統架構

### 2.1 軟硬體分層

```
┌─────────────────────────────────────────────┐
│                Application / app_main       │
│   啟動 ptpd、讀取 status、輸出 log / GPIO    │
└──────────────────────┬──────────────────────┘
                       │
┌──────────────────────▼──────────────────────┐
│            components/ptpd/ptpd.c           │
│  狀態機、封包處理、Sync、Delay、Pdelay       │
└───────────────┬───────────────┬─────────────┘
                │               │
┌───────────────▼───────┐ ┌─────▼────────────────┐
│     ptpv2.h / msg     │ │     servo / status   │
│ 訊息格式、header、seq │ │ offset/path delay 計算│
└───────────────┬───────┘ └─────┬────────────────┘
                │               │
┌───────────────▼────────────────▼─────────────┐
│              L2TAP / raw Ethernet            │
│   EtherType 0x88F7 收發、MAC 過濾、timestamp  │
└──────────────────────┬───────────────────────┘
                       │
┌──────────────────────▼───────────────────────┐
│                   DM9058 MAC                 │
│      TX/RX hardware timestamp + SPI 存取      │
└──────────────────────────────────────────────┘
```

### 2.2 模組責任

| 模組 | 責任 |
|------|------|
| `components/ptpd/ptpd.c` | 主狀態機、封包分派、Sync、Delay、Pdelay 控制流程 |
| `components/ptpd/ptpv2.h` | PTPv2 / Pdelay message type 與封包結構 |
| L2TAP transport | 建立 raw Ethernet frame、過濾 EtherType 0x88F7、保留 RX/TX 時間戳 |
| DM9058 driver | 提供硬體時間戳與 MAC 收發能力 |
| app / status layer | 啟動 daemon、顯示同步狀態、對外暴露 path delay / offset |

完整 802.1AS 移植的關鍵不在單一模組，而在於這五層是否用同一個 profile 假設在工作。

## 3. 核心流程分析

### 3.1 L2 封包收發

目前實作已建立在 L2TAP raw Ethernet 之上，這是導入 gPTP 的正確基礎。最重要的行為有三個：

1. 只處理 EtherType `0x88F7` 的 PTP frame。
2. 依 `messageType` 決定封包要走哪條 handler 路徑。
3. 對 event 類封包保留硬體 TX/RX timestamp。

從完整 gPTP 角度看，這條路徑之後還需要再確認兩件事：

- 送出的 gPTP 訊息是否都符合 802.1AS 目的 MAC 規則。
- header 中的 `transportSpecific`、`correctionField` 與 log interval 是否與 profile 一致。

### 3.2 Sync / Follow_Up 流程

目前 Sync 路徑大致延續精簡版 ptpd 的 two-step 模式：

1. Master 送出 `Sync`。
2. Slave 收到 `Sync`，記錄 RX 時間戳 `T2`。
3. Master 再送 `Follow_Up`，帶回 `T1`。
4. Slave 以 `(T2 - T1)` 結合 correction field 與 path delay 做 offset 補償。

概念式可寫成：

$$
offset\_from\_master \approx (T2 - T1) - correction - path\_delay
$$

這條路徑本身不是問題；真正影響是否符合 802.1AS 的，是下列三點：

- `path_delay` 的來源是否真的來自 P2P peer delay。
- Sync / Follow_Up 的 header 是否符合 802.1AS profile。
- master/slave 狀態轉換是否與 announce / BMCA 流程一致。

### 3.3 Pdelay 流程

這是目前分支最接近 gPTP 核心要求的部分。

#### 3.3.1 四個時間戳

```
[Requester]                               [Responder]
    │                                          │
    │ t1 = Pdelay_Req TX timestamp             │
    │────── Pdelay_Req(seq=N) ────────────────►│
    │                                          │ t2 = Pdelay_Req RX timestamp
    │◄───── Pdelay_Resp(seq=N, t2) ────────────│
    │ t4 = Pdelay_Resp RX timestamp            │ t3 = Pdelay_Resp TX timestamp
    │◄───── Pdelay_Resp_Follow_Up(seq=N, t3) ──│
```

#### 3.3.2 計算方式

目前分支採用 two-step Pdelay：

$$
peer\_delay = \frac{(t2 - t1) + (t4 - t3) - correction}{2}
$$

其中 `correction` 為 `Pdelay_Resp` 與 `Pdelay_Resp_Follow_Up` header 的 correction field 累加值。

#### 3.3.3 實作意義

在完整 802.1AS 中，這個值應對應 `peerMeanPathDelay` 一類的語義；但目前分支為了保持改動面小，並未完整重建原版資料集，而是直接把結果寫回既有 `path_delay_ns`。這是合理的工程折衷，但文件必須明確說清楚，否則在後續重構時很容易把 E2E 與 P2P 路徑混淆。

### 3.4 Sync 與 Pdelay 的關聯

這是目前文件最需要講清楚的一段。

Pdelay 不是獨立功能，也不是單純多出三種封包而已。它的存在是為了讓 Sync 計算中的傳輸延遲補償，從原本的 E2E path delay 來源，切換成符合 802.1AS 要求的 peer delay。

系統層面的資料流可以簡化為：

1. Master 週期性送出 `Sync` / `Follow_Up`。
2. Slave 從 Sync 路徑得到 `(T2 - T1)`。
3. Pdelay 路徑持續更新鄰居鏈路的 `peer_delay`。
4. Servo 在 offset 計算時扣除該延遲值。

因此，若 `path_delay_ns` 沒有正確切換到 P2P 來源，或者 Pdelay 還沒收斂，整體同步品質就不可能達到真正的 gPTP 行為。

## 4. 狀態機與執行時控制

### 4.1 目前分支的實際控制邏輯

雖然本分支沒有完整移植原版 ptpd 的所有資料集與 BMCA 流程，但執行上仍然存在幾個核心狀態：

- 尚未選定同步來源。
- 已選定同步來源，開始收 Sync / Follow_Up。
- 允許送 DelayReq 或 PdelayReq。
- 等待 Pdelay follow-up。
- 依 offset / path delay 收斂狀態更新 status。

也就是說，目前程式的控制核心比較像「簡化的同步循環」，而不是完整的 802.1AS port state machine。

### 4.2 若要對齊完整 802.1AS，至少要補哪些狀態語義

從完整實作角度，文件上應明確追蹤下列狀態責任：

- Listening：尚未鎖定 master，只接收 announce / sync 類資訊。
- Master：送出 Sync、Follow_Up、Announce，並回應 Pdelay。
- Slave：接收 Sync、Follow_Up，並主動送出 PdelayReq。
- Passive：僅監聽與參與 BMCA，不主導同步。
- Fault / Reinitialize：時間戳異常、封包錯誤或超時後的恢復路徑。

若未來真的要宣稱「完整 802.1AS 實作」，這些語義不能只存在文件裡，還必須在程式與 status API 中有可追蹤的對應欄位。

## 5. 目前程式已修改的實作點

### 5.1 Kconfig

現有分支已新增或調整下列配置概念：

- `NETUTILS_PTPD_MECHANISM_E2E`
- `NETUTILS_PTPD_MECHANISM_P2P`
- `NETUTILS_PTPD_PDELAY_INTERVAL_MSEC`
- 僅在 E2E 時顯示或啟用 `SEND_DELAYREQ`

這讓目前專案至少能在編譯期切換 delay mechanism，但完整 gPTP 版本仍建議把 profile 級設定集中管理，而不是只停留在 menuconfig 分流。

### 5.2 封包定義

目前已增加：

- `PTP_MSGTYPE_PDELAY_REQ = 2`
- `PTP_MSGTYPE_PDELAY_RESP = 3`
- `PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP = 10`
- 對應的 `struct ptp_pdelay_req_s`
- 對應的 `struct ptp_pdelay_resp_s`
- 對應的 `struct ptp_pdelay_resp_follow_up_s`

這些封包定義已足以支撐 two-step Pdelay 路徑，但若要完整符合 802.1AS，仍需回頭檢查：

- header 各欄位是否都符合 profile。
- 封包的目的 MAC 是否與訊息型別一致。
- correction field 是否在 one-step / two-step 模式切換時保持一致語義。

### 5.3 內部狀態欄位

目前 `ptpd.c` 已新增 Pdelay 相關狀態，例如：

- `pdelay_req_seq`
- `pdelay_t1/t2/t3/t4`
- `waiting_pdelay_follow_up`
- `pdelay_interval`
- `pdelay_resp_correction_ns`

這些欄位已足夠支撐 two-step requester / responder 的運作，但如果要繼續往完整實作推進，建議下一步不是再加更多零散欄位，而是重整成有明確語義的資料集模型，例如：

- 當前 port state
- peer delay dataset
- sync state dataset
- announce / foreign master dataset

## 6. 與完整 802.1AS 實作的差距

目前最重要的差距有六項。

### 6.1 profile 一致性仍未完成

文件、封包與傳輸路徑必須統一回答同一件事：哪些訊息已經進入 802.1AS profile，哪些還只是沿用一般 PTP 假設。若這件事沒有講清楚，後續互通測試會很混亂。

### 6.2 `transportSpecific` 與 header 細節仍待補齊

完整 gPTP 不只是在 L2 上送封包，還要求 header 中的 profile 欄位一致。若訊息型別雖然正確，但 header 仍沿用一般 PTP 設定，與標準裝置的互通風險仍高。

### 6.3 BMCA 與 announce 流程仍是精簡版思維

若系統仍只依賴簡化的 source selection 邏輯，就很難稱得上完整 802.1AS。至少需要把 announce、master selection、timeout 與狀態切換整理成可驗證的流程。

### 6.4 `path_delay_ns` 仍是工程折衷欄位

它現在同時承載 E2E 與 P2P 語義，對短期維護是好的，但對完整 gPTP 文件與 status API 來說不夠清楚。完整版本建議至少拆出：

- `peer_delay_ns`
- `delay_mechanism`
- `offset_from_master_ns`

### 6.5 one-step Pdelay 尚未支援

這不一定是錯，但文件必須明說：現階段只支援 two-step Pdelay，因為 responder 的真實 TX timestamp 只能在 MAC 完成送出後取得。若要支援 one-step，driver 或硬體必須能在線上修正 correction field。

### 6.6 互通驗證不足

若沒有與 Linux `ptp4l`、封包擷取工具與跨裝置時序量測一起驗證，再完整的文件也只是一種設計假說。

## 7. 完整 gPTP 移植與重構建議

若要把目前分支真正推進到完整 802.1AS，可按下列順序重構。

### 7.1 第一階段：統一 profile 與傳輸語義

目標：讓所有 gPTP 相關封包都使用一致的 L2 profile 假設。

建議項目：

1. 統一檢查各 message type 的目的 MAC。
2. 明確設定與驗證 `transportSpecific = 0x1`。
3. 整理 log interval、announce interval、pdelay interval 的 profile 預設值。
4. 在文件中列出哪些訊息已符合 802.1AS，哪些尚未完成。

### 7.2 第二階段：重整內部資料模型

目標：讓 `path_delay_ns`、offset、state flag 不再只是零散欄位，而是能映射到完整同步語義。

建議項目：

1. 拆出 `peer_delay_ns` 與 `mean_path_delay_ns`。
2. 明確保存 `offset_from_master`、`last_sync_t1/t2`、`last_pdelay_t1/t2/t3/t4`。
3. 建立可對外暴露的 status 結構，避免 app 端依賴隱含語義。

### 7.3 第三階段：補齊狀態機

目標：讓 master / slave / passive / listening / fault 狀態具備可追蹤行為，而不是只靠條件旗標分散驅動。

建議項目：

1. 整理 announce 處理與 source selection 流程。
2. 為 timeout、reselect、fault recovery 建立清楚的狀態轉移。
3. 將 Pdelay requester / responder 的啟停條件掛到 port state，而不是只依賴零散旗標。

### 7.4 第四階段：補齊互通與量測

目標：把文件中的設計主張轉成可驗證結果。

建議項目：

1. 與 Linux `ptp4l` 的 gPTP 模式互通測試。
2. 用 Wireshark 或封包擷取檢查 MAC、EtherType、messageType、sequenceId、correctionField。
3. 量測雙板同步抖動、收斂時間與 peer delay 穩定度。

## 8. 驗證策略

### 8.1 基本建置驗證

```bash
idf.py build
```

這只能證明配置沒有壞，不能證明同步行為正確。

### 8.2 執行期 log 驗證

理想情況下，slave 端至少應看到：

```text
ptpd: Got sync packet, seq N
ptpd: Got follow-up packet, seq N
ptpd: Sent pdelay req, seq M
ptpd: Got pdelay-resp, seq M
ptpd: Got pdelay-resp-follow-up, seq M
ptpd: Peer delay: X ns (avg: Y ns)
```

### 8.3 封包層驗證

至少檢查下列項目：

- `Pdelay_Req` 的目的 MAC 是否為 `01:80:C2:00:00:0E`
- `Pdelay_Resp` / `Pdelay_Resp_Follow_Up` 的 `sequenceId` 是否一致
- `Sync` / `Follow_Up` 的 header 是否與 profile 假設一致
- EtherType 是否穩定為 `0x88F7`
- correction field 是否在 two-step 流程中保持合理值

### 8.4 同步品質驗證

至少追蹤下列指標：

- `peer_delay_ns` 或 `path_delay_ns` 是否收斂
- offset 是否在合理範圍內穩定
- GPIO pulse 對齊是否優於或不差於原 E2E 路徑
- 切換 master 或重新連線後，系統是否能恢復同步

### 8.5 互通驗證

若文件最終要宣稱「完整 802.1AS 實作」，則至少應加入：

- 與 Linux `ptp4l` 的互通紀錄
- 與不同 peer 裝置的 announce / sync / pdelay 交握結果
- 封包欄位對照與失敗案例整理

## 9. 結論

目前這個 ESP-IDF + L2TAP + DM9058 分支，已經具備導入 gPTP 所需的最重要基礎能力：L2 raw Ethernet、硬體時間戳、two-step Sync，以及 two-step Pdelay requester / responder。這使它不再只是一般 PTP 範例，而是可以被當成 gPTP 移植起點的實作骨架。

但如果要把文件與程式都提升到「完整 802.1AS 實作」的層級，還必須補齊 profile 一致性、header 細節、狀態機、資料模型與互通驗證。這份文件因此不只是在描述現況，也明確提供了從目前分支走向完整 gPTP 實作的重構路線。

後續若要繼續演進，建議優先順序如下：

1. 先統一 L2 profile 與 header 行為。
2. 再重整 `path_delay_ns` 與同步狀態資料模型。
3. 然後補齊 BMCA / announce / port state 語義。
4. 最後用 `ptp4l` 與實機量測完成互通驗證。
