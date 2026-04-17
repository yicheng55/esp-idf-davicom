# L2TAP PTPD P2P 實作手冊

> 適用範例：esp-idf/examples/ethernet/ptp
> 目標：在現有 L2TAP + DM9058 架構上，為 ESP-IDF 精簡版 ptpd 補上可編譯、可驗證的 Peer-to-Peer (P2P) path delay 實作
> 當前範圍：編譯期切換 E2E / P2P、雙 ESP 裝置直連驗證、Two-step Pdelay 流程

本手冊描述目前這個分支已經加入的 P2P 代碼路徑、Kconfig、封包行為與驗證方式。它不重複 [docs/L2TAP_PTPD_Manual.md](docs/L2TAP_PTPD_Manual.md) 已說明的 L2TAP、DM9058 transport 與一般 PTP 背景，而是聚焦在「為了讓現有 ptpd 支援 Pdelay，到底改了哪些地方」。

## 1. 實作範圍

目前版本的 P2P 支援是「最小可用、可維護」路線，而不是完整移植 gPTP/802.1AS 全部行為。

已完成的部分：

- 新增編譯期切換的 path delay mechanism：E2E 或 P2P。
- 新增 `Pdelay_Req`、`Pdelay_Resp`、`Pdelay_Resp_Follow_Up` 的訊息型別與封包結構。
- 讓 `ptpd.c` 能根據訊息型別，自動把 Pdelay 類封包送到 `01:80:C2:00:00:0E`。
- 新增 Pdelay requester/responder handler，沿用現有 L2TAP IREC 硬體時間戳。
- 把 peer delay 結果寫回現有 `path_delay_ns` 路徑，直接供 offset 計算使用。
- 已通過 `idf.py build`。

尚未完成或刻意不做的部分：

- 不支援執行期切換 E2E/P2P。
- 不支援 One-step P2P。
- 不處理完整 802.1AS profile 細節，例如 `transportSpecific`、TC/BC 行為與 profile-specific BMCA 差異。
- 不保證與 Linux `ptp4l` 的完整 gPTP 模式互通；第一階段目標是雙 ESP 範例互通。

## 2. 架構決策

### 2.1 為什麼保留 E2E 路徑

原本範例的 E2E 邏輯已經可用，且 app 層、README、示波驗證流程都建立在它之上。直接把整個 daemon 改成 P2P-only 會破壞既有範例用途，也會讓除錯成本升高。因此這一版採用：

- Kconfig 決定編譯期 delay mechanism
- 保留既有 `Delay_Req/Delay_Resp` 代碼
- P2P 只接入需要新增的 handler 與封包路由

### 2.2 為什麼共用 `path_delay_ns`

精簡版 ptpd 沒有原版 `peerMeanPathDelay` 與 `CurrentDS.meanPathDelay` 的完整資料模型。若照原版重新拆欄位，改動會牽涉到 status API、app 層、文件與更多測試。

因此這一版採用折衷策略：

- E2E 模式：`path_delay_ns` 表示 end-to-end path delay
- P2P 模式：`path_delay_ns` 表示 peer delay

對本範例來說，這個欄位唯一用途是讓 offset 計算補償傳輸延遲，因此共用不會破壞目前同步路徑。

## 3. 實際修改點

### 3.1 Kconfig

檔案：[components/ptpd/Kconfig.projbuild](../components/ptpd/Kconfig.projbuild)

新增：

- `NETUTILS_PTPD_MECHANISM_E2E`
- `NETUTILS_PTPD_MECHANISM_P2P`
- `NETUTILS_PTPD_PDELAY_INTERVAL_MSEC`

調整：

- `CONFIG_NETUTILS_PTPD_SEND_DELAYREQ` 僅在 E2E 模式可用
- `MAX_PATH_DELAY_NS`、`DELAYREQ_AVGCOUNT`、`PATH_DELAY_STABILITY_NS` 改為 E2E/P2P 共用

設計原因：

- 保持 menuconfig 入口簡單
- 避免為 P2P 再複製一份數值幾乎相同的 path delay 濾波設定
- 明確限制目前 P2P 只在 `CLIENT + SERVER + TWOSTEP_SYNC` 條件下可用

### 3.2 PTPv2 封包定義

檔案：[components/ptpd/ptpv2.h](../components/ptpd/ptpv2.h)

新增的 message type：

- `PTP_MSGTYPE_PDELAY_REQ = 2`
- `PTP_MSGTYPE_PDELAY_RESP = 3`
- `PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP = 10`

新增的結構：

- `struct ptp_pdelay_req_s`
- `struct ptp_pdelay_resp_s`
- `struct ptp_pdelay_resp_follow_up_s`

這些結構刻意沿用目前 `ptp_delay_resp_s` 的欄位布局，方便直接使用現有 `timespec_to_ptp_format()` / `ptp_format_to_timespec()` 轉換函式。

### 3.3 ptpd state

檔案：[components/ptpd/ptpd.c](../components/ptpd/ptpd.c)

`struct ptp_state_s` 新增：

- `pdelay_req_seq`
- `pdelay_t1/t2/t3/t4`
- `waiting_pdelay_follow_up`
- `pdelay_interval`
- `pdelay_resp_correction_ns`

對應關係：

- `t1`：送出 `Pdelay_Req` 的本地 TX 硬體時間戳
- `t2`：對端收到 `Pdelay_Req` 的時間，由 `Pdelay_Resp.requestReceiptTimestamp` 帶回
- `t3`：對端送出 `Pdelay_Resp` 的時間，由 `Pdelay_Resp_Follow_Up.originTimestamp` 帶回
- `t4`：本地收到 `Pdelay_Resp` 的硬體時間戳

## 4. 封包路由與 MAC 位址

### 4.1 送出路徑

`ptp_create_eth_frame()` 會根據 `messageType` 選 MAC：

- `Pdelay_*` 類：`01:80:C2:00:00:0E`
- 其餘訊息：`01:1B:19:00:00:00`

這樣做的原因是目前 daemon 已經統一使用 L2TAP raw Ethernet 傳輸，所以只要在組 Ethernet header 時分流，不需要再碰 socket 類型或 EtherType。

### 4.2 接收路徑

L2TAP 仍然只靠 `EtherType 0x88F7` 過濾。目的 MAC 是否為 peer-delay multicast，不由 L2TAP 處理，而是靠：

- 底層 Ethernet 驅動 MAC filter 已預先加入 `01:80:C2:00:00:0E`
- `ptp_process_rx_packet()` 依 messageType 分派給對應 handler

## 5. Pdelay 流程

### 5.1 Requester

當前條件：

- `selected_source_valid == true`
- `can_send_delayreq == true`
- 已達到 `CONFIG_NETUTILS_PTPD_PDELAY_INTERVAL_MSEC`

流程：

1. `ptp_send_pdelay_req()` 建立 `Pdelay_Req`
2. 透過 `ptp_net_send()` 取得 `t1`
3. 等待 `Pdelay_Resp`
4. 收到 `Pdelay_Resp` 後保存 `t2` 與 `t4`
5. 若為 two-step，等待 `Pdelay_Resp_Follow_Up`
6. 收到 follow-up 後取得 `t3`
7. 用 `(t2 - t1 + t4 - t3 - correction) / 2` 計算 peer delay

### 5.2 Responder

收到 `Pdelay_Req` 後：

1. 以 RX 硬體時間戳作為 `requestReceiptTimestamp` 回送 `Pdelay_Resp`
2. 同一個 `sequenceId` 再送 `Pdelay_Resp_Follow_Up`
3. `originTimestamp` 填入 `Pdelay_Resp` 的實際 TX 硬體時間戳

### 5.3 為什麼只支援 Two-step

在目前軟體路徑中，`Pdelay_Resp` 的真實 TX 時間只有在 `write()` 完成後才能拿到。這意味著：

- 若要做 One-step，必須在封包離開 MAC 的同時，把 `(t3 - t2)` 即時寫入 `correctionField`
- 這不是當前精簡版 `ptpd.c` 可以在軟體層完成的事

因此本版明確只支援 Two-step P2P，並在 Kconfig 上做限制。

## 6. Peer Delay 計算

目前計算式來自原版 ptpd 的 two-step Pdelay 演算法，並做最小化移植：

$$
peer\_delay = \frac{(t2 - t1) + (t4 - t3) - correction}{2}
$$

其中：

- `correction` 由 `Pdelay_Resp` 與 `Pdelay_Resp_Follow_Up` header 的 `correctionField` 累加而來
- 在雙 ESP 直連第一階段場景中，這個值通常為 0

計算完成後，不新增新的 servo 欄位，而是直接呼叫共用的 path delay averaging helper，把結果寫回 `state->path_delay_ns`。

## 7. 目前限制

### 7.1 只保證雙 ESP 直連

目前沒有實作 Transparent Clock、Boundary Clock 或 802.1AS profile-specific 行為，因此不應把這版直接視為完整 gPTP 實作。

### 7.2 `ptpd_status_s` 尚未擴充

公開 API 仍沿用原本的 `path_delay_ns` 欄位，因此：

- E2E 模式下它表示 path delay
- P2P 模式下它表示 peer delay

app 層若需要對外明確區分，後續可再擴充 status struct。

### 7.3 One-step P2P 尚未做

如果未來需要 One-step P2P，建議往下列方向延伸：

- 由 MAC/driver 支援 Pdelay correctionField 線上改寫
- 或在 driver 中提供 responder turnaround correction 的硬體輔助欄位

## 8. 建議驗證流程

### 8.1 menuconfig

在 `PTP Daemon Configuration` 中設定：

- `PTPD client/server`：開啟
- `Enable client support`：開啟
- `Enable server support`：開啟
- `PTP server sends two-step synchronization packets`：開啟
- `PTP client path delay mechanism`：選 `Peer-to-Peer (Pdelay_Req / Pdelay_Resp)`
- `PTP client peer delay request interval (ms)`：建議先用 `1000`

### 8.2 build

```bash
idf.py build
```

### 8.3 觀察 log

理想情況下，slave 端應看到：

```text
ptpd: Got sync packet, seq N
ptpd: Got follow-up packet, seq N
ptpd: Sent pdelay req, seq M
ptpd: Got pdelay-resp, seq M
ptpd: Got pdelay-resp-follow-up, seq M
ptpd: Peer delay: X ns (avg: Y ns)
```

### 8.4 驗證點

- `Pdelay_Req` 的目的 MAC 應為 `01:80:C2:00:00:0E`
- `Pdelay_Resp` / `Pdelay_Resp_Follow_Up` 的 `sequenceId` 應與 request 相同
- `path_delay_ns` 應收斂到穩定值，而不是持續暴衝
- GPIO pulse 對齊不應比原 E2E 直連更差

## 9. 下一步

若要把這版往完整 gPTP 再推進，建議順序如下：

1. 擴充 `ptpd_status_s`，把 `delay_mechanism` 與 `peer_delay_ns` 明確公開
2. 補 `transportSpecific` 與 profile-specific header 行為
3. 加入與 Linux `ptp4l` 的互通驗證
4. 評估是否要把 One-step P2P 下放到 driver/hardware path 完成

在那之前，這份實作比較適合作為「在現有 ESP-IDF ptpd 精簡版中導入 P2P 的第一階段」。