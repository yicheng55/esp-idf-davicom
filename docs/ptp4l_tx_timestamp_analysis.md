# ptp4l 如何讀取驅動的 `shhwtstamps`

## 概述

ptp4l 透過 Linux **socket error queue** 機制，從核心驅動呼叫 `skb_tstamp_tx(skb, &shhwtstamps)` 後，將硬體時間戳傳遞到 userspace。整個流程分為四層。

---

## 第 1 層：核心驅動端（kernel driver）

```c
// 驅動在 TX 完成後呼叫：
skb_tstamp_tx(skb, &shhwtstamps);
```

核心會將 `shhwtstamps.hwtstamp`（一個 `ktime_t`）打包成三個 `timespec64` 陣列，透過 **socket error queue** 送回 userspace，格式為：

| 索引 | 內容 |
|------|------|
| `ts[0]` | Software timestamp（軟體時間戳） |
| `ts[1]` | Legacy hardware timestamp |
| `ts[2]` | **Raw hardware timestamp** ← 即 `shhwtstamps.hwtstamp` |

---

## 第 2 層：Socket 初始化（`sk.c` → `sk_timestamping_init`）

```c
// sk.c: sk_timestamping_init()
flags = SOF_TIMESTAMPING_TX_HARDWARE |
        SOF_TIMESTAMPING_RX_HARDWARE |
        SOF_TIMESTAMPING_RAW_HARDWARE;

setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPING, &timestamping, sizeof(timestamping));
setsockopt(fd, SOL_SOCKET, SO_SELECT_ERR_QUEUE, &flags, sizeof(flags));
```

同時透過 `ioctl(SIOCSHWTSTAMP)` 告訴驅動啟用 HW timestamping：

| `TS_HARDWARE` / `TS_LEGACY_HW` | `tx_type = HWTSTAMP_TX_ON` |
|---|---|
| `TS_ONESTEP` | `tx_type = HWTSTAMP_TX_ONESTEP_SYNC` |
| `TS_P2P1STEP` | `tx_type = HWTSTAMP_TX_ONESTEP_P2P` |

---

## 第 3 層：讀取 TX timestamp（`transport.c` → `transport_txts` → `sk_receive`）

TX 完成後，`transport.c` 呼叫：

```c
// transport_txts() in transport.c
cnt = sk_receive(fda->fd[FD_EVENT], pkt, len, NULL, hwts, MSG_ERRQUEUE);
```

`sk_receive` 在 `sk.c` 內部執行兩個動作：

### ① `poll()` 等待 error queue 就緒

```c
struct pollfd pfd = { fd, sk_events, 0 };
res = poll(&pfd, 1, sk_tx_timeout);  // 預設 1ms timeout
```

- `sk_events` 預設為 `POLLPRI`
- 若核心不支援 `SO_SELECT_ERR_QUEUE`，則退回使用 `POLLERR`

### ② `recvmsg(MSG_ERRQUEUE)` 收取 ancillary data

```c
cnt = recvmsg(fd, &msg, flags);  // flags = MSG_ERRQUEUE

for (cm = CMSG_FIRSTHDR(&msg); cm != NULL; cm = CMSG_NXTHDR(&msg, cm)) {
    if (SOL_SOCKET == level && SO_TIMESTAMPING == type) {
        ts = (struct timespec *) CMSG_DATA(cm);  // 取得 3 個 timespec
    }
    if (SOL_SOCKET == level && SO_TIMESTAMPNS == type) {
        sw = (struct timespec *) CMSG_DATA(cm);
        hwts->sw = timespec_to_tmv(*sw);         // 軟體時間戳 (fupsync 用)
    }
}
```

---

## 第 4 層：選取正確的 timestamp

```c
// sk.c: sk_receive() 最後的 switch
switch (hwts->type) {
case TS_SOFTWARE:
    hwts->ts = timespec_to_tmv(ts[0]);   // 軟體時間戳
    break;
case TS_HARDWARE:
case TS_ONESTEP:
case TS_P2P1STEP:
    hwts->ts = timespec_to_tmv(ts[2]);   // ← shhwtstamps.hwtstamp 在這裡！
    break;
case TS_LEGACY_HW:
    hwts->ts = timespec_to_tmv(ts[1]);
    break;
}
```

---

## 完整資料流

```
[NIC 驅動 / kernel]
  skb_tstamp_tx(skb, &shhwtstamps)
       │
       │  核心將 shhwtstamps.hwtstamp 存入 socket error queue
       ▼
[kernel → userspace ancillary data]
  struct timespec ts[3]
  ts[0] = software timestamp
  ts[1] = legacy hardware timestamp
  ts[2] = RAW hardware timestamp  (= shhwtstamps.hwtstamp)
       │
       ▼
[transport.c]  transport_txts()
  → sk_receive(fda->fd[FD_EVENT], ..., MSG_ERRQUEUE)
       │
       ▼
[sk.c]  sk_receive()
  ① poll()           — 等待 error queue POLLPRI / POLLERR
  ② recvmsg(MSG_ERRQUEUE) — 從 cmsg SO_TIMESTAMPING 取出 ts[]
  ③ hwts->ts = ts[2] — (TS_HARDWARE 模式)
       │
       ▼
[port.c]
  使用 msg->hwts.ts 計算 path delay / offset
```

---

## 關鍵結論

ptp4l **不直接以 ioctl 讀取驅動的時間戳**，而是：

1. 啟動時透過 `SIOCSHWTSTAMP` 告知驅動開啟硬體時間戳功能。
2. 每次發送事件封包後，透過 `poll()` 等待核心通知。
3. 呼叫 `recvmsg(MSG_ERRQUEUE)` 從 socket ancillary data (`cmsg`) 取出 `SO_TIMESTAMPING` 的第三個 `timespec`（`ts[2]`）。
4. 此 `ts[2]` 即是核心從 `shhwtstamps.hwtstamp` 複製而來的硬體時間戳。
