#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import base64
import zlib
import urllib.request
import urllib.parse

# Mermaid 图表代码
mermaid_code = '''sequenceDiagram
    participant HW as DM9058硬體
    participant ISR as DM9058_isr_handler
    participant Task as esp32_DM9058_task
    participant Recv as esp32_DM9058_receive
    participant Frame as DM9058_frame_to_rx_buffer
    participant PTP_TS as DM9058_handle_rx_ptp_timestamp
    participant Stack as emac->eth->stack_input
    participant L2TAP as L2TAP Filter<br/>(esp_vfs_l2tap_eth_filter_frame)
    participant Queue as L2TAP RX Queue
    participant Daemon as ptp_daemon<br/>(PTPD Task)
    participant Process as ptp_process_rx_packet
    participant Sync as ptp_process_sync

    HW->>ISR: 1. Sync封包接收中斷
    ISR->>Task: 2. vTaskNotifyGive()

    Task->>Task: 3. 檢查ISR_PR狀態
    Task->>Recv: 4. parent.receive()

    Recv->>Frame: 5. DM9058_frame_to_rx_buffer()
    Frame->>Frame: 6. 讀取RX頭部(4字節)
    Frame->>Frame: 7. 讀取Sync封包內容

    Frame->>PTP_TS: 8. DM9058_handle_rx_ptp_timestamp()
    PTP_TS->>PTP_TS: 9. 檢查PTP啟用狀態
    PTP_TS->>PTP_TS: 10. 解析RX頭部時戳資訊
    PTP_TS->>PTP_TS: 11. 從DM9058讀取時戳(8 bytes)
    PTP_TS->>PTP_TS: 12. 解碼並存到<br/>emac->last_rx_timestamp
    PTP_TS->>PTP_TS: 13. rx_timestamp_valid = true

    Frame->>Recv: 14. 返回封包長度
    Recv->>Task: 15. 返回Sync封包(去CRC)

    Task->>Task: 16. malloc新緩衝
    Task->>Task: 17. memcpy Sync封包
    Task->>Stack: 18. stack_input(buffer, len)

    Stack->>L2TAP: 19. esp_vfs_l2tap_eth_filter_frame()
    L2TAP->>L2TAP: 20. 檢查EtherType過濾器<br/>(0x88F7 for PTP)
    L2TAP->>L2TAP: 21. 檢查L2TAP_FLAG_TS<br/>(時戳功能啟用?)
    L2TAP->>L2TAP: 22. 取得RX時戳<br/>(從info參數)
    L2TAP->>Queue: 23. 將Sync封包+時戳<br/>放入RX Queue

    Daemon->>Daemon: 24. poll()等待socket可讀
    Daemon->>Daemon: 25. pollfds[0].revents觸發
    Daemon->>Daemon: 26. ptp_net_recv()
    Daemon->>Queue: 27. read(ptp_socket, ...)

    Queue->>Daemon: 28. 返回Sync封包+時戳
    Daemon->>Daemon: 29. 從L2TAP_IREC中<br/>提取timespec

    Daemon->>Process: 30. ptp_process_rx_packet()
    Process->>Process: 31. 檢查domain和長度
    Process->>Process: 32. 識別PTP_MSGTYPE_SYNC

    Process->>Sync: 33. ptp_process_sync()
    Sync->>Sync: 34. 驗證來源身份
    Sync->>Sync: 35. 檢查TWO_STEP標誌

    alt TWO_STEP模式
        Sync->>Sync: 36a. 儲存rxtime等待Follow_Up
    else ONE_STEP模式
        Sync->>Sync: 36b. 提取originTimestamp
        Sync->>Sync: 36c. ptp_update_local_clock()
        Sync->>Sync: 36d. 使用rxtime時戳調整時鐘
    end'''

def generate_svg_via_kroki(mermaid_code, output_file):
    """使用 Kroki API 生成 SVG (POST 方法)"""
    import json

    url = 'https://kroki.io/mermaid/svg'

    # 构建 POST 请求数据
    data = json.dumps({
        'diagram_source': mermaid_code,
        'diagram_type': 'mermaid',
        'output_format': 'svg'
    }).encode('utf-8')

    print(f'正在從 Kroki API 下載 SVG (使用 POST)...')

    try:
        # 创建请求
        req = urllib.request.Request(
            url,
            data=data,
            headers={
                'Content-Type': 'application/json',
                'User-Agent': 'Mozilla/5.0'
            },
            method='POST'
        )

        # 下载 SVG
        with urllib.request.urlopen(req, timeout=30) as response:
            svg_content = response.read()

        # 保存到文件
        with open(output_file, 'wb') as f:
            f.write(svg_content)

        print(f'✓ SVG 文件已成功生成: {output_file}')
        print(f'  文件大小: {len(svg_content)} 字節')
        return True
    except Exception as e:
        print(f'✗ 錯誤: {e}')
        return False

if __name__ == '__main__':
    import os

    output_file = os.path.join(os.path.dirname(__file__), 'Sync封包流程圖.svg')

    if generate_svg_via_kroki(mermaid_code, output_file):
        print(f'\n已將 Mermaid 圖表轉換為 SVG 格式!')
        print(f'您可以使用任何 SVG 查看器或瀏覽器打開此文件。')
    else:
        print(f'\nSVG 生成失敗。請檢查網路連接或手動使用以下方法:')
        print(f'1. 打開 https://mermaid.live/')
        print(f'2. 貼上 Mermaid 代碼')
        print(f'3. 點擊 "Export" -> "SVG"')
