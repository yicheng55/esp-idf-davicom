/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * gPTP network layer – ESP-IDF L2TAP implementation.
 * Replaces ptpd-2.0.0 dep/net.c which used lwIP pbuf callbacks.
 *
 * All PTP frames for gPTP (IEEE 802.1AS) are sent to the gPTP
 * multicast MAC  01:80:C2:00:00:0E  (EtherType 0x88F7).
 * A single L2TAP socket filters on that EtherType.
 *
 * Incoming frames are classified into:
 *   eventQ   – SYNC, DELAY_REQ, PDELAY_REQ, PDELAY_RESP   (need HW timestamp)
 *   generalQ – FOLLOW_UP, DELAY_RESP, PDELAY_RESP_FOLLOW_UP, ANNOUNCE
 * and queued in static ring buffers for the protocol state machine.
 */

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/poll.h>

#include "gptp_defs.h"
#include "gptp_net.h"
#include "esp_vfs_l2tap.h"
#include "esp_eth_driver.h"
#include "esp_eth.h"
#include "lwip/prot/ethernet.h"   /* ETH_HEADER_LEN, struct eth_hdr */

/* Ethernet header = 6 (dst) + 6 (src) + 2 (EtherType) = 14 bytes */
#define ETH_HDR_LEN  14u

/* gPTP multicast MAC (IEEE 802.1AS, used for ALL messages) */
static const uint8_t GPTP_MCAST_MAC[6] = {0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E};

/* ============================================================
 *  Queue helpers (static ring buffer, no dynamic allocation)
 * ============================================================ */

static void netQInit(BufQueue *q)
{
    memset(q, 0, sizeof(*q));
}

static Boolean netQPut(BufQueue *q, const uint8_t *buf, uint16_t len, const TimeInternal *ts)
{
    if (q->count >= PBUF_QUEUE_SIZE) return FALSE;

    GptpPkt *pkt = &q->pkts[q->put];
    uint16_t copy_len = (len > GPTP_PKT_SIZE) ? GPTP_PKT_SIZE : len;
    memcpy(pkt->buf, buf, copy_len);
    pkt->len = copy_len;
    if (ts) pkt->ts = *ts;
    else    memset(&pkt->ts, 0, sizeof(pkt->ts));

    q->put = (q->put + 1) % PBUF_QUEUE_SIZE;
    q->count++;
    return TRUE;
}

static ssize_t netQGet(BufQueue *q, uint8_t *buf, TimeInternal *ts)
{
    if (q->count <= 0) return 0;

    GptpPkt *pkt = &q->pkts[q->get];
    memcpy(buf, pkt->buf, pkt->len);
    if (ts) *ts = pkt->ts;
    ssize_t len = pkt->len;

    q->get = (q->get + 1) % PBUF_QUEUE_SIZE;
    q->count--;
    return len;
}

/* ============================================================
 *  Classify message: event vs. general
 * ============================================================ */
static Boolean isEventMessage(uint8_t messageType)
{
    switch (messageType & 0x0F) {
    case SYNC:       /* 0x0 */
    case DELAY_REQ:  /* 0x1 */
    case PDELAY_REQ: /* 0x2 */
    case PDELAY_RESP:/* 0x3 */
        return TRUE;
    default:
        return FALSE;
    }
}

/* ============================================================
 *  Drain L2TAP fd: read all available frames into queues
 * ============================================================ */
void gptp_net_poll_and_queue(NetPath *netPath)
{
    struct pollfd pfd = { .fd = netPath->fd, .events = POLLIN };
    int ret;

    while (1) {
        ret = poll(&pfd, 1, 0);
        if (ret <= 0 || !(pfd.revents & POLLIN)) break;

        /* Receive with L2TAP extended buffer to get HW timestamp */
        union {
            uint8_t info_recs_buff[L2TAP_IREC_SPACE(sizeof(struct timespec))];
            l2tap_irec_hdr_t align;
        } u;

        uint8_t frame_buf[ETH_HDR_LEN + PACKET_SIZE];

        l2tap_extended_buff_t ext = {
            .info_recs_len  = sizeof(u.info_recs_buff),
            .info_recs_buff = u.info_recs_buff,
            .buff           = frame_buf,
            .buff_len       = sizeof(frame_buf),
        };

        l2tap_irec_hdr_t *ts_info = L2TAP_IREC_FIRST(&ext);
        ts_info->len  = L2TAP_IREC_LEN(sizeof(struct timespec));
        ts_info->type = L2TAP_IREC_TIME_STAMP;

        int n = read(netPath->fd, &ext, 0);
        if (n <= 0) break;

        /* Strip Ethernet header to get raw PTP payload */
        if ((uint32_t)n <= ETH_HDR_LEN) continue;
        const uint8_t *ptp_payload = frame_buf + ETH_HDR_LEN;
        uint16_t       ptp_len     = (uint16_t)((uint32_t)n - ETH_HDR_LEN);

        if (ptp_len < 1) continue;

        /* Extract HW timestamp */
        TimeInternal ts = {0, 0};
        if (ts_info->type == L2TAP_IREC_TIME_STAMP) {
            struct timespec *tsp = (struct timespec *)ts_info->data;
            ts.seconds     = (Integer32)tsp->tv_sec;
            ts.nanoseconds = (Integer32)tsp->tv_nsec;
        }

        uint8_t msgType = ptp_payload[0] & 0x0F;

        if (isEventMessage(msgType)) {
            if (!netQPut(&netPath->eventQ, ptp_payload, ptp_len, &ts)) {
                ESP_LOGW(GPTP_TAG, "gptp_net: eventQ full, dropping msgType=%u", msgType);
            }
        } else {
            if (!netQPut(&netPath->generalQ, ptp_payload, ptp_len, NULL)) {
                ESP_LOGW(GPTP_TAG, "gptp_net: generalQ full, dropping msgType=%u", msgType);
            }
        }
    }
}

/* ============================================================
 *  Send helper: build Ethernet frame + request TX timestamp
 * ============================================================ */
static ssize_t gptp_l2tap_send(int fd, const uint8_t *src_mac,
                                const Octet *ptp_buf, UInteger16 ptp_len,
                                TimeInternal *tx_time)
{
    /* Build complete Ethernet frame: dst(6)+src(6)+type(2)+payload */
    uint8_t frame[ETH_HDR_LEN + PACKET_SIZE];
    if (ptp_len > PACKET_SIZE) ptp_len = PACKET_SIZE;

    memcpy(frame + 0, GPTP_MCAST_MAC, 6);           /* dst */
    memcpy(frame + 6, src_mac, 6);                   /* src */
    frame[12] = (uint8_t)(PTP_ETHERTYPE >> 8);
    frame[13] = (uint8_t)(PTP_ETHERTYPE & 0xFF);
    memcpy(frame + ETH_HDR_LEN, ptp_buf, ptp_len);

    uint16_t total = (uint16_t)(ETH_HDR_LEN + ptp_len);

    union {
        uint8_t info_recs_buff[L2TAP_IREC_SPACE(sizeof(struct timespec))];
        l2tap_irec_hdr_t align;
    } u;

    l2tap_extended_buff_t ext = {
        .info_recs_len  = sizeof(u.info_recs_buff),
        .info_recs_buff = u.info_recs_buff,
        .buff           = frame,
        .buff_len       = total,
    };

    l2tap_irec_hdr_t *ts_info = L2TAP_IREC_FIRST(&ext);
    ts_info->len  = L2TAP_IREC_LEN(sizeof(struct timespec));
    ts_info->type = L2TAP_IREC_TIME_STAMP;

    int ret = write(fd, &ext, 0);

    if (ret > 0 && tx_time && ts_info->type == L2TAP_IREC_TIME_STAMP) {
        struct timespec *tsp = (struct timespec *)ts_info->data;
        tx_time->seconds     = (Integer32)tsp->tv_sec;
        tx_time->nanoseconds = (Integer32)tsp->tv_nsec;
        DBGV("gptp TX ts: %ds %dns", tx_time->seconds, tx_time->nanoseconds);
    }

    return ret;
}

/* ============================================================
 *  netInit – open L2TAP, set interface + EtherType filter
 * ============================================================ */
Boolean netInit(NetPath *netPath, PtpClock *ptpClock)
{
    const char *ifaceName = ptpClock->rtOpts->ifaceName;
    DBG("netInit: interface=%s", ifaceName);

    netPath->fd = open("/dev/net/tap", O_RDWR);
    if (netPath->fd < 0) {
        ERROR("netInit: open /dev/net/tap failed: %d", errno);
        return FALSE;
    }

    if (ioctl(netPath->fd, L2TAP_S_INTF_DEVICE, ifaceName) < 0) {
        ERROR("netInit: L2TAP_S_INTF_DEVICE failed: %d", errno);
        close(netPath->fd);
        return FALSE;
    }

    uint16_t eth_type = PTP_ETHERTYPE;
    if (ioctl(netPath->fd, L2TAP_S_RCV_FILTER, &eth_type) < 0) {
        ERROR("netInit: L2TAP_S_RCV_FILTER failed: %d", errno);
        close(netPath->fd);
        return FALSE;
    }

    if (ioctl(netPath->fd, L2TAP_S_TIMESTAMP_EN) < 0) {
        ERROR("netInit: L2TAP_S_TIMESTAMP_EN failed: %d", errno);
        close(netPath->fd);
        return FALSE;
    }

    /* Retrieve the Ethernet driver handle to access MAC address */
    esp_eth_handle_t eth_handle = NULL;
    if (ioctl(netPath->fd, L2TAP_G_DEVICE_DRV_HNDL, &eth_handle) < 0) {
        ERROR("netInit: L2TAP_G_DEVICE_DRV_HNDL failed: %d", errno);
        close(netPath->fd);
        return FALSE;
    }

    esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, netPath->hwaddr);
    ESP_LOGI(GPTP_TAG, "netInit: MAC=%02x:%02x:%02x:%02x:%02x:%02x",
             netPath->hwaddr[0], netPath->hwaddr[1], netPath->hwaddr[2],
             netPath->hwaddr[3], netPath->hwaddr[4], netPath->hwaddr[5]);

    /* Store MAC address as UUID (EUI-48) for clock identity */
    memcpy(ptpClock->portUuidField, netPath->hwaddr, PTP_UUID_LENGTH);

    /* Add gPTP multicast filters so the MAC passes these frames up */
    uint8_t addr[6];
    memcpy(addr, GPTP_MCAST_MAC, 6);
    esp_eth_ioctl(eth_handle, ETH_CMD_ADD_MAC_FILTER, addr);
    /* Also add standard 802.3 PTP multicast so we can see both */
    addr[0]=0x01; addr[1]=0x1B; addr[2]=0x19;
    addr[3]=0x00; addr[4]=0x00; addr[5]=0x00;
    esp_eth_ioctl(eth_handle, ETH_CMD_ADD_MAC_FILTER, addr);

    netQInit(&netPath->eventQ);
    netQInit(&netPath->generalQ);

    return TRUE;
}

Boolean netShutdown(NetPath *netPath)
{
    if (netPath->fd >= 0) {
        close(netPath->fd);
        netPath->fd = -1;
    }
    netQInit(&netPath->eventQ);
    netQInit(&netPath->generalQ);
    return TRUE;
}

/* ============================================================
 *  netSelect – return > 0 if queues have data
 *  (actual draining happens in gptp_net_poll_and_queue)
 * ============================================================ */
Integer32 netSelect(NetPath *netPath, const TimeInternal *timeout)
{
    (void)timeout;
    return (netPath->eventQ.count > 0 || netPath->generalQ.count > 0) ? 1 : 0;
}

ssize_t netRecvEvent(NetPath *netPath, Octet *buf, TimeInternal *time)
{
    return netQGet(&netPath->eventQ, (uint8_t *)buf, time);
}

ssize_t netRecvGeneral(NetPath *netPath, Octet *buf, TimeInternal *time)
{
    return netQGet(&netPath->generalQ, (uint8_t *)buf, time);
}

/* ============================================================
 *  Send functions
 *  In gPTP (802.1AS) all messages go to 01:80:C2:00:00:0E.
 *  Event messages request HW TX timestamps; general messages do not.
 * ============================================================ */

ssize_t netSendEvent(NetPath *netPath, const Octet *buf,
                     UInteger16 length, TimeInternal *time)
{
    return gptp_l2tap_send(netPath->fd, netPath->hwaddr, buf, length, time);
}

ssize_t netSendGeneral(NetPath *netPath, const Octet *buf, UInteger16 length)
{
    return gptp_l2tap_send(netPath->fd, netPath->hwaddr, buf, length, NULL);
}

/* P2P peer event/general – same MAC as regular event/general in 802.1AS */
ssize_t netSendPeerEvent(NetPath *netPath, const Octet *buf,
                         UInteger16 length, TimeInternal *time)
{
    return gptp_l2tap_send(netPath->fd, netPath->hwaddr, buf, length, time);
}

ssize_t netSendPeerGeneral(NetPath *netPath, const Octet *buf, UInteger16 length)
{
    return gptp_l2tap_send(netPath->fd, netPath->hwaddr, buf, length, NULL);
}

void netEmptyEventQ(NetPath *netPath)
{
    netQInit(&netPath->eventQ);
}
