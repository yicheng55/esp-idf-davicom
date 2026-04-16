/* net.c */

#include "../ptpd.h"
#include "lwip/inet.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "netconf.h"

#define ETH_HDR_LEN 14u
#define VLAN_ETH_HDR_LEN 18u

/* Forward declarations for file-local queue helpers (avoid implicit decl on ARMCC) */
static void netQInit(BufQueue *pQ);
static Boolean netQPut(BufQueue *pQ, void *pbuf);
static void netQEmpty(BufQueue *pQ);

static Integer32 findIface(const Octet *ifaceName, Octet *uuid, NetPath *netPath);

static NetPath *g_ptp_netpath_l2 = NULL;
static Enumeration8 g_ptp_transport_l2 = TRANSPORT_UDP_IPV4;

static const uint8_t PTP_MAC_IEEE_802_3[6]   = {0x01, 0x1B, 0x19, 0x00, 0x00, 0x00};
static const uint8_t PTP_MAC_IEEE_802_1AS[6] = {0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E};

static Boolean isEventMessageType(UInteger8 messageType)
{
    switch (messageType)
    {
    case SYNC:
    case DELAY_REQ:
    case PDELAY_REQ:
    case PDELAY_RESP:
        return TRUE;
    default:
        return FALSE;
    }
}

static int netRecvL2Callback(struct pbuf *p)
{
    static UInteger32 g_l2_seen = 0;
    static UInteger32 g_l2_queued = 0;
    uint16_t ethType;
    uint16_t hdrLen = ETH_HDR_LEN;
    UInteger8 messageType;
    BufQueue *targetQ;

    if (p == NULL || g_ptp_netpath_l2 == NULL)
        return 0;

    if (p->tot_len < ETH_HDR_LEN)
        return 0;

    /* Read Ethertype (big-endian) */
    {
        const uint8_t *b = (const uint8_t *)p->payload;
        ethType = ((uint16_t)b[12] << 8) | (uint16_t)b[13];

        /* Handle 802.1Q VLAN tag */
        if (ethType == ETHTYPE_VLAN)
        {
            if (p->tot_len < VLAN_ETH_HDR_LEN)
                return 0;
            ethType = ((uint16_t)b[16] << 8) | (uint16_t)b[17];
            hdrLen  = VLAN_ETH_HDR_LEN;
        }
    }

    if (ethType != PTP_ETHERTYPE)
        return 0;

    g_l2_seen++;

    /* Strip Ethernet header (and optional VLAN tag) */
    if (pbuf_remove_header(p, hdrLen) != 0)
        return 0;

    if (p->tot_len < 1)
    {
        pbuf_free(p);
        return 1;
    }

    messageType = ((const uint8_t *)p->payload)[0] & 0x0F;
    targetQ     = isEventMessageType(messageType) ? &g_ptp_netpath_l2->eventQ : &g_ptp_netpath_l2->generalQ;

    if (!netQPut(targetQ, p))
    {
        pbuf_free(p);
        ERROR("netRecvL2Callback: queue full\n");
        return 1;
    }

    g_l2_queued++;
    if ((g_l2_queued % 64u) == 0u)
    {
        DBGV("L2 PTP RX: seen=%lu queued=%lu lastType=%u\n",
             (unsigned long)g_l2_seen, (unsigned long)g_l2_queued, (unsigned)messageType);
    }

    return 1;
}

static Boolean netInitL2(NetPath *netPath, PtpClock *ptpClock)
{
    ip_addr_t interfaceAddr;

    DBG("netInitL2\n");

    /* Find interface to get MAC / uuid */
    interfaceAddr.addr = findIface(ptpClock->rtOpts->ifaceName, ptpClock->portUuidField, netPath);
    (void)interfaceAddr;

    netPath->eventPcb = NULL;
    netPath->generalPcb = NULL;
    netPath->multicastAddr = 0;
    netPath->peerMulticastAddr = 0;
    netPath->unicastAddr = 0;

    netQInit(&netPath->eventQ);
    netQInit(&netPath->generalQ);

    g_ptp_netpath_l2 = netPath;
    g_ptp_transport_l2 = ptpClock->rtOpts->transportType;

    /* Only intercept frames in L2 / gPTP modes */
    netconf_register_ptp_callback(netRecvL2Callback);
    DBGV("netInitL2: registered L2 RX callback, transport=%d\n", (int)g_ptp_transport_l2);

    return TRUE;
}

static ssize_t netSendL2(const Octet *buf, UInteger16 length, TimeInternal *time)
{
    struct netif *iface = netif_default;
    struct pbuf *p;
    uint8_t *out;
    const uint8_t *dstMac;
    err_t result;

    if (iface == NULL || iface->linkoutput == NULL)
    {
        ERROR("netSendL2: netif_default unavailable\n");
        return 0;
    }

    dstMac = (g_ptp_transport_l2 == TRANSPORT_IEEE_802_1AS) ? PTP_MAC_IEEE_802_1AS : PTP_MAC_IEEE_802_3;

    p = pbuf_alloc(PBUF_RAW, (uint16_t)(ETH_HDR_LEN + length), PBUF_RAM);
    if (p == NULL)
    {
        ERROR("netSendL2: Failed to allocate Tx Buffer\n");
        return 0;
    }

#if LWIP_PTP
    p->time_sec  = 0;
    p->time_nsec = 0;
#endif

    out = (uint8_t *)p->payload;
    memcpy(out + 0, dstMac, 6);
    memcpy(out + 6, iface->hwaddr, 6);
    out[12] = (uint8_t)((PTP_ETHERTYPE >> 8) & 0xFF);
    out[13] = (uint8_t)(PTP_ETHERTYPE & 0xFF);
    memcpy(out + ETH_HDR_LEN, buf, length);

    result = iface->linkoutput(iface, p);
    if (result != ERR_OK)
    {
        ERROR("netSendL2: linkoutput failed (%d)\n", result);
        pbuf_free(p);
        return 0;
    }

    if (time != NULL)
    {
#if LWIP_PTP
        time->seconds     = p->time_sec;
        time->nanoseconds = p->time_nsec;
        if (time->seconds == 0 && time->nanoseconds == 0)
        {
            DBGV("netSendL2: Hardware timestamp not available, using software timestamp\n");
            getTime(time);
        }
#else
        getTime(time);
#endif
    }

    pbuf_free(p);
    return length;
}

/**
  * @brief  Initialize network queue
  * @param  pQ the queue to be initialized
  * @retval None
  */
static void netQInit(BufQueue *pQ)
{
    pQ->get = 0;
    pQ->put = 0;
    pQ->count = 0;
}

/**
  * @brief  Put data to the network queue
  * @param  pQ the queue to be used
  * @param  pbuf the packet to be put to the queue
  * @retval Boolean success
  */
static Boolean netQPut(BufQueue *pQ, void *pbuf)
{
    if (pQ->count >= PBUF_QUEUE_SIZE)
        return FALSE;

    pQ->pbuf[pQ->put] = pbuf;

    pQ->put = (pQ->put + 1) % PBUF_QUEUE_SIZE;

    pQ->count++;

    return TRUE;
}

/**
  * @brief  Get data from the network queue
  * @param  pQ the queue to be used
  * @retval void* pointer to pbuf or NULL
  */
static void *netQGet(BufQueue *pQ)
{
    void *pbuf;

    if (!pQ->count)
        return NULL;

    pbuf = pQ->pbuf[pQ->get];

    pQ->get = (pQ->get + 1) % PBUF_QUEUE_SIZE;

    pQ->count--;

    return pbuf;
}

/**
  * @brief  Free all pbufs in the queue
  * @param  pQ the queue to be used
  * @retval None
  */
static void netQEmpty(BufQueue * pQ)
{

    struct pbuf * p;
    int cnt = pQ->count;

    for (;cnt > 0; cnt--)
    {
        p = (struct pbuf*)netQGet(pQ);

        if (p) pbuf_free(p);
    }
}

/**
  * @brief  Check if something is in the queue
  * @param  pQ the queue to be used
  * @retval Boolean TRUE if success
  */
static Boolean netQCheck(BufQueue *pQ)
{
    if (!pQ->count)
        return FALSE;

    return TRUE;
}

/**
  * @brief  shut down the UDP and network stuff
  * @retval TRUE if success
  */
Boolean netShutdown(NetPath *netPath)
{

    ip_addr_t multicastAaddr;

    DBG("netShutdown\n");

    /* stop L2 interception */
    if (g_ptp_netpath_l2 == netPath)
    {
        netconf_register_ptp_callback(NULL);
        g_ptp_netpath_l2 = NULL;
        g_ptp_transport_l2 = TRANSPORT_UDP_IPV4;
    }

    netQEmpty(&netPath->eventQ);
    netQEmpty(&netPath->generalQ);

    /* L2 mode uses no multicast group / UDP PCBs */
    if (netPath->eventPcb == NULL && netPath->generalPcb == NULL)
    {
        netPath->multicastAddr = 0;
        netPath->peerMulticastAddr = 0;
        netPath->unicastAddr = 0;
        return TRUE;
    }

    /* leave multicast group */
    if (netPath->multicastAddr != 0)
    {
        multicastAaddr.addr = netPath->multicastAddr;
        igmp_leavegroup(IP_ADDR_ANY, &multicastAaddr);
    }

    /* Disconnect and close the Event UDP interface */

    if (netPath->eventPcb)
    {
        udp_disconnect(netPath->eventPcb);
        udp_remove(netPath->eventPcb);
        netPath->eventPcb = NULL;
    }

    /* Disconnect and close the General UDP interface */
    if (netPath->generalPcb)
    {
        udp_disconnect(netPath->generalPcb);
        udp_remove(netPath->generalPcb);
        netPath->generalPcb = NULL;
    }

    netPath->multicastAddr = 0;

    netPath->unicastAddr = 0;

    /* Clear the network addresses. */
    netPath->multicastAddr = 0;
    netPath->unicastAddr = 0;

    /* Return a success code. */
    return TRUE;
}

/**
  * @brief  Find interface to be used
  * @param  ifaceName name of the required interface (will be filled with appropriate interface name if not set)
  * @param  uuid will be filled with MAC address of the interface
  * @param  netPath network object
  * @retval Integer32 IPv4 address of the interface
  */
static Integer32 findIface(const Octet *ifaceName, Octet *uuid, NetPath *netPath)
{

    struct netif * iface;
    iface = netif_default;
    memcpy(uuid, iface->hwaddr, iface->hwaddr_len);
    return iface->ip_addr.addr;
}

/**
  * @brief  Processing an incoming message on the Event port.
  * @param  arg the user argument
  * @param  pcb the tcp_pcb that has received the data
  * @param  p the packet buffer
  * @param  addr the addres of sender
  * @param  port the port number of sender
  * @retval None
  */
static void netRecvEventCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                 ip_addr_t *addr, u16_t port)
{
    NetPath *netPath = (NetPath *)arg;
    static UInteger32 g_udp_event_queued = 0;

    /* Place the incoming message on the Event Port QUEUE. */

    if (!netQPut(&netPath->eventQ, p))
    {
        pbuf_free(p);
        p = NULL;
        ERROR("netRecvEventCallback: queue full\n");
        return;
    }

    g_udp_event_queued++;
    if ((g_udp_event_queued % 64u) == 0u)
    {
        DBGV("UDP PTP RX(event): queued=%lu\n", (unsigned long)g_udp_event_queued);
    }
}

/**
  * @brief  Processing an incoming message on the General port.
  * @param  arg the user argument
  * @param  pcb the tcp_pcb that has received the data
  * @param  p the packet buffer
  * @param  addr the addres of sender
  * @param  port the port number of sender
  * @retval None
  */
static void netRecvGeneralCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                   ip_addr_t *addr, u16_t port)
{
    NetPath *netPath = (NetPath *)arg;
    static UInteger32 g_udp_general_queued = 0;

    /* Place the incoming message on the Event Port QUEUE. */

    if (!netQPut(&netPath->generalQ, p))
    {
        pbuf_free(p);
        p = NULL;
        ERROR("netRecvGeneralCallback: queue full\n");
        return;
    }

    g_udp_general_queued++;
    if ((g_udp_general_queued % 64u) == 0u)
    {
        DBGV("UDP PTP RX(general): queued=%lu\n", (unsigned long)g_udp_general_queued);
    }
}

/**
  * @brief  Start all of the UDP stuff
  * @param  netPath network object
  * @param  ptpClock PTP clock object
  * @retval Boolean success
  */
Boolean netInit(NetPath *netPath, PtpClock *ptpClock)
{

    ip_addr_t interfaceAddr;

    ip_addr_t netAddr;
    char addrStr[NET_ADDRESS_LENGTH];

    DBG("netInit\n");

    /* In UDP mode, ensure no L2 interception remains registered */
    if (ptpClock->rtOpts->transportType == TRANSPORT_UDP_IPV4)
    {
        netconf_register_ptp_callback(NULL);
        DBGV("netInit: UDP/IPv4 mode, L2 callback disabled\n");
    }
    else
    {
        return netInitL2(netPath, ptpClock);
    }

    /* find a network interface */
    interfaceAddr.addr = findIface(ptpClock->rtOpts->ifaceName, ptpClock->portUuidField, netPath);

    if (!(interfaceAddr.addr))
    {

        goto fail01;
    }

    /* Open lwIP raw udp interfaces for the event port. */
    netPath->eventPcb = udp_new();

    if (NULL == netPath->eventPcb)
    {
        ERROR("netInit: Failed to open Event UDP PCB\n");
        goto fail02;
    }

    /* Open lwIP raw udp interfaces for the general port. */
    netPath->generalPcb = udp_new();

    if (NULL == netPath->generalPcb)
    {
        ERROR("netInit: Failed to open General UDP PCB\n");
        goto fail03;
    }

    /* Initialize the buffer queues. */
    netQInit(&netPath->eventQ);

    netQInit(&netPath->generalQ);

    /* Configure network (broadcast/unicast) addresses. */
    netPath->unicastAddr = 0; /* disable unicast */

    /*Init General multicast IP address*/
    memcpy(addrStr, DEFAULT_PTP_DOMAIN_ADDRESS, NET_ADDRESS_LENGTH);

    if (!inet_aton(addrStr, &netAddr))
    {
        ERROR("netInit: failed to encode multi-cast address: %s\n", addrStr);
        goto fail04;
    }

    DBG("netInit: multicast address set directly: %s (0x%08x)\n", addrStr, netAddr.addr);

    netPath->multicastAddr = netAddr.addr;

    /* join multicast group (for receiving) on specified interface */
    igmp_joingroup(&interfaceAddr, (ip_addr_t *)&netAddr);


    /*Init Peer multicast IP address*/
    memcpy(addrStr, PEER_PTP_DOMAIN_ADDRESS, NET_ADDRESS_LENGTH);

    if (!inet_aton(addrStr, &netAddr))
    {
        ERROR("netInit: failed to encode peer multi-cast address: %s\n", addrStr);
        goto fail04;
    }

    DBG("netInit: peer multicast address set directly: %s (0x%08x)\n", addrStr, netAddr.addr);

    netPath->peerMulticastAddr = netAddr.addr;

    /* join peer multicast group (for receiving) on specified interface */
    igmp_joingroup(&interfaceAddr, (ip_addr_t *)&netAddr);


    /* multicast send only on specified interface */
    netPath->eventPcb->mcast_ip4.addr = netPath->multicastAddr;
    netPath->generalPcb->mcast_ip4.addr = netPath->multicastAddr;

    /* Establish the appropriate UDP bindings/connections for events. */
    udp_recv(netPath->eventPcb, (udp_recv_fn)netRecvEventCallback, netPath);
    udp_bind(netPath->eventPcb, IP_ADDR_ANY, PTP_EVENT_PORT);
    /*  udp_connect(netPath->eventPcb, &netAddr, PTP_EVENT_PORT); */

    /* Establish the appropriate UDP bindings/connections for general. */
    udp_recv(netPath->generalPcb, (udp_recv_fn)netRecvGeneralCallback, netPath);
    udp_bind(netPath->generalPcb, IP_ADDR_ANY, PTP_GENERAL_PORT);
    /*  udp_connect(netPath->generalPcb, &netAddr, PTP_GENERAL_PORT); */

    /* Return a success code. */
    return TRUE;

    /*
    fail05:
        udp_disconnect(netPath->eventPcb);
        udp_disconnect(netPath->generalPcb);
    */
fail04:
    udp_remove(netPath->generalPcb);
fail03:
    udp_remove(netPath->eventPcb);
fail02:
fail01:
    return FALSE;
}

/**
  * @brief  Wait for a packet to come in on either port.  For now, there is no wait.
  * Simply check to see if a packet is available on either port and return 1,
  * otherwise return 0.
  * @param  netPath network object
  * @param  timeout not used
  * @retval Integer32 number > 0 if there are some data
  */
Integer32 netSelect(NetPath *netPath, const TimeInternal *timeout)
{
    /* Check the packet queues.  If there is data, return TRUE. */
    if (netQCheck(&netPath->eventQ) || netQCheck(&netPath->generalQ))
        return 1;

    return 0;
}

/**
  * @brief  Delete all waiting packets in Event queue
  * @param  netPath network object
  * @retval None
  */
void netEmptyEventQ(NetPath *netPath)
{
    netQEmpty(&netPath->eventQ);
}

static ssize_t netRecv(Octet *buf, TimeInternal *time, BufQueue * msgQueue)
{
    ssize_t length;
    int i, j;

    /* get actual buffer */

    struct pbuf * p, *pcopy;
    p = (struct pbuf*)netQGet(msgQueue);

    if (!p)
    {
        return 0;
    }

    pcopy = p;

    /* Here, p points to a valid PBUF structure.  Verify that we have
     * enough space to store the contents. */

    if (p->tot_len > PACKET_SIZE)
    {
        ERROR("netRecv: received truncated message\n");
        return 0;
    }

    if (NULL != time)
    {
#if LWIP_PTP
        time->seconds = p->time_sec;
        time->nanoseconds = p->time_nsec;
#else
        getTime(time);
#endif
    }

    /* Copy the PBUF payload into the buffer. */
    j = 0;

    length = p->tot_len;

    for (i = 0; i < length; i++)
    {
        buf[i] = ((u8_t *)pcopy->payload)[j++];

        if (j == pcopy->len)
        {
            pcopy = pcopy->next;
            j = 0;
        }
    }

    /* Free up the pbuf (chain). */
    pbuf_free(p);

    return length;
}

ssize_t netRecvEvent(NetPath *netPath, Octet *buf, TimeInternal *time)
{
    return netRecv(buf, time, &netPath->eventQ);
}

ssize_t netRecvGeneral(NetPath *netPath, Octet *buf, TimeInternal *time)
{
    return netRecv(buf, time, &netPath->generalQ);
}

static ssize_t netSend(const Octet *buf, UInteger16 length, TimeInternal *time, const Integer32 * addr, struct udp_pcb * pcb)
{
    err_t result;

    struct pbuf * p;

    /* Allocate the tx pbuf based on the current size. */
    p = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);

    if (NULL == p)
    {
        ERROR("netSend: Failed to allocate Tx Buffer\n");
        goto fail01;
    }

#if LWIP_PTP
    /* 初始化時間戳欄位，避免讀取到記憶體池中的舊值 */
    p->time_sec = 0;
    p->time_nsec = 0;
#endif

    /* Copy the incoming data into the pbuf payload. */
    result = pbuf_take(p, buf, length);

    if (ERR_OK != result)
    {
        ERROR("netSend: Failed to copy data to Pbuf (%d)\n", result);
        goto fail02;
    }

    /* send the buffer. */
    result = udp_sendto(pcb, p, (void *)addr, pcb->local_port);

    if (ERR_OK != result)
    {
        ERROR("netSend: Failed to send data (%d)\n", result);
        goto fail02;
    }

    if (NULL != time)
    {
#if LWIP_PTP
        /* 
         * 注意：此處的時間戳是從 pbuf 中讀取
         * 底層驅動應該在發送時填入硬體時間戳
         * 如果時間戳為 0，表示底層驅動尚未填入或發送失敗
         */
        time->seconds = p->time_sec;
        time->nanoseconds = p->time_nsec;
        
        /* 添加時間戳有效性檢查 */
        if (time->seconds == 0 && time->nanoseconds == 0)
        {
            /* 時間戳未被填入，使用軟體時間戳作為降級方案 */
            DBGV("netSend: Hardware timestamp not available, using software timestamp\n");
            getTime(time);
        }
        else
        {
            DBGV("netSend: Hardware TX timestamp: %ds %dns\n", time->seconds, time->nanoseconds);
        }
#else
        /* TODO: use of loopback mode */
        /*
        time->seconds = 0;
        time->nanoseconds = 0;
        */
        getTime(time);
#endif
        DBGV("netSend: %ds %dns\n", time->seconds, time->nanoseconds);
    } else {
        DBGV("netSend\n");
    }


fail02:
    pbuf_free(p);

fail01:
    return length;

    /*  return (0 == result) ? length : 0; */
}

ssize_t netSendEvent(NetPath *netPath, const Octet *buf, UInteger16 length, TimeInternal *time)
{
    if (netPath->eventPcb == NULL && netPath->generalPcb == NULL)
        return netSendL2(buf, length, time);
    return netSend(buf, length, time, &netPath->multicastAddr, netPath->eventPcb);
}

ssize_t netSendGeneral(NetPath *netPath, const Octet *buf, UInteger16 length)
{
    if (netPath->eventPcb == NULL && netPath->generalPcb == NULL)
        return netSendL2(buf, length, NULL);
    return netSend(buf, length, NULL, &netPath->multicastAddr, netPath->generalPcb);
}

ssize_t netSendPeerGeneral(NetPath *netPath, const Octet *buf, UInteger16 length)
{
    if (netPath->eventPcb == NULL && netPath->generalPcb == NULL)
        return netSendL2(buf, length, NULL);
    return netSend(buf, length, NULL, &netPath->peerMulticastAddr, netPath->generalPcb);
}

ssize_t netSendPeerEvent(NetPath *netPath, const Octet *buf, UInteger16 length, TimeInternal* time)
{
    if (netPath->eventPcb == NULL && netPath->generalPcb == NULL)
        return netSendL2(buf, length, time);
    return netSend(buf, length, time, &netPath->peerMulticastAddr, netPath->eventPcb);
}
