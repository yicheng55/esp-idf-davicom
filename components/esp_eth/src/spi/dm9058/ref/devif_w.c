#include "lwip/opt.h" //[TEMP for LWIP_PTP]
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "../../dm9058_u2510_if/platform_info.h" //"all, as main.c including"
#include "dm9058_edriver_extend/dm9058_ptp.h"
//int rx_hash_igmp(uint32_t group, enum netif_mac_filter_action action);
err_t rx_igmp_mac_filter_w(struct netif *netif, const ip4_addr_t *group, enum netif_mac_filter_action action);

void dm9058_tx_ptp_if(uint8_t *buff, uint16_t len, struct pbuf *p);
uint16_t dm9058_rx_ptp_if(uint8_t *buffer);

int input_mode;
//extern int input_mode; //return from "dm9058_conf()"

/*struct proc_forward_t dm9058dev = {
	dm9058_init, //.init
	dm9058_rx, //.rx
	dm9058_tx, //.tx

#if (EDRIVER_ADDING_PTP && LWIP_PTP)
	dm9058_init_ptp, //.init_ptp
	dm9058_rx_ptp, //.rx_ptp

	rx_ptptime_debug, //.ptptime_process_rx
	dm9058_ptptime_gettime, //.ptptime_gettime
	dm9058_ptptime_settime, //ptptime_settime.
	dm9058_ptptime_updateoffset, //.ptptime_updateoffset
	dm9058_ptptime_adjfreq, //.ptptime_adjfreq
#endif
};*/

/* Initialize the Ethernet driver 
 * Usually, called by "low_level_init()"
 * cmp //void DM_ETH_Init(adr);
 */
const uint8_t *DM_ETH_Init_W(/*const*/ uint8_t *adr)
{
	input_mode = dm9058_conf();
#if (EDRIVER_ADDING_PTP && LWIP_PTP)
	return dm9058_init_ptp(adr);
#else
	return dm9058_init(adr);
#endif
}

void tx_manager_dispatch_w(uint8_t *buffer, uint16_t len, struct pbuf *p)
{
#if (EDRIVER_ADDING_PTP && LWIP_PTP)
	dm9058_tx_ptp_if(buffer, len, p);
#else
	dm9058_tx(buffer, len);
#endif
}

uint16_t rx_manager_dispatch_w(uint8_t *buf)
{
#if (EDRIVER_ADDING_PTP && LWIP_PTP)
	uint16_t len = dm9058_rx_ptp_if(buf); //low_level_input_timestamp(buf);
#else
	uint16_t len = dm9058_rx(buf);
#endif
	if (len) {
		dump_data(buf, len); //dm_eth_input_hexdump(buf, len);
	}
	return len;
}

/* IGMP callback for joining/leaving multicast groups */
//err_t rx_hash_igmp_w(const ip4_addr_t *group, enum netif_mac_filter_action action)
//{
//    switch (action) {
//        case NETIF_ADD_MAC_FILTER:
//			dm9058_rx_mode_add_hash(group->addr);
//            break;
//            
//        case NETIF_DEL_MAC_FILTER:
//			dm9058_rx_mode_del_hash(group->addr);
//            break;
//        default:
//            return ERR_ARG;
//    }
//    return ERR_OK;
//}

err_t rx_igmp_mac_filter_w(struct netif *netif, const ip4_addr_t *group, enum netif_mac_filter_action action)
{
	//return rx_hash_igmp_w(group, action);

	//if (!rx_hash_igmp(group->addr, action))
	//	return ERR_OK;
	//return ERR_ARG;
	
    switch (action) {
        case NETIF_ADD_MAC_FILTER:
			dm9058_rx_mode_add_hash(group->addr);
            break;
            
        case NETIF_DEL_MAC_FILTER:
			dm9058_rx_mode_del_hash(group->addr);
            break;
        default:
            return ERR_ARG; //return 1;
    }
    return ERR_OK; //return 0;
}
