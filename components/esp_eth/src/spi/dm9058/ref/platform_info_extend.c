/**
 **************************************************************************
 * @file     platform_info.c
 * @version  v1.0.1
 * @date     2024-12-12
 * @brief    DM9058 Ethernet driver info file
 **************************************************************************
 *
 * To restructure and improve the file to enhance readability, maintainability,
 * and potentially performance.
 * Last updated: 2024-12-12
 *
 */
#include "lwip/netif.h"
#include "netconf.h"
#include "../../dm9058_u2510_if/platform_info.h"
uint16_t link_display(uint16_t regvalue, struct netif *netif);
void dm_wait2_periodic_stat(void);

/**
 * @brief  Notify user about link status changes
 * @note   Handles network interface state changes
 * @param  netif: pointer to network interface structure
 * @retval none
 */
void ethernetif_notify_conn_changed(struct netif *netif)
{
    if (netif_is_link_up(netif)) {
        netif_set_up(netif);
#if LWIP_DHCP
		printf("(up) notify\r\n");
		printf("----------------------------- dhcp_start(netif)  ------------------ \r\n");
		printf(" LINK-UP, on link_changed, Call dhcp_start(netif)\r\n");
        dhcp_start(netif);
		#if 1 //state = DM_ENUM_COMBINED_WAIT_NET;
		dm_wait2_periodic_stat();
		#endif
#else
		printf("(up %d.%d.%d.%d) notify\r\n",
			ip4_addr1(&netif->ip_addr), ip4_addr2(&netif->ip_addr),
			ip4_addr3(&netif->ip_addr), ip4_addr4(&netif->ip_addr));
#endif
    } else {
		printf("(down)\r\n");
        netif_set_down(netif);
    }
}

/**
 * @brief = _dm_eth_polling_downup()
 * @brief  Set network interface link status
 * @note   Manages the netif link status based on physical link state
 * @param  argument: pointer to network interface structure
 * @retval 1 if link status changed to up, 0 otherwise
 */
int ethernetif_set_link(void const *argument)
{
	int on_dhcp = LWIP_DHCP;
    struct netif *netif = (struct netif *)argument;
    //int linkchg_up = 0;
	uint16_t regvalue = dm9058_link_update();
	link_display(regvalue, netif);
	if (!netif_is_link_up(netif) && regvalue) {
		netif_set_link_up(netif);
		//printf("dm9058 link up\r\n");
		printf("%s dm9058 link up\r\n", on_dhcp ? "DHCP" : "static IP");
		//#if _LWIP_MQTT
		//#endif
		return 1; //linkchg_up = 1;
	} else if (netif_is_link_up(netif) && !regvalue) {
		netif_set_link_down(netif);
		//printf("dm9058 link down\r\n");
		printf("%s dm9058 link down\r\n", on_dhcp ? "DHCP" : "static IP");
		//#if _LWIP_MQTT
		//#endif
	}
    return 0; //return linkchg_up;
}
