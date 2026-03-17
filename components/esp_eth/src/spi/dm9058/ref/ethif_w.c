/**
 **************************************************************************
 * @file     ethif.c
 * @version  v1.0.1
 * @date     2025-10-15
 * @brief    DM9058 Ethernet driver uip interface support file (or referred to be as eth.c)
 **************************************************************************
 *
 * To restructure and improve the file to enhance readability, maintainability,
 * and potentially performance.
 * Last updated: 2025-10-15
 *
 */
#include "lwip/opt.h"
#include "lwip/netif.h"
#include "../../dm9058_u2510_if/platform_info.h" //"all, as main.c including"
//#include "../../dm9058_u2510_if/_dhcp_status.h"
//#include "../../dm9058_u2510_if/_ip_status.h"

extern struct netif netif;

uint8_t *identified_tcpip_mac(void)
{
	return (uint8_t *)netif.hwaddr;
}

uint8_t *identified_tcpip_ip(void)
{
	return (uint8_t *)&netif.ip_addr.addr;
}
uint8_t *identified_tcpip_gw(void)
{
	return (uint8_t *)&netif.gw.addr;
}
uint8_t *identified_tcpip_mask(void)
{
	return (uint8_t *)&netif.netmask.addr;
}

/**
 * @brief  Network configuration functions
 */
//const uint8_t *DM_ETH_Ip_Configuration(const uint8_t *ip)
//{
////	static uint8_t ip_printag = 0x1;
////	if (ip_printag & 0x01) {
////		ip_printag &= ~0x01;
////		identify_tcpip_ip(ip);
////		return dm_eth_show_identified_ip(ip ? "config ip" : "candidate ip");
////	}
//	return identify_tcpip_ip(ip);
//}
//const uint8_t *DM_ETH_Gw_Configuration(const uint8_t *ip)
//{
////	static uint8_t gw_printag = 0x1;
////	if (gw_printag & 0x01) {
////		gw_printag &= ~0x01;
////		identify_tcpip_gw(ip);
////		return dm_eth_show_identified_gw(ip ? "config gw" : "candidate gw");
////	}
//	return identify_tcpip_gw(ip);
//}
//const uint8_t *DM_ETH_Mask_Configuration(const uint8_t *ip)
//{
//  return identify_tcpip_mask(ip);
//}
