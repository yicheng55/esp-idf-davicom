#ifndef PLATFORM_INFO_EXT_H
#define PLATFORM_INFO_EXT_H

/* WILL such as devif-w-api */
 const uint8_t *DM_ETH_Init_W(uint8_t *adr);
 void tx_manager_dispatch_w(uint8_t *buffer, uint16_t len, struct pbuf *p);
 uint16_t rx_manager_dispatch_w(uint8_t *buf);
 void rx_manager_dispatch_pbuf(uint8_t *buf, struct pbuf *p);
 err_t rx_igmp_mac_filter_w(struct netif *netif, const ip4_addr_t *group, enum netif_mac_filter_action action);
 
#endif //PLATFORM_INFO_EXT_H
