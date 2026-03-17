#include "lwip/pbuf.h"
#include "../../dm9058_u2510_if/platform_info.h" //"all, as main.c including"
#include "dm9058_edriver_extend/dm9058_ptp.h"
#include "devif_ptp.h"
void rx_tstamp_2_pbuf(struct pbuf *p);

extern uint8_t ts_receivedata[4];
extern uint8_t ts_buf[8];

//[ethif.info header file]
#define TX_MANAGER_RAW	0
#define TX_MANAGER_PTP	1
struct proc_info_t {
	/*
	 * .eth_tx_dynamic
	 */
	int tx_manager;
	/*void (*tx_dispatch)(uint8_t *buffer, uint16_t l, struct pbuf *p);
	void (*tx_update)(void);*/
};

struct proc_info_t dm9058etc = {
	/*
	 * .eth_tx_dynamic
	 */
	TX_MANAGER_RAW,
	/*
	tx_manager_dispatch_w,
	tx_manager_update, //.tx_update (NOT called this way.)
	*/
};

void tx_manager_update(void) {
	dm9058etc.tx_manager = TX_MANAGER_PTP; //ptp_inst.eth_tx_dynamic = ptp_inst.eth_tx_operate_ptp;
	printf("[TS-DEBUG] tx_manager_update: Switched to PTP mode (tx_manager=%d)\r\n", 
		dm9058etc.tx_manager);
}

void dm9058_tx_ptp_if_trans(uint8_t *buff, uint16_t len, struct pbuf *p)
{	
#if (EDRIVER_ADDING_PTP && LWIP_PTP)
	switch(dm9058etc.tx_manager) {
		case TX_MANAGER_RAW:
			dm9058_tx(buff, len);
			break;
		case TX_MANAGER_PTP:
			dm9058_tx_ptp(buff, len, p);
			break;
	}
#else
  dm9058_tx(buff, len);
#endif
}
void rx_manager_dispatch_pbuf_trans(uint8_t *buf, struct pbuf *p)
{
	switch(dm9058etc.tx_manager) {
		case TX_MANAGER_RAW:
			break;
		case TX_MANAGER_PTP:
			rx_tstamp_2_pbuf(p);
			rx_ptptime_debug(buf, p->tot_len); //if (len)
			break;
	}
}

void rx_tstamp_2_pbuf(struct pbuf *p)
{
	if (p != NULL) {
		if (ts_receivedata[1] & RSR_RXTS_EN) {
			p->time_sec = (uint32_t)ts_buf[3] | (uint32_t)ts_buf[2] << 8 | (uint32_t)ts_buf[1] << 16 | (uint32_t)ts_buf[0] << 24;
			p->time_nsec = (uint32_t)ts_buf[7] | (uint32_t)ts_buf[6] << 8 | (uint32_t)ts_buf[5] << 16 | (uint32_t)ts_buf[4] << 24;
		}
	}
}
