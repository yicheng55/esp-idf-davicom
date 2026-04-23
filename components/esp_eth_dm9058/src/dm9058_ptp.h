/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __DM9058_PTP_H
#define __DM9058_PTP_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#include "lwip/opt.h" //[TEMP for LWIP_PTP]
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/prot/ieee.h"

#include "netconf.h"

// PTP FIELD
#ifndef PTP_ETHERTYPE
/* Single source of truth: lwIP's IEEE ethertype definition (0x88F7U). */
#define PTP_ETHERTYPE ((uint16_t)ETHTYPE_PTP) /* Layer 2 PTP */
#endif
#define PTP_EVENT_PORT                        319    // UDP PTP EVENT
#define PTP_GENERAL_PORT                      320    // UDP PTP GENERAL

/**
 * @brief DM9058 PTP Debug Control
 * @note  Set to 1 to enable PTP debug messages, 0 to disable
 */
//#define DM9058_PTP_DEBUG 	1
#ifndef DM9058_PTP_DEBUG
  #define DM9058_PTP_DEBUG 0
#endif

#if DM9058_PTP_DEBUG
  #define dm9058_ptp_dbg(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
  #define dm9058_ptp_dbg(fmt, ...) ((void)0)
#endif

// 02H TX Control Reg
#define TCR_TSEN_CAP                 			TCR_RSV_BIT7
#define TCR_TS1STEP_EMIT             			TCR_TJDIS //TCR_DIS_JABBER_TIMER

// 06H RX Status Reg
// BIT(5),PTP use the same bit, timestamp is available
// BIT(3),PTP use the same bit, this is odd parity rx TimeStamp
// BIT(2),PTP use the same bit: 1 => 8-bytes, 0 => 4-bytes, for timestamp length
#define RSR_RXTS_EN                           (1 << 5)
#define RSR_RXTS_PARITY                       (1 << 3)
#define RSR_RXTS_LEN                          (1 << 2)
#define RSR_PTP_BITS                          (RSR_RXTS_EN | RSR_RXTS_PARITY | RSR_RXTS_LEN)

#if EDRIVER_ADDING_PTP && LWIP_PTP
struct ptptime_t {
  s32_t tv_sec;
  s32_t tv_nsec;
};

int is_issue_ptp_tstamp_emit(uint8_t *buf);
int is_issue_ptp_tstamp_tsen(uint8_t *buf);
void V51_JJ_tx_ptptime(struct ptptime_t *ptimestamp, char *head);
uint8_t ptp_tx_tstamp_parse_packet(uint8_t *buffer, struct pbuf *p);
void ptp_tx_tstamp_pass_to(uint8_t *buffer, struct pbuf *p, uint8_t tcr_wr);

  //[API]uip
const uint8_t *dm9058_init_ptp(const uint8_t *adr);
#endif

#if EDRIVER_ADDING_PTP && LWIP_PTP //(ptpd-2.0.0\src\dep\sys_time.c is using)
void v51_ptptime_add_offset(struct ptptime_t *timestamp);
void dm9058_ptptime_gettime(struct ptptime_t * timestamp);
void dm9058_ptptime_settime(struct ptptime_t * timestamp);
void dm9058_ptptime_updateoffset(struct ptptime_t * timeoffset);
void dm9058_ptptime_adjfreq(int32_t Adj);

/**
 * @brief Function aliases for compatibility with different naming conventions
 *
 * These aliases map the v51/dm9058 naming conventions to the dm9058_ptp_* naming
 * used throughout the project for consistency.
 */
#define dm9058_ptp_get_time(ts)           dm9058_ptptime_gettime(ts)
//#define dm9058_ptp_time_set_time(ts)      dm9058_ptptime_settime(ts)
//#define dm9058_ptp_time_update_offset(ts) dm9058_ptptime_updateoffset(ts)
//#define dm9058_ptp_time_adj_freq(adj)     dm9058_ptptime_adjfreq(adj)

/* PTP get_time latency measurement functions */
uint64_t dm9058_measure_ptp_get_time_latency(uint32_t iterations);
uint64_t dm9058_measure_ptp_get_time_with_dwt(uint32_t iterations);
#endif

//void v51_ptptime_gettime(struct ptptime_t *timestamp);
//void v51_ptptime_settime(struct ptptime_t *timestamp);
uint16_t dm9058_rx_ptp(uint8_t *buffer, uint8_t *receivedata, uint8_t *ts_bff); //uint16_t dm9058_rx_ptp(uint8_t *buff, uint8_t *ts_bff);
void dm9058_tx_ptp(uint8_t *buff, uint16_t len, struct pbuf *p);
//void _ptp_rx_tstamp_process(struct pbuf *p, uint8_t *buffer, uint8_t *receivedata, uint8_t *ts_bff, uint16_t len);
void ptp_rx_tstamp_process(struct pbuf *p, uint8_t *buffer, uint8_t *receivedata, uint8_t *ts_bff, uint16_t len);

uint16_t cspi_rx_head_ptp(uint8_t *receivedata, uint8_t *ts_bff);
uint16_t rx_head_takelen_ptp(uint8_t *receivedata, uint8_t *ts_bff);
void cspi_rx_tstamp_mem(uint8_t *receivedata, uint8_t *ts_bff);

void v51_ptptime_set_tx_timestamp_reg(uint8_t val);

void v51_getPtpClockRate1(uint32_t *rateValuePtr, uint8_t *sign);
void v51_adjust_ptp_frequency(uint32_t adjustment, int8_t direction);
//void v51_adjustPtpFrequency1(uint32_t adjustment, uint8_t direction);
//void v51_ptptime_zero_rate(void);

void v51_putPtpClockRate(int32_t rateValue/*, uint8_t sign*/);
void v51_pushPtpClockRate(int reverse);
void v51_updatePtpClockRate(int64_t signed_addend); //(int32_t adjust_value, uint8_t adjust_direction);

void internaltime_as_ymd_s(char *head, const s32_t seconds, s32_t nanoseconds);

#if EDRIVER_ADDING_PTP && LWIP_PTP
void impl_dm9058_get_tx_timestamp(struct ptptime_t *pTimeStamp);
void print_as_ymd(char *head, const struct ptptime_t *time);
void print_as_ymd_s(int rnd, char *head, const struct ptptime_t *time);
#endif

#ifdef __cplusplus
}
#endif

#endif //__DM9058_PTP_H
