/**
 **************************************************************************
 * @file     devif_ptp.h
 * @brief    DM9058 PTP device interface header
 * 
 * @details  Declares interface functions for PTP packet processing
 *           and timestamp handling in the DM9058 driver.
 * 
 * @version  v1.0.0
 * @author   Joseph CHANG <joseph_chang@davicom.com.tw>
 * @copyright (c) 2023-2025 Davicom Semiconductor, Inc.
 * @date     2024-12-12
 **************************************************************************
 */
#ifndef DEVIF_PTP_H
#define DEVIF_PTP_H

#include <stdint.h>
#include "lwip/pbuf.h"

/**
 * @brief  Debug function for RX PTP timestamp information
 * @param  buffer: Pointer to received packet buffer
 * @param  len: Length of the packet
 * @retval None
 */
void rx_ptptime_debug(uint8_t *buffer, uint16_t len);

/**
 * @brief  Dispatch RX packet to PTP processing buffer
 * @param  buf: Pointer to packet buffer
 * @param  p: Pointer to pbuf structure
 * @retval None
 */
void rx_manager_dispatch_pbuf_trans(uint8_t *buf, struct pbuf *p);

/**
 * @brief  Get PTP header from packet buffer
 * @param  buffer: Pointer to packet buffer
 * @param  len: Length of the packet
 * @param  ts_en: Timestamp enable flag
 * @retval Pointer to PTP header or NULL if not found
 */
char *get_ptp_header(uint8_t *buffer, uint16_t len, uint8_t ts_en);

#endif /* DEVIF_PTP_H */
