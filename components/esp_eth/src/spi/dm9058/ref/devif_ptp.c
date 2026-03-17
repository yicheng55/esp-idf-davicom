#include "lwip/pbuf.h"
#include "../../dm9058_u2510_if/platform_info.h" //"all, as main.c including"
#include "dm9058_edriver_extend/dm9058_ptp.h"
void dm9058_tx_ptp_if_trans(uint8_t *buff, uint16_t len, struct pbuf *p);
void rx_manager_dispatch_pbuf_trans(uint8_t *buf, struct pbuf *p);
void publish_sync_info(char *payload);

#if (EDRIVER_ADDING_PTP && LWIP_PTP)

uint8_t ts_receivedata[4];
uint8_t ts_buf[8];

//[ethif.info header file]
struct proc_forward_t {
	const uint8_t *(*init)(const uint8_t *adr);
	uint16_t (*rx)(uint8_t *buffer);
	void (*tx)(uint8_t *buf, uint16_t len);

#if (EDRIVER_ADDING_PTP && LWIP_PTP)
	const uint8_t *(*init_ptp)(const uint8_t *adr); //void (*init_ptp)(uint8_t sel); //void (*eth_init)(void);

	// .eth_rx_static
	uint16_t (*rx_ptp)(uint8_t *buffer, uint8_t *receivedata, uint8_t *ts_bff);
	void (*ptptime_process_rx)(uint8_t *buffer, uint16_t len);

	void (*ptptime_gettime)(struct ptptime_t * timestamp);
	void (*ptptime_settime)(struct ptptime_t * timestamp);
	void (*ptptime_updateoffset)(struct ptptime_t * timeoffset);
	void (*ptptime_adjfreq)(int32_t Adj);
#endif
};

void dm9058_tx_ptp_if(uint8_t *buff, uint16_t len, struct pbuf *p)
{
	dm9058_tx_ptp_if_trans(buff, len, p);
}
//uint16_t low_level_input_timestamp(uint8_t *buffer)
//{
//	return dm9058_rx_ptp(buffer, ts_receivedata, ts_buf); //dm9058dev.rx_ptp(buffer, receivedata, ts_bff); //DM_ETH_Input_W_ptp
//}
uint16_t dm9058_rx_ptp_if(uint8_t *buffer)
{
#if (EDRIVER_ADDING_PTP && LWIP_PTP)
	return dm9058_rx_ptp(buffer, ts_receivedata, ts_buf); //dm9058dev.rx_ptp(buffer, receivedata, ts_bff); //DM_ETH_Input_W_ptp
#else
	return dm9058_rx(buf); //dm9058dev.rx(buffer);
#endif
}

void rx_manager_dispatch_pbuf(uint8_t *buf, struct pbuf *p)
{
	rx_manager_dispatch_pbuf_trans(buf, p);
}

//[mqtt help debug!]
#include "ptpd.h" //#include "ptpd_v51.h" //TEMP
char *get_ptp_header(uint8_t *p, uint16_t len, int rxts_en_packet);
void buffer_ts_time(uint8_t *buffer, TimeInternal *pTimeTmp);
char *parse_pbuf_ptp_packet1(uint8_t *buf, int in_offset);

/* to "dm9058_ptp.c" */
extern PtpClock ptpClock; //for debug log
extern struct ptptime_t delayREQ_T3, master_recv_extra_T3;
#define MQTT_PAYLOAD_BUFFER_SIZE					256

void pack_sync_info(char *buf, struct ptptime_t *pts_arrive, TimeInternal *pComeInPkt_t1, TimeInternal *pt2)
{
	sprintf(buf, "{ \"TimeStamp\" : \"D (Sync_arrive %d.%09d T2)(Unpack %d.%09d T1) diff %u s %u ns\" }",
		pts_arrive->tv_sec, pts_arrive->tv_nsec,
		pComeInPkt_t1->seconds, pComeInPkt_t1->nanoseconds,
		pt2->seconds, pt2->nanoseconds); // " .%d", ++ssc
}

void rx_ptptime_debug(uint8_t *buffer, uint16_t len)
{
	struct ptptime_t trapped_delayRESP_T4;
	char *ptp_hdr = get_ptp_header(buffer, len, ts_receivedata[1] & RSR_RXTS_EN);
	if (ptp_hdr)
	{
			MsgHeader    header;
			struct ptptime_t ts_arrive;

//			if ((header.versionPTP == ptpClock_v51.portDS.versionNumber) &&
//				(header.domainNumber == ptpClock_v51.defaultDS.domainNumber))
//			{
//			}
#if 1
			if (ts_receivedata[1] & RSR_RXTS_EN) { //=(rxts_en_packet)
				ts_arrive.tv_nsec =
					(uint32_t)ts_buf[7] | (uint32_t)ts_buf[6] << 8 | (uint32_t)ts_buf[5] << 16 | (uint32_t)ts_buf[4] << 24;
				ts_arrive.tv_sec =
					(uint32_t)ts_buf[3] | (uint32_t)ts_buf[2] << 8 | (uint32_t)ts_buf[1] << 16 | (uint32_t)ts_buf[0] << 24;
			}
#endif
#if 1
			msgUnpackHeader((const Octet *)(buffer+14+20+8), &header);
			switch (header.messageType)
			{
				
				case ANNOUNCE:
//						printf("in ANNOUNCE\r\n"); //handleAnnounce(ptpClock, isFromSelf);
				break;

				case SYNC:
//						printf("in SYNC\r\n"); //handleSync(ptpClock, &time, isFromSelf);
				#if 1			// disable yi-cheng 2025-11-01
				if (1) {
					TimeInternal comeInPkt_t1;
					TimeInternal t2;
					/* char payload[MQTT_PAYLOAD_BUFFER_SIZE]; */
					/* TODO: Uncomment when pack_sync_info is enabled */

					//[show diff to master.]
					buffer_ts_time(buffer+14+20+8, &comeInPkt_t1); //UnpackTimestamp((const Octet *)(buffer+14+20+8), &unpk_timestamp);
					//arrive_timestamp.tv_sec, arrive_timestamp.tv_nsec
					t2.seconds = ts_arrive.tv_sec;
					t2.nanoseconds = ts_arrive.tv_nsec;
						subTime(&t2, &t2, &comeInPkt_t1);

						dm9058_ptp_dbg("%s rx SYNC: slave OnRcv.msgSync(%u.%09u T2) unPack(%u.%09u T1) diff to master %10d s %11d ns\r\n",
							PTPD_HEADER_MCU_DESC,
							ts_arrive.tv_sec,
							ts_arrive.tv_nsec,
							comeInPkt_t1.seconds, 
							comeInPkt_t1.nanoseconds,
							t2.seconds, t2.nanoseconds);
//						pack_sync_info(payload, &ts_arrive, &comeInPkt_t1, &t2);
//						publish_sync_info(payload);
					}
					#endif
				break;

				case FOLLOW_UP:
					dm9058_ptp_dbg("in follow_up\r\n"); //handleFollowUp(ptpClock, isFromSelf)
				break;

				case DELAY_REQ:
					/* master action
					 * master rx delayREQ_T4;
					 */
					if (ptpClock.portDS.portState == PTP_MASTER) {
						trapped_delayRESP_T4.tv_sec = ts_arrive.tv_sec;
						trapped_delayRESP_T4.tv_nsec = ts_arrive.tv_nsec;

						/* master's check (peek) (if slave do a favor packet has ts.) */
						TimeInternal master_rcvDlyReq_extra_timeTmp;
						TimeInternal internalTime;
						#if SLAVE_MAKE_DELAY_REQ_FAVOR_TS
						/* master's check (peek) (when slave do a favor make the packet has ts.) */
						#endif
						buffer_ts_time(buffer+14+20+8, &master_rcvDlyReq_extra_timeTmp);
						master_recv_extra_T3.tv_sec = master_rcvDlyReq_extra_timeTmp.seconds;
						master_recv_extra_T3.tv_nsec = master_rcvDlyReq_extra_timeTmp.nanoseconds;

						internalTime.seconds = trapped_delayRESP_T4.tv_sec;
						internalTime.nanoseconds = trapped_delayRESP_T4.tv_nsec;

						subTime(&internalTime, &internalTime, &master_rcvDlyReq_extra_timeTmp);
						dm9058_ptp_dbg("recv DELAY_REQ at tstamp(%u.%09u) diff to slave %u sec %u ns\r\n",
							trapped_delayRESP_T4.tv_sec, trapped_delayRESP_T4.tv_nsec,
							internalTime.seconds, internalTime.nanoseconds);
					}
				break;

				case PDELAY_REQ:
					dm9058_ptp_dbg("in pdelay_req\r\n"); //handlePDelayReq(ptpClock, &time, isFromSelf)
				break;

				case DELAY_RESP:
				if (1) {
					Boolean isFromCurrentParent = FALSE;
					Boolean isCurrentRequest = FALSE;

					isFromCurrentParent = isSamePortIdentity( //DIS
						&ptpClock.parentDS.parentPortIdentity,
						&header.sourcePortIdentity); //&ptpClock.msgTmpHeader.sourcePortIdentity

					isCurrentRequest = isSamePortIdentity( //DIS
						&ptpClock.portDS.portIdentity,
						&ptpClock.msgTmp.resp.requestingPortIdentity);

					isFromCurrentParent = TRUE; //CAST
					isCurrentRequest = TRUE; //CAST
					if (((ptpClock.sentDelayReqSequenceId - 1) == header.sequenceId) //ptpClock.msgTmpHeader.sequenceId
						&& isCurrentRequest && isFromCurrentParent)
					{
						TimeInternal timeTmp;
						TimeInternal resp_receiveTimestamp;
						TimeInternal internalTime;

						buffer_ts_time(buffer+14+20+8, &resp_receiveTimestamp);
						timeTmp.seconds = resp_receiveTimestamp.seconds;
						timeTmp.nanoseconds = resp_receiveTimestamp.nanoseconds;
						internalTime.seconds = delayREQ_T3.tv_sec;
						internalTime.nanoseconds = delayREQ_T3.tv_nsec;
						//abs
						subTime(&timeTmp, &timeTmp, &internalTime);
						//abs(),abs()
						dm9058_ptp_dbg("in DELAY_RESP from-master-pk_ts(%u.%09u) diff to master %u sec %u ns\r\n",
							resp_receiveTimestamp.seconds, resp_receiveTimestamp.nanoseconds,
							timeTmp.seconds, timeTmp.nanoseconds);
					}
				}
				break;

				case PDELAY_RESP:
				dm9058_ptp_dbg("in pdelay_reap\r\n"); //handlePDelayResp(ptpClock, &time, isFromSelf)
				break;

				case PDELAY_RESP_FOLLOW_UP:
				dm9058_ptp_dbg("in pdelay_resp_follow_up\r\n"); //handlePDelayRespFollowUp(ptpClock, isFromSelf)
				break;

				case MANAGEMENT:
				dm9058_ptp_dbg("in management\r\n"); //handleManagement(ptpClock, isFromSelf)
				break;

				case SIGNALING:
				dm9058_ptp_dbg("in signaling\r\n"); //handleSignaling(ptpClock, isFromSelf)
				break;

				default:
				dm9058_ptp_dbg("in Unknow\r\n");
				break;
			}

			do { /* dump */
				char *ab1 = parse_pbuf_ptp_packet1(/*p*/ buffer, 14+20+8);
				//if (!strcmp(ab1, "Announce"))
				//	printkey("//");
				//printkey("<------- DM_ETH_input, UDP: Len %u, parse packet (%s)(%s)\r\n", len, abc, ab1);

				if (!strcmp(ab1, "Delay_Req"))
					dump_data(buffer, len); //dm_eth_input_hexdump(/*p->payload*/ buffer, len); //dm_eth_davicom_hexdump(p->payload, l);
			} while(0);
#endif
	}
}
#endif //EDRIVER_ADDING_PTP && LWIP_PTP
