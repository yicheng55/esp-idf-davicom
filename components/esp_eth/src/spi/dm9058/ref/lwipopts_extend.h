//
// Development options, could be also by included by LWIP project's "lwiiopts.h"
// 20251027

/* Enable DHCP to test it, disable UDP checksum to easier inject packets */
#define LWIP_DHCP                       1 //1 //1 //0 //1 //0
#define EDRIVER_ADDING_PTP				0	//1
#define LWIP_PTP                        0 // 1 //1
#define IP_REASSEMBLY                   0 //[jj]
#define LWIP_NO_CTYPE_H					1 //[jj.20250729]
#define OPTIONAL_SYSTICK_CORE_CM4		FALSE //[JJ.20250820]
#define LWIP_PROVIDE_ERRNO				  //[20250826]
#define LWIPERF_APP						1
#define LWIP_MQTT						0 //1 //1 //[20250828]
//#define LWIP_PHY_DOWN_MEASUREMENT		1 //[20250905]
//#define LWIP_PHY_DOWN_SECONDS			50 //[20250905]
#if 1
//[Joseph] use debug config
//#define MQTT_DEBUG              		LWIP_DBG_ON //[20250911] //| LWIP_DBG_HALT //LWIP_DBG_OFF
//#define IP_DEBUG                      LWIP_DBG_ON //[20250911]
//#define TCP_INPUT_DEBUG                 LWIP_DBG_ON //[20250911]
//#define DHCP_DEBUG                      LWIP_DBG_ON //[20250919]
#endif
#define SLAVE_MAKE_DELAY_REQ_FAVOR_TS	1 //[prepare 250916]
#if LWIPERF_APP
#define TCP_TMR_INTERVAL       			10 //[20251003] /* 250, The TCP timer interval in milliseconds. */
#endif