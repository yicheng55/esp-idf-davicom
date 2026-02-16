/**
 * @file     dm9058.c
 * @brief    DM9058 Ethernet Controller Driver for uip, LWIP TCP/IP Stack
 * @version  V1.6.1
 * @date     2025-02-27
 *
 * @author   Joseph CHANG <joseph_chang@davicom.com.tw>
 * @copyright (c) 2023-2026 Davicom Semiconductor, Inc.
 *
 * @details
 *   This driver implements the hardware abstraction layer for the DM9058
 *   Ethernet controller, providing initialization, transmit, and receive
 *   functionality integrated with the LWIP TCP/IP stack.
 *
 *   Key Features:
 *   - Full integration with uip and LWIP TCP/IP stack
 *   - Support for interrupt and polling modes
 *   - Configurable MAC address handling
 *   - Error detection and recovery mechanisms
 *
 * @note    First verification: AT32F415
 */
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>

#include "dm9058.h"
#include "dm9058_constants.h"

const struct instance_data {
		int tx_checksuming;
			//~ int tx_stack_ip_gen_checksuming;
			//~ int tx_stack_udp_gen_checksuming;
			//~ int tx_stack_tcp_gen_checksuming;
		int rx_checksuming;
			// ! (int) rx_stack_check_checksuming;
        int force_mode_10m;
        int force_flow_control;
        int force_rcr_all;
        int force_stop_if_dm9058_not_found;
} dm9058_def_inf =
/* Instances Configuration */
/*const struct instance_data dm9058_def_inf = */ {
	DEFAULT_TX_CHECKSUM_ON,
    DEFAULT_RX_CHECKSUM_OFF,
    FORCE_MODE_10M_OFF,
    FORCE_FLOW_CONTROL_ON,
    FORCE_RCR_ALL_ON,  // Enable multicast reception for PTP packets
    FORCE_STOP_IF_DM9058_NOT_FOUND_ON,
};
const struct instance_data *dm9058_inf = &dm9058_def_inf;

/* inint internal */
struct innr_data
{
	struct internal_config
	{
		int dev_found; // individual on/off, on if chip_id found
		uint32_t ext_line;
	} dev;
} lab_data = {
	{
		DMS_NETCARD_NOT_YET,
		0xffffffff,
	},
};

struct internal_config *lab_inf = &lab_data.dev;

mac_t final_mac = {0, 0, 0, 0, 0, 0};

char *dm9058_info(void)
{
  return dm_version_info();
}

/**
 * @brief  Core spi and interrupt initialization
 *
 */
int dm9058_conf(void)
{
  HAL_spi_info(0);
  HAL_spi_info(1);

  HAL_boards_initialize_spi();
#if defined(DMPLUG_INT)
  lab_inf->ext_line = HAL_boards_initialize_int();
#endif

  HAL_boards_spi_info();
  hal_active_interrupt_info();
  HAL_spi_speed_info();
  return hal_active_interrupt_mode();
}

/**
 * @brief  Core initialization implementation
 *
 * @param  adr  MAC address to configure
 * @return      Pointer to configured MAC address
 * @note   This function performs complete initialization of the DM9058 including
 *         MAC address configuration and basic setup
 */
const uint8_t *dm9058_init(const uint8_t *adr)
{
  /* verify device */
  HAL_spi_info(2);
  if (!env_chip_id_and_ticks())
  {
    check_force_stop();
    printf("\r\n");
    return NULL;
  }

  return init_setup(adr);
}

/**
 * @brief  Receive packet from DM9058
 *
 * @param  buff  Buffer to store received packet
 * @return       Length of received packet or 0 if no packet
 *
 * @note   This function handles the complete receive process including
 *         error checking and buffer management
 */
uint16_t dm9058_rx(uint8_t *buf)
{
  if (cspi_rx_ready())
    return cspi_rx_read(buf, cspi_rx_head());
  return 0;
}

/**
 * @brief  Transmit packet through DM9058
 *
 * @param  buf   Buffer containing packet to transmit
 * @param  len   Length of packet
 *
 * @note   This function handles the transmit process
 *         Yicheng55 recommand, delay check in advance, if not ready Don't send.
 */
void dm9058_tx(uint8_t *buf, uint16_t len)
{
  len = cspi_tx_packet_len(len);
  if (len) {
	cspi_tx_write(buf, len);
	//cspi_tx_req(buf, TCR_TXREQ);
	tx_compl_timeout(TCR_TXREQ, 500); //while (HAL_read_reg(DM9058_TCR) & TCR_TXREQ) ;
//	printf(" ------------------dm9058_tx(buffer, l, p) TCR_TXREQ(%02x) ----- %02x%02x%02x%02x%02x%02x len %d\r\n",
//			TCR_TXREQ, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], len);
  }
}

/**
 * @brief  Get DM9058 status
 *
 * @param	void
 * @return      16-bit status data
 *
 * @note   This function get 16-bit data
 *         16-bit status data (NCR | NSR << 8)
 *         User can check bit[14] for link status,
 *         bit[14] 1 is link up state
 */
uint16_t dm9058_status(void)
{
  return cspi_get_status();
}

/**
 * @brief  Read comprehensive register information
 *
 * @param  stat  Buffer to store status information (6 bytes)
 * 			stat[0]: NCR
 * 			stat[1]: NSR
 * 			stat[2]: PID.D[15:8]
 * 			stat[3]: PID[7:0]
 * 			stat[4]: BMSR.D[15:8]
 * 			stat[5]: BMSR.D[7:0]
 *
 * @note   Combines chip ID, control status, and PHY status
 */
void dm9058_read_regs_info(uint8_t *stat)
{
  uint16_t cs;
  uint32_t pbm;

  pbm = cspi_phy_read(PHY_STATUS_REG);
  pbm |= cspi_get_chipid() << 16;
  cs = cspi_get_status();

  stat[0] = cs & 0xff;
  stat[1] = (cs >> 8) & 0xff;
  stat[2] = (pbm >> 24) & 0xff;
  stat[3] = (pbm >> 16) & 0xff;
  stat[4] = (pbm >> 8) & 0xff;
  stat[5] = pbm & 0xff;
}

uint16_t dm9058_link_update(void)
{
  uint16_t cs = cspi_get_status() >> 8;
  return (cs & NSR_LINKST) >> 6;
}

/* ver.c */
char *hal_get_dm9058_date(void)
{
	return (char *)dm9058_support_date;
}

char *hal_get_dm9058_release_num(void)
{
	return (char *)dm9058_release_num;
}

int hal_active_interrupt_mode(void)
{
  //dm9058_inf->interrupt_mode;
#if defined(DMPLUG_INT)
  #if defined(INT_CLKOUT)
	return INPUT_MODE_INTERRUPT_CLKOUT;
  #else
	return INPUT_MODE_INTERRUPT;
  #endif
#else
  return INPUT_MODE_POLL;
#endif
}

char *hal_active_interrupt_desc(void)
{
#if defined(DMPLUG_INT)
  #if defined(INT_CLKOUT)
	return "INT CLKOUT";
  #else
	return "INT";
  #endif
#else
  return "POL";
#endif
}

void hal_active_interrupt_info(void)
{
//#if defined(DMPLUG_INT)
//	printf("[CHK] at32f437_spi1 'int' Running...\r\n");
//#else
//	printf("[CHK] at32f437_spi1 'poll' Running...\r\n");
//#endif
	printf("[core] dm9058_constants '%s' Running...\r\n", hal_active_interrupt_desc());
}

//void dm9058_interrupt_set(uint32_t exint_line)
//{
//  if (lab_inf->ext_line == exint_line)
//  {
//    dm9058_interrupt_event = 1;
//  }
//}

/* Interrupt Accessing */
int dm9058_interrupt_get(void)
{
//  if (!hal_active_interrupt_mode())
//    return 1;
#if defined(DMPLUG_INT)
  if (HAL_get_int_event(lab_inf->ext_line)) {
    cspi_disble_irq();
    return 1;
  }
  return 0;
#else
  return 1;
#endif
}
void dm9058_interrupt_reset(void)
{
//  if (hal_active_interrupt_mode())
//  {
//    cspi_enable_irq();
//  }
#if defined(DMPLUG_INT)
  cspi_enable_irq();
#endif
}

//#include "dm9058_JJ222.c"

// ------------------------------------------------------------------------------
// DAVICOM_HalTicks increase by SysTick_Handler or added by dm9058 driver itself.
// Both are OK.
//
// If by SysTick_Handler, davicom_haltick() cyclic called,
// If by driver itself, davicom_haltickcount() indeed keep called until time
//  dm9058_clock_time() get increased!!
// ------------------------------------------------------------------------------
#define DRIVER_INCREASE_MODE_CARDINALITY 10000 // 1000000 NG //100000 ok now
static uint32_t DAVICOM_HalTicks = 0;

uint32_t davicom_haltickcount(void)
{
  static uint32_t hal_tick_count = 0;

  /*[Not only by SysTick counting][if take this, experiment to get 1000000 correction with your system.]*/
  hal_tick_count++;
  if (hal_tick_count >= DRIVER_INCREASE_MODE_CARDINALITY)
  {
    hal_tick_count = 0;
    DAVICOM_HalTicks++;
  }
  return DAVICOM_HalTicks;
}

/**
 * @brief  Get current clock time in milliseconds
 * @retval Current clock time
 */
uint32_t dm9058_clock_time(void)
{
  return davicom_haltickcount();
}

// ---------------
/* Delay Functions
 */
void ctick_delay_us(uint32_t nus)
{
  uint32_t start = dm9058_clock_time();
  while ((dm9058_clock_time() - start) < ((nus + 999) / 1000))
    ;
}

void ctick_delay_ms(uint16_t nms)
{
  uint32_t start = dm9058_clock_time();
  while ((dm9058_clock_time() - start) < nms)
    ;
}

void ncr_compl_timeout(uint32_t timeout_ms)
{
	while (timeout_ms)
	{
		timeout_ms--;
		ctick_delay_ms(1);
		if (!(HAL_read_reg(DM9058_NCR) & DM9058_NCR_RESET))
			return;
		if (!timeout_ms)
			printf("ncr_compl_timeout(uint32_t timeout_ms) -- Check condition!\r\n");
	}
}

void tx_pointer_timeout(uint32_t timeout_us)
{
	HAL_write_reg(0x55, 0x02);
	while (timeout_us)
	{
		timeout_us--;
		ctick_delay_us(1);
		if (!(HAL_read_reg(0x55) & 0x02))
			return;
		if (!timeout_us)
			printf("tx_chk_timeout(uint32_t timeout_us) -- Check condition!\r\n");
	}
}

void tx_compl_timeout(uint8_t tcr_wr, uint32_t timeout_us)
{
	HAL_write_reg(DM9058_TCR, tcr_wr);
	while (timeout_us)
	{
		timeout_us--;
		ctick_delay_us(1);
		if (!(HAL_read_reg(DM9058_TCR) & TCR_TXREQ))
			return;
		if (!timeout_us)
			printf("tx_chk_timeout(uint32_t timeout_us) -- Check condition!\r\n");
	}
}

#define is_multicast_ether_addr(a)               (0x01 & a[0])
int is_zero_ether_addr(const uint8_t *addr)
{
  return (*(const uint16_t *)(addr + 0) |
          *(const uint16_t *)(addr + 2) |
          *(const uint16_t *)(addr + 4)) == 0;
}

int validate_macaddr(const uint8_t *adr)
{
  return (!is_multicast_ether_addr(adr) && !is_zero_ether_addr(adr));
}

uint8_t *identified_eth_mac(void) // dr_mget_final_mac()
{
  return final_mac;
}
uint8_t *identify_eth_mac(const mac_t param) // dr_mset_final_mac()
{
  if (param)
    memcpy(final_mac, param, MAC_ADDR_LENGTH);
  else
  {
    final_mac[0] = 0x00;
    final_mac[1] = 0x60;
    final_mac[2] = 0x6e;
    final_mac[3] = (uint8_t)rand() + 0; /*need real random*/
    final_mac[4] = (uint8_t)rand() + 1; /*need real random*/
    final_mac[5] = (uint8_t)rand() + 2; /*need real random*/
  }
  return final_mac;
}

void cspi_write_regs(uint8_t reg, const uint8_t *buf, uint16_t len)
{
  uint16_t i;
  for (i = 0; i < len; i++, reg++)
  {
    HAL_write_reg(reg, buf[i]);
  }
}

void cspi_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
  uint16_t i;
  for (i = 0; i < len; i++, reg++)
  {
    buf[i] = HAL_read_reg(reg);
  }
}

/*******************************************************************************
 * PHY Access Functions
 ******************************************************************************/
uint16_t cspi_phy_read(uint16_t uReg)
{
  int w = 0;
  uint16_t uData;

  HAL_write_reg(DM9058_EPAR, DM9058_PHY | uReg);
  HAL_write_reg(DM9058_EPCR, 0xc);
  ctick_delay_us(1);

  while (HAL_read_reg(DM9058_EPCR) & 0x1)
  {
    ctick_delay_us(1);
    if (++w >= 500)
      break; // Timeout
  }

  HAL_write_reg(DM9058_EPCR, 0x0);
  uData = (HAL_read_reg(DM9058_EPDRH) << 8) | HAL_read_reg(DM9058_EPDRL);
  return uData;
}

void cspi_phy_write(uint16_t reg, uint16_t value)
{
  int w = 0;

  HAL_write_reg(DM9058_EPAR, DM9058_PHY | reg);
  HAL_write_reg(DM9058_EPDRL, (value & 0xff));
  HAL_write_reg(DM9058_EPDRH, ((value >> 8) & 0xff));
  HAL_write_reg(DM9058_EPCR, 0xa);
  ctick_delay_us(1);

  while (HAL_read_reg(DM9058_EPCR) & 0x1)
  {
    ctick_delay_us(1);
    if (++w >= 500)
      break; // Timeout
  }

  HAL_write_reg(DM9058_EPCR, 0x0);
}

/**
 * @brief  Power on PHY and wait for stabilization
 *
 * @param  nms  Delay time in milliseconds
 */
void cspi_phycore_on(uint16_t nms)
{
  // HAL_write_reg(DM9058_GPCR, 0x01)
  HAL_write_reg(DM9058_GPR, 0x00); /* Power on PHY */
  ctick_delay_ms(nms);
}

/**
 * @brief  Perform NCR reset and wait for completion
 *
 * @param  nms  Delay time in milliseconds
 */
void cspi_ncr_reset(uint16_t nms)
{
  HAL_write_reg(DM9058_NCR, DM9058_NCR_RESET);
  ncr_compl_timeout(nms);
 //ctick_delay_ms(nms);
}

/**
 * @brief  Configure default software settings
 */
void cspi_soft_default(void)
{
//  BUS_SETUP1(cspi_setup_buswork); //cspi_setup_buswork();
  HAL_write_reg(DM9058_MBNDRY, BOUND_CONF_BIT);
  HAL_write_reg(DM9058_PPCR, PPCR_PAUSE_COUNT);
  HAL_write_reg(DM9058_LMCR, LMCR_MODE1);
  HAL_write_reg(DM9058_INTR, INTR_ACTIVE_LOW);
  if (dm9058_inf->tx_checksuming)
	HAL_write_reg(DM9058_CSCR, TCSCR_IPCS_ENABLE | TCSCR_UDPCS_ENABLE | TCSCR_TCPCS_ENABLE);
  if (dm9058_inf->rx_checksuming)
	HAL_write_reg(DM9058_RCSSR, RCSSR_RCSEN | RCSSR_DCSE);
  #if defined(INT_CLKOUT)
    HAL_write_reg(DM9058_IPCOCR, IPCOCR_CLKOUT | IPCOCR_DUTY_LEN);
  #endif
}

/**
 * @brief  Core reset implementation
 */
void cspi_core_reset(void)
{
  cspi_phycore_on(25);
  cspi_ncr_reset(5);

  /* Jabber function disabled refer to bench test
   * meeting advice 20250226
   */
  //cspi_phy_write(18, 0x7000);

  cspi_soft_default();
}

#define trace_print_eth_mac(s, adr)                          \
  printf("%s mac address %02x:%02x:%02x:%02x:%02x:%02x\r\n", \
         s, adr[0], adr[1], adr[2], adr[3], adr[4], adr[5])
#define trace_array_eth_mac(head)                                                 \
  do                                                                              \
  {                                                                               \
    uint8_t addr[6];                                                              \
    memcpy(addr, identified_eth_mac(), MAC_ADDR_LENGTH); /*only for a trace log*/ \
    trace_print_eth_mac(head, addr);                                              \
  } while (0)

void cspi_macaddr_process(const uint8_t *param_adr)
{
  uint8_t c_adr[6];

  if (param_adr && validate_macaddr(param_adr))
  {
    identify_eth_mac(param_adr); // save into final_mac, accept param_adr MAC addr
    cspi_set_par(identified_eth_mac());
    return;
  }

  cspi_get_par(c_adr);

  if (validate_macaddr(c_adr))
  {
    identify_eth_mac(c_adr);                 // save into final_mac, use chip load MAC address
    //trace_array_eth_mac("macaddr_process:"); //"chip.mac.e", trace a mac addr from chip, only for a trace log
  }
  else
  {
    /* while invalid, do configure MAC address */
    trace_print_eth_mac("invalid.mac.s", c_adr);
    identify_eth_mac(NULL);              // save into final_mac, use random MAC address
    trace_array_eth_mac("random.mac.e"); // a mac addr from param_adr or candidated random, only for a trace log
    cspi_set_par(identified_eth_mac());
  }
}

/**
 * @brief  Start DM9058 with MAC configuration
 *
 * @param  adr  MAC address to configure
 * @return      Pointer to configured MAC address
 */
void cspi_core_start1(void)
{
  /* Initialize receive settings */
#if defined(DMPLUG_INT)
  HAL_clr_int_event(lab_inf->ext_line); //dm9058_interrupt_event = 0;
  HAL_enable_mcu_irq();
#endif
  cspi_set_recv();
}

/* Device found, in case for lwip xnetifs' multi-cards checking
 *  for if not all multi-cards is dm9058 chip_id found.
 * //const uint_8 *dm9058_all_start(const uint8_t *adr)
 */
const uint8_t *init_setup(const uint8_t *adr)
{
  lab_inf->dev_found = DMS_NETCARD_FOUND;
  //cspi_setup_buswork();
  cspi_core_reset();         // env_reset_process();
  cspi_macaddr_process(adr); //.init
  cspi_core_start1();
  return identified_eth_mac();
}

/**
 * @brief  Read DM9058 chip identification
 * @return 16-bit chip ID (PIDL | PIDH << 8)
 */
uint16_t cspi_get_chipid(void)
{
  uint8_t buff[2];

  cspi_read_regs(DM9058_PIDL, buff, 2);
  return buff[0] | buff[1] << 8;
}

/**
 * @brief  Read control and status registers
 * @return 16-bit status (NCR | NSR << 8)
 */
uint16_t cspi_get_status(void)
{
  uint8_t buff[2];

  cspi_read_regs(DM9058_NCR, buff, 2);
  return buff[0] | buff[1] << 8;
}

/**
 * @brief  Initialize and verify DM9058 device
 *
 * @param  id  Pointer to store chip ID
 * @return     1 if device found, 0 if not found
 */
int env_chip_id_and_ticks(void)
{
  uint16_t id;
  int ps = 3; /* Times to check */
  uint32_t n = 0, heartbeat_stamp = dm9058_clock_time();
  uint8_t rev = HAL_read_reg(0x5c);

  /* Read and verify chip ID */
  id = cspi_get_chipid();
  if (id == 0x9000)
  {
    id = 0x9058;
  }

  printf("[DRIVER %s mode] %s: %04x\r\n", hal_active_interrupt_desc(),
         (id == 0x9051) ? setup_found_str : setup_not_found_str, id);

  /* Read and verify heartbeat */
  while (!n && --ps >= 0)
  {
    n = 1000000; // 1M times
    while (n--)
      dm9058_clock_time();

    n = dm9058_clock_time() - heartbeat_stamp;
    if (n <= 1)
      printf("%s: %u%s, increase cardinality %lu (pls engineer tune this to smaller value!! such as %lu)\r\n",
             n ? "[HEARTBEAT WARN]" : "[HEARTBEAT ERROR]",
             n,
             n == 1 ? ", increase type" : " !",
             DRIVER_INCREASE_MODE_CARDINALITY,
             DRIVER_INCREASE_MODE_CARDINALITY / 10);
  }

  return (id == 0x9051) && n ? 1 : 0;
}

// -----------------------------------------------------------------------
void check_force_stop(void)
{
  if (dm9058_inf->force_stop_if_dm9058_not_found)
    while (1)
      ; //"system stop"
}

/**
 * @brief  Configure physical address registers
 *
 * @param  adr  MAC address array
 */
void cspi_get_par(uint8_t *adr)
{
  cspi_read_regs(DM9058_PAR, adr, 6);
}

void cspi_set_par(const uint8_t *adr)
{
  cspi_write_regs(DM9058_PAR, adr, 6);
}

/**
 * @brief  Enable interrupts and read/write ISR register
 * @return Combined ISR status (upper 8 bits always 0xFF)
 */
uint16_t dm9058_isr_enab(void)
{
  uint16_t isrs = HAL_read_reg(DM9058_ISR);
  HAL_write_reg(DM9058_ISR, (uint8_t)isrs);
  return isrs | (0xff << 8);
}

void dm9058_imr_disab(void)
{
  HAL_write_reg(DM9058_IMR, IMR_PAR);
}

void dm9058_imr_enab(void)
{
  HAL_write_reg(DM9058_IMR, hal_active_interrupt_mode() == INPUT_MODE_POLL ? IMR_POL_DEFAULT : IMR_INT_DEFAULT);
}

void cspi_set_imr(void)
{
  dm9058_imr_enab(); // HAL_write_reg(_DM9058_IMR, _IMR_DEFAULT);
}

void cspi_disble_irq(void)
{
  dm9058_imr_disab();
}

void cspi_enable_irq(void)
{
  dm9058_isr_enab();
  dm9058_imr_enab(); // cspi_set_imr()
}

/**
 * @brief  Configure multicast address registers
 */
void cspi_set_mar(void)
{
  int i;

  for (i = 0; i < 8; i++)
  {
    HAL_write_reg(DM9058_MAR + i, (i == 7) ? 0x80 : 0x00);
  }
}

//static
void cspi_set_fcr(void)
{
  if (dm9058_inf->force_flow_control)
  {
    HAL_write_reg(DM9058_FCR, FCR_DEFAULT);       //_FCR_DEFAULT1
    cspi_phy_write(PHY_ADV_REG, 0x0400 | 0x01e1); // phy_write(04, flow)
  }
}

//static
void cspi_set_rcr(void)
{
  uint8_t val = RCR_DEFAULT | RCR_RXEN;
  HAL_write_reg(DM9058_RCR, (dm9058_inf->force_rcr_all) ? val | RCR_ALL | RCR_PRMSC
														: val);
}

/**
 * @brief  Configure receive settings and interrupts
 */
void cspi_set_recv(void)
{
  //.cspi_set_par(adr); NOT here!
  /* Configure multicast addresses */
  cspi_set_mar(); //= "dm9058_multicast_init(void)"

  cspi_set_fcr();

  /* Configure interrupt settings */
  cspi_set_imr();

  /* Configure receive settings */
  SET_RCR(); //cspi_set_rcr();
}

// RX Byte Read Function
uint8_t cspi_read_rxb(void)
{
  HAL_read_reg(DM9058_MRCMDX);
  return HAL_read_reg(DM9058_MRCMDX);
}

uint16_t cspi_get_rwpa(void) // wt
{
  return (uint16_t)HAL_read_reg(DM9058_RWPAL) |     /* 0x24 */
         (uint16_t)HAL_read_reg(DM9058_RWPAH) << 8; /* 0x25 */
}

uint16_t cspi_get_mrrl(void) // rd
{
  return (uint16_t)HAL_read_reg(DM9058_MRRL) |     /* 0x74 */
         (uint16_t)HAL_read_reg(DM9058_MRRH) << 8; /* 0x75 */
}

/**
 * @brief  Read RX write pointer and memory data read address
 *
 * @param  rwpa_wt  Pointer to store write pointer
 * @param  mdra_rd  Pointer to store read address
 */
void dm9058_read_rx_pointers(uint16_t *rwpa_wt, uint16_t *mdra_rd)
{
  *rwpa_wt = cspi_get_rwpa();
  *mdra_rd = cspi_get_mrrl();
}

int rx_pointers_equ(uint16_t dummy_rwpa, uint16_t rd_now)
{
	return dummy_rwpa == rd_now;
}

/**
 * @brief  Display RX buffer statistics
 *
 * @param  htc  History counter array
 * @param  n    Size of array
 */
static void dm9058_show_rxbstatistic(uint8_t *htc, int n)
{
  int i;

  printf("SHW rxbStatistic, 254 wrngs\r\n");

  for (i = 0; i < (n + 2); i++)
  {
    if (!(i % 32) && i)
      printf("\r\n");
    if (!(i % 32) || !(i % 16))
      printf("%02x:", i);
    if (!(i % 8))
      printf(" ");
    if (i == 0 || i == 1)
    {
      printf("  ");
      continue;
    }

    /* (drv_print && PRINT_SEMA == SEMA_ON)? */
    printf("%d ", htc[i - 2]);
  }
  printf("\r\n");
}

/**
 * @brief  Process RX buffer fire time
 *
 * @param  histc   History counter array
 * @param  csize   Size of array
 * @param  i       Current index
 * @param  rxb     RX buffer value
 * @return         TIMES_TO_RST if reset needed, 0 otherwise
 */
static uint8_t ret_fire_time(uint8_t *histc, int csize, int i, uint8_t rxb)
{
  printf(" _dm9058f rxb %02x (times %2d)%c\r\n",
         rxb, histc[i],
         (histc[i] == 2) ? '*' : ' ');

  if (histc[i] >= TIMES_TO_RST)
  {
    dm9058_show_rxbstatistic(histc, csize);
    histc[i] = 1;
    return TIMES_TO_RST;
  }

  return 0;
}

/**
 * @brief  Evaluate RX buffer status and handle errors
 *
 * @param  rxb  RX buffer value to evaluate
 * @return      0 if successful, error code otherwise
 */
//static
uint16_t env_evaluate_rxb(uint8_t rxb)
{
  int i;
  static uint8_t histc[254] = {0};
  uint8_t times = 1;

  for (i = 0; i < sizeof(histc); i++)
  {
    if (rxb == (i + 2))
    {
      histc[i]++;
      times = ret_fire_time(histc, sizeof(histc), i, rxb);

      if (times == 0)
        return 0;

      return env_err_rsthdlr3("_dm9058f rxb error accumunation times : %u\r\n", times);
    }
  }

  return env_err_rsthdlr3("dm9 impossible path error times : %u\r\n", times);
}

//static
uint16_t env_evaluate_rxb_zero(void)
{
  uint16_t dummy_rwpa, rd_now;

  dm9058_read_rx_pointers(&dummy_rwpa, &rd_now);
  if (rx_pointers_equ(dummy_rwpa, rd_now))
	  return 0;
#if 0
  printf("rxb zero && wrong rx pointers, rwpa %04x mrr %04x\r\n",
		dummy_rwpa, rd_now);
#endif
  return 0; //or return env_err_rsthdlr3(..
}

uint16_t rx_head_takelen(uint8_t *receivedata)
{
  uint16_t rx_len;
  uint8_t rx_status;

  /* Read packet header */
  rx_status = receivedata[1];
  rx_len = receivedata[2] + (receivedata[3] << 8);

  /* Validate packet status and length */
  DM9058_RX_BREAK((rx_status & RSR_ERR_BITS),
                  return env_err_rsthdlr3("_dm9058f rx_status error : 0x%02x\r\n",
                                          rx_status));
  DM9058_RX_BREAK((rx_len > DRVBUF_POOL_BUFSIZE),
                  return env_err_rsthdlr3("_dm9058f rx_len error : %u\r\n",
                                          rx_len));
  return rx_len;
}

/**
 * @brief  Read receive packet header
 *
 * @param  receivedata  Buffer to store header data (4 bytes)
 */
uint16_t cspi_rx_head(void) //(uint8_t *receivedata)
{
  uint8_t receivedata[4];
  HAL_read_mem(receivedata, 4);
  HAL_write_reg(DM9058_ISR, 0x80);

  return rx_head_takelen(receivedata);
}

/**
 * @brief  Read receive packet data
 *
 * @param  buff    Buffer to store packet data
 * @param  rx_len  Length of data to read
 */
uint16_t cspi_rx_read(uint8_t *buff, uint16_t rx_len)
{
  if (rx_len)
  {
    /* Read packet data */
    HAL_read_mem(buff, PAD_LEN(rx_len));
    HAL_write_reg(DM9058_ISR, 0x80);
	BUS_OPS1(bus_work, buff, PAD_LEN(rx_len));
    //DM9058_BUS_WORK(RSV_MODE && rsv_word, bus_work(buff, PAD_LEN(rx_len)));
    return rx_len;
  }
  return 0;
}

int cspi_rx_ready(void)
{
  uint8_t rxbyte; //= 0x25;

  /* Read and validate RX byte */
  rxbyte = cspi_read_rxb();

  //if (dm9058_inf->mode.checksuming == DEFAULT_CHECKSUM_ON)
  if (dm9058_inf->rx_checksuming) //if (!dm9058_inf->rx_stack_check_checksuming)
  {
    DM9058_RX_BREAK(((rxbyte & 0x03) != DM9058_PKT_RDY && rxbyte != 0), // Need a suitable value instead.
                    return env_evaluate_rxb(rxbyte));
  }
  else
    DM9058_RX_BREAK((rxbyte != DM9058_PKT_RDY && rxbyte != 0),
                    return env_evaluate_rxb(rxbyte));

  DM9058_RX_BREAK((rxbyte == 0),
                  return env_evaluate_rxb_zero());
  return 1;
}

/**
 * @brief  Write transmit packet data
 *
 * @param  buf  Buffer containing packet data
 * @param  len  Length of data to write
 */
uint16_t cspi_tx_len(uint16_t len)
{
  /* Set packet length */
  HAL_write_reg(DM9058_TXPLL, len & 0xff);
  HAL_write_reg(DM9058_TXPLH, (len >> 8) & 0xff);
  return len;
}

uint16_t cspi_tx_packet_len(uint16_t len)
{
  return SET_TXPL(len);
}

void cspi_tx_write(uint8_t *buf, uint16_t len)
{
  /* Padding if needed */
  HAL_write_mem(buf, PAD_LEN(len)); /* 8/16-bit */
}

//void cspi_tx_req(uint8_t *buf, uint8_t tcr_wrt)
//{
//  HAL_write_reg(DM9058_TCR, tcr_wrt);
//}

/**
 * @brief  Variable argument error handler with reset
 *
 * @param  format   Error format string
 * @param  ...      Variable arguments
 * @return          0 after reset completion
 */
uint16_t env_err_rsthdlr3(const char *format, ...)
{
  char bff[180];
  int val;
  va_list args;

  va_start(args, format);
  val = va_arg(args, int);
  sprintf(bff, format, val);
  printf("%s", bff);
  va_end(args);

//  cspi_setup_buswork();
  cspi_core_reset();
  cspi_core_start1();
  return 0;
}

/* Multicast table for DM9058 */
#if 0
//#define HASH_BITS 6  // For 64 buckets (2^6 = 64)

//uint32_t calculate_crc32(const uint8_t *data, size_t length) {
//    uint32_t crc = 0xFFFFFFFF;
//    uint32_t polynomial = 0x04C11DB7;

//    for (size_t i = 0; i < length; i++) {
//        crc ^= (data[i] << 24);
//        for (int j = 0; j < 8; j++) {
//            if (crc & 0x80000000)
//                crc = (crc << 1) ^ polynomial;
//            else
//                crc <<= 1;
//        }
//    }
//    return crc;
//}

//static uint8_t calculate_hash(const uint8_t *mac_addr) {
//    // Calculate CRC32 of the MAC address (6 bytes)
//    uint32_t crc = calculate_crc32(mac_addr, 6);
//    
//    // Extract the least significant HASH_BITS
//    return (crc & ((1 << HASH_BITS) - 1));
//}
#endif
#if 0
//#define HASH_BITS 6  // For 64 buckets (2^6 = 64)

//uint32_t calculate_crc32(const uint8_t *data, size_t length) {
//    uint32_t crc = 0xFFFFFFFF;
//    uint32_t polynomial = 0x04C11DB7;

//    for (size_t i = 0; i < length; i++) {
//        crc ^= (data[i] << 24);
//        for (int j = 0; j < 8; j++) {
//            if (crc & 0x80000000)
//                crc = (crc << 1) ^ polynomial;
//            else
//                crc <<= 1;
//        }
//    }
//    // Final XOR for standard CRC32 post-processing
//    return crc ^ 0xFFFFFFFF;
//}

//static uint8_t calculate_hash(const uint8_t *mac_addr) {
//    // Calculate CRC32 of the MAC address (6 bytes)
//    uint32_t crc = calculate_crc32(mac_addr, 6);
//    
//    // Extract the least significant HASH_BITS
//    return (crc & ((1 << HASH_BITS) - 1));
//}
#endif

/* Convert IP multicast address to MAC multicast address */
static void ip_to_mac_multicast(uint32_t ip_addr, uint8_t *mac_addr)
{
    mac_addr[0] = 0x01;
    mac_addr[1] = 0x00;
    mac_addr[2] = 0x5E;
    mac_addr[3] = (uint8_t)((ip_addr >> 8) & 0x7F); //((ip_addr >> 16) & 0x7F);
    mac_addr[4] = (uint8_t)((ip_addr >> 16) & 0xFF); //((ip_addr >> 8) & 0xFF);
    mac_addr[5] = (uint8_t)((ip_addr >> 24) & 0xFF); //(ip_addr & 0xFF);
}

//static int ether_crc_fail001(int length, const unsigned char *data)
//{
//  int crc = -1;
//  static unsigned const ethernet_polynomial = 0x04c11db7U;
//  while (--length >= 0)
//  {
//    unsigned char current_octet = *data++;
//    int bit;
//    for (bit = 0; bit < 8; bit++, current_octet >>= 1)
//    {
//      crc = (crc << 1) ^
//            ((crc < 0) ^ (current_octet & 1) ? ethernet_polynomial : 0);
//    }
//  }
//  return crc;
//}

static uint32_t ether_crc(int length, const unsigned char *data)
{
//  int crc = -1;
//  static unsigned const ethernet_polynomial = 0x04c11db7U;
//  while (--length >= 0)
//  {
//    unsigned char current_octet = *data++;
//    int bit;
//    for (bit = 0; bit < 8; bit++, current_octet >>= 1)
//    {
//      crc = (crc << 1) ^
//            ((crc < 0) ^ (current_octet & 1) ? ethernet_polynomial : 0);
//    }
//  }
//  return crc;
    int i, j;
    uint32_t crc = 0xFFFFFFFF;

    for (j = 0; j < length; j++) {
        crc ^= data[j];
        for (i = 0; i < 8; i++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320; // reversed polynomial
            else
                crc = crc >> 1;
        }
    }
	return crc;
}

static uint8_t MacHash(const uint8_t *Mac) {
	return ether_crc(6, Mac) & 0x3F;
//	return 31; //test
#if 0
//    int i, j;
//    uint32_t Crc = 0xFFFFFFFF;

//    for (j = 0; j < 6; j++) {
//        Crc ^= Mac[j];
//        for (i = 0; i < 8; i++) {
//            if (Crc & 1)
//                Crc = (Crc >> 1) ^ 0xEDB88320; // reversed polynomial
//            else
//                Crc = Crc >> 1;
//        }
//    }
//	return Crc & 0x3F;
#endif
#if 0
//  uint32_t crc32 = ether_crc_fail001(6, Mac);
//  uint8_t crc6 = (crc32 >> 26) & 0x3f;
//  return crc6;
#endif
}

static void Dm9058_Multicast_Modify_add(const unsigned char hash_value)
{
	/* calculate crc32 value of mac address */
	unsigned char hash_group = hash_value / 8;
	unsigned char hash_bit = hash_value % 8;
	//printf("      MAC HASH: hash_val %u (byte_offset %u bit_offset %u)\r\n",
		//hash_value, hash_group, hash_bit);

	uint8_t mcast_hash[8];
#if 1
	int i;
	for (i = 0; i < 8; i++)
	{
		mcast_hash[i] = HAL_read_reg(DM9058_MAR + i);
	}
//	printf("      IN HASH: %02x %02x %02x %02x  %02x %02x %02x %02x\r\n",
//		mcast_hash[0], mcast_hash[1], mcast_hash[2], mcast_hash[3],
//		mcast_hash[4], mcast_hash[5], mcast_hash[6], mcast_hash[7]);
#else
	mcast_hash[hash_group] = HAL_read_reg(DM9058_MAR + hash_group);
	printf("      IN HASH: byte_offset %u, read as %02x\r\n",
		hash_group, mcast_hash[hash_group]);
#endif

	//if (action)
	mcast_hash[hash_group] |= (1 << hash_bit);

	HAL_write_reg(DM9058_MAR + hash_group, mcast_hash[hash_group]);

#if 0
	mcast_hash[hash_group] = HAL_read_reg(DM9058_MAR + hash_group);
	printf("      OUT HASH: byte_offset %u, read as %02x\r\n",
		hash_group, mcast_hash[hash_group]);
#else
	for (i = 0; i < 8; i++)
	{
		mcast_hash[i] = HAL_read_reg(DM9058_MAR + i);
	}
	printf("      OUT HASH: %02x %02x %02x %02x  %02x %02x %02x %02x\r\n",
		mcast_hash[0], mcast_hash[1], mcast_hash[2], mcast_hash[3],
		mcast_hash[4], mcast_hash[5], mcast_hash[6], mcast_hash[7]);
#endif
}

static void Dm9058_Multicast_Modify_del(const unsigned char hash_value)
{
	/* calculate crc32 value of mac address */
	unsigned char hash_group = hash_value / 8;
	unsigned char hash_bit = hash_value % 8;
	//printf("      MAC HASH: hash_val %u (byte_offset %u bit_offset %u)\r\n",
		//hash_value, hash_group, hash_bit);

	uint8_t mcast_hash[8];
#if 1
	int i;
	for (i = 0; i < 8; i++)
	{
		mcast_hash[i] = HAL_read_reg(DM9058_MAR + i);
	}
//	printf("      IN HASH: %02x %02x %02x %02x  %02x %02x %02x %02x\r\n",
//		mcast_hash[0], mcast_hash[1], mcast_hash[2], mcast_hash[3],
//		mcast_hash[4], mcast_hash[5], mcast_hash[6], mcast_hash[7]);
#else
	mcast_hash[hash_group] = HAL_read_reg(DM9058_MAR + hash_group);
	printf("      IN HASH: byte_offset %u, read as %02x\r\n",
		hash_group, mcast_hash[hash_group]);
#endif

	//if (action)
	//else
	mcast_hash[hash_group] &= ~(1 << hash_bit);

	HAL_write_reg(DM9058_MAR + hash_group, mcast_hash[hash_group]);

#if 0
	mcast_hash[hash_group] = HAL_read_reg(DM9058_MAR + hash_group);
	printf("      OUT HASH: byte_offset %u, read as %02x\r\n",
		hash_group, mcast_hash[hash_group]);
#else
	for (i = 0; i < 8; i++)
	{
		mcast_hash[i] = HAL_read_reg(DM9058_MAR + i);
	}
	printf("      OUT HASH: %02x %02x %02x %02x  %02x %02x %02x %02x\r\n",
		mcast_hash[0], mcast_hash[1], mcast_hash[2], mcast_hash[3],
		mcast_hash[4], mcast_hash[5], mcast_hash[6], mcast_hash[7]);
#endif
}

void dm9058_rx_mode_add_hash(const uint32_t ip_addr)
{
    uint8_t mac[6];


    ip_to_mac_multicast(ip_addr, mac);
	printf("[PTPD] ip_to_mac_multicast: ip %d.%d.%d.%d to mac %02x:%02x:%02x:%02x:%02x:%02x\r\n",
		(ip_addr >> 0) & 0xff, (ip_addr >> 8) & 0xff, (ip_addr >> 16) & 0xff, ip_addr >> 24,
		mac[0], mac[1],
		mac[2], mac[3],
		mac[4], mac[5]);
    uint8_t hash_val = MacHash(mac);
	printf("     .MAC HASH: %02x:%02x:%02x:%02x:%02x:%02x= hash_val %u\r\n",
		mac[0], mac[1],
		mac[2], mac[3],
		mac[4], mac[5],
		hash_val);
	Dm9058_Multicast_Modify_add(hash_val);
}

void dm9058_rx_mode_del_hash(const uint32_t ip_addr)
{
    uint8_t mac[6];

    ip_to_mac_multicast(ip_addr, mac);
    uint8_t hash_val = MacHash(mac);
	printf("     .MAC HASH: %02x:%02x:%02x:%02x:%02x:%02x= hash_val %u\r\n",
		mac[0], mac[1],
		mac[2], mac[3],
		mac[4], mac[5],
		hash_val);
	Dm9058_Multicast_Modify_del(hash_val);
}

//void Dm9058_Multicast_Modify(const unsigned char hash_value, unsigned char action)
//{
//	/* calculate crc32 value of mac address */
//	unsigned char hash_group = hash_value / 8;
//	unsigned char hash_bit = hash_value % 8;
//	printf("      MAC HASH: hash_val %u (byte_offset %u bit_offset %u)\r\n",
//		hash_value, hash_group, hash_bit);

//	uint8_t mcast_hash[8];
//#if 1
//	int i;
//	for (i = 0; i < 8; i++)
//	{
//		mcast_hash[i] = HAL_read_reg(DM9058_MAR + i);
//	}
//	printf("      IN HASH: %02x %02x %02x %02x  %02x %02x %02x %02x\r\n",
//		mcast_hash[0], mcast_hash[1], mcast_hash[2], mcast_hash[3],
//		mcast_hash[4], mcast_hash[5], mcast_hash[6], mcast_hash[7]);
//#else
//	mcast_hash[hash_group] = HAL_read_reg(DM9058_MAR + hash_group);
//	printf("      IN HASH: byte_offset %u, read as %02x\r\n",
//		hash_group, mcast_hash[hash_group]);
//#endif

//	if (action)
//		mcast_hash[hash_group] |= (1 << hash_bit);
//	else
//		mcast_hash[hash_group] &= ~(1 << hash_bit);
//	printf("      ON HASH: byte_offset %u, write to %02x\r\n",
//		hash_group, mcast_hash[hash_group]);

//	HAL_write_reg(DM9058_MAR + hash_group, mcast_hash[hash_group]);

//#if 0
//	mcast_hash[hash_group] = HAL_read_reg(DM9058_MAR + hash_group);
//	printf("      OUT HASH: byte_offset %u, read as %02x\r\n",
//		hash_group, mcast_hash[hash_group]);
//#else
//	for (i = 0; i < 8; i++)
//	{
//		mcast_hash[i] = HAL_read_reg(DM9058_MAR + i);
//	}
//	printf("      OUT HASH: %02x %02x %02x %02x  %02x %02x %02x %02x\r\n",
//		mcast_hash[0], mcast_hash[1], mcast_hash[2], mcast_hash[3],
//		mcast_hash[4], mcast_hash[5], mcast_hash[6], mcast_hash[7]);
//#endif
//}
