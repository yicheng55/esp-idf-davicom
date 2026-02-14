/**
 *******************************************************************************
 * @file    dm9058_ptp.c
 * @brief   DM9058 PTP (Precision Time Protocol) Implementation for AT32F403A
 *
 * @details This file implements PTP timestamp functions for the DM9058
 *          Ethernet controller with IEEE 1588-2008 support.
 *
 *          Features:
 *          - Hardware timestamp capture for TX/RX packets
 *          - PTP clock rate adjustment and frequency correction
 *          - Time synchronization with nanosecond precision
 *          - PTP packet timestamping support
 *
 * @version 1.6.1
 * @author  Joseph CHANG / yicheng55
 * @copyright (c) 2023-2026 Davicom Semiconductor, Inc.
 * @date    2025-06-29
 *******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "lwip/opt.h"
#include "lwip/prot/ethernet.h"
#include "ethernetif.h"
#include "core/dm9058_constants.h"
#include "dm9058_ptp.h"
#include "../../dm9058_u2510_if/platform_info.h"
#define DM_DEBUG_TYPE 0
#include "../../dm9058_u2510_if/nosys/dbg_ops.h"
#define delay_us ctick_delay_us
#define delay_ms ctick_delay_ms

#include "ptpd.h"

/* Network protocol constants */
#define ETH_HLEN              14
#define ETH_P_IP              0x0800
#define ETH_P_IPV6            0x86DD
#define ETH_P_LLDP            0x88CC
#define IPPROTO_UDP           17
#define IPPROTO_IGMP          2
#define PTP_ETHERTYPE         ((uint16_t)ETHTYPE_PTP)  /* 0x88F7 */

#if LWIP_PTP
int32_t v51_getPtpClockRate(int64_t *ptrRateValue);

  /* DM9058 PTP Constants */
  #define V51_ADJ_FREQ_BASE_ADDEND     171.7987 /* Base addend for frequency adjustment */
  #define V51_ADJ_FREQ_BASE_ADDEND_FT  171.7987
  #define V51_ADJ_FREQ_BASE_ADDEND_Q16 11259106 /* Q16 format: 171.7987 * 2^16 */
  // #define ADJ_FREQ_MAX                 5120000  /* Maximum adjustment value */
  #define MAX_RETRY_COUNT              3 /* Maximum retry for register read */

  /* PTP Register Control Values */
  #define ADJUST_SLOWER_CTRL           0x60       /* Control value for slower adjustment */
  #define ADJUST_FASTER_CTRL           0x20       /* Control value for faster adjustment */
  #define MAX_ADJUSTMENT               0xEFFFFFFF /* Maximum adjustment value */

/* Global Variables */
uint8_t g_time_buf[8]; /* Time buffer for PTP operations */


/**
 * @brief  Initialize DM9058 PTP functionality
 * @param  sel: PTP mode selection based on PTP_NETWORK_TRANSPORT
 *              - TRANSPORT_UDP_IPV4: PTP over UDP/IPv4
 *              - TRANSPORT_UDP_IPV6: PTP over UDP/IPv6
 *              - TRANSPORT_IEEE_802_3: PTP over Ethernet II (EtherType)
 * @retval Pointer to initialized address or NULL on error
 */
const uint8_t *dm9058_init_ptp(const uint8_t *adr)
{
  adr = dm9058_init(adr); // dm9058dev.init(adr); //dm9058_init(adr);
  if (!adr)
  {
    printf("[edrv.init].error\r\n\r\n");
    return NULL;
  }

  struct ptptime_t ts;
  int64_t          rate;
  uint8_t          ptp_offset_addr;   /* Timestamp offset (Register 0x65) */
  uint8_t          ptp_checksum_addr; /* Checksum offset (Register 0x66) */

  /* PTP restart sequence */
  HAL_write_reg(0x60, 0x01); /* PTP restart bit */
  delay_ms(1);
  HAL_write_reg(0x60, 0x00);

  /* Enable PTP functionality */
  HAL_write_reg(0x61, 0x01); /* PTP Enable */

  /* Disable TX timestamp capture initially */
  HAL_write_reg(0x02, 0x00); /* Disable TX PTP timestamp */

  /* Master/Slave Mode & 1588 Version Register */
  HAL_write_reg(0x64, 0x12); /* RX_EN=0x10 | multicast=0x02 */

  /* TX One Step configuration */
  HAL_write_reg(0x63, 0x00); /* TX One Step disabled */

  /* Configure 1-step sync packet offsets based on network transport type */
  #if defined(PTP_NETWORK_TRANSPORT)
    #if (PTP_NETWORK_TRANSPORT == TRANSPORT_UDP_IPV4)
      /* PTP over UDP/IPv4 */
      printf("PTP Transport: UDP over IPv4\n");
      ptp_offset_addr   = 0x4E;  /* Timestamp offset for IPv4 */
      ptp_checksum_addr = 0x3C;  /* Checksum offset for IPv4 */
    
    #elif (PTP_NETWORK_TRANSPORT == TRANSPORT_UDP_IPV6)
      /* PTP over UDP/IPv6 */
      printf("PTP Transport: UDP over IPv6\n");
      ptp_offset_addr   = 0x62;  /* Timestamp offset for IPv6 */
      ptp_checksum_addr = 0x50;  /* Checksum offset for IPv6 */
    
    #elif (PTP_NETWORK_TRANSPORT == TRANSPORT_IEEE_802_3)
      /* PTP over Ethernet II (EtherType) */
      printf("PTP Transport: Ethernet II (EtherType 0x88F7)\n");
      ptp_offset_addr   = 0x32;  /* Timestamp offset for Ethernet II */
      ptp_checksum_addr = 0x20;  /* Checksum offset for Ethernet II */
    
    #else
      /* Default fallback to IPv4 */
      printf("PTP Transport: Unknown, using IPv4 defaults\n");
      ptp_offset_addr   = 0x4E;
      ptp_checksum_addr = 0x3C;
    #endif
  #else
    /* Default to IPv4 if PTP_NETWORK_TRANSPORT not defined */
    printf("PTP Transport: Not defined, using IPv4 defaults\n");
    ptp_offset_addr   = 0x4E;
    ptp_checksum_addr = 0x3C;
  #endif

  /* 1-step sync packet configuration - Timestamp offset (Register 0x65) */
  HAL_write_reg(0x65, ptp_offset_addr);
  
  /* 1-step sync packet configuration - Checksum offset (Register 0x66) */
  HAL_write_reg(0x66, ptp_checksum_addr);

  /* Set initial time */
  ts.tv_sec  = 2025030616; /* Default time in seconds */
  ts.tv_nsec = 0;
  dm9058_ptptime_settime(&ts); // v51_ptptime_settime

  /* Read back and display initial time */
  dm9058_ptptime_gettime(&ts); // v51_ptptime_gettime
  printf("****** dm9058_ptptime_gettime: %u sec %u ns\r\n", ts.tv_sec, ts.tv_nsec);
  //  printf("Init PTP time: %u.%09u\r\n", ts.tv_sec, ts.tv_nsec);

  /* Get initial rate */
  v51_getPtpClockRate(&rate);

  /* Note: tx_manager_update() will be called after PTPd_Init() completes */

  // 會關閉 led 功能，暫不啟用
  // DA1082S_E1_design_report_part1.pdf datasheet.
  // 實際測試 dm9058_ptp_pps_set GP2 enable, mac reg[0x3C] = 0xA0, bit 7~4 = 0xA, bit 3~0 = 0x0
  // Set MAC REG_3CH = 0xa0
  // GP1: 80ns pulse per 1 sec
  // GP2: toggle per 1 sec
  // cspi_write_reg(0x3C, 0xA0);
  // HAL_write_reg(0x3C, 0xA0);

  // Set MAC REG_3CH = 0xb0
  // LNKLED: 80ns pulse per 1 sec
  // SPDLED: toggle per 1 sec
  // cspi_write_reg(0x3C, 0xB0);
  // HAL_write_reg(0x3C, 0xB0);

  return adr;
}

// #include "ptpd.h" //#include "ptpd_v51.h" //TEMP

/**
 * @brief  將時間偏移量加入到 PTP 時鐘中
 * @param  timestamp: PTP 時間結構指標，包含要增加的偏移量（秒和納秒）
 * @retval none
 * @note   此函式將時間偏移值寫入硬體暫存器，用於微調 PTP 時鐘
 *         時間格式為小端序（nanoseconds 在前，seconds 在後）
 */
void v51_ptptime_add_offset(struct ptptime_t *timestamp)
{
  int     i;
  uint8_t time_buf[8];

  /* Convert to timestamp format (nanoseconds first, then seconds) */
  time_buf[0] = (uint8_t)(timestamp->tv_nsec & 0x000000FF);
  time_buf[1] = (uint8_t)((timestamp->tv_nsec & 0x0000FF00) >> 8);
  time_buf[2] = (uint8_t)((timestamp->tv_nsec & 0x00FF0000) >> 16);
  time_buf[3] = (uint8_t)((timestamp->tv_nsec & 0xFF000000) >> 24);
  time_buf[4] = (uint8_t)(timestamp->tv_sec & 0x000000FF);
  time_buf[5] = (uint8_t)((timestamp->tv_sec & 0x0000FF00) >> 8);
  time_buf[6] = (uint8_t)((timestamp->tv_sec & 0x00FF0000) >> 16);
  time_buf[7] = (uint8_t)((timestamp->tv_sec & 0xFF000000) >> 24);

  /* Reset index and write time data */
  HAL_write_reg(0x61, 0x80);

  /* Write 8 bytes of time data */
  for (i = 0; i < 8; i++)
    HAL_write_reg(0x68, time_buf[i]);

  /* Apply time setting */
  HAL_write_reg(0x61, 0x10); // 0x10
  // HAL_write_reg(0x61, 0x11);         // 0x10 | 0x01
}

/**
 * @brief  從 DM9058 硬體讀取當前 PTP 時間
 * @param  timestamp: 用於儲存讀取時間的 PTP 時間結構指標
 * @retval none
 * @note   從硬體暫存器 0x68 讀取 8 個位元組的時間資料
 *         時間格式為小端序（nanoseconds 在前 4 bytes，seconds 在後 4 bytes）
 */
void dm9058_ptptime_gettime(struct ptptime_t *timestamp)
{
  //  timestamp->tv_nsec = 0; //emac_ptpsubsecond2nanosecond(EMAC_PTP->tsl);
  //  timestamp->tv_sec = 0; //EMAC_PTP->tsh;

  /* Setup register to read PTP clock time */
  HAL_write_reg(0x61, 0x84); /* Read PTP Clock Time */

  /* Read 8 bytes of time data using memory read */
  HAL_read_reg_mem(0x68, g_time_buf, 8);

  /* Convert to timestamp format (nanoseconds first, then seconds) */
  timestamp->tv_nsec = (uint32_t)g_time_buf[0] | (uint32_t)g_time_buf[1] << 8 | (uint32_t)g_time_buf[2] << 16 |
                       (uint32_t)g_time_buf[3] << 24;
  timestamp->tv_sec = (uint32_t)g_time_buf[4] | (uint32_t)g_time_buf[5] << 8 | (uint32_t)g_time_buf[6] << 16 |
                      (uint32_t)g_time_buf[7] << 24;
}

/**
 * @brief  將 PTP 時間結構轉換為納秒數
 * @param  ptp_time: PTP 時間結構指標
 * @retval 納秒數（64位元）
 */
static inline uint64_t ptptime_to_nanoseconds(const struct ptptime_t *ptp_time)
{
  const uint64_t NS_PER_SEC = 1000000000ULL;
  return (uint64_t)ptp_time->tv_sec * NS_PER_SEC + (uint64_t)ptp_time->tv_nsec;
}

/**
 * @brief  測量 dm9058_ptp_get_time 函式的執行時間
 * @param  iterations: 測量次數用於平均（0 表示使用預設值 1000）
 * @retval 平均執行時間（奈秒）
 * @note   使用 PTP 時間戳本身來測量，測量三次調用的時間差（start->dummy->end），
 *         包含兩次 get_time 調用的總時間
 */
uint64_t dm9058_measure_ptp_get_time_latency(uint32_t iterations)
{
  struct ptptime_t ptp_time_start, ptp_time_end, ptp_time_dummy;
  uint64_t         total_time_ns = 0;
  uint32_t         primask;
  const uint32_t   DEFAULT_ITERATIONS = 1000;
  const uint32_t   MEASURE_CALLS      = 2; /* 測量兩次調用的時間差 */

  /* 使用預設迭代次數 */
  if (iterations == 0)
  {
    iterations = DEFAULT_ITERATIONS;
  }

  /* 關閉中斷以獲得穩定測量 */
  primask = __get_PRIMASK();
  __disable_irq();

  /* 執行多次測量以獲得平均值 */
  for (uint32_t i = 0; i < iterations; i++)
  {
    /* 記錄開始時間 */
    dm9058_ptptime_gettime(&ptp_time_start);

    /* 執行被測函式（測量一次調用的開銷） */
    dm9058_ptptime_gettime(&ptp_time_dummy);

    /* 記錄結束時間 */
    dm9058_ptptime_gettime(&ptp_time_end);

    /* 計算並累加時間差（從第一次到第三次調用的時間，包含兩次調用） */
    total_time_ns += (ptptime_to_nanoseconds(&ptp_time_end) - ptptime_to_nanoseconds(&ptp_time_start));
  }

  /* 恢復中斷 */
  __set_PRIMASK(primask);

  /* 計算平均時間 */
  const uint64_t avg_two_calls = total_time_ns / iterations;
  const uint64_t avg_one_call  = avg_two_calls / MEASURE_CALLS;

  /* 輸出測量結果 */
  printf("[PTP-LATENCY] Measured over %lu iterations:\r\n", (unsigned long)iterations);
  printf("  Average time (2 calls): %llu ns\r\n", (unsigned long long)avg_two_calls);
  printf("  Estimated single call: %llu ns (%.3f us)\r\n", (unsigned long long)avg_one_call,
         (double)avg_one_call / 1000.0);

  return avg_one_call;
}

/**
 * @brief  使用 DWT cycle counter 精確測量執行時間
 * @param  iterations: 測量次數
 * @retval 平均執行時間（奈秒）
 * @note   需要啟用 DWT cycle counter，提供更精確的 CPU 週期級測量
 */
uint64_t dm9058_measure_ptp_get_time_with_dwt(uint32_t iterations)
{
  struct ptptime_t ptp_time;
  uint64_t         total_cycles = 0;
  uint32_t         start_cycle, end_cycle;
  uint32_t         i;
  uint32_t         primask;

  if (iterations == 0)
    iterations = 1000;

  /* 啟用 DWT cycle counter */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* 關閉中斷 */
  primask = __get_PRIMASK();
  __disable_irq();

  for (i = 0; i < iterations; i++)
  {
    /* 重置並開始計數 */
    DWT->CYCCNT = 0;
    start_cycle = DWT->CYCCNT;

    /* 執行被測函式 */
    dm9058_ptptime_gettime(&ptp_time);

    /* 讀取結束計數 */
    end_cycle = DWT->CYCCNT;

    /* 累加週期數 */
    total_cycles += (end_cycle - start_cycle);
  }

  /* 恢復中斷 */
  __set_PRIMASK(primask);

  /* 計算平均 CPU 週期 */
  uint64_t avg_cycles = total_cycles / iterations;

  /* 動態取得 CPU 頻率 */
  crm_clocks_freq_type crm_clocks = {0};
  crm_clocks_freq_get(&crm_clocks);
  uint32_t cpu_freq_mhz = crm_clocks.sclk_freq / 1000000; // 從系統時鐘取得

  uint64_t avg_time_ns = (avg_cycles * 1000) / cpu_freq_mhz;

  printf("[PTP-LATENCY-DWT] Measured over %lu iterations:\r\n", iterations);
  printf("  Average CPU cycles: %llu\r\n", avg_cycles);
  printf("  Average time: %llu ns (%.3f us)\r\n", avg_time_ns, (double)avg_time_ns / 1000.0);
  printf("  Actual CPU freq: %lu MHz\r\n", cpu_freq_mhz);

  return avg_time_ns;
}

static int64_t last_rate = 0; /* Last cumulative adjustment value */

/**
 * @brief  設定 DM9058 PTP 時鐘時間
 * @param  timestamp: PTP 時間結構指標，包含要設定的時間（秒和納秒）
 * @retval none
 * @note   此函式會執行 PTP 重啟序列並寫入新的時間值
 *         同時會重置累積調整值 last_rate
 *         時間格式為小端序（nanoseconds 在前，seconds 在後）
 */
void dm9058_ptptime_settime(struct ptptime_t *timestamp)
{
  int     i;
  uint8_t time_buf[8];

  /* Convert timestamp to byte array (nanoseconds first, then seconds) */
  time_buf[0] = (uint8_t)(timestamp->tv_nsec & 0x000000FF);
  time_buf[1] = (uint8_t)((timestamp->tv_nsec & 0x0000FF00) >> 8);
  time_buf[2] = (uint8_t)((timestamp->tv_nsec & 0x00FF0000) >> 16);
  time_buf[3] = (uint8_t)((timestamp->tv_nsec & 0xFF000000) >> 24);
  time_buf[4] = (uint8_t)(timestamp->tv_sec & 0x000000FF);
  time_buf[5] = (uint8_t)((timestamp->tv_sec & 0x0000FF00) >> 8);
  time_buf[6] = (uint8_t)((timestamp->tv_sec & 0x00FF0000) >> 16);
  time_buf[7] = (uint8_t)((timestamp->tv_sec & 0xFF000000) >> 24);

  /* PTP restart sequence */
  HAL_write_reg(0x60, 0x01); /* PTP restart bit */
  delay_us(2);
  HAL_write_reg(0x60, 0x00);
  last_rate = 0; /* Reset accumulated adjustment */

  /* Reset index and write time data */
  HAL_write_reg(0x61, 0x80); /* Reset register index */

  /* Write 8 bytes of time data */
  for (i = 0; i < 8; i++)
  {
    HAL_write_reg(0x68, time_buf[i]);
  }

  /* Apply time setting */
  HAL_write_reg(0x61, 0x09); /* Write PTP Clock Time | PTP Enable */

  printf("****** dm9058_ptptime_settime: %u sec %u ns\r\n", timestamp->tv_sec, timestamp->tv_nsec);
}

/**
 * @brief  調整 PTP 時鐘頻率
 * @param  adjustment: 調整值（32位元無符號整數）
 * @param  direction: 調整方向（1: 減慢時鐘，0: 加快時鐘）
 * @retval none
 * @note   調整值不能超過 MAX_ADJUSTMENT (0xEFFFFFFF)
 *         控制值：direction=1 使用 ADJUST_SLOWER_CTRL (0x60)
 *                 direction=0 使用 ADJUST_FASTER_CTRL (0x20)
 */
void v51_adjust_ptp_frequency(uint32_t adjustment, int8_t direction)
{
  uint8_t rate_bytes[4] = {0};
  uint8_t control_value;

  /* Parameter validation */
  if (adjustment > MAX_ADJUSTMENT)
  {
    printf("[PTP] Error: Adjustment value 0x%08lX exceeds maximum allowed\n", adjustment);
    return;
  }

  /* Convert 32-bit adjustment to byte array (little-endian) */
  rate_bytes[0] = (uint8_t)(adjustment);
  rate_bytes[1] = (uint8_t)(adjustment >> 8);
  rate_bytes[2] = (uint8_t)(adjustment >> 16);
  rate_bytes[3] = (uint8_t)(adjustment >> 24);

  /* Reset register index */
  HAL_write_reg(0x61, 0x80);

  /* Write each byte */
  for (uint8_t i = 0; i < 4; i++)
  {
    HAL_write_reg(0x68, rate_bytes[i]);
  }

  /* Apply adjustment */
  control_value = (direction == 1) ? ADJUST_SLOWER_CTRL : ADJUST_FASTER_CTRL;
  HAL_write_reg(0x61, control_value);
}


#define v51_ptp_time_adj_freq dm9058_ptptime_adjfreq

/**
 * @brief  Adjust PTP frequency based on adj value
 * @param  adj: Adjustment value in ppb
 * @retval none
 */
void v51_ptp_time_adj_freq(int32_t adj)
{
  // dm9058 ptp clock 25M hz 計算參數: 2^32*40/10^9 = 171.79869184 rate = 1ppb.
  int64_t signed_addend = (int64_t)(adj * V51_ADJ_FREQ_BASE_ADDEND);
  int64_t delta_rate = signed_addend - last_rate;

  // 決定最終調整方向與值
  uint32_t adjust_value;
  int8_t   adjust_direction;

  if (delta_rate < 0)
  {
    adjust_direction = 1; // 慢下來
    adjust_value     = (uint32_t)(-delta_rate);
  }
  else
  {
    adjust_direction = 0; // 加快
    adjust_value     = (uint32_t)delta_rate;
  }

  // 呼叫硬體調整函數
  v51_adjust_ptp_frequency(adjust_value, adjust_direction);

  // 更新累積調整值
  last_rate = signed_addend;
}

/**
 * @brief  更新 PTP 時鐘偏移量
 * @param  timeoffset: 時間偏移量結構指標（秒和納秒）
 * @retval none
 * @note   此函式讀取當前時間，加上偏移量後設定新時間
 *         會自動處理納秒的溢位和下溢（正規化到 0-999999999 範圍）
 */
void dm9058_ptptime_updateoffset(struct ptptime_t *timeoffset)
{
  struct ptptime_t current_time;
  struct ptptime_t new_time;

  /* Get current time */
  dm9058_ptptime_gettime(&current_time); // v51_ptptime_gettime

  /* Calculate new time by adding offset */
  new_time.tv_sec  = current_time.tv_sec + timeoffset->tv_sec;
  new_time.tv_nsec = current_time.tv_nsec + timeoffset->tv_nsec;

  /* Handle nanosecond overflow/underflow */
  if (new_time.tv_nsec >= 1000000000)
  {
    new_time.tv_sec += 1;
    new_time.tv_nsec -= 1000000000;
  }
  else if (new_time.tv_nsec < 0)
  {
    new_time.tv_sec -= 1;
    new_time.tv_nsec += 1000000000;
  }

  /* Set the new time */
  dm9058_ptptime_settime(&new_time); // v51_ptptime_settime

  printf("dm9058_ptptime_updateoffset: offset %ds %dns applied\r\n", timeoffset->tv_sec, timeoffset->tv_nsec);
}

/**
 * @brief  Get TX timestamp from DM9058
 * @param  pTimeStamp: Pointer to timestamp structure
 * @retval none
 */
void impl_dm9058_get_tx_timestamp(struct ptptime_t *pTimeStamp)
{
  uint8_t timeStampBuf[8] = {0};
  int     i;

  /* Reset timestamp read index */
  HAL_write_reg(0x61, 0x80);

  /* Set TX timestamp read mode */
  HAL_write_reg(0x62, 0x01);

  /* Read 8 bytes of timestamp data */
  for (i = 0; i < 8; i++)
  {
    timeStampBuf[i] = HAL_read_reg(0x68);
  }

  /* Convert to standard timestamp format (nanoseconds first, then seconds) */
  pTimeStamp->tv_nsec = (uint32_t)timeStampBuf[0] | (uint32_t)timeStampBuf[1] << 8 | (uint32_t)timeStampBuf[2] << 16 |
                        (uint32_t)timeStampBuf[3] << 24;

  pTimeStamp->tv_sec = (uint32_t)timeStampBuf[4] | (uint32_t)timeStampBuf[5] << 8 | (uint32_t)timeStampBuf[6] << 16 |
                       (uint32_t)timeStampBuf[7] << 24;
}

/**
 * @brief  Helper function for rate debug logging
 * @param  buffer: Rate buffer data
 * @retval none
 */
static void logRateDebugInfo(const uint8_t *buffer)
{
  /* Extract rate from buffer (bytes 0-3, little-endian) */
  uint32_t rate =
      (uint32_t)buffer[3] << 24 | (uint32_t)buffer[2] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[0];

  /* Extract sign from buffer (byte 4) */
  uint8_t sign = buffer[4] & 0x01;

  /* Determine sign character and direction */
  char        signChar  = ((sign & 0x01) == 0) ? '+' : '-';
  const char *direction = (sign == 0) ? "faster" : "slower";

  /* Print formatted rate information */
  //  printf("dm9058 Rate: %c0x%08X (%c%ld) - Clock adjusted to be %s\r\n", signChar, rate, signChar, (long)rate,
  //  direction);
  printf("****** dm9058 Rate: %c0x%08X (%c%ld) - Clock adjusted to be %s\r\n", signChar, rate, signChar, (long)rate,
         direction);
}

/**
 * @brief  Reset and setup PTP clock read mode
 * @retval none
 */
static void reset_ptp_clock_read_mode(void)
{
  HAL_write_reg(0x69, 0x01); /* Enable rate read mode */
  HAL_write_reg(0x61, 0x80); /* Reset register index */
}

/**
 * @brief  Validate register index value
 * @param  index: Current index value
 * @param  position: Index position (0-3)
 * @retval 1 if index is correct, 0 if error
 */
static uint8_t validate_register_index(uint8_t index, uint8_t position)
{
  const uint8_t expected_indices[] = {0x10, 0x20, 0x30, 0x40};
  return (index == expected_indices[position]);
}

/**
 * @brief  Read PTP clock rate value
 * @param  rateBuffer: Buffer to store rate value
 * @param  reg_index: Buffer to store register indices
 * @retval 1 if read successful, 0 if failed
 */
static uint8_t read_ptp_clock_rate(uint8_t *rateBuffer, uint8_t *reg_index)
{
  uint8_t retry_count = 0;
  uint8_t success     = 0;

  while (retry_count < MAX_RETRY_COUNT && !success)
  {
    uint8_t i;

    reset_ptp_clock_read_mode();
    success = 1;

    /* Read 8 bytes of data */
    for (i = 0; i < 8 && success; i++)
    {
      rateBuffer[i] = HAL_read_reg(0x68);
      reg_index[i]  = HAL_read_reg(0x69);

      /* Check only first 4 index values */
      if (i < 4 && !validate_register_index(reg_index[i], i))
      {
        success = 0;
        retry_count++;
        printf("Invalid index at position %d: 0x%02X (retry %d)\n", i, reg_index[i], retry_count);
        break;
      }
    }
  }

  return success;
}

/**
 * @brief  Get current PTP clock rate adjustment value
 * @param  ptrRateValue: Pointer to store rate value
 * @retval 0 on success, negative on error
 */
int32_t v51_getPtpClockRate(int64_t *ptrRateValue)
{
  uint8_t       rateBuffer[8]     = {0};
  uint8_t       reg_index[8]      = {0};
  uint32_t      absoluteRate      = 0;
  const int32_t success           = 0;
  const int32_t invalidParamError = -1;
  const int32_t readError         = -2;

  /* Validate input parameter */
  if (!ptrRateValue)
  {
    return invalidParamError;
  }

  /* Attempt to read clock rate */
  if (!read_ptp_clock_rate(rateBuffer, reg_index))
  {
    printf("Failed to read PTP clock rate after %d attempts\n", MAX_RETRY_COUNT);
    return readError;
  }

  /* Combine 32-bit rate value (little-endian format) */
  absoluteRate = (uint32_t)rateBuffer[3] << 24 | (uint32_t)rateBuffer[2] << 16 | (uint32_t)rateBuffer[1] << 8 |
                 (uint32_t)rateBuffer[0];

  /* Set final value based on sign bit */
  *ptrRateValue = ((rateBuffer[4] & 0x01) == 0) ? (int64_t)absoluteRate : -(int64_t)absoluteRate;

  /* Output debug information */
  logRateDebugInfo(rateBuffer);

  return success;
}

/* match library
 *  is_packet_match(type, DELAY_REQ)
 */

/**
 * @brief  檢查是否為 PTP SYNC 封包
 * @param  msgtype: PTP 訊息類型
 * @retval 1: 是 SYNC 封包，0: 不是
 */
int is_ptp_sync_packet(u8_t msgtype) { return (msgtype == SYNC) ? 1 : 0; }

/**
 * @brief  檢查是否為 PTP DELAY_REQ 封包
 * @param  msgtype: PTP 訊息類型
 * @retval 1: 是 DELAY_REQ 封包，0: 不是
 */
int is_ptp_delayreq_packet(u8_t msgtype) { return (msgtype == DELAY_REQ) ? 1 : 0; }

/**
 * @brief  檢查是否為 PTP DELAY_RESP 封包
 * @param  msgtype: PTP 訊息類型
 * @retval 1: 是 DELAY_RESP 封包，0: 不是
 */
int is_ptp_delayresp_packet(u8_t msgtype) { return (msgtype == DELAY_RESP) ? 1 : 0; }

/**
 * @brief  檢查是否為 PTP PDELAY_REQ 封包
 * @param  msgtype: PTP 訊息類型
 * @retval 1: 是 PDELAY_REQ 封包，0: 不是
 */
int is_ptp_pdelayreq_packet(u8_t msgtype) { return (msgtype == PDELAY_REQ) ? 1 : 0; }

/**
 * @brief  檢查是否為 PTP PDELAY_RESP 封包
 * @param  msgtype: PTP 訊息類型
 * @retval 1: 是 PDELAY_RESP 封包，0: 不是
 */
int is_ptp_pdelayresp_packet(u8_t msgtype) { return (msgtype == PDELAY_RESP) ? 1 : 0; }

/**
 * @brief  取得 PTP 封包的來源埠號
 * @param  buf: 封包緩衝區指標
 * @retval UDP 來源埠號（主機位元組順序）
 * @note   跳過乙太網路標頭 (ETH_HLEN=14 bytes) 和 IP 標頭後讀取 UDP 標頭
 */
u16_t get_ptp_src_port(uint8_t *buf)
{
  /* L2 PTP (0x88F7) has no UDP ports */
  struct eth_hdr *eth = (struct eth_hdr *)buf;
  u16_t           proto = PP_NTOHS(eth->type);
  if (proto != ETH_P_IP)
    return 0;

  buf += ETH_HLEN;
  struct ip_hdr *ip = (struct ip_hdr *)buf;
  if (IPH_PROTO(ip) != IPPROTO_UDP)
    return 0;

  u16_t iphdr_len = (u16_t)(IPH_HL(ip) * 4U);
  struct udp_hdr *udp = (struct udp_hdr *)(buf + iphdr_len);
  return PP_NTOHS(udp->src);
}

/**
 * @brief  取得 PTP 封包的目的埠號
 * @param  buf: 封包緩衝區指標
 * @retval UDP 目的埠號（主機位元組順序）
 * @note   跳過乙太網路標頭 (ETH_HLEN=14 bytes) 和 IP 標頭後讀取 UDP 標頭
 */
u16_t get_ptp_dest_port(uint8_t *buf)
{
  /* L2 PTP (0x88F7) has no UDP ports */
  struct eth_hdr *eth = (struct eth_hdr *)buf;
  u16_t           proto = PP_NTOHS(eth->type);
  if (proto != ETH_P_IP)
    return 0;

  buf += ETH_HLEN;
  struct ip_hdr *ip = (struct ip_hdr *)buf;
  if (IPH_PROTO(ip) != IPPROTO_UDP)
    return 0;

  u16_t iphdr_len = (u16_t)(IPH_HL(ip) * 4U);
  struct udp_hdr *udp = (struct udp_hdr *)(buf + iphdr_len);
  return PP_NTOHS(udp->dest);
}

/**
 * @brief  驗證是否為有效的 PTP 訊息類型
 * @param  buf: 封包緩衝區指標
 * @retval 1: 是有效的 PTP 訊息，0: 不是
 * @note   檢查條件：
 *         - 協定為 UDP (17)
 *         - 來源埠號為 PTP_EVENT_PORT (319) 或 PTP_GENERAL_PORT (320)
 *         - 目的埠號為 PTP_EVENT_PORT (319) 或 PTP_GENERAL_PORT (320)
 */
int valid_ptp_message_type(uint8_t *buf)
{
  /* Support both:
   * - L2 PTP: EtherType 0x88F7 (no UDP ports)
   * - L3/L4 PTP: UDP ports 319/320
   */
  struct eth_hdr *eth = (struct eth_hdr *)buf;
  u16_t           proto = PP_NTOHS(eth->type);

  if (proto == PTP_ETHERTYPE)
    return 1;

  if (proto != ETH_P_IP)
    return 0;

  buf += ETH_HLEN;
  struct ip_hdr *ip = (struct ip_hdr *)buf;
  if (IPH_PROTO(ip) != IPPROTO_UDP)
    return 0;

  u16_t iphdr_len = (u16_t)(IPH_HL(ip) * 4U);
  struct udp_hdr *udp = (struct udp_hdr *)(buf + iphdr_len);

  u16_t src = PP_NTOHS(udp->src);
  u16_t dst = PP_NTOHS(udp->dest);

  if ((src == PTP_EVENT_PORT || src == PTP_GENERAL_PORT) && (dst == PTP_EVENT_PORT || dst == PTP_GENERAL_PORT))
    return 1;

  return 0;
}

/**
 * @brief  解析並取得 PTP 訊息類型
 * @param  buf: 封包緩衝區指標
 * @retval PTP 訊息類型（messageType 欄位）
 * @note   跳過乙太網路標頭(14)、IP標頭(20)、UDP標頭(8)後解析 PTP 標頭
 *         使用 msgUnpackHeader 函式解析 PTP 標頭結構
 */
u8_t get_ptp_message_type005(uint8_t *buf)
{
  MsgHeader header;
  struct eth_hdr *eth = (struct eth_hdr *)buf;
  u16_t           proto = PP_NTOHS(eth->type);

  if (proto == PTP_ETHERTYPE)
  {
    /* L2 PTP header begins after Ethernet header */
    msgUnpackHeader((const Octet *)(buf + ETH_HLEN), &header);
    return header.messageType;
  }

  /* UDP/IPv4 PTP */
  buf += ETH_HLEN;
  struct ip_hdr *ip = (struct ip_hdr *)buf;
  u16_t          iphdr_len = (u16_t)(IPH_HL(ip) * 4U);
  msgUnpackHeader((const Octet *)(buf + iphdr_len + sizeof(struct udp_hdr)), &header);
  return header.messageType;
}

/**
 * @brief  檢查是否需要發出 PTP 時間戳記（1-step 模式）
 * @param  buf: 封包緩衝區指標
 * @retval 1: 需要發出時間戳記，0: 不需要
 * @note   條件：有效的 PTP 訊息 && SYNC 封包 && 使用 1-step 模式 (TWO_STEP_FLAG=FALSE)
 *         && 來源和目的埠號都是 PTP_EVENT_PORT (319)
 */
int is_issue_ptp_tstamp_emit(uint8_t *buf)
{
  if (!valid_ptp_message_type(buf))
    return 0;

  u8_t  msgtype   = get_ptp_message_type005(buf);
  u16_t src_port  = get_ptp_src_port(buf);
  u16_t dest_port = get_ptp_dest_port(buf);
  return (src_port == PTP_EVENT_PORT && dest_port == PTP_EVENT_PORT && is_ptp_sync_packet(msgtype) &&
          DEFAULT_TWO_STEP_FLAG == FALSE)
      /* PTP_EVENT_PORT is 319 */
      /*|| is_ptp_delayreq_packet(msgtype)*/;
}

/**
 * @brief  檢查是否需要啟用 PTP 時間戳記捕獲
 * @param  buf: 封包緩衝區指標
 * @retval 1: 需要啟用時間戳記捕獲，0: 不需要
 * @note   條件：有效的 PTP 訊息 && 
 *         ((SYNC 封包 && 2-step 模式) || DELAY_REQ || PDELAY_REQ)
 */
int is_issue_ptp_tstamp_tsen(uint8_t *buf)
{
  if (!valid_ptp_message_type(buf))
    return 0;

  u8_t msgtype = get_ptp_message_type005(buf);
  /*
   * P2P (peer delay) needs responder's TX timestamp (T3) for PDELAY_RESP,
   * which is carried in PDELAY_RESP_FOLLOW_UP (2-step) or embedded (1-step).
   *
   * If we don't enable TSEN_CAP for PDELAY_RESP, upper layers will see zero
   * timestamps and fall back to software time (netSendL2: "Hardware timestamp
   * not available"), breaking P2P delay measurement accuracy.
   */
  return (is_ptp_sync_packet(msgtype) && DEFAULT_TWO_STEP_FLAG == TRUE) ||
         is_ptp_delayreq_packet(msgtype) ||
         is_ptp_pdelayreq_packet(msgtype) ||
         is_ptp_pdelayresp_packet(msgtype);
}

/**
 * @brief  檢查封包是否為 DELAY_REQ 類型
 * @param  buf: 封包緩衝區指標
 * @retval 1: 是 DELAY_REQ 封包，0: 不是
 * @note   先驗證是否為有效的 PTP 訊息，再檢查訊息類型
 */
int is_issue_ptp_tstamp_delayreq(uint8_t *buf)
{
  if (!valid_ptp_message_type(buf))
    return 0;

  u8_t msgtype = get_ptp_message_type005(buf);
  return is_ptp_delayreq_packet(msgtype);
}

/**
 * @brief  檢查封包是否為 DELAY_RESP 類型
 * @param  buf: 封包緩衝區指標
 * @retval 1: 是 DELAY_RESP 封包，0: 不是
 * @note   先驗證是否為有效的 PTP 訊息，再檢查訊息類型
 */
int is_issue_ptp_tstamp_delayresp_packet(uint8_t *buf)
{
  if (!valid_ptp_message_type(buf))
    return 0;

  u8_t msgtype = get_ptp_message_type005(buf);
  return is_ptp_delayresp_packet(msgtype);
}

/**
 * @brief  將 PTP 傳送封包的時間戳記欄位清零
 * @param  buffer: 封包緩衝區指標
 * @retval none
 * @note   跳過乙太網路(14)、IP(20)、UDP(8)標頭，清除 originTimestamp 欄位
 *         - secondsField.msb (偏移 34, 2 bytes)
 *         - secondsField.lsb (偏移 36, 4 bytes)
 *         - nanosecondsField (偏移 40, 4 bytes)
 */
void ptp_tx_tstamp_zero(uint8_t *buffer)
{
  const uint8_t *buf;

  /* Locate PTP message start for both L2 (0x88F7) and UDP/IPv4 (319/320) */
  struct eth_hdr *eth = (struct eth_hdr *)buffer;
  u16_t           proto = PP_NTOHS(eth->type);

  if (proto == PTP_ETHERTYPE)
  {
    buf = (buffer + ETH_HLEN);
  }
  else
  {
    /* UDP/IPv4 PTP */
    uint8_t *ipbuf = buffer + ETH_HLEN;
    struct ip_hdr *ip = (struct ip_hdr *)ipbuf;
    u16_t          iphdr_len = (u16_t)(IPH_HL(ip) * 4U);
    buf = (ipbuf + iphdr_len + sizeof(struct udp_hdr));
  }

  /*Sync message*/
  *(UInteger16 *)(buf + 34) = 0; // flip16(originTimestamp->secondsField.msb);
  *(UInteger32 *)(buf + 36) = 0; // flip32(originTimestamp->secondsField.lsb);
  *(UInteger32 *)(buf + 40) = 0; // flip32(originTimestamp->nanosecondsField);
}

/**
 * @brief  取得 PTP 傳送時間戳記的包裝函式
 * @param  ptimestamp: 用於儲存時間戳記的結構指標
 * @param  head: 標頭字串（用於除錯輸出）
 * @retval none
 * @note   此函式為 impl_dm9058_get_tx_timestamp 的包裝函式
 */
void V51_JJ_tx_ptptime(struct ptptime_t *ptimestamp, char *head) // ethif.c
{
  impl_dm9058_get_tx_timestamp(ptimestamp); // dm9058_ptp.c
}

/**
 * @brief  Receive packet with PTP timestamp
 * @param  buff: Packet buffer
 * @param  ts_bff: Timestamp buffer (8 bytes)
 * @retval Packet length
 */
uint16_t dm9058_rx_ptp(uint8_t *buffer, uint8_t *receivedata,
                       uint8_t *ts_bff) // dm9058_rx_ptp(uint8_t *buff, uint8_t *ts_bff)
{
  if (cspi_rx_ready())
    return cspi_rx_read(buffer, cspi_rx_head_ptp(receivedata, ts_bff));
  return 0;
}

// int ptp_rx_tstamp_process_flg = 0;
// void set_ptp_rx_tstamp_process(int val)
//{
//	ptp_rx_tstamp_process_flg = val;
// }

/**
 * @brief  傳送 PTP 封包並處理時間戳記
 * @param  buff: 封包緩衝區指標
 * @param  len: 封包長度
 * @param  p: pbuf 結構指標
 * @retval none
 * @note   處理流程：
 *         1. 重置傳送指標（修正長時間運行後的 sync 封包錯誤）
 *         2. 解析封包並設定 TCR 控制旗標
 *         3. 寫入封包資料
 *         4. 等待傳送完成
 *         5. 將時間戳記傳遞到 pbuf 結構
 */
void dm9058_tx_ptp(uint8_t *buff, uint16_t len, struct pbuf *p)
{
  /* Testing 2025-03-03 */
  // Fix sync packet error occurring after 3 hours of running.
  tx_pointer_timeout(500); // HAL_write_reg(0x55, 0x02); while (HAL_read_reg(0x55) & 0x02) ;
  len = cspi_tx_packet_len(len);
  if (len)
  {
    uint8_t tcr_wr;
    tcr_wr = ptp_tx_tstamp_parse_packet(buff, p);
    cspi_tx_write(buff, len);
    tx_compl_timeout(tcr_wr, 500);
    ptp_tx_tstamp_pass_to(buff, p, tcr_wr);
  }
}

/**
 * @brief  解析 PTP 封包類型並回傳類型字串
 * @param  buf: 封包緩衝區指標
 * @param  off: 偏移量
 * @retval PTP 封包類型字串 ("Announce", "Sync", "Delay_Resp", "Delay_Req", "Unknown")
 * @note   根據 PTP 標頭的訊息類型欄位和控制欄位判斷封包類型
 */
char *parse_buf_ptp_packet1(uint8_t *buf, int off)
{
  if (buf[off] == 0x8b && buf[32 + off] == CTRL_OTHER)
    return "Announce";
  else if (buf[off] == 0x80 && buf[32 + off] == CTRL_SYNC)
    return "Sync";
  else if (buf[off] == 0x89 && buf[32 + off] == CTRL_DELAY_RESP)
    return "Delay_Resp";
  else if (buf[off] == 0x81 && buf[32 + off] == CTRL_DELAY_REQ)
    return "Delay_Req";
  else
    return "Unknown";
}

/**
 * @brief  解析 pbuf 中的 PTP 封包類型
 * @param  buf: 封包緩衝區指標
 * @param  in_offset: 輸入偏移量
 * @retval PTP 封包類型字串
 * @note   此函式為 parse_buf_ptp_packet1 的包裝函式
 */
char *parse_pbuf_ptp_packet1(uint8_t *buf, int in_offset) { return parse_buf_ptp_packet1(buf, in_offset); }

/**
 * @brief  取得 PTP 標頭位置
 * @param  p: 封包緩衝區指標
 * @param  len: 封包長度
 * @param  rxts_en_packet: 接收時間戳記啟用旗標
 * @retval PTP 標頭位置指標，如果不是 PTP 封包則回傳 NULL
 * @note   支援 Layer 2 PTP (EtherType 0x88F7) 和 Layer 3/4 PTP (UDP port 319/320)
 *         會過濾 LLDP、IGMP 等非 PTP 封包
 */
char *get_ptp_header(uint8_t *p, uint16_t len, int rxts_en_packet)
{
  int             printflg = 1;
  struct eth_hdr *eth      = (struct eth_hdr *)p;
  u8             *ptp_hdr;
  u16             proto;
  struct ip_hdr  *ip;

  // Skip Ethernet header
  p += ETH_HLEN;
  proto = eth->type;
  proto = PP_HTONS(eth->type); // ntohs(eth->type);

  // Check for Layer 2 PTP
  if (proto == PTP_ETHERTYPE)
  {
    return (char *)p;
  }

  if (proto == ETH_P_LLDP && printflg)
  {
    static int ll2_pnt = 2;
    if (ll2_pnt)
    {
      ll2_pnt--;
      printf("get_ptp_header LLDP/LL2 packet\r\n");
    }
    return NULL;
  }

  // Handle IPv4
  ip = (struct ip_hdr *)p;
  if (proto == ETH_P_IP)
  {
    if (IPH_PROTO(ip) == IPPROTO_UDP)
    {
      struct udp_hdr *udp = (struct udp_hdr *)(p + sizeof(struct ip_hdr));
      if (PP_HTONS(udp->dest) == PTP_EVENT_PORT || PP_HTONS(udp->dest) == PTP_GENERAL_PORT)
      {
        ptp_hdr = (u8 *)udp + sizeof(struct udp_hdr);
        return (char *)ptp_hdr;
      }
      return NULL;
    }
    if (IPH_PROTO(ip) == IPPROTO_IGMP && printflg)
    {
      return NULL;
    }
  }

  if (rxts_en_packet && printflg)
  {
    printf("dst.ip %d.%d.%d.%d \r\n", (ip->dest.addr >> 0) & 0xff, (ip->dest.addr >> 8) & 0xff,
           (ip->dest.addr >> 16) & 0xff, (ip->dest.addr >> 24) & 0xff);

    /* rare, extra 24.1.0.224 */
    if (((ip->dest.addr >> 24) & 0xff) == 224 && ((ip->dest.addr >> 16) & 0xff) == 0 &&
        ((ip->dest.addr >> 8) & 0xff) == 1 && ((ip->dest.addr >> 0) & 0xff) == 24)
    {
      printf("=get_ptp_header dst 24.1.0.224 packet\r\n");
      return NULL;
    }
    printf("low_level_input_ (Check not a PTP packet) rx len %u dumpdata %d\r\n", len, 40);
    dump_data(eth->dest.addr, 40);
  }
  return NULL; // Not a PTP packet
}

/**
 * @brief  從 Delay_Resp 封包中提取接收時間戳記
 * @param  buffer: 封包緩衝區指標
 * @param  pTimeTmp: 用於儲存時間的 TimeInternal 結構指標
 * @retval none
 * @note   Delay_Resp 訊息結構：
 *         - PTP 標頭: 34 bytes (0-33)
 *         - receiveTimestamp: 從偏移 34 開始
 *           - secondsField.msb: 2 bytes 偏移 34-35 (big-endian)
 *           - secondsField.lsb: 4 bytes 偏移 36-39 (big-endian)
 *           - nanosecondsField: 4 bytes 偏移 40-43 (big-endian)
 */
void buffer_ts_time(uint8_t *buffer, TimeInternal *pTimeTmp)
{
  /* Extract receiveTimestamp from Delay_Resp message packet
   * Based on PTP message structure and msgUnpackDelayResp function
   *
   * Delay_Resp message structure:
   * - PTP header: 34 bytes (0-33)
   * - receiveTimestamp: starts at offset 34
   *   - secondsField.msb: 2 bytes at offset 34-35 (big-endian)
   *   - secondsField.lsb: 4 bytes at offset 36-39 (big-endian)
   *   - nanosecondsField: 4 bytes at offset 40-43 (big-endian)
   */

  /* Skip PTP header (34 bytes) and extract timestamp fields */
  uint8_t *timestamp_ptr = buffer + 34;

  /* Extract secondsField.msb (16-bit, big-endian) */
  //    uint16_t seconds_msb = (uint16_t)timestamp_ptr[0] << 8 | (uint16_t)timestamp_ptr[1];

  /* Extract secondsField.lsb (32-bit, big-endian) */
  uint32_t seconds_lsb = (uint32_t)timestamp_ptr[2] << 24 | (uint32_t)timestamp_ptr[3] << 16 |
                         (uint32_t)timestamp_ptr[4] << 8 | (uint32_t)timestamp_ptr[5];

  /* Extract nanosecondsField (32-bit, big-endian) */
  uint32_t nanoseconds = (uint32_t)timestamp_ptr[6] << 24 | (uint32_t)timestamp_ptr[7] << 16 |
                         (uint32_t)timestamp_ptr[8] << 8 | (uint32_t)timestamp_ptr[9];

  /* Convert to TimeInternal format */
  /* For most practical purposes, we only use the lsb part of seconds */
  pTimeTmp->seconds     = (int32_t)seconds_lsb;
  pTimeTmp->nanoseconds = (int32_t)nanoseconds;
}

struct ptptime_t delayREQ_T3, master_recv_extra_T3;

  #if (EDRIVER_ADDING_PTP && LWIP_PTP) || 1
/**
 * @brief  解析 PTP 傳送封包並設定 TCR 控制旗標
 * @param  buffer: 封包緩衝區指標
 * @param  p: pbuf 結構指標
 * @retval TCR 控制旗標值
 * @note   根據封包類型設定不同的 TCR 旗標：
 *         - TCR_TXREQ: 傳送請求（所有封包）
 *         - TCR_TS1STEP_EMIT: 1-step 時間戳記發出
 *         - TCR_TSEN_CAP: 啟用時間戳記捕獲
 *         對於需要 1-step 的封包，會先將時間戳記欄位清零
 */
uint8_t ptp_tx_tstamp_parse_packet(uint8_t *buffer, struct pbuf *p)
{
  uint8_t tcr_wr = TCR_TXREQ;
  if (is_issue_ptp_tstamp_emit(buffer))
  {
    tcr_wr |= TCR_TS1STEP_EMIT;
  }
  if (is_issue_ptp_tstamp_tsen(buffer))
  {
    tcr_wr |= TCR_TSEN_CAP;
    if (is_issue_ptp_tstamp_delayreq(buffer))
    {
      tcr_wr |= TCR_TS1STEP_EMIT;
      dm9058_ptp_dbg("%s [issue dlyreq, slave optional provide 1-step ts] tcr_wr %02x\r\n", PTPD_HEADER_MCU_DESC,
                     tcr_wr);
    }
  }

  // Fix sync packet with zero ts before Tx_Req.
  if (tcr_wr & TCR_TS1STEP_EMIT)
  {
    ptp_tx_tstamp_zero(buffer);
  }
  return tcr_wr;
}

/**
 * @brief  將 PTP 傳送時間戳記傳遞到 pbuf 和全域變數
 * @param  buffer: 封包緩衝區指標
 * @param  p: pbuf 結構指標
 * @param  tcr_wr: TCR 控制旗標值
 * @retval none
 * @note   處理內容：
 *         - 如果啟用 TSEN_CAP，讀取硬體時間戳記並儲存到 pbuf
 *         - 對於 DELAY_REQ 封包，儲存時間戳記到 delayREQ_T3
 *         - 對於 DELAY_RESP 封包（主機端），計算並顯示時間差
 */
void ptp_tx_tstamp_pass_to(uint8_t *buffer, struct pbuf *p, uint8_t tcr_wr)
{
  struct ptptime_t timestamp;
  
  if (tcr_wr & TCR_TSEN_CAP)
  {
    V51_JJ_tx_ptptime(&timestamp, "<stepT1.T3>*V51_TX.11");

    // Store timestamp in pbuf
    p->time_sec  = timestamp.tv_sec;
    p->time_nsec = timestamp.tv_nsec;
    
    if (is_issue_ptp_tstamp_delayreq(buffer))
    {
      dm9058_ptp_dbg("%s (slave on ISSUE_DELAYREQ) pk_ts(%u.%09u)\r\n", PTPD_HEADER_MCU_DESC, timestamp.tv_sec,
                     timestamp.tv_nsec);
    }
  }

  if (is_issue_ptp_tstamp_delayreq(buffer))
  {
    delayREQ_T3 = timestamp;
    dm9058_ptp_dbg("%s (slave save ISSUE_DELAYREQ, delayREQ_T3) TSEN_CAP ts(%u.%09u)\r\n", PTPD_HEADER_MCU_DESC,
                   delayREQ_T3.tv_sec, delayREQ_T3.tv_nsec);
  }

  /* master's check ('delayRESP_T4' belong to master) */
  if (is_issue_ptp_tstamp_delayresp_packet(buffer))
  {
    TimeInternal t4, t3;
    TimeInternal internalTime;
    buffer_ts_time(buffer + 14 + 20 + 8, &t4);
    internalTime.seconds     = t4.seconds;
    internalTime.nanoseconds = t4.nanoseconds;

    t3.seconds     = master_recv_extra_T3.tv_sec;
    t3.nanoseconds = master_recv_extra_T3.tv_nsec;
    subTime(&internalTime, &internalTime, &t3);

    dm9058_ptp_dbg("%s tx DELAY_RESP: master msgPackDelayResp on send(%u.%09u) diff to slave %10d s %11d ns\r\n",
                   PTPD_HEADER_MCU_DESC, t4.seconds, t4.nanoseconds, internalTime.seconds, internalTime.nanoseconds);
  }
}
  #endif

/**
 * @brief  Read receive packet header
 * @param  receivedata  Buffer to store header data (4 bytes)
 * @param  ts_bff       Timestamp buffer
 * @retval Packet length
 */
uint16_t cspi_rx_head_ptp(uint8_t *receivedata, uint8_t *ts_bff)
{
  HAL_read_mem(receivedata, 4);
  HAL_write_reg(DM9058_ISR, 0x80);
  return rx_head_takelen_ptp(receivedata, ts_bff);
}

/**
 * @brief  讀取接收封包的時間戳記資料
 * @param  receivedata: 接收資料緩衝區（包含接收狀態）
 * @param  ts_bff: 時間戳記緩衝區
 * @retval none
 * @note   根據 RSR_RXTS_EN 和 RSR_RXTS_LEN 旗標決定讀取長度：
 *         - RSR_RXTS_LEN=1: 讀取 8 bytes（完整時間戳記）
 *         - RSR_RXTS_LEN=0: 讀取 4 bytes（簡化時間戳記）
 */
void cspi_rx_tstamp_mem(uint8_t *receivedata, uint8_t *ts_bff)
{
  if (receivedata[1] & RSR_RXTS_EN)
  {
    if (receivedata[1] & RSR_RXTS_LEN)
    {
      HAL_read_mem(ts_bff, 8);
    }
    else
    {
      HAL_read_mem(ts_bff, 4);
    }
    HAL_write_reg(DM9058_ISR, 0x80);
  }
}

/**
 * @brief  解析接收封包標頭並取得封包長度
 * @param  receivedata: 接收資料緩衝區（4 bytes 標頭）
 * @param  ts_bff: 時間戳記緩衝區
 * @retval 封包長度，錯誤時回傳錯誤碼
 * @note   驗證接收狀態和長度：
 *         - 檢查 RSR_ERR_BITS（排除 PTP 相關位元）
 *         - 檢查長度不超過 DRVBUF_POOL_BUFSIZE
 *         - 讀取時間戳記資料（如果啟用）
 */
uint16_t rx_head_takelen_ptp(uint8_t *receivedata, uint8_t *ts_bff)
{
  uint16_t rx_len;
  uint8_t  rx_status;

  /* Read packet header */
  rx_status = receivedata[1];
  rx_len    = receivedata[2] + (receivedata[3] << 8);

  /* Validate packet status and length */
  DM9058_RX_BREAK((rx_status & (RSR_ERR_BITS & ~RSR_PTP_BITS)),
                  return env_err_rsthdlr3("_dm9058f rx_status error : 0x%02x\r\n", rx_status));
  DM9058_RX_BREAK((rx_len > DRVBUF_POOL_BUFSIZE), return env_err_rsthdlr3("_dm9058f rx_len error : %u\r\n", rx_len));

  cspi_rx_tstamp_mem(receivedata, ts_bff);
  return rx_len;
}

int32_t put_rateValue;

/**
 * @brief  儲存 PTP 時鐘速率值到全域變數
 * @param  rateValue: 速率值（ppb 單位）
 * @retval none
 * @note   此函式用於記錄當前的時鐘速率調整值
 */
void v51_putPtpClockRate(int32_t rateValue)
{
  put_rateValue = rateValue;
}

/**
 * @brief  Update PTP clock rate with signed addend
 * @param  signed_addend: Signed adjustment value
 * @retval none
 */
void v51_updatePtpClockRate(int64_t signed_addend)
{
  /* Convert 64-bit addend to 32-bit rate value */
  /* This conversion depends on the specific algorithm used */
  int32_t rate_ppb = (int32_t)(signed_addend >> 16); /* Example conversion */

  v51_putPtpClockRate(rate_ppb);
}

typedef struct
{
  int year;
  int month;
  int day;
} ymd_t;

ymd_t data_ydt;

/**
 * @brief  將 Unix 時間戳（秒）轉換為年月日（UTC 時間）
 * @param  seconds: Unix 時間戳（從 1970-01-01 00:00:00 UTC 開始的秒數）
 * @retval 指向 ymd_t 結構的指標，包含年、月、日
 * @note   考慮閏年規則：
 *         - 能被 4 整除且不能被 100 整除的年份
 *         - 或能被 400 整除的年份
 */
ymd_t *convert_seconds_to_ymd(uint32_t seconds)
{
  uint32_t days = seconds / 86400;
  int year  = 1970;
  int month = 1;
  int day   = 1;
  int month_days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}, i;
  while (1)
  {
    int is_leap      = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    int days_in_year = is_leap ? 366 : 365;
    if (days >= days_in_year)
    {
      days -= days_in_year;
      year++;
    }
    else
    {
      break;
    }
  }

  if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))
  {
    month_days[1] = 29;
  }
  for (i = 0; i < 12; i++)
  {
    if (days >= month_days[i])
    {
      days -= month_days[i];
      month++;
    }
    else
    {
      day = days + 1;
      break;
    }
  }

  data_ydt.year  = year;
  data_ydt.month = month;
  data_ydt.day   = day;
  return &data_ydt;
}

/**
 * @brief  以年月日格式列印內部時間
 * @param  head: 標頭字串
 * @param  seconds: 秒數（Unix 時間戳）
 * @param  nanoseconds: 納秒數
 * @retval none
 * @note   輸出格式：[head]: [seconds] s [nanoseconds] ns (v51) YYYY-MM-DD
 */
void internaltime_as_ymd_s(char *head, const s32_t seconds, s32_t nanoseconds)
{
  ymd_t *date = convert_seconds_to_ymd(seconds);
  printf("%s:%10d s %11d ns (v51) %04d-%02d-%02d\r\n", head, seconds, nanoseconds, date->year, date->month, date->day);
}

/**
 * @brief  Print timestamp in year-month-day format
 * @param  head: Header string
 * @param  time: Pointer to timestamp structure
 * @retval none
 */
void print_as_ymd(char *head, const struct ptptime_t *time)
{
  if (time == NULL)
    return;

  do
  {
    uint32_t days    = time->tv_sec / 86400;
    uint32_t hours   = (time->tv_sec % 86400) / 3600;
    uint32_t minutes = (time->tv_sec % 3600) / 60;
    uint32_t seconds = time->tv_sec % 60;

    printf("%s %ld days, %02ld:%02ld:%02ld.%09ld\r\n", head ? head : "[TIME]", days, hours, minutes, seconds,
           time->tv_nsec);
  } while (0);
}

/**
 * @brief  Print timestamp in year-month-day format with round parameter
 * @param  rnd: Rounding parameter
 * @param  head: Header string
 * @param  time: Pointer to timestamp structure
 * @retval none
 */
void print_as_ymd_s(int rnd, char *head, const struct ptptime_t *time)
{
  struct ptptime_t rounded_time = *time;

  if (time == NULL)
    return;

  /* Apply rounding if requested */
  if (rnd > 0 && rnd < 9)
  {
    int      i;
    uint32_t divisor = 1;
    for (i = 0; i < (9 - rnd); i++)
    {
      divisor *= 10;
    }
    rounded_time.tv_nsec = (rounded_time.tv_nsec / divisor) * divisor;
  }

  print_as_ymd(head, &rounded_time);
}
#endif
