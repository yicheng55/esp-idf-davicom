/*
 * DM9058 PTP 優化整合 - 現行 API 參考範例
 *
 * 這個檔案不是要直接編進 ESP-IDF build 的元件，而是用來記錄目前
 * DM9058 PTP driver、esp_eth_time、L2TAP、ptpd 之間的正確對接方式。
 *
 * 重點：
 * 1. PTP enable 使用 esp_eth_ptp_dm9058_enable_config_t。
 * 2. MAC 自動 PTP 處理使用 ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS 控制。
 * 3. one-step / two-step 由 CONFIG_ETH_DM9058_PTP_TWO_STEP_MODE 決定。
 * 4. RX timestamp 透過 stack_input_info() -> L2TAP info record 隨封包傳遞。
 * 5. 不使用「最後一筆 RX timestamp」輪詢模式。
 */

/* Reference snippets only; this file is not compiled as an ESP-IDF component. */
#if 0

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_eth.h"
#include "esp_eth_driver.h"
#include "esp_eth_mac_spi.h"
#include "esp_log.h"
#include "esp_vfs_l2tap.h"

#include "examples/ethernet/ptp/components/esp_eth_time/esp_eth_time.h"

static const char *TAG = "dm9058_ptp_ref";

#define ETH_TYPE_PTP 0x88F7
#define ETH_HEADER_LEN 14

/*
 * 範例 1：以 driver ioctl 直接啟用 DM9058 PTP。
 *
 * 如果是在 examples/ethernet/ptp 中，通常會使用 esp_eth_clock_init()
 * 包裝這段流程；這裡保留底層 ioctl 寫法，方便對照 driver 介面。
 */
esp_err_t dm9058_ptp_enable_direct(esp_eth_handle_t eth_handle,
                                   esp_eth_ptp_dm9058_transport_t transport)
{
    ESP_RETURN_ON_FALSE(eth_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "eth handle is null");

    esp_eth_ptp_dm9058_enable_config_t ptp_cfg = {
        .enable = true,
        .transport = transport,
    };

    return esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_ENABLE, &ptp_cfg);
}

/*
 * 範例 2：使用 esp_eth_time 包裝層啟用 CLOCK_PTP_SYSTEM。
 *
 * ptp_main.c 目前依 CONFIG_NETUTILS_PTPD_IEEE_802_1AS 選擇 transport：
 * - 啟用 802.1AS profile 時使用 ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_1AS
 * - 否則使用 ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3
 */
esp_err_t dm9058_ptp_clock_init_for_l2_example(esp_eth_handle_t eth_handle)
{
    ESP_RETURN_ON_FALSE(eth_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "eth handle is null");

    esp_eth_clock_cfg_t clock_cfg = {
        .eth_hndl = eth_handle,
#if CONFIG_NETUTILS_PTPD_IEEE_802_1AS
        .transport = ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_1AS,
#else
        .transport = ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3,
#endif
    };

    return esp_eth_clock_init(CLOCK_PTP_SYSTEM, &clock_cfg);
}

/*
 * 範例 3：啟用或停用 MAC 自動 PTP 處理。
 *
 * 目前 esp_eth_mac_new_dm9058() 會將 ptp_auto_process 預設為 true。
 * 若應用程式要測試手動路徑，可以用這個 ioctl 關閉。
 */
esp_err_t dm9058_ptp_set_auto_process(esp_eth_handle_t eth_handle, bool enable)
{
    ESP_RETURN_ON_FALSE(eth_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "eth handle is null");

    bool auto_process = enable;
    return esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_PTP_AUTO_PROCESS, &auto_process);
}

/*
 * 範例 4：PTP clock get / set / adjust。
 *
 * esp_eth_time.c 會把 CLOCK_PTP_SYSTEM 對應到 DM9058 PTP time ioctl。
 */
esp_err_t dm9058_ptp_set_time(uint32_t seconds, uint32_t nanoseconds)
{
    struct timespec ts = {
        .tv_sec = seconds,
        .tv_nsec = nanoseconds,
    };

    return (esp_eth_clock_settime(CLOCK_PTP_SYSTEM, &ts) == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t dm9058_ptp_get_time(struct timespec *out_time)
{
    ESP_RETURN_ON_FALSE(out_time != NULL, ESP_ERR_INVALID_ARG, TAG, "out_time is null");

    return (esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, out_time) == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t dm9058_ptp_adjust_frequency(double freq_scale)
{
    esp_eth_clock_adj_param_t adj = {
        .mode = ETH_CLK_ADJ_FREQ_SCALE,
        .freq_scale = freq_scale,
    };

    return (esp_eth_clock_adjtime(CLOCK_PTP_SYSTEM, &adj) == 0) ? ESP_OK : ESP_FAIL;
}

/*
 * 範例 5：直接使用 esp_eth_transmit_ctrl_vargs() 取得 TX timestamp。
 *
 * L2TAP write path 內部也是使用同一個 MAC transmit_ctrl_vargs() 能力。
 * ctrl 指向 eth_mac_time_t 時，DM9058 MAC 會在送出後嘗試寫回 TX timestamp。
 */
esp_err_t dm9058_ptp_transmit_with_tx_timestamp(esp_eth_handle_t eth_handle,
                                                uint8_t *frame,
                                                uint32_t frame_len,
                                                eth_mac_time_t *out_tx_time)
{
    ESP_RETURN_ON_FALSE(eth_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "eth handle is null");
    ESP_RETURN_ON_FALSE(frame != NULL && frame_len > 0, ESP_ERR_INVALID_ARG, TAG, "invalid frame");
    ESP_RETURN_ON_FALSE(out_tx_time != NULL, ESP_ERR_INVALID_ARG, TAG, "out_tx_time is null");

    memset(out_tx_time, 0, sizeof(*out_tx_time));
    return esp_eth_transmit_ctrl_vargs(eth_handle, out_tx_time, 2, frame, frame_len);
}

/*
 * 範例 6：若未走 transmit_ctrl path，也可以讀取 DM9058 最近一次 TX timestamp。
 *
 * 對 ptpd / L2TAP 而言，優先使用封包綁定的 info record；這個 ioctl 較適合
 * driver bring-up 或診斷用途。
 */
esp_err_t dm9058_ptp_get_last_tx_timestamp(esp_eth_handle_t eth_handle,
                                           eth_mac_time_t *out_tx_time)
{
    ESP_RETURN_ON_FALSE(eth_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "eth handle is null");
    ESP_RETURN_ON_FALSE(out_tx_time != NULL, ESP_ERR_INVALID_ARG, TAG, "out_tx_time is null");

    return esp_eth_ioctl(eth_handle, ETH_MAC_DM9058_CMD_G_PTP_TX_TIME, out_tx_time);
}

/*
 * L2TAP extended buffer helper。
 *
 * read/write 的 size 傳 0 時，esp_vfs_l2tap 會把參數視為 l2tap_extended_buff_t。
 * info record 使用 L2TAP_IREC_TIME_STAMP，資料格式是 struct timespec。
 */
typedef union {
    uint8_t bytes[L2TAP_IREC_SPACE(sizeof(struct timespec))];
    l2tap_irec_hdr_t align;
} dm9058_l2tap_timestamp_record_t;

static void dm9058_l2tap_prepare_timestamp_record(l2tap_extended_buff_t *ext_buff,
                                                  dm9058_l2tap_timestamp_record_t *record,
                                                  void *frame,
                                                  size_t frame_len)
{
    memset(record, 0, sizeof(*record));
    memset(ext_buff, 0, sizeof(*ext_buff));

    ext_buff->info_recs_len = sizeof(record->bytes);
    ext_buff->info_recs_buff = record->bytes;
    ext_buff->buff = frame;
    ext_buff->buff_len = frame_len;

    l2tap_irec_hdr_t *ts_info = L2TAP_IREC_FIRST(ext_buff);
    ts_info->len = L2TAP_IREC_LEN(sizeof(struct timespec));
    ts_info->type = L2TAP_IREC_TIME_STAMP;
}

static bool dm9058_l2tap_get_timestamp(l2tap_extended_buff_t *ext_buff,
                                       struct timespec *out_time)
{
    if (ext_buff == NULL || out_time == NULL) {
        return false;
    }

    l2tap_irec_hdr_t *ts_info = L2TAP_IREC_FIRST(ext_buff);
    if (ts_info == NULL || ts_info->type != L2TAP_IREC_TIME_STAMP) {
        return false;
    }

    struct timespec ts = *(struct timespec *)ts_info->data;
    if (ts.tv_sec < 0 || ts.tv_nsec < 0 || ts.tv_nsec >= 1000000000L) {
        return false;
    }

    *out_time = ts;
    return true;
}

/*
 * 範例 7：建立 L2TAP socket 並啟用 timestamp。
 *
 * interface_name 對 PTP example 通常是 "ETH_0"，需與 esp_netif if_key 一致。
 */
int dm9058_l2tap_open_ptp_socket(const char *interface_name, esp_eth_handle_t *out_eth_handle)
{
    if (interface_name == NULL || out_eth_handle == NULL) {
        errno = EINVAL;
        return -1;
    }

    int fd = open("/dev/net/tap", 0);
    if (fd < 0) {
        ESP_LOGE(TAG, "failed to open l2tap socket: errno=%d", errno);
        return -1;
    }

    if (ioctl(fd, L2TAP_S_INTF_DEVICE, interface_name) < 0) {
        ESP_LOGE(TAG, "failed to bind l2tap interface: errno=%d", errno);
        close(fd);
        return -1;
    }

    uint16_t eth_type_filter = ETH_TYPE_PTP;
    if (ioctl(fd, L2TAP_S_RCV_FILTER, &eth_type_filter) < 0) {
        ESP_LOGE(TAG, "failed to set PTP ethertype filter: errno=%d", errno);
        close(fd);
        return -1;
    }

    if (ioctl(fd, L2TAP_G_DEVICE_DRV_HNDL, out_eth_handle) < 0) {
        ESP_LOGE(TAG, "failed to get eth handle: errno=%d", errno);
        close(fd);
        return -1;
    }

    if (ioctl(fd, L2TAP_S_TIMESTAMP_EN) < 0) {
        ESP_LOGE(TAG, "failed to enable l2tap timestamp: errno=%d", errno);
        close(fd);
        return -1;
    }

    return fd;
}

/*
 * 範例 8：透過 L2TAP 送出 PTP frame 並取回 TX timestamp。
 */
esp_err_t dm9058_l2tap_send_ptp_frame(int fd,
                                      uint8_t *eth_frame,
                                      size_t eth_frame_len,
                                      struct timespec *out_tx_time)
{
    ESP_RETURN_ON_FALSE(fd >= 0, ESP_ERR_INVALID_ARG, TAG, "invalid fd");
    ESP_RETURN_ON_FALSE(eth_frame != NULL && eth_frame_len >= ETH_HEADER_LEN,
                        ESP_ERR_INVALID_ARG, TAG, "invalid ethernet frame");

    dm9058_l2tap_timestamp_record_t record;
    l2tap_extended_buff_t ext_buff;
    dm9058_l2tap_prepare_timestamp_record(&ext_buff, &record, eth_frame, eth_frame_len);

    int ret = write(fd, &ext_buff, 0);
    if (ret < 0) {
        ESP_LOGE(TAG, "l2tap write failed: errno=%d", errno);
        return ESP_FAIL;
    }

    if (out_tx_time != NULL && !dm9058_l2tap_get_timestamp(&ext_buff, out_tx_time)) {
        ESP_LOGW(TAG, "TX timestamp was not returned in L2TAP info record");
    }

    return ESP_OK;
}

/*
 * 範例 9：透過 L2TAP 接收 PTP frame 並取回 RX timestamp。
 *
 * DM9058 driver 會先在 MAC RX task 中將可用 RX timestamp 傳給 stack_input_info()。
 * L2TAP 再把這個 timestamp 填入 extended buffer 的 info record。
 */
esp_err_t dm9058_l2tap_receive_ptp_frame(int fd,
                                         uint8_t *eth_frame,
                                         size_t eth_frame_capacity,
                                         size_t *out_frame_len,
                                         struct timespec *out_rx_time)
{
    ESP_RETURN_ON_FALSE(fd >= 0, ESP_ERR_INVALID_ARG, TAG, "invalid fd");
    ESP_RETURN_ON_FALSE(eth_frame != NULL && eth_frame_capacity >= ETH_HEADER_LEN,
                        ESP_ERR_INVALID_ARG, TAG, "invalid rx buffer");
    ESP_RETURN_ON_FALSE(out_frame_len != NULL, ESP_ERR_INVALID_ARG, TAG, "out_frame_len is null");

    dm9058_l2tap_timestamp_record_t record;
    l2tap_extended_buff_t ext_buff;
    dm9058_l2tap_prepare_timestamp_record(&ext_buff, &record, eth_frame, eth_frame_capacity);

    int ret = read(fd, &ext_buff, 0);
    if (ret < 0) {
        ESP_LOGE(TAG, "l2tap read failed: errno=%d", errno);
        return ESP_FAIL;
    }

    *out_frame_len = ext_buff.buff_len;

    if (out_rx_time != NULL && !dm9058_l2tap_get_timestamp(&ext_buff, out_rx_time)) {
        ESP_LOGD(TAG, "RX timestamp was not attached to this frame");
    }

    return ESP_OK;
}

/*
 * 範例 10：完整初始化順序摘要。
 *
 * 實際 PTP example 的 main flow 約為：
 * 1. example_eth_init() 建立 Ethernet driver。
 * 2. esp_vfs_l2tap_intf_register() 註冊 L2TAP。
 * 3. 建立 esp_netif 並啟動 esp_eth_start()。
 * 4. Ethernet connected 後呼叫 esp_eth_clock_init() 啟用 DM9058 PTP。
 * 5. ptpd_start("ETH_0")，ptpd 內部開啟 L2TAP timestamp。
 */
esp_err_t dm9058_ptp_reference_start_sequence(esp_eth_handle_t eth_handle)
{
    ESP_RETURN_ON_FALSE(eth_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "eth handle is null");

    ESP_RETURN_ON_ERROR(dm9058_ptp_clock_init_for_l2_example(eth_handle),
                        TAG, "failed to init PTP clock");

    ESP_RETURN_ON_ERROR(dm9058_ptp_set_auto_process(eth_handle, true),
                        TAG, "failed to enable auto PTP processing");

    struct timespec now = {0};
    ESP_RETURN_ON_ERROR(dm9058_ptp_get_time(&now), TAG, "failed to get PTP time");

    ESP_LOGI(TAG, "DM9058 PTP clock ready: %lld.%09ld",
             (long long)now.tv_sec, now.tv_nsec);
    return ESP_OK;
}

/*
 * 實作提醒：
 *
 * - RX timestamp 應走 L2TAP info record 或 stack_input_info() metadata。
 * - 若上層一直拿不到 RX timestamp，先確認封包是否為 SYNC、DELAY_REQ、
 *   PDELAY_REQ、PDELAY_RESP 這類 event message。
 * - 若使用 802.1AS profile，transportSpecific / majorSdoId 必須與 daemon
 *   profile filter 一致。
 * - two-step 設定要同時檢查 DM9058 Kconfig 與 ptpd Kconfig。
 * - Target Time / callback 目前在 DM9058 driver case 中仍未正式支援。
 */
#endif
