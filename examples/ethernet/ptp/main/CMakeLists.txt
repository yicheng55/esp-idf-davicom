idf_component_register(SRCS "ptp_main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES ptpd gptp esp_eth_time esp_event esp_eth esp_netif driver)
