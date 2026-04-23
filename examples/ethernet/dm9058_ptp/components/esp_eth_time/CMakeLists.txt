idf_component_register(SRCS "esp_eth_time.c"
                       REQUIRES esp_eth_dm9058
                       PRIV_REQUIRES esp_eth
                       INCLUDE_DIRS ".")
