idf_component_register(SRCS "ethernet_init.c"
                       PRIV_REQUIRES esp_driver_gpio esp_eth esp_eth_dm9058
                       INCLUDE_DIRS ".")
