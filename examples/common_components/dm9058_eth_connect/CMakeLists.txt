idf_component_register(SRCS "eth_connect_dm9058.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_netif esp_eth esp_event
                                     esp_driver_gpio esp_driver_spi
                                     esp_eth_dm9058)

if(CONFIG_EXAMPLE_DM9058_ENABLE_PTP)
    idf_component_optional_requires(PRIVATE esp_eth_time)
endif()
