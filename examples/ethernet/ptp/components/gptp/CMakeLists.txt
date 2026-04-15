idf_component_register(SRCS "gptp.c"
                       PRIV_REQUIRES esp_eth esp_netif esp_timer esp_eth_time
                       INCLUDE_DIRS "." "./include")

target_sources(${COMPONENT_LIB} PRIVATE "gptp.c")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
