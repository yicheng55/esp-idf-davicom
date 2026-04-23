idf_component_register(SRCS "ptpd.c"
                       PRIV_REQUIRES esp_eth esp_netif esp_eth_time esp_eth_dm9058
                       INCLUDE_DIRS "." "./include")

target_sources(${COMPONENT_LIB} PRIVATE "ptpd.c")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
