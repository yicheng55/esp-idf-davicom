idf_component_register(SRCS "ptpd.c"
                       REQUIRES esp_eth
                       PRIV_REQUIRES esp_netif
                       INCLUDE_DIRS "." "./include")

target_sources(${COMPONENT_LIB} PRIVATE "ptpd.c")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
# ESP_PTP guards ESP32-specific code in ptpd.c and ptpd.h
target_compile_definitions(${COMPONENT_LIB} PUBLIC ESP_PTP=1)
