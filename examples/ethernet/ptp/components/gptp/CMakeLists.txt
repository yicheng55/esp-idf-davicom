idf_component_register(
    SRCS
        "gptp_main.c"
        "gptp_arith.c"
        "gptp_bmc.c"
        "gptp_net.c"
        "gptp_protocol.c"
        "gptp_servo.c"
        "gptp_timer.c"
    INCLUDE_DIRS
        "."
        "include"
    PRIV_REQUIRES
        esp_eth
        esp_netif
        esp_eth_time
        freertos
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-function
    -Wno-unused-variable
)
