idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    return()
endif()

set(srcs)
set(include)
set(priv_include)

# CONFIG_ETH_SPI_ETHERNET_DM9058 is not evaluated during CMAKE_BUILD_EARLY_EXPANSION,
# so always declare REQUIRES; only gate sources and include dirs.
if(CONFIG_ETH_SPI_ETHERNET_DM9058)
    set(srcs "src/esp_eth_mac_dm9058.c"
             "src/esp_eth_phy_dm9058.c"
             "src/esp_eth_ptp_dm9058.c")
    set(include "include")
    set(priv_include "src")
endif()

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS ${include}
                       PRIV_INCLUDE_DIRS ${priv_include}
                       REQUIRES esp_eth
                       PRIV_REQUIRES esp_eth log esp_driver_spi esp_driver_gpio esp_timer)
