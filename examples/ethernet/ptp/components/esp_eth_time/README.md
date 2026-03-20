# ESP Ethernet Time Control Component Example

This example component provides a wrapper around management of the internal Ethernet MAC Time (Time Stamping) system which is normally accessed via `esp_eth_ioctl` commands. The component is offering a more intuitive API mimicking POSIX `clock_settime`, `clock_gettime` group of time functions and so making it easier to integrate with existing systems.

Note: RX timestamps can be delivered either by pull ioctl or by push metadata depending on the MAC driver implementation. In DM9058 pure-push mode, RX timestamps are attached to received frames via L2TAP metadata and `ETH_MAC_*_CMD_G_PTP_RX_TIME` is not supported.
