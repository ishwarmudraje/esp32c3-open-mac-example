# Introduction
A simple open wifi example for open source Wi-Fi Tx and Rx on the ESP32-C3. For the main project, please refer to [esp32-open-mac](https://esp32-open-mac.be/) which reverse-engineered the Wi-Fi driver on the Tensilica-based ESP32. 

# Background
Highly recommend Jasper's blog post [esp32-open-mac](https://esp32-open-mac.be/) regarding the reverse engineering process. For the C3, we do not have reverse engineer from scratch as the RF chip on both the variants are similar. The Wi-Fi driver for the C3 only requires a different mapping of peripheral registers. Therefore, ESP32-Open-Mac can be ported to the C3. However, I have not been able to do it. This repo serves as an example for basic Tx and Rx, which were interested in for another project. 

# Requirements
The example has been tested with ESP-IDF v5.2.2 on an ESP32-C3. No IPv4 stack is used. In [main.c](main/main.c), a raw 802.3 UDP packet is crafted. Replace the MAC addresses with the UDP listener and your ESP's mac addresses. Also enter the IP address assigned to the listener. Make sure the ESP's arbitrary IP address is also in the router's IP range. Currently, only Wi-Fi networks with no security are supported. 

# Examples
## WiFi Driver
[wifi_driver.h](main/wifi_driver.h) contains the register mappings used by the C3. [wifi_driver.c](main/wifi_driver.c) implements the transmission and receive tasks as well as defines the hardware Wi-Fi task  that runs continuously monitoring a FreeRTOS queue (say "hardware queue") for packets to transmit as well as queueing packets from the peripheral onto another queue (say "MAC queue"). This queue is in-turn monitored by a "MAC-layer" task that then is free to pass it on to its IP stack of choice. Similarly, the MAC task also queues packets for transmission from the IP stack on the aforementioned hardware queue.

## WiFi MAC
[wifi_mac.c](main/wifi_mac.c) defines the MAC layer task. 

## Net Setup
[net_setup.c](main/net_setup.c) connects to the router using the proprietary driver and then kills it after association is complete. Then, the custom interrupt for the Wi-Fi peripheral is registered with the interrupt system of the ESP. The method to set up interrupts on the RISC-V core is different from Xtensa.

## Register addresses
|                |               |
| -------------- | ------------- |
| WIFI_MAC_BASE | 0x60033000 |
| WIFI_BASE_RX_DSCR | 0x60033088 |
| WIFI_NEXT_RX_DSCR | 0x6003308c |
| WIFI_LAST_RX_DSCR | 0x60033090 |
| WIFI_LAST_RXBUF_CONFIG | 0x6003309c |
| WIFI_MAC_BITMASK_084 | 0x60033084 |
| WIFI_TX_CONFIG | 0x60033d04 |
| WIFI_MAC_CTRL | 0x60033ca0 |
| WIFI_TX_PLCP0 | 0x60033d08 |
| WIFI_TX_PLCP1 | 0x600342f8 |
| WIFI_TX_PLCP1_2 | 0x600342fc |
| WIFI_TX_HTSIG | 0x60034310 |
| WIFI_TX_PLCP2 | 0x60034314 |
| WIFI_TX_DURATION | 0x60034318 |
| WIFI_TX_STATUS | 0x60033cb0 |
| WIFI_TX_CLR | 0x60033cac |
| WIFI_TX_GET_ERR | 0x60033ca8 |
| WIFI_TX_CLR_ERR | 0x60033ca4 |
| WIFI_INT_STATUS_GET | 0x60033c3c |
| WIFI_INT_STATUS_CLR | 0x60033c40 |
| PWR_INT_STATUS_GET | 0x60035118 |
| PWR_INT_STATUS_CLR | 0x6003511c |
| WIFI_MAC_CCA_REG | 0x60033c50 |
| WIFI_OFFSET_REG | 0x60033c64 |