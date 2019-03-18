/*
 * mbed SDK
 * Copyright (c) 2017 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Automatically generated configuration file.
// DO NOT EDIT, content will be overwritten.

#ifndef __MBED_CONFIG_DATA__
#define __MBED_CONFIG_DATA__

// Configuration parameters
#define MBED_CONF_ATMEL_RF_ASSUME_SPACED_SPI                 0                                                                                                // set by library:atmel-rf
#define MBED_CONF_ATMEL_RF_FULL_SPI_SPEED                    7500000                                                                                          // set by library:atmel-rf
#define MBED_CONF_ATMEL_RF_FULL_SPI_SPEED_BYTE_SPACING       250                                                                                              // set by library:atmel-rf
#define MBED_CONF_ATMEL_RF_IRQ_THREAD_STACK_SIZE             1024                                                                                             // set by library:atmel-rf
#define MBED_CONF_ATMEL_RF_LOW_SPI_SPEED                     3750000                                                                                          // set by library:atmel-rf
#define MBED_CONF_ATMEL_RF_PROVIDE_DEFAULT                   0                                                                                                // set by library:atmel-rf
#define MBED_CONF_ATMEL_RF_USE_SPI_SPACING_API               0                                                                                                // set by library:atmel-rf
#define MBED_CONF_CELLULAR_DEBUG_AT                          0                                                                                                // set by library:cellular
#define MBED_CONF_CELLULAR_RANDOM_MAX_START_DELAY            0                                                                                                // set by library:cellular
#define MBED_CONF_CELLULAR_USE_APN_LOOKUP                    1                                                                                                // set by library:cellular
#define MBED_CONF_DRIVERS_UART_SERIAL_RXBUF_SIZE             256                                                                                              // set by library:drivers
#define MBED_CONF_DRIVERS_UART_SERIAL_TXBUF_SIZE             256                                                                                              // set by library:drivers
#define MBED_CONF_ESP8266_DEBUG                              0                                                                                                // set by library:esp8266
#define MBED_CONF_ESP8266_PROVIDE_DEFAULT                    0                                                                                                // set by library:esp8266
#define MBED_CONF_ESP8266_SOCKET_BUFSIZE                     8192                                                                                             // set by library:esp8266
#define MBED_CONF_EVENTS_PRESENT                             1                                                                                                // set by library:events
#define MBED_CONF_EVENTS_SHARED_DISPATCH_FROM_APPLICATION    0                                                                                                // set by library:events
#define MBED_CONF_EVENTS_SHARED_EVENTSIZE                    256                                                                                              // set by library:events
#define MBED_CONF_EVENTS_SHARED_HIGHPRIO_EVENTSIZE           256                                                                                              // set by library:events
#define MBED_CONF_EVENTS_SHARED_HIGHPRIO_STACKSIZE           1024                                                                                             // set by library:events
#define MBED_CONF_EVENTS_SHARED_STACKSIZE                    2048                                                                                             // set by library:events
#define MBED_CONF_EVENTS_USE_LOWPOWER_TIMER_TICKER           0                                                                                                // set by library:events
#define MBED_CONF_LORA_ADR_ON                                1                                                                                                // set by library:lora
#define MBED_CONF_LORA_APPLICATION_EUI                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}                                                 // set by library:lora
#define MBED_CONF_LORA_APPLICATION_KEY                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} // set by library:lora
#define MBED_CONF_LORA_APPSKEY                               {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} // set by library:lora
#define MBED_CONF_LORA_APP_PORT                              15                                                                                               // set by library:lora
#define MBED_CONF_LORA_AUTOMATIC_UPLINK_MESSAGE              1                                                                                                // set by library:lora
#define MBED_CONF_LORA_DEVICE_ADDRESS                        0x00000000                                                                                       // set by library:lora
#define MBED_CONF_LORA_DEVICE_EUI                            {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}                                                 // set by library:lora
#define MBED_CONF_LORA_DOWNLINK_PREAMBLE_LENGTH              5                                                                                                // set by library:lora
#define MBED_CONF_LORA_DUTY_CYCLE_ON                         1                                                                                                // set by library:lora
#define MBED_CONF_LORA_DUTY_CYCLE_ON_JOIN                    1                                                                                                // set by library:lora
#define MBED_CONF_LORA_FSB_MASK                              {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x00FF}                                                         // set by library:lora
#define MBED_CONF_LORA_FSB_MASK_CHINA                        {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF}                                                 // set by library:lora
#define MBED_CONF_LORA_LBT_ON                                0                                                                                                // set by library:lora
#define MBED_CONF_LORA_MAX_SYS_RX_ERROR                      5                                                                                                // set by library:lora
#define MBED_CONF_LORA_NB_TRIALS                             12                                                                                               // set by library:lora
#define MBED_CONF_LORA_NWKSKEY                               {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} // set by library:lora
#define MBED_CONF_LORA_OVER_THE_AIR_ACTIVATION               1                                                                                                // set by library:lora
#define MBED_CONF_LORA_PHY                                   EU868                                                                                            // set by library:lora
#define MBED_CONF_LORA_PUBLIC_NETWORK                        1                                                                                                // set by library:lora
#define MBED_CONF_LORA_TX_MAX_SIZE                           64                                                                                               // set by library:lora
#define MBED_CONF_LORA_UPLINK_PREAMBLE_LENGTH                8                                                                                                // set by library:lora
#define MBED_CONF_LORA_WAKEUP_TIME                           5                                                                                                // set by library:lora
#define MBED_CONF_LWIP_ADDR_TIMEOUT                          5                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_ADDR_TIMEOUT_MODE                     1                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_DEBUG_ENABLED                         0                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_DEFAULT_THREAD_STACKSIZE              512                                                                                              // set by library:lwip
#define MBED_CONF_LWIP_ENABLE_PPP_TRACE                      0                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_ETHERNET_ENABLED                      1                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_IPV4_ENABLED                          1                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_IPV6_ENABLED                          0                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_IP_VER_PREF                           4                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_MEM_SIZE                              16362                                                                                            // set by library:lwip[LPC1768]
#define MBED_CONF_LWIP_PPP_THREAD_STACKSIZE                  768                                                                                              // set by library:lwip
#define MBED_CONF_LWIP_SOCKET_MAX                            4                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_TCPIP_THREAD_STACKSIZE                1200                                                                                             // set by library:lwip
#define MBED_CONF_LWIP_TCP_ENABLED                           1                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_TCP_MAXRTX                            6                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_TCP_SERVER_MAX                        4                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_TCP_SOCKET_MAX                        4                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_UDP_SOCKET_MAX                        4                                                                                                // set by library:lwip
#define MBED_CONF_LWIP_USE_MBED_TRACE                        0                                                                                                // set by library:lwip
#define MBED_CONF_MCR20A_PROVIDE_DEFAULT                     0                                                                                                // set by library:mcr20a
#define MBED_CONF_NSAPI_DEFAULT_MESH_TYPE                    THREAD                                                                                           // set by library:nsapi
#define MBED_CONF_NSAPI_DEFAULT_STACK                        LWIP                                                                                             // set by library:nsapi
#define MBED_CONF_NSAPI_DEFAULT_WIFI_SECURITY                NONE                                                                                             // set by library:nsapi
#define MBED_CONF_NSAPI_DNS_CACHE_SIZE                       3                                                                                                // set by library:nsapi
#define MBED_CONF_NSAPI_DNS_RESPONSE_WAIT_TIME               5000                                                                                             // set by library:nsapi
#define MBED_CONF_NSAPI_DNS_RETRIES                          0                                                                                                // set by library:nsapi
#define MBED_CONF_NSAPI_DNS_TOTAL_ATTEMPTS                   3                                                                                                // set by library:nsapi
#define MBED_CONF_NSAPI_PRESENT                              1                                                                                                // set by library:nsapi
#define MBED_CONF_NSAPI_SOCKET_STATS_ENABLE                  0                                                                                                // set by library:nsapi
#define MBED_CONF_NSAPI_SOCKET_STATS_MAX_COUNT               10                                                                                               // set by library:nsapi
#define MBED_CONF_PLATFORM_CRASH_CAPTURE_ENABLED             0                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_CTHUNK_COUNT_MAX                  8                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_DEFAULT_SERIAL_BAUD_RATE          9600                                                                                             // set by library:platform
#define MBED_CONF_PLATFORM_ERROR_ALL_THREADS_INFO            0                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_ERROR_DECODE_HTTP_URL_STR         "\nFor more info, visit: https://armmbed.github.io/mbedos-error/?error=0x%08X"                   // set by library:platform
#define MBED_CONF_PLATFORM_ERROR_FILENAME_CAPTURE_ENABLED    0                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_ERROR_HIST_ENABLED                0                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_ERROR_HIST_SIZE                   4                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_ERROR_REBOOT_MAX                  1                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_FATAL_ERROR_AUTO_REBOOT_ENABLED   0                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_FORCE_NON_COPYABLE_ERROR          0                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_MAX_ERROR_FILENAME_LEN            16                                                                                               // set by library:platform
#define MBED_CONF_PLATFORM_POLL_USE_LOWPOWER_TIMER           0                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_STDIO_BAUD_RATE                   9600                                                                                             // set by library:platform
#define MBED_CONF_PLATFORM_STDIO_BUFFERED_SERIAL             0                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_STDIO_CONVERT_NEWLINES            0                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_STDIO_CONVERT_TTY_NEWLINES        0                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_STDIO_FLUSH_AT_EXIT               1                                                                                                // set by library:platform
#define MBED_CONF_PLATFORM_USE_MPU                           1                                                                                                // set by library:platform
#define MBED_CONF_RTOS_IDLE_THREAD_STACK_SIZE                512                                                                                              // set by library:rtos
#define MBED_CONF_RTOS_IDLE_THREAD_STACK_SIZE_TICKLESS_EXTRA 256                                                                                              // set by library:rtos
#define MBED_CONF_RTOS_MAIN_THREAD_STACK_SIZE                4096                                                                                             // set by library:rtos
#define MBED_CONF_RTOS_PRESENT                               1                                                                                                // set by library:rtos
#define MBED_CONF_RTOS_THREAD_STACK_SIZE                     4096                                                                                             // set by library:rtos
#define MBED_CONF_RTOS_TIMER_THREAD_STACK_SIZE               768                                                                                              // set by library:rtos
#define MBED_CONF_S2LP_PROVIDE_DEFAULT                       0                                                                                                // set by library:s2lp
#define MBED_CONF_TARGET_BOOT_STACK_SIZE                     0x400                                                                                            // set by library:rtos[*]
#define MBED_CONF_TARGET_DEEP_SLEEP_LATENCY                  0                                                                                                // set by target:Target
#define MBED_CONF_TARGET_MPU_ROM_END                         0x0fffffff                                                                                       // set by target:Target
#define MBED_CONF_TARGET_NETWORK_DEFAULT_INTERFACE_TYPE      ETHERNET                                                                                         // set by target:LPC1768
#define MBED_CONF_TARGET_US_TICKER_TIMER                     3                                                                                                // set by target:LPC1768
#define MEM_ALLOC                                            malloc                                                                                           // set by library:mbed-trace
#define MEM_FREE                                             free                                                                                             // set by library:mbed-trace
#define NSAPI_PPP_AVAILABLE                                  0                                                                                                // set by library:lwip
#define NSAPI_PPP_IPV4_AVAILABLE                             1                                                                                                // set by library:lwip
#define NSAPI_PPP_IPV6_AVAILABLE                             0                                                                                                // set by library:lwip
// Macros
#define UNITY_INCLUDE_CONFIG_H                                                                                                                                // defined by library:utest
#define _RTE_                                                                                                                                                 // defined by library:rtos

#endif
