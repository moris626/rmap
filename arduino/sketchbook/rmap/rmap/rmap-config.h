/**********************************************************************
Copyright (C) 2017  Marco Baldinetti <m.baldinetti@digiteco.it>
authors:
Marco Baldinetti <m.baldinetti@digiteco.it>

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
**********************************************************************/

#ifndef _RMAP_CONFIG_H
#define _RMAP_CONFIG_H

#include <stima_module.h>
#include <sensors_config.h>

/**********************************************************************
* MODULE
*********************************************************************/
/*!
\def MODULE_VERSION
Module version.
*/
#define MODULE_VERSION                                (3)

/*!
\def MODULE_TYPE
Type of module. It is defined in stima_module.h
*/
#define MODULE_TYPE                                   (STIMA_MODULE_TYPE_REPORT_GSM)

/*!
\def TASKS_COUNT
Max number of tasks.
*/
#define TASKS_COUNT                                   (10)

/*!
\def USE_POWER_DOWN
Enable or disable power down.
*/
#define USE_POWER_DOWN                                (false)

/*!
\def DATA_PROCESSING_RETRY_COUNT_MAX
Retry count for data savings
*/
#define DATA_PROCESSING_RETRY_COUNT_MAX               (2)

/*!
\def DATA_PROCESSING_RETRY_DELAY_MS
Delay for retry in data savings
*/
#define DATA_PROCESSING_RETRY_DELAY_MS                (500)

#define DATA_SAVING_RETRY_COUNT_MAX                   (3)
#define DATA_SAVING_DELAY_MS                          (1000)

#define MQTT_RETRY_COUNT_MAX                          (3)
#define MQTT_DELAY_MS                                 (1000)

#define GSM_CONNECT_RETRY_COUNT_MAX                   (10)
#define GSM_CONNECT_DELAY_MS                          (5000)

#define ETHERNET_CONNECT_RETRY_COUNT_MAX              (3)
#define ETHERNET_CONNECT_DELAY_MS                     (1000)

#define SUPERVISOR_CONNECTION_TIMEOUT_MS              (50000)

/*!
\def SDCARD_CHIP_SELECT_PIN
Chip select for SD card module
*/
#define SDCARD_CHIP_SELECT_PIN                        (7)

/*!
\def SDCARD_RETRY_COUNT_MAX
Retry count for management sd card
*/
#define SDCARD_RETRY_COUNT_MAX                        (5)

/*!
\def SDCARD_RETRY_DELAY_MS
Delay for retry in sd card management
*/
#define SDCARD_RETRY_DELAY_MS                         (100)

/*!
\def SDCARD_MQTT_PTR_FILE_NAME
Pointer for sensor data
*/
#define SDCARD_MQTT_PTR_FILE_NAME                     ("mqtt_ptr.txt")

/*!
\def NTP_RETRY_COUNT_MAX
Retry count for get time from NTP server
*/
#define NTP_RETRY_COUNT_MAX                           (5)

/*!
\def NTP_RETRY_DELAY_MS
Delay for retry for get time from NTP server
*/
#define NTP_RETRY_DELAY_MS                            (100)

/*!
\def NTP_TIME_FOR_RESYNC_S
Delay for retry for get time from NTP server
*/
#define NTP_TIME_FOR_RESYNC_S                         (SECS_PER_DAY)

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)

/*!
\def ETHERNET_RETRY_COUNT_MAX
Retry count for management ethernet
*/
#define ETHERNET_RETRY_COUNT_MAX                      (5)

/*!
\def ETHERNET_RETRY_DELAY_MS
Delay for retry in ethernet management
*/
#define ETHERNET_RETRY_DELAY_MS                       (ETHERNET_ATTEMPT_MS)

/*!
\def W5500_CHIP_SELECT_PIN
Chip select for Ethernet module W5500
*/
#define W5500_CHIP_SELECT_PIN                         (8)

#define W5500_RESET_PIN                               (4)

#elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)

/*!
\def GSM_RETRY_COUNT_MAX
Retry count for management GSM
*/
#define GSM_RETRY_COUNT_MAX                           (1)

/*!
\def GSM_RETRY_DELAY_MS
Delay for retry in GSM management
*/
#define GSM_RETRY_DELAY_MS                            (GSM_ATTEMPT_MS)

/*!
\def GSM_MQTT_TIMEOUT_MS
Timeout for mqtt connection on gsm.
*/
#define GSM_MQTT_TIMEOUT_MS                           (GSM_TIMEOUT_MS)

/*!
\def GSM_ON_OFF_PIN
Chip select for Ethernet module W5500
*/
#define GSM_ON_OFF_PIN                                (SIM800_ON_OFF_PIN)

#if (USE_SIM_800L)
#define GSM_RESET_PIN                                 (SIM800_RESET_PIN)
#endif

#define GSM_APN                                       (GSM_DEFAULT_APN)
#define GSM_USERNAME                                  (GSM_DEFAULT_USERNAME)
#define GSM_PASSWORD                                  (GSM_DEFAULT_PASSWORD)

#endif


/**********************************************************************
* CONFIGURATION
*********************************************************************/
/*!
\def CONFIGURATION_DEFAULT_TH_ADDRESS
Default i2c address.
*/
#define CONFIGURATION_DEFAULT_TH_ADDRESS              (I2C_TH_DEFAULT_ADDRESS)

/*!
\def CONFIGURATION_DEFAULT_RAIN_ADDRESS
Default i2c address.
*/
#define CONFIGURATION_DEFAULT_RAIN_ADDRESS            (I2C_RAIN_DEFAULT_ADDRESS)

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
/*!
\def CONFIGURATION_DEFAULT_ETHERNET_DHCP_ENABLE
Default DHCP status.
*/
#define CONFIGURATION_DEFAULT_ETHERNET_DHCP_ENABLE    (ETHERNET_DEFAULT_DHCP_ENABLE)

/*!
\def CONFIGURATION_DEFAULT_ETHERNET_MAC
Default mac address.
*/
#define CONFIGURATION_DEFAULT_ETHERNET_MAC            (ETHERNET_DEFAULT_MAC)

/*!
\def CONFIGURATION_DEFAULT_ETHERNET_IP
Default ip address.
*/
#define CONFIGURATION_DEFAULT_ETHERNET_IP             (ETHERNET_DEFAULT_IP)

/*!
\def CONFIGURATION_DEFAULT_ETHERNET_NETMASK
Default netmask address.
*/
#define CONFIGURATION_DEFAULT_ETHERNET_NETMASK        (ETHERNET_DEFAULT_NETMASK)

/*!
\def CONFIGURATION_DEFAULT_ETHERNET_GATEWAY
Default gateway address.
*/
#define CONFIGURATION_DEFAULT_ETHERNET_GATEWAY        (ETHERNET_DEFAULT_GATEWAY)

/*!
\def CONFIGURATION_DEFAULT_ETHERNET_PRIMARY_DNS
Default primary dns address.
*/
#define CONFIGURATION_DEFAULT_ETHERNET_PRIMARY_DNS    (ETHERNET_DEFAULT_PRIMARY_DNS)

#elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
/*!
\def CONFIGURATION_DEFAULT_GSM_APN
Default gsm apn.
*/
#define CONFIGURATION_DEFAULT_GSM_APN                 (GSM_APN_TIM)

/*!
\def CONFIGURATION_DEFAULT_GSM_USERNAME
Default gsm apn.
*/
#define CONFIGURATION_DEFAULT_GSM_USERNAME            (GSM_USERNAME)

/*!
\def CONFIGURATION_DEFAULT_GSM_PASSWORD
Default gsm apn.
*/
#define CONFIGURATION_DEFAULT_GSM_PASSWORD            (GSM_PASSWORD)

#endif

/*!
\def CONFIGURATION_DEFAULT_NTP_SERVER
Default ntp server.
*/
#define CONFIGURATION_DEFAULT_NTP_SERVER              (NTP_DEFAULT_SERVER)

/*!
\def CONFIGURATION_DEFAULT_MQTT_PORT
Default mqtt server port.
*/
#define CONFIGURATION_DEFAULT_MQTT_PORT               (MQTT_DEFAULT_PORT)

/*!
\def CONFIGURATION_DEFAULT_MQTT_SERVER
Default mqtt server.
*/
#define CONFIGURATION_DEFAULT_MQTT_SERVER             (MQTT_DEFAULT_SERVER)

/*!
\def CONFIGURATION_DEFAULT_MQTT_ROOT_TOPIC
Default mqtt root topic.
*/
#define CONFIGURATION_DEFAULT_MQTT_ROOT_TOPIC         (MQTT_DEFAULT_ROOT_TOPIC)

/*!
\def CONFIGURATION_DEFAULT_MQTT_SUBSCRIBE_TOPIC
Default mqtt subscribe topic.
*/
#define CONFIGURATION_DEFAULT_MQTT_SUBSCRIBE_TOPIC    (MQTT_DEFAULT_SUBSCRIBE_TOPIC)

/*!
\def CONFIGURATION_DEFAULT_MQTT_USERNAME
Default mqtt username.
*/
#define CONFIGURATION_DEFAULT_MQTT_USERNAME           (MQTT_DEFAULT_USERNAME)

/*!
\def CONFIGURATION_DEFAULT_MQTT_PASSWORD
Default mqtt password.
*/
#define CONFIGURATION_DEFAULT_MQTT_PASSWORD           (MQTT_DEFAULT_PASSWORD)

/*!
\def RESET_CONFIGURATION_PIN
Input pin for reset configuration at startup.
*/
#define CONFIGURATION_RESET_PIN                       (8)

/**********************************************************************
* WATCHDOG
*********************************************************************/
/*!
\def WDT_TIMER
Timer for generating watchdog interrupt for periodically check to avoid malfunctions

Possible value for WDT_TIMER are: WDTO_15MS, WDTO_30MS, WDTO_60MS, WDTO_120MS, WDTO_250MS, WDTO_500MS, WDTO_1S, WDTO_2S, WDTO_4S, WDTO_8S
*/
#define WDT_TIMER                                     (WDTO_8S)

/**********************************************************************
* RTC
*********************************************************************/
/*!
\def RTC_TIMER
Timer for generating real time clock interrupt for generating
the tick rate for awaken the microprocessor and execute timed tasks

Possible value for RTC_TIMER are any value between 1 and 255,
interpreted as seconds or minutes according to the definition of
RTC_FREQUENCY.

High values correspond to high energy savings.
*/
#define RTC_TIMER                                     (1)

/*!
\def RTC_FREQUENCY
Interprets the value of RTC_TIMER as seconds or minutes.
Possible value for RTC_FREQUENCY are:
PCF8563_TIMER_FREQUENCY_SECONDS and PCF8563_TIMER_FREQUENCY_MINUTES
*/
#define RTC_FREQUENCY                                 (PCF8563_CLKOUT_FREQUENCY_SECONDS)

/*!
\def RTC_INTERRUPT_PIN
Interrupt pin for rtc awake.
*/
#define RTC_INTERRUPT_PIN                             (6)

/*!
\def USE_RTC_TASK
Enable or disable timed tasks by RTC.
*/
#define USE_RTC_TASK                                  (true)

/**********************************************************************
* SENSORS
*********************************************************************/
#define USE_SENSORS_COUNT                             (USE_SENSOR_ITH + USE_SENSOR_MTH + USE_SENSOR_NTH + USE_SENSOR_XTH + USE_SENSOR_TBS + USE_SENSOR_TBR)

#if (USE_SENSORS_COUNT == 0)
#error No sensor used. Are you sure? If not, enable it in RmapConfig/sensors_config.h
#else
#endif

#endif
