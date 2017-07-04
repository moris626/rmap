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

#include <sensors_config.h>

/**********************************************************************
 * MODULE
 *********************************************************************/
/*!
  \def MODULE_VERSION
  Module version.
*/
#define MODULE_VERSION                              (3)

/*!
  \def MODULE_TYPE
  Type of module. It is defined in registers.h
*/
#define MODULE_TYPE                                 (STIMA_MODULE_TYPE_REPORT_ETH)  // STIMA_MODULE_TYPE_REPORT_GSM

/*!
  \def USE_POWER_DOWN
  Enable or disable power down.
*/
#define USE_POWER_DOWN                              (false)

/*!
  \def SDCARD_RETRY_MAX_COUNT
  Retry count for sd card
*/
#define SDCARD_RETRY_MAX_COUNT                      (5)

/*!
  \def SDCARD_FILES_COUNT
  Number of files
*/
#define SDCARD_FILES_COUNT                          (2)

/*!
  \def SDCARD_FILES_NAME_MAX_LENGTH
  Max length of filename
*/
#define SDCARD_FILES_NAME_MAX_LENGTH                (20)

/*!
  \def SDCARD_RETRY_DELAY_MS
  Delay for retry in sd card management
*/
#define SDCARD_RETRY_DELAY_MS                       (3000)

/*!
  \def SDCARD_PTR_DATA_FILE_NAME
  Pointer for sensor data
*/
#define SDCARD_PTR_DATA_FILE_NAME                   ("ptr_data.txt")

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
/*!
  \def W5500_CHIP_SELECT_PIN
  Chip select for Ethernet module W5500
*/
#define W5500_CHIP_SELECT_PIN                       (8)

#define W5500_RESET_PIN                             (4)

/*!
  \def NTP_RETRY_MAX_COUNT
  Retry count for get time from NTP server
*/
#define NTP_RETRY_MAX_COUNT                         (5)

/*!
  \def NTP_RETRY_DELAY_MS
  Delay for retry for get time from NTP server
*/
#define NTP_RETRY_DELAY_MS                          (100)
#endif

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
#endif

/*!
  \def SDCARD_CHIP_SELECT_PIN
  Chip select for SD card module
*/
#define SDCARD_CHIP_SELECT_PIN                      (7)

/**********************************************************************
 * CONFIGURATION
 *********************************************************************/
 /*!
   \def CONFIGURATION_DEFAULT_TH_ADDRESS
   Default i2c address.
 */
 #define CONFIGURATION_DEFAULT_TH_ADDRESS           (I2C_TH_DEFAULT_ADDRESS)

 /*!
   \def CONFIGURATION_DEFAULT_RAIN_ADDRESS
   Default i2c address.
 */
 #define CONFIGURATION_DEFAULT_RAIN_ADDRESS         (I2C_RAIN_DEFAULT_ADDRESS)

 #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
 /*!
 \def CONFIGURATION_DEFAULT_ETHERNET_DHCP_ENABLE
 Default DHCP status.
 */
 #define CONFIGURATION_DEFAULT_ETHERNET_DHCP_ENABLE (ETHERNET_DEFAULT_DHCP_ENABLE)

 /*!
 \def CONFIGURATION_DEFAULT_ETHERNET_MAC
 Default mac address.
 */
 #define CONFIGURATION_DEFAULT_ETHERNET_MAC         (ETHERNET_DEFAULT_MAC)

 /*!
 \def CONFIGURATION_DEFAULT_ETHERNET_IP
 Default ip address.
 */
 #define CONFIGURATION_DEFAULT_ETHERNET_IP          (ETHERNET_DEFAULT_IP)

 /*!
 \def CONFIGURATION_DEFAULT_ETHERNET_NETMASK
 Default netmask address.
 */
 #define CONFIGURATION_DEFAULT_ETHERNET_NETMASK     (ETHERNET_DEFAULT_NETMASK)

 /*!
 \def CONFIGURATION_DEFAULT_ETHERNET_GATEWAY
 Default gateway address.
 */
 #define CONFIGURATION_DEFAULT_ETHERNET_GATEWAY     (ETHERNET_DEFAULT_GATEWAY)

 /*!
 \def CONFIGURATION_DEFAULT_ETHERNET_PRIMARY_DNS
 Default primary dns address.
 */
 #define CONFIGURATION_DEFAULT_ETHERNET_PRIMARY_DNS (ETHERNET_DEFAULT_PRIMARY_DNS)

 /*!
 \def CONFIGURATION_DEFAULT_NTP_SERVER
   Default ntp server.
 */
 #define CONFIGURATION_DEFAULT_NTP_SERVER           (NTP_DEFAULT_SERVER)

 #endif

 /*!
   \def CONFIGURATION_DEFAULT_MQTT_SERVER
   Default mqtt server.
 */
 #define CONFIGURATION_DEFAULT_MQTT_SERVER          (MQTT_DEFAULT_SERVER)

 /*!
   \def CONFIGURATION_DEFAULT_MQTT_ROOT_PATH
   Default mqtt root path.
 */
 #define CONFIGURATION_DEFAULT_MQTT_ROOT_PATH       (MQTT_DEFAULT_ROOT_PATH)

 /*!
   \def CONFIGURATION_DEFAULT_MQTT_USERNAME
   Default mqtt username.
 */
 #define CONFIGURATION_DEFAULT_MQTT_USERNAME        (MQTT_DEFAULT_USERNAME)

 /*!
 \def CONFIGURATION_DEFAULT_MQTT_PASSWORD
 Default mqtt password.
 */
 #define CONFIGURATION_DEFAULT_MQTT_PASSWORD        (MQTT_DEFAULT_PASSWORD)

/*!
  \def RESET_CONFIGURATION_PIN
  Input pin for reset configuration at startup.
*/
#define CONFIGURATION_RESET_PIN                     (8)

/**********************************************************************
 * WATCHDOG
 *********************************************************************/
/*!
  \def WDT_TIMER
  Timer for generating watchdog interrupt for:
  1) generating the tick rate for awaken the microprocessor and execute timed tasks
  2) periodically check to avoid malfunctions

  note:
  1) while the interrupt is generated exactly at the expected time,
  the execution of the timed tasks differs from about 150 to 200 ms from the ideal WDT_TIMER,
  due to the time needed to awaken the microprocessor
  2) the tick rate for timed task must be a multiple of WDT_TIMER

  Possible value for WDT_TIMER are: WDTO_15MS, WDTO_30MS, WDTO_60MS, WDTO_120MS, WDTO_250MS, WDTO_500MS, WDTO_1S, WDTO_2S, WDTO_4S, WDTO_8S

  High values correspond to high energy savings.
*/
#define WDT_TIMER                                   (WDTO_8S)

/*!
  \def WDT_INTERRUPT_COUNT_DEFAULT
  Watchdog interrupt counter.
  After WDT_INTERRUPT_COUNT_DEFAULT * WDT_TIMER [ ms | s ] the cpu will reboot due to a malfunction.
*/
#define WDT_INTERRUPT_COUNT_DEFAULT                 (2)

/*!
  \def USE_WDT_TASK
  Enable or disable timed tasks by WDT.
*/
#define USE_WDT_TASK                                (false)

/*!
  \def USE_WDT_TO_WAKE_UP_FROM_SLEEP
  Enable or disable watchdog wake up from sleep
*/
#define USE_WDT_TO_WAKE_UP_FROM_SLEEP               (false)

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
#define RTC_TIMER                                   (1)

/*!
  \def RTC_FREQUENCY
  Interprets the value of RTC_TIMER as seconds or minutes.
  Possible value for RTC_FREQUENCY are:
  PCF8563_TIMER_FREQUENCY_SECONDS and PCF8563_TIMER_FREQUENCY_MINUTES
*/
#define RTC_FREQUENCY                               (PCF8563_TIMER_FREQUENCY_MINUTES)

/*!
  \def RTC_INTERRUPT_PIN
  Interrupt pin for rtc awake.
*/
#define RTC_INTERRUPT_PIN                           (3)

/*!
  \def USE_RTC_TASK
  Enable or disable timed tasks by RTC.
*/
#define USE_RTC_TASK                                (true)

/**********************************************************************
 * SENSORS
 *********************************************************************/
#define VALUES_TO_READ_FROM_SENSOR_COUNT            (2)
#define SENSORS_RETRY_COUNT_MAX                     (5)
#define SENSORS_RETRY_DELAY_MS                      (50)

#define USE_SENSORS_COUNT                           (USE_SENSOR_ITH + USE_SENSOR_MTH + USE_SENSOR_NTH + USE_SENSOR_XTH + USE_SENSOR_TBS + USE_SENSOR_TBR)

#if (USE_SENSORS_COUNT == 0)
#error No sensor used. Are you sure? If not, enable it in RmapConfig/sensors_config.h
#else
#endif

#endif
