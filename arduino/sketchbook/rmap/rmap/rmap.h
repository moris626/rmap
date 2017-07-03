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

#include "rmap-config.h"
#include <debug.h>
#include <hardware_config.h>
#include <mqtt_config.h>
#include <ntp_config.h>
#include <rmap_util.h>
#include <sleep_utility.h>
#include <eeprom_utility.h>
#include <Wire.h>
#include <pcf8563.h>
#include <Time.h>
#include <typedef.h>
#include <SensorDriver.h>
// #include <SdFat.h>

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
#include <ethernet_config.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <ntp.h>
#endif

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
#endif

/**********************************************************************
 * TYPEDEF
 *********************************************************************/
/*!
  \typedef
  \struct configuration_t
  EEPROM saved configuration this module.
*/
typedef struct {
  uint8_t module_type;      //!< module type saved in eeprom. If matching the MODULE_TYPE, the configuration is up to date
  uint8_t module_version;   //!< module version saved in eeprom. If matching the MODULE_VERSION, the configuration is up to date
  uint8_t i2c_rain_address;
  uint8_t i2c_th_address;

  #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
  bool is_dhcp_enable;
  uint8_t ethernet_mac[ETHERNET_MAC_LENGTH];
  uint8_t ip[ETHERNET_IP_LENGTH];
  uint8_t netmask[ETHERNET_IP_LENGTH];
  uint8_t gateway[ETHERNET_IP_LENGTH];
  uint8_t primary_dns[ETHERNET_IP_LENGTH];
  #endif

  char mqtt_server[MQTT_SERVER_LENGTH];
  char mqtt_root_path[MQTT_ROOT_PATH_LENGTH];
  char mqtt_username[MQTT_USERNAME_LENGTH];
  char mqtt_password[MQTT_PASSWORD_LENGTH];
  char ntp_server[NTP_SERVER_LENGTH];
} configuration_t;

/*!
  \typedef
  \struct readable_data_t
  Readable data through i2c bus.
*/
typedef struct {
  uint8_t module_type;      //!< module type defined in MODULE_TYPE
  uint8_t module_version;   //!< module version defined in MODULE_VERSION
} readable_data_t;

/*!
  \typedef
  \struct writable_data_t
  Writable data through i2c bus.
*/
typedef struct {
  uint8_t i2c_rain_address;
  uint8_t i2c_th_address;

  #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
  bool is_dhcp_enable;
  uint8_t ethernet_mac[ETHERNET_MAC_LENGTH];
  uint8_t ip[ETHERNET_IP_LENGTH];
  uint8_t netmask[ETHERNET_IP_LENGTH];
  uint8_t gateway[ETHERNET_IP_LENGTH];
  uint8_t primary_dns[ETHERNET_IP_LENGTH];
  #endif

  char mqtt_server[MQTT_SERVER_LENGTH];
  char mqtt_root_path[MQTT_ROOT_PATH_LENGTH];
  char mqtt_username[MQTT_USERNAME_LENGTH];
  char mqtt_password[MQTT_PASSWORD_LENGTH];
  char ntp_server[NTP_SERVER_LENGTH];
} writable_data_t;

/*!
  \typedef
  \struct wdt_timer_t
  Watchdog running parameter.
*/
typedef struct {
  uint16_t value;               //!< current value of watchdog timer
  uint8_t interrupt_count;      //!< watchdog elapsed timer counter
} wdt_timer_t;

/**********************************************************************
 * ENUMERATION
 *********************************************************************/
/*!
  \enum state
  Module state machine.
*/
enum {
  INIT,
  #if (USE_POWER_DOWN)
  ENTER_POWER_DOWN,
  #endif
  TASKS_EXECUTION,
  END
} state;

typedef enum {
  INIT_SENSOR,
  PREPARE_SENSOR,
  IS_SENSOR_PREPARED,
  GET_SENSOR,
  IS_SENSOR_GETTED,
  READ_SENSOR,
  END_SENSOR_READING,
  END_TASK,
  END_SENSOR,
  WAIT_STATE
} sensor_state_t;

typedef enum {
  INIT_TIME,
  SEND_REQUEST,
  WAIT_RESPONSE,
  SET_SYNC_PROVIDER,
  SET_RTC_TIME,
  END_TIME,
  WAIT_FOR_NTP_RETRY
} time_state_t;

/**********************************************************************
 * GLOBAL VARIABLE
 *********************************************************************/
/*!
  \var configuration
  Configuration for this module.
*/
configuration_t configuration;

/*!
  \var wdt_timer
  Watchdog timer variable.
*/
volatile wdt_timer_t wdt_timer;

/*!
  \var readable_data_1
  First readable i2c register.
*/
volatile readable_data_t readable_data_1;

/*!
  \var readable_data_2
  Second readable i2c register.
*/
volatile readable_data_t readable_data_2;

/*!
  \var readable_data_read_ptr
  Pointer for read data in i2c readable register.
*/
volatile readable_data_t *readable_data_read_ptr;

/*!
  \var readable_data_write_ptr
  Pointer for write data in i2c readable register.
*/
volatile readable_data_t *readable_data_write_ptr;

/*!
  \var readable_data_temp_ptr
  Temporary pointer for exchange read and write pointer for i2c readable register.
*/
volatile readable_data_t *readable_data_temp_ptr;

/*!
  \var writable_data
  Writable i2c register.
*/
writable_data_t writable_data;

/*!
  \var writable_data
  Pointer for read and write data in i2c writable register.
*/
writable_data_t *writable_data_ptr;

/*!
  \var ready_tasks_count
  Number of tasks ready to execute.
*/
volatile uint8_t ready_tasks_count;

/*!
  \var awakened_event_occurred_time_ms
  System time (in millisecond) when the system has awakened from power down.
*/
uint32_t awakened_event_occurred_time_ms;

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
uint32_t ethernet_task_attempt_occurred_time_ms;
EthernetUDP ethernetUdpClient;
#endif

SensorDriver *sensors[USE_SENSORS_COUNT];
uint8_t sensors_count;

uint8_t sensors_end_readings_count = 0;
uint32_t absolute_millis_for_sensors_read_ms;

bool is_first_run;

#if (USE_SENSOR_TBS || USE_SENSOR_TBR)
bool is_sensors_rain_prepared;
bool is_sensors_rain_setted;
#endif

#if (USE_SENSOR_STH || USE_SENSOR_ITH || USE_SENSOR_MTH || USE_SENSOR_NTH || USE_SENSOR_XTH)
bool is_sensors_th_prepared;
bool is_sensors_th_setted;
#endif

#if (USE_JSON)
char json_buffer[JSON_BUFFER_LENGTH];
#endif

value_t temperature;
value_t humidity;
rain_t rain;

uint8_t next_hour_for_sensor_reading;
uint8_t next_minute_for_sensor_reading;
uint8_t sensor_reading_day;
uint8_t sensor_reading_month;
uint16_t sensor_reading_year;
uint8_t sensor_reading_hour;
uint8_t sensor_reading_minute;

time_state_t time_state;
sensor_state_t sensor_state;

/**********************************************************************
 * FUNCTIONS
 *********************************************************************/
void init_systems(void);
void init_buffers(void);
void init_tasks(void);
void init_pins(void);
void init_wire(void);
void init_spi(void);
void init_rtc(void);
void init_sensors(void);

/*! \fn void print_configuration(void)
 *  \brief Print configuration.
 *  \return void.
 */
void print_configuration(void);

/*! \fn void load_configuration(void)
 *  \brief Load configuration from EEPROM.
 *  \return void.
 */
void load_configuration(void);

/*! \fn void save_configuration(bool is_default)
 *  \brief Save configuration to EEPROM.
 *  \param is_default: if true save default configuration; if false save current configuration.
 *  \return void.
 */
void save_configuration(bool);

void init_sensors(void);

void setNextTimeForSensorReading(uint8_t *, uint8_t *);

/**********************************************************************
 * TASKS
 *********************************************************************/

/*! \fn void sensors_reading_task(void)
 *  \brief Read sensors.
 *  \return void.
 */
void sensors_reading_task(void);

volatile bool is_event_sensors_reading;

#if (USE_WDT_TASK)
volatile bool is_event_wdt;

/*! \fn void wdt_task(void)
 *  \brief Temporized task.
 *  \return void.
 */
void wdt_task(void);
#endif

#if (USE_RTC_TASK)
volatile bool is_event_rtc;

/*! \fn void rtc_task(void)
 *  \brief Temporized task.
 *  \return void.
 */
void rtc_task(void);
#endif

/*! \fn void time_task(void)
 *  \brief manage ntp and rtc time.
 *  \return void.
 */
void time_task(void);

bool is_event_time;

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
/*! \fn void ethernet_task(void)
 *  \brief manage ethernet connection.
 *  \return void.
 */
void ethernet_task(void);

bool is_event_ethernet;
#endif

/**********************************************************************
 * INTERRUPT HANDLER
 *********************************************************************/

 /*! \fn void rtc_interrupt_handler(void)
  *  \brief Real time clock interrupt handler.
  *  \return void.
  */
#if (USE_RTC_TASK)
void rtc_interrupt_handler(void);
#endif
