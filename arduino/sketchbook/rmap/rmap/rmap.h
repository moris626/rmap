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
#include <typedef.h>
#include <debug.h>
#include <hardware_config.h>
#include <json_config.h>
#include <ntp_config.h>
#include <SPI.h>
#include <Wire.h>
#include <SdFat.h>
#include <pcf8563.h>
#include <ntp.h>
#include <Time.h>
#include <SensorDriver.h>
#include <sleep_utility.h>
#include <eeprom_utility.h>
#include <rmap_utility.h>
#include <i2c_utility.h>
#include <sdcard_utility.h>

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
#include <ethernet_config.h>
#include <Ethernet2.h>

#elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
#include <gsm_config.h>
#include <sim800Client.h>
#include <Sim800IPStack.h>

#endif

#include <IPStack.h>
#include <Countdown.h>
#include <MQTTClient.h>

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

   uint16_t mqtt_port;
   char mqtt_server[MQTT_SERVER_LENGTH];
   char mqtt_root_topic[MQTT_ROOT_TOPIC_LENGTH];
   char mqtt_subscribe_topic[MQTT_SUBSCRIBE_TOPIC_LENGTH];
   char mqtt_username[MQTT_USERNAME_LENGTH];
   char mqtt_password[MQTT_PASSWORD_LENGTH];
   char ntp_server[NTP_SERVER_LENGTH];

   sensor_t sensors[USE_SENSORS_COUNT];

   #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
   bool is_dhcp_enable;
   uint8_t ethernet_mac[ETHERNET_MAC_LENGTH];
   uint8_t ip[ETHERNET_IP_LENGTH];
   uint8_t netmask[ETHERNET_IP_LENGTH];
   uint8_t gateway[ETHERNET_IP_LENGTH];
   uint8_t primary_dns[ETHERNET_IP_LENGTH];

   #elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
   char gsm_apn[GSM_APN_LENGTH];
   char gsm_username[GSM_APN_LENGTH];
   char gsm_password[GSM_APN_LENGTH];

   #endif
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
Writable data.
*/
typedef struct {
   uint16_t mqtt_port;
   char mqtt_server[MQTT_SERVER_LENGTH];
   char mqtt_root_topic[MQTT_ROOT_TOPIC_LENGTH];
   char mqtt_subscribe_topic[MQTT_SUBSCRIBE_TOPIC_LENGTH];
   char mqtt_username[MQTT_USERNAME_LENGTH];
   char mqtt_password[MQTT_PASSWORD_LENGTH];
   char ntp_server[NTP_SERVER_LENGTH];

   sensor_t sensors[USE_SENSORS_COUNT];

   #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
   bool is_dhcp_enable;
   uint8_t ethernet_mac[ETHERNET_MAC_LENGTH];
   uint8_t ip[ETHERNET_IP_LENGTH];
   uint8_t netmask[ETHERNET_IP_LENGTH];
   uint8_t gateway[ETHERNET_IP_LENGTH];
   uint8_t primary_dns[ETHERNET_IP_LENGTH];

   #elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
   char gsm_apn[GSM_APN_LENGTH];
   char gsm_username[GSM_APN_LENGTH];
   char gsm_password[GSM_APN_LENGTH];

   #endif
} writable_data_t;

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
   SUPERVISOR_INIT,
   SUPERVISOR_RTC_LEVEL_TASK,
   SUPERVISOR_CONNECTION_LEVEL_TASK,
   SUPERVISOR_NTP_LEVEL_TASK,
   SUPERVISOR_MANAGE_LEVEL_TASK,
   SUPERVISOR_END,
   SUPERVISOR_WAIT_STATE
} supervisor_state_t;

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
typedef enum {
   ETHERNET_INIT,
   ETHERNET_CONNECT,
   ETHERNET_OPEN_UDP_SOCKET,
   ETHERNET_END,
   ETHERNET_WAIT_STATE
} ethernet_state_t;

#elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
typedef enum {
   GSM_INIT,
   GSM_SWITCH_ON,
   GSM_AUTOBAUD,
   GSM_SETUP,
   GSM_START_CONNECTION,
   GSM_CHECK_OPERATION,
   GSM_OPEN_UDP_SOCKET,
   GSM_STOP_CONNECTION,
   GSM_SUSPEND,
   GSM_WAIT_FOR_SWITCH_OFF,
   GSM_SWITCH_OFF,
   GSM_END,
   GSM_WAIT_STATE
} gsm_state_t;

#endif

typedef enum {
   SENSORS_READING_INIT,
   SENSORS_READING_PREPARE,
   SENSORS_READING_IS_PREPARED,
   SENSORS_READING_GET,
   SENSORS_READING_IS_GETTED,
   SENSORS_READING_READ,
   SENSORS_READING_NEXT,
   SENSORS_READING_END,
   SENSORS_READING_WAIT_STATE
} sensors_reading_state_t;

typedef enum {
   TIME_INIT,
   TIME_SEND_ONLINE_REQUEST,
   TIME_WAIT_ONLINE_RESPONSE,
   TIME_SET_SYNC_RTC_PROVIDER,
   TIME_END,
   TIME_WAIT_STATE
} time_state_t;

typedef enum {
   DATA_SAVING_INIT,
   DATA_SAVING_OPEN_SDCARD,   // if not already open
   DATA_SAVING_OPEN_FILE,
   DATA_SAVING_SENSORS_LOOP,  // loop from 0 to sensors_count
   DATA_SAVING_DATA_LOOP,     // loop from 0 to data_count
   DATA_SAVING_WRITE_FILE,
   DATA_SAVING_CLOSE_FILE,
   DATA_SAVING_END,
   DATA_SAVING_WAIT_STATE
} data_saving_state_t;

typedef enum {
   MQTT_INIT,

   MQTT_OPEN_SDCARD,    // if not already open
   MQTT_OPEN_PTR_FILE,
   MQTT_PTR_READ,
   MQTT_PTR_FIND,
   MQTT_PTR_FOUND,
   MQTT_PTR_END,

   MQTT_OPEN,
   MQTT_CHECK,
   MQTT_CONNECT,
   MQTT_SUBSCRIBE,

   MQTT_OPEN_DATA_FILE,

   MQTT_SENSORS_LOOP,   // loop from 0 to sensors_count
   MQTT_DATA_LOOP,      // loop from 0 to data_count
   MQTT_SD_LOOP,        // loop from first row to last row of data file
   MQTT_PUBLISH,

   MQTT_CLOSE_DATA_FILE,

   MQTT_DISCONNECT,

   MQTT_PTR_UPDATE,
   MQTT_CLOSE_PTR_FILE,
   MQTT_CLOSE_SDCARD,

   MQTT_END,
   MQTT_WAIT_STATE
} mqtt_state_t;

// typedef enum {
//    DATA_PROCESSING_INIT,
//    INIT_SDCARD_SERVICE,
//    OPEN_SDCARD_PTR_DATA_FILES,
//    OPEN_SDCARD_WRITE_DATA_FILE,
//    OPEN_SDCARD_READ_DATA_FILE,
//    READ_PTR_DATA,
//    FIND_PTR_DATA,
//    FOUND_PTR_DATA,
//    END_FIND_PTR,
//    END_SDCARD_SERVICE,
//    INIT_MQTT_SERVICE,
//    CHECK_CLIENT_CONNECTION_STATUS,
//    CONNECT_MQTT_SERVICE,
//    SUBSCRIBE_MQTT_SERVICE,
//    END_MQTT_SERVICE,
//    LOOP_JSON_TO_MQTT,
//    LOOP_SD_TO_MQTT,
//    LOOP_MQTT_TO_X,
//    WRITE_DATA_TO_X,
//    UPDATE_PTR_DATA,
//    DATA_PROCESSING_END,
//    DATA_PROCESSING_WAIT_STATE
// } data_processing_state_t;

/**********************************************************************
* GLOBAL VARIABLE
*********************************************************************/
/*!
\var configuration
Configuration for this module.
*/
configuration_t configuration;

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

SdFat SD;
File read_data_file;
File write_data_file;
File mqtt_ptr_file;

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
EthernetUDP eth_udp_client;
EthernetClient eth_tcp_client;
IPStack ipstack(eth_tcp_client);

#elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
sim800Client s800;
IPStack ipstack(s800);

#endif

#define IP_STACK_TIMEOUT_MS      (MQTT_TIMEOUT_MS)

MQTT::Client<IPStack, Countdown, MQTT_ROOT_TOPIC_LENGTH+MQTT_SENSOR_TOPIC_LENGTH+MQTT_MESSAGE_LENGTH, 1> mqtt_client = MQTT::Client<IPStack, Countdown, MQTT_ROOT_TOPIC_LENGTH+MQTT_SENSOR_TOPIC_LENGTH+MQTT_MESSAGE_LENGTH, 1>(ipstack, IP_STACK_TIMEOUT_MS);

SensorDriver *sensors[USE_SENSORS_COUNT];
uint8_t sensors_count;

bool is_first_run;
bool is_time_set;

bool is_client_connected;
bool is_client_udp_socket_open;
bool is_event_client_executed;
bool do_ntp_sync;
time_t last_ntp_sync;

bool is_sdcard_open;
bool is_sdcard_error;

bool is_ptr_found;
bool is_ptr_updated;

bool is_mqtt_subscribed;

char json_sensors_data[USE_SENSORS_COUNT][JSON_BUFFER_LENGTH];

value_t temperature;
value_t humidity;
rain_t rain;

uint8_t next_hour_for_sensor_reading;
uint8_t next_minute_for_sensor_reading;
uint8_t next_second_for_sensor_reading;

volatile tmElements_t sensor_reading_time;
time_t ptr_date_time;

supervisor_state_t supervisor_state;

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
ethernet_state_t ethernet_state;

#elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
gsm_state_t gsm_state;

#endif

time_state_t time_state;
volatile sensors_reading_state_t sensors_reading_state;
data_saving_state_t data_saving_state;
mqtt_state_t mqtt_state;

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
void init_wdt(void);

void check_tasks();

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

void setNextTimeForSensorReading(uint8_t *, uint8_t *, uint8_t *);

bool mqttConnect(char *username, char *password);
bool mqttPublish(const char *topic, const char *message);
void mqttRxCallback(MQTT::MessageData &md);

/**********************************************************************
* TASKS
*********************************************************************/

/*! \fn void supervisor_task(void)
*  \brief Start other tasks.
*  \return void.
*/
void supervisor_task(void);

volatile bool is_event_supervisor;

/*! \fn void sensors_reading_task(void)
*  \brief Read sensors.
*  \return void.
*/
void sensors_reading_task(void);

volatile bool is_event_sensors_reading;

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

volatile bool is_event_time;
bool is_event_time_executed;

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
/*! \fn void ethernet_task(void)
*  \brief manage ethernet connection.
*  \return void.
*/
void ethernet_task(void);

bool is_event_ethernet;

#elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
/*! \fn void gsm_task(void)
*  \brief manage gsm connection.
*  \return void.
*/
void gsm_task(void);

bool is_event_gsm;

#endif

/*! \fn void data_saving_task(void)
*  \brief save data in sdcard.
*  \return void.
*/
void data_saving_task(void);

bool is_event_data_saving;

/*! \fn void mqtt_task(void)
*  \brief send data over MQTT.
*  \return void.
*/
void mqtt_task(void);

bool is_event_mqtt;

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
