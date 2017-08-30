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

/**********************************************************************
* DEBUG
*********************************************************************/
/*! \file debug_config.h
\brief debug_config include file

debug_config include file
*/
#include <debug_config.h>

/*!
\def SERIAL_TRACE_LEVEL
Debug level for this sketch.
*/
#define SERIAL_TRACE_LEVEL RMAP_SERIAL_TRACE_LEVEL

#include "rmap.h"

void setup() {
   init_wdt();
   TRACE_BEGIN(230400);
   init_pins();
   load_configuration();
   init_buffers();
   init_wire();
   init_spi();
   init_rtc();
   init_system();
}

void loop() {
   switch (state) {
      case INIT:
         init_tasks();
         init_sensors();

         state = TASKS_EXECUTION;
         wdt_reset();
      break;

      #if (USE_POWER_DOWN)
      case ENTER_POWER_DOWN:
         wdt_disable();
         power_down(&awakened_event_occurred_time_ms);
         state = TASKS_EXECUTION;
         init_wdt();
      break;
      #endif

      case TASKS_EXECUTION:
         #if (USE_RTC_TASK)
         if (is_event_rtc) {
            rtc_task();
            wdt_reset();
         }
         #endif

         if (is_event_supervisor) {
            supervisor_task();
            wdt_reset();
         }

         #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
         if (is_event_ethernet) {
            ethernet_task();
            wdt_reset();
         }

         #elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
         if (is_event_gsm) {
            gsm_task();
            wdt_reset();
         }

         #endif

         if (is_event_sensors_reading) {
            sensors_reading_task();
            wdt_reset();
         }

         if (is_event_data_saving) {
            data_saving_task();
            wdt_reset();
         }

         if (is_event_mqtt) {
            mqtt_task();
            wdt_reset();
         }

         if (is_event_time) {
            time_task();
            wdt_reset();
         }

         wdt_reset();

         if (ready_tasks_count == 0) {
            state = END;
         }
      break;

      case END:
         #if (USE_POWER_DOWN)
         state = ENTER_POWER_DOWN;
         #else
         state = TASKS_EXECUTION;
         #endif
         wdt_reset();
      break;
   }
}

void init_buffers() {
   readable_data_read_ptr = &readable_data_1;
   readable_data_write_ptr = &readable_data_2;
   writable_data_ptr = &writable_data;

   readable_data_write_ptr->module_type = MODULE_TYPE;
   readable_data_write_ptr->module_version = MODULE_VERSION;
   memcpy((void *) readable_data_read_ptr, (const void*) readable_data_write_ptr, sizeof(readable_data_t));
}

void init_tasks() {
   noInterrupts();
   ready_tasks_count = 0;

   is_event_supervisor = true;
   ready_tasks_count++;

   is_event_time = false;
   is_event_sensors_reading = false;
   is_event_data_saving = false;
   is_event_mqtt = false;

   #if (USE_RTC_TASK)
   is_event_rtc = false;
   #endif

   #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
   is_event_ethernet = false;
   ethernet_state = ETHERNET_INIT;

   #elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
   is_event_gsm = false;
   gsm_state = GSM_INIT;

   #endif

   supervisor_state = SUPERVISOR_INIT;
   time_state = TIME_INIT;
   sensors_reading_state = SENSORS_READING_INIT;
   data_saving_state = DATA_SAVING_INIT;
   mqtt_state = MQTT_INIT;

   is_client_connected = false;
   is_client_udp_socket_open = false;

   do_ntp_sync = false;
   is_time_set = false;
   last_ntp_sync = -NTP_TIME_FOR_RESYNC_S;

   is_sdcard_error = false;
   is_sdcard_open = false;

   is_ptr_found = false;
   is_ptr_updated = false;

   is_mqtt_subscribed = false;

   interrupts();
}

void init_pins() {
   pinMode(CONFIGURATION_RESET_PIN, INPUT_PULLUP);

   pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);

   pinMode(SDCARD_CHIP_SELECT_PIN, OUTPUT);
   digitalWrite(SDCARD_CHIP_SELECT_PIN, HIGH);

   #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
   pinMode(W5500_CHIP_SELECT_PIN, OUTPUT);
   digitalWrite(W5500_CHIP_SELECT_PIN, HIGH);

   Ethernet.w5500_cspin = W5500_CHIP_SELECT_PIN;

   #elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
   s800.init(GSM_ON_OFF_PIN);

   #endif
}

void init_wire() {
   uint8_t i2c_bus_state = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()

   if (i2c_bus_state) {
      SERIAL_ERROR("I2C bus error: Could not clear!!!\r\n");
   }

   switch (i2c_bus_state) {
      case 1:
      SERIAL_ERROR("SCL clock line held low\r\n");
      break;

      case 2:
      SERIAL_ERROR("SCL clock line held low by slave clock stretch\r\n");
      break;

      case 3:
      SERIAL_ERROR("SDA data line held low\r\n");
      break;

      default:
      Wire.begin();
      Wire.setClock(I2C_BUS_CLOCK);
      break;
   }
}

void init_spi() {
   SPI.begin();
}

void init_rtc() {
   Pcf8563::disableAlarm();
   Pcf8563::disableTimer();
   Pcf8563::disableClockout();
   Pcf8563::setClockoutFrequency(RTC_FREQUENCY);
   Pcf8563::enableClockout();
   attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), rtc_interrupt_handler, RISING);
}

void init_system() {
   #if (USE_POWER_DOWN)
   set_sleep_mode(SLEEP_MODE_PWR_DOWN);
   awakened_event_occurred_time_ms = millis();
   #endif
   state = INIT;
}

void init_wdt() {
   wdt_disable();
   wdt_reset();
   wdt_enable(WDT_TIMER);
}

void init_sensors () {
   is_first_run = true;
   sensors_count = 0;

   SERIAL_INFO("Sensors...\r\n");

   for (uint8_t i=0; i<USE_SENSORS_COUNT; i++) {
      SensorDriver::createAndSetup(configuration.sensors[i].driver, configuration.sensors[i].type, configuration.sensors[i].address, sensors, &sensors_count);
      SERIAL_INFO("--> %u: %s-%s: %s\t [ %s ]\r\n", sensors_count, configuration.sensors[i].driver, configuration.sensors[i].type, configuration.sensors[i].mqtt_topic, sensors[i]->isSetted() ? OK_STRING : FAIL_STRING);
   }

   SERIAL_INFO("\r\n");
}

void setNextTimeForSensorReading (uint8_t *next_hour, uint8_t *next_minute, uint8_t *next_second) {
   bool is_update = false;
   *next_hour = hour();
   *next_second = 0;

   if (*next_minute == 255) {
      *next_minute = minute();
      *next_minute = *next_minute - (*next_minute % REPORT_MINUTES) + REPORT_MINUTES;
      is_update = true;
   }
   else if (minute() == *next_minute) {
      *next_minute += REPORT_MINUTES;
      is_update = true;
   }

   if (is_update) {
      if (*next_minute > 59) {
         *next_minute -= 60;
         (*next_hour)++;
      }

      if (*next_hour > 23)
      *next_hour = *next_hour - 24;
   }
}

bool mqttConnect(char *username, char *password) {
   MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
   data.MQTTVersion = 3;
   data.clientID.cstring = (char*)"proviamoci123";
   data.username.cstring = (char*)username;
   data.password.cstring = (char*)password;
   data.cleansession = false;

   return !mqtt_client.connect(data);
}

bool mqttPublish(const char *topic, const char *message) {
   MQTT::Message tx_message;
   tx_message.qos = MQTT::QOS1;
   tx_message.retained = false;
   tx_message.dup = false;
   tx_message.payload = (void*)message;
   tx_message.payloadlen = strlen(message) + 1;

   return !mqtt_client.publish(topic, tx_message);
}

void mqttRxCallback(MQTT::MessageData &md) {
   MQTT::Message &rx_message = md.message;
   SERIAL_DEBUG("%s\r\n", (char*)rx_message.payload);
   SERIAL_DEBUG("--> qos %u\r\n", rx_message.qos);
   SERIAL_DEBUG("--> retained %u\r\n", rx_message.retained);
   SERIAL_DEBUG("--> dup %u\r\n", rx_message.dup);
   SERIAL_DEBUG("--> id %u\r\n", rx_message.id);
}

void print_configuration() {
   char stima_name[20];
   getStimaNameByType(stima_name, configuration.module_type);
   SERIAL_INFO("--> type: %s\r\n", stima_name);
   SERIAL_INFO("--> version: %d\r\n", configuration.module_version);

   #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
   SERIAL_INFO("--> dhcp: %s\r\n", configuration.is_dhcp_enable ? "on" : "off");
   SERIAL_INFO("--> ethernet mac: ");
   SERIAL_INFO_ARRAY(configuration.ethernet_mac, ETHERNET_MAC_LENGTH, UINT8, "%02X:");
   SERIAL_INFO("\r\n");

   if (!configuration.is_dhcp_enable) {
      SERIAL_INFO("--> ip: ");
      SERIAL_INFO_ARRAY(configuration.ip, ETHERNET_IP_LENGTH, UINT8, "%u.");
      SERIAL_INFO("\r\n");
      SERIAL_INFO("--> netmask: ");
      SERIAL_INFO_ARRAY(configuration.netmask, ETHERNET_IP_LENGTH, UINT8, "%u.");
      SERIAL_INFO("\r\n");
      SERIAL_INFO("--> gateway: ");
      SERIAL_INFO_ARRAY(configuration.gateway, ETHERNET_IP_LENGTH, UINT8, "%u.");
      SERIAL_INFO("\r\n");
      SERIAL_INFO("--> primary dns: ");
      SERIAL_INFO_ARRAY(configuration.primary_dns, ETHERNET_IP_LENGTH, UINT8, "%u.");
      SERIAL_INFO("\r\n");
   }

   #elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
   SERIAL_INFO("--> gsm apn: %s\r\n", configuration.gsm_apn);
   SERIAL_INFO("--> gsm username: %s\r\n", configuration.gsm_username);
   SERIAL_INFO("--> gsm password: %s\r\n", configuration.gsm_password);

   #endif

   SERIAL_INFO("--> ntp server: %s\r\n", configuration.ntp_server);

   SERIAL_INFO("--> mqtt server: %s\r\n", configuration.mqtt_server);
   SERIAL_INFO("--> mqtt port: %u\r\n", configuration.mqtt_port);
   SERIAL_INFO("--> mqtt root topic: %s\r\n", configuration.mqtt_root_topic);
   SERIAL_INFO("--> mqtt subscribe topic: %s\r\n", configuration.mqtt_subscribe_topic);
   SERIAL_INFO("--> mqtt username: %s\r\n", configuration.mqtt_username);
   SERIAL_INFO("--> mqtt password: %s\r\n", configuration.mqtt_password);

   SERIAL_INFO("\r\n");
}

void save_configuration(bool is_default) {
   if (is_default) {
      SERIAL_INFO("Save default configuration... [ OK ]\r\n");
      configuration.module_type = MODULE_TYPE;
      configuration.module_version = MODULE_VERSION;

      // DEBUG
      uint8_t i=0;

      strcpy(configuration.sensors[i].driver, SENSOR_DRIVER_I2C);
      strcpy(configuration.sensors[i].type, SENSOR_TYPE_TBS);
      strcpy(configuration.sensors[i].mqtt_topic, "1,0,900/1,-,-,-/");
      configuration.sensors[i].address = CONFIGURATION_DEFAULT_RAIN_ADDRESS;
      i++;

      strcpy(configuration.sensors[i].driver, SENSOR_DRIVER_I2C);
      strcpy(configuration.sensors[i].type, SENSOR_TYPE_ITH);
      strcpy(configuration.sensors[i].mqtt_topic, "254,0,0/103,2000,-,-/");
      configuration.sensors[i].address = CONFIGURATION_DEFAULT_TH_ADDRESS;
      i++;

      strcpy(configuration.sensors[i].driver, SENSOR_DRIVER_I2C);
      strcpy(configuration.sensors[i].type, SENSOR_TYPE_NTH);
      strcpy(configuration.sensors[i].mqtt_topic, "3,0,0/103,2000,-,-/");
      configuration.sensors[i].address = CONFIGURATION_DEFAULT_TH_ADDRESS;
      i++;

      strcpy(configuration.sensors[i].driver, SENSOR_DRIVER_I2C);
      strcpy(configuration.sensors[i].type, SENSOR_TYPE_MTH);
      strcpy(configuration.sensors[i].mqtt_topic, "0,0,0/103,2000,-,-/");
      configuration.sensors[i].address = CONFIGURATION_DEFAULT_TH_ADDRESS;
      i++;

      strcpy(configuration.sensors[i].driver, SENSOR_DRIVER_I2C);
      strcpy(configuration.sensors[i].type, SENSOR_TYPE_XTH);
      strcpy(configuration.sensors[i].mqtt_topic, "2,0,0/103,2000,-,-/");
      configuration.sensors[i].address = CONFIGURATION_DEFAULT_TH_ADDRESS;
      i++;
      // END DEBUG

      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
      char temp_string[20];
      configuration.is_dhcp_enable = CONFIGURATION_DEFAULT_ETHERNET_DHCP_ENABLE;
      strcpy(temp_string, CONFIGURATION_DEFAULT_ETHERNET_MAC);
      macStringToArray(configuration.ethernet_mac, temp_string);
      strcpy(temp_string, CONFIGURATION_DEFAULT_ETHERNET_IP);
      ipStringToArray(configuration.ip, temp_string);
      strcpy(temp_string, CONFIGURATION_DEFAULT_ETHERNET_NETMASK);
      ipStringToArray(configuration.netmask, temp_string);
      strcpy(temp_string, CONFIGURATION_DEFAULT_ETHERNET_GATEWAY);
      ipStringToArray(configuration.gateway, temp_string);
      strcpy(temp_string, CONFIGURATION_DEFAULT_ETHERNET_PRIMARY_DNS);
      ipStringToArray(configuration.primary_dns, temp_string);

      #elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
      strcpy(configuration.gsm_apn, GSM_APN_TIM);
      strcpy(configuration.gsm_username, GSM_USERNAME);
      strcpy(configuration.gsm_password, GSM_PASSWORD);

      #endif

      strcpy(configuration.ntp_server, CONFIGURATION_DEFAULT_NTP_SERVER);

      configuration.mqtt_port = CONFIGURATION_DEFAULT_MQTT_PORT;
      strcpy(configuration.mqtt_server, CONFIGURATION_DEFAULT_MQTT_SERVER);
      strcpy(configuration.mqtt_root_topic, CONFIGURATION_DEFAULT_MQTT_ROOT_TOPIC);
      strcpy(configuration.mqtt_subscribe_topic, CONFIGURATION_DEFAULT_MQTT_SUBSCRIBE_TOPIC);
      strcpy(configuration.mqtt_username, CONFIGURATION_DEFAULT_MQTT_USERNAME);
      strcpy(configuration.mqtt_password, CONFIGURATION_DEFAULT_MQTT_PASSWORD);
   }
   else {
      SERIAL_INFO("Save configuration... [ OK ]\r\n");

      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
      configuration.is_dhcp_enable = writable_data.is_dhcp_enable;
      #endif

      strcpy(configuration.ntp_server, writable_data.ntp_server);

      strcpy(configuration.mqtt_server, writable_data.mqtt_server);
      strcpy(configuration.mqtt_root_topic, writable_data.mqtt_root_topic);
      strcpy(configuration.mqtt_subscribe_topic, writable_data.mqtt_subscribe_topic);
      strcpy(configuration.mqtt_username, writable_data.mqtt_username);
      strcpy(configuration.mqtt_password, writable_data.mqtt_password);
   }

   ee_write(&configuration, CONFIGURATION_EEPROM_ADDRESS, sizeof(configuration_t));

   print_configuration();
}

void load_configuration() {
   ee_read(&configuration, CONFIGURATION_EEPROM_ADDRESS, sizeof(configuration_t));

   if (configuration.module_type != MODULE_TYPE || configuration.module_version != MODULE_VERSION || digitalRead(CONFIGURATION_RESET_PIN) == LOW) {
      save_configuration(CONFIGURATION_DEFAULT);
   }
   else {
      SERIAL_INFO("Load configuration... [ OK ]\r\n");
      print_configuration();
   }
}

void rtc_interrupt_handler() {
   #if (USE_RTC_TASK)

   if (second() == next_second_for_sensor_reading && minute() == next_minute_for_sensor_reading) {
      setNextTimeForSensorReading(&next_hour_for_sensor_reading, &next_minute_for_sensor_reading, &next_second_for_sensor_reading);
      SERIAL_INFO("Next acquisition scheduled at: %02u:%02u:%02u\r\n", next_hour_for_sensor_reading, next_minute_for_sensor_reading, next_second_for_sensor_reading);
      sensor_reading_time.Day = day();
      sensor_reading_time.Month = month();
      sensor_reading_time.Year = CalendarYrToTm(year());
      sensor_reading_time.Hour = hour();
      sensor_reading_time.Minute = minute();
      sensor_reading_time.Second = next_second_for_sensor_reading;

      noInterrupts();
      if (!is_event_sensors_reading) {
         is_event_sensors_reading = true;
         ready_tasks_count++;
      }
      interrupts();
   }

   noInterrupts();
   if (!is_event_rtc) {
      is_event_rtc = true;
      ready_tasks_count++;
   }
   interrupts();

   #endif
}

void supervisor_task() {
   static supervisor_state_t state_after_wait;
   static uint32_t delay_ms;
   static uint32_t start_time_ms;

   static bool is_supervisor_first_run = true;
   static bool is_time_updated;

   #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
   bool *is_event_client = &is_event_ethernet;
   #elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
   bool *is_event_client = &is_event_gsm;
   #endif

   switch (supervisor_state) {
      case SUPERVISOR_INIT:
         start_time_ms = 0;
         is_time_updated = false;
         is_event_time_executed = false;
         is_event_client_executed = false;

         if (is_supervisor_first_run) {
            supervisor_state = SUPERVISOR_RTC_LEVEL_TASK;
            SERIAL_TRACE("SUPERVISOR_INIT ---> SUPERVISOR_RTC_LEVEL_TASK\r\n");
         }
         else {
            supervisor_state = SUPERVISOR_CONNECTION_LEVEL_TASK;
            SERIAL_TRACE("SUPERVISOR_INIT ---> SUPERVISOR_CONNECTION_LEVEL_TASK\r\n");
         }
      break;

      case SUPERVISOR_RTC_LEVEL_TASK:
         // first time
         noInterrupts();
         if (!is_event_time && !is_event_time_executed && !is_time_set) {
            time_state = TIME_INIT;
            is_event_time = true;
            ready_tasks_count++;
         }
         interrupts();

         // success
         if (!is_event_time && is_event_time_executed && is_time_set) {
            is_time_updated = true;
            supervisor_state = SUPERVISOR_CONNECTION_LEVEL_TASK;
            SERIAL_TRACE("SUPERVISOR_RTC_LEVEL_TASK ---> SUPERVISOR_CONNECTION_LEVEL_TASK\r\n");
         }

         // error: BLOCK !!!!
         if (!is_event_time && is_event_time_executed && !is_time_set) {
            supervisor_state = SUPERVISOR_MANAGE_LEVEL_TASK;
            SERIAL_TRACE("SUPERVISOR_RTC_LEVEL_TASK ---> SUPERVISOR_MANAGE_LEVEL_TASK\r\n");
         }
      break;

      case SUPERVISOR_CONNECTION_LEVEL_TASK:
         if (start_time_ms == 0) {
            start_time_ms = millis();
         }

         if (!do_ntp_sync && (now() - last_ntp_sync > NTP_TIME_FOR_RESYNC_S)) {
            do_ntp_sync = true;
         }

         // first time
         noInterrupts();
         if (!*is_event_client && !is_event_client_executed && !is_client_connected) {
            *is_event_client = true;
            ready_tasks_count++;
         }
         interrupts();

         // success
         if (!*is_event_client && is_client_connected) {
            // do ntp sync
            if (is_client_udp_socket_open && do_ntp_sync) {
               is_event_time_executed = false;
               supervisor_state = SUPERVISOR_NTP_LEVEL_TASK;
               SERIAL_TRACE("SUPERVISOR_CONNECTION_LEVEL_TASK ---> SUPERVISOR_NTP_LEVEL_TASK\r\n");
            }
            // other operation
            else {
               do_ntp_sync = false;
               supervisor_state = SUPERVISOR_MANAGE_LEVEL_TASK;
               SERIAL_TRACE("SUPERVISOR_CONNECTION_LEVEL_TASK ---> SUPERVISOR_MANAGE_LEVEL_TASK\r\n");
            }
         }

         // error
         if ((!*is_event_client && is_event_client_executed && !is_client_connected) || (millis() - start_time_ms > SUPERVISOR_CONNECTION_TIMEOUT_MS)) {
            supervisor_state = SUPERVISOR_MANAGE_LEVEL_TASK;
            SERIAL_TRACE("SUPERVISOR_CONNECTION_LEVEL_TASK ---> SUPERVISOR_MANAGE_LEVEL_TASK\r\n");
         }
      break;

      case SUPERVISOR_NTP_LEVEL_TASK:
         // first time
         noInterrupts();
         if (!is_event_time && !is_event_time_executed && is_time_set && is_client_connected) {
            time_state = TIME_INIT;
            is_event_time = true;
            ready_tasks_count++;
         }
         interrupts();

         // success
         if (!is_event_time && is_event_time_executed && is_time_set && is_client_connected) {
            last_ntp_sync = now();
            is_time_updated = true;
         }

         // success or error
         if ((!is_event_time && is_event_time_executed && is_time_set && is_client_connected) || (!is_event_time && is_event_time_executed && !is_time_set && is_client_connected)) {
            do_ntp_sync = false;

            #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
            noInterrupts();
            if (!*is_event_client) {
               *is_event_client = true;
               ready_tasks_count++;
            }
            interrupts();
            #endif

            supervisor_state = SUPERVISOR_MANAGE_LEVEL_TASK;
            SERIAL_TRACE("SUPERVISOR_NTP_LEVEL_TASK ---> SUPERVISOR_MANAGE_LEVEL_TASK\r\n");
         }

      break;

      case SUPERVISOR_MANAGE_LEVEL_TASK:
         if (is_time_updated) {
            SERIAL_INFO("Current date and time is: %02u/%02u/%04u %02u:%02u:%02u\r\n\r\n", day(), month(), year(), hour(), minute(), second());
         }

         if (is_supervisor_first_run && is_time_set) {
            next_second_for_sensor_reading = 0;
            next_minute_for_sensor_reading = 255;
            setNextTimeForSensorReading(&next_hour_for_sensor_reading, &next_minute_for_sensor_reading, &next_second_for_sensor_reading);

            SERIAL_INFO("Acquisition scheduling...\r\n");
            SERIAL_INFO("--> observations every %u minutes\r\n", OBSERVATIONS_MINUTES);
            SERIAL_INFO("--> report every %u minutes\r\n", REPORT_MINUTES);
            SERIAL_INFO("--> starting at: %02u:%02u:%02u\r\n\r\n", next_hour_for_sensor_reading, next_minute_for_sensor_reading, next_second_for_sensor_reading);
         }

         #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
         noInterrupts();
         if (!is_event_mqtt) {
            is_event_mqtt = true;
            ready_tasks_count++;
         }
         interrupts();

         #elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
         if (!*is_event_client) {
            noInterrupts();
            if (!is_event_mqtt) {
               is_event_mqtt = true;
               ready_tasks_count++;
            }
            interrupts();
         }

         #endif

         #if (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_INFO)
         delay_ms = DEBUG_WAIT_FOR_SLEEP_MS;
         start_time_ms = millis();
         state_after_wait = SUPERVISOR_END;
         supervisor_state = SUPERVISOR_WAIT_STATE;
         #else
         supervisor_state = SUPERVISOR_END;
         #endif
         SERIAL_TRACE("SUPERVISOR_MANAGE_LEVEL_TASK ---> SUPERVISOR_END\r\n");
      break;

      case SUPERVISOR_END:
         is_supervisor_first_run = false;
         noInterrupts();
         is_event_supervisor = false;
         ready_tasks_count--;
         interrupts();

         supervisor_state = SUPERVISOR_INIT;
         SERIAL_TRACE("SUPERVISOR_END ---> SUPERVISOR_INIT\r\n");
      break;

      case SUPERVISOR_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            supervisor_state = state_after_wait;
         }
      break;
   }
}

#if (USE_RTC_TASK)
void rtc_task() {
   setTime(Pcf8563::getTime());
   noInterrupts();
   is_event_rtc = false;
   ready_tasks_count--;
   interrupts();
}
#endif

void time_task() {
   static uint8_t retry;
   static time_state_t state_after_wait;
   static uint32_t delay_ms;
   static uint32_t start_time_ms;
   int32_t seconds_since_1900;
   bool is_ntp_request_ok;

   switch (time_state) {
      case TIME_INIT:
         is_ntp_request_ok = false;
         seconds_since_1900 = 0;
         retry = 0;
         state_after_wait = TIME_INIT;

         if (is_client_connected) {
            time_state = TIME_SEND_ONLINE_REQUEST;
         }
         else time_state = TIME_SET_SYNC_RTC_PROVIDER;

      break;

      case TIME_SEND_ONLINE_REQUEST:
         #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
         //while (eth_udp_client.parsePacket() > 0);
         is_ntp_request_ok = Ntp::sendRequest(&eth_udp_client, configuration.ntp_server);

         #elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
         is_ntp_request_ok = Ntp::sendRequest(&s800);

         #endif

         // success
         if (is_ntp_request_ok) {
            retry = 0;
            time_state = TIME_WAIT_ONLINE_RESPONSE;
         }
         // retry
         else if (++retry < NTP_RETRY_COUNT_MAX) {
            delay_ms = NTP_RETRY_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = TIME_SEND_ONLINE_REQUEST;
            time_state = TIME_WAIT_STATE;
         }
         // fail: use old rtc time
         else {
            SERIAL_ERROR("NTP request... [ %s ]\r\n", FAIL_STRING);
            time_state = TIME_SET_SYNC_RTC_PROVIDER;
         }
      break;

      case TIME_WAIT_ONLINE_RESPONSE:
         #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
         seconds_since_1900 = Ntp::getResponse(&eth_udp_client);

         #elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
         seconds_since_1900 = Ntp::getResponse(&s800);

         #endif

         // success
         if (seconds_since_1900) {
            setTime(seconds_since_1900);
            Pcf8563::setDate(day(), month(), year()-2000, weekday()-1, 0);
            Pcf8563::setTime(hour(), minute(), second());
            SERIAL_TRACE("Current NTP date and time: %02u/%02u/%04u %02u:%02u:%02u\r\n", day(), month(), year(), hour(), minute(), second());
            time_state = TIME_SET_SYNC_RTC_PROVIDER;
         }
         // retry
         else if (++retry < NTP_RETRY_COUNT_MAX) {
            delay_ms = NTP_RETRY_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = TIME_WAIT_ONLINE_RESPONSE;
            time_state = TIME_WAIT_STATE;
         }
         // fail
         else {
            time_state = TIME_SET_SYNC_RTC_PROVIDER;
         }
      break;

      case TIME_SET_SYNC_RTC_PROVIDER:
         setSyncProvider(Pcf8563::getTime);
         SERIAL_TRACE("Current RTC date and time: %02u/%02u/%04u %02u:%02u:%02u\r\n", day(), month(), year(), hour(), minute(), second());
         time_state = TIME_END;
      break;

      case TIME_END:
         is_time_set = true;
         is_event_time_executed = true;
         noInterrupts();
         is_event_time = false;
         ready_tasks_count--;
         interrupts();
         time_state = TIME_INIT;
      break;

      case TIME_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            time_state = state_after_wait;
         }
      break;
   }
}

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
void ethernet_task() {
   static uint8_t retry;
   static ethernet_state_t state_after_wait;
   static uint32_t delay_ms;
   static uint32_t start_time_ms;

   switch (ethernet_state) {
      case ETHERNET_INIT:
         retry = 0;
         is_client_connected = false;
         is_client_udp_socket_open = false;
         state_after_wait = ETHERNET_INIT;
         ethernet_state = ETHERNET_CONNECT;
      break;

      case ETHERNET_CONNECT:
         if (configuration.is_dhcp_enable) {
            if (Ethernet.begin(configuration.ethernet_mac)) {
               is_client_connected = true;
               SERIAL_INFO("Ethernet: DHCP [ OK ]\r\n");
            }
            else {
               SERIAL_ERROR("Ethernet: DHCP [ FAIL ]\r\n");
            }
         }
         else {
            Ethernet.begin(configuration.ethernet_mac, IPAddress(configuration.ip), IPAddress(configuration.primary_dns), IPAddress(configuration.gateway), IPAddress(configuration.netmask));
            is_client_connected = true;
            SERIAL_INFO("Ethernet: Static [ OK ]\r\n");
         }

         // success
         if (is_client_connected) {
            w5500.setRetransmissionTime(2000);
            w5500.setRetransmissionCount(2);
            SERIAL_INFO("--> ip: ");
            SERIAL_INFO_ARRAY((void *)(&Ethernet.localIP()[0]), ETHERNET_IP_LENGTH, UINT8, "%u.");
            SERIAL_INFO("\r\n");
            SERIAL_INFO("--> netmask: ");
            SERIAL_INFO_ARRAY((void *)(&Ethernet.subnetMask()[0]), ETHERNET_IP_LENGTH, UINT8, "%u.");
            SERIAL_INFO("\r\n");
            SERIAL_INFO("--> gateway: ");
            SERIAL_INFO_ARRAY((void *)(&Ethernet.gatewayIP()[0]), ETHERNET_IP_LENGTH, UINT8, "%u.");
            SERIAL_INFO("\r\n");
            SERIAL_INFO("--> primary dns: ");
            SERIAL_INFO_ARRAY((void *)(&Ethernet.dnsServerIP()[0]), ETHERNET_IP_LENGTH, UINT8, "%u.");
            SERIAL_INFO("\r\n");
            ethernet_state = ETHERNET_OPEN_UDP_SOCKET;
         }
         // retry
         else if ((++retry) < ETHERNET_RETRY_COUNT_MAX) {
            delay_ms = ETHERNET_RETRY_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = ETHERNET_CONNECT;
            ethernet_state = ETHERNET_WAIT_STATE;
         }
         // fail
         else {
            retry = 0;
            ethernet_state = ETHERNET_END;
         }
      break;

      case ETHERNET_OPEN_UDP_SOCKET:
         if (eth_udp_client.begin(ETHERNET_DEFAULT_LOCAL_UDP_PORT)) {
            SERIAL_TRACE("--> udp socket local port: %u [ OK ]\r\n", ETHERNET_DEFAULT_LOCAL_UDP_PORT);
            is_client_udp_socket_open = true;
            ethernet_state = ETHERNET_END;
         }
         // retry
         else if ((++retry) < ETHERNET_RETRY_COUNT_MAX) {
            delay_ms = ETHERNET_RETRY_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = ETHERNET_OPEN_UDP_SOCKET;
            ethernet_state = ETHERNET_WAIT_STATE;
         }
         // fail
         else {
            SERIAL_ERROR("--> udp socket on local port: %u [ FAIL ]\r\n", ETHERNET_DEFAULT_LOCAL_UDP_PORT);
            retry = 0;
            ethernet_state = ETHERNET_INIT;
         }
      break;

      case ETHERNET_END:
         SERIAL_INFO("\r\n");
         is_event_client_executed = true;
         noInterrupts();
         is_event_ethernet = false;
         ready_tasks_count--;
         interrupts();
         ethernet_state = ETHERNET_INIT;
      break;

      case ETHERNET_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            ethernet_state = state_after_wait;
         }
      break;
   }
}

#elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
void gsm_task() {
   static gsm_state_t state_after_wait;
   static uint32_t delay_ms;
   static uint32_t start_time_ms;
   static bool is_error;
   sim800_status_t sim800_status;
   uint8_t sim800_connection_status;
   static uint8_t power_off_mode = SIM800_POWER_OFF_BY_AT_COMMAND;

   switch (gsm_state) {
      case GSM_INIT:
         is_error = false;
         is_client_connected = false;
         sim800_connection_status = 0;
         state_after_wait = GSM_INIT;
         gsm_state = GSM_SWITCH_ON;
      break;

      case GSM_SWITCH_ON:
         sim800_status = s800.switchOn();

         // success
         if (sim800_status == SIM800_OK) {
            gsm_state = GSM_AUTOBAUD;
         }
         else if (sim800_status == SIM800_ERROR) {
            gsm_state = GSM_END;
         }
         // wait...
      break;

      case GSM_AUTOBAUD:
         sim800_status = s800.initAutobaud();

         // success
         if (sim800_status == SIM800_OK) {
            delay_ms = SIM800_WAIT_FOR_AUTOBAUD_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = GSM_SETUP;
            gsm_state = GSM_WAIT_STATE;

         }
         // fail
         else if (sim800_status == SIM800_ERROR) {
            gsm_state = GSM_WAIT_FOR_SWITCH_OFF;
         }
         // wait...
      break;

      case GSM_SETUP:
         sim800_status = s800.setup();

         // success
         if (sim800_status == SIM800_OK) {
            gsm_state = GSM_START_CONNECTION;
         }
         // fail
         else if (sim800_status == SIM800_ERROR) {
            is_error = true;
            gsm_state = GSM_WAIT_FOR_SWITCH_OFF;
         }
         // wait...
      break;

      case GSM_START_CONNECTION:
         sim800_status = s800.startConnection(GSM_APN, GSM_USERNAME, GSM_PASSWORD);

         // success
         if (sim800_status == SIM800_OK) {
            gsm_state = GSM_CHECK_OPERATION;
         }
         // fail
         else if (sim800_status == SIM800_ERROR) {
            is_error = true;
            gsm_state = GSM_WAIT_FOR_SWITCH_OFF;
         }
         // wait...
      break;

      case GSM_CHECK_OPERATION:
         // open udp socket for query NTP
         if (do_ntp_sync) {
            gsm_state = GSM_OPEN_UDP_SOCKET;
         }
         // wait for mqtt send terminate
         else {
            is_client_connected = true;
            is_event_client_executed = true;
            gsm_state = GSM_SUSPEND;
            state_after_wait = GSM_STOP_CONNECTION;
         }
      break;

      case GSM_OPEN_UDP_SOCKET:
         sim800_connection_status = s800.connection("UDP", configuration.ntp_server, NTP_SERVER_PORT);

         // success
         if (sim800_connection_status == 1) {
            is_client_udp_socket_open = true;
            is_client_connected = true;
            is_event_client_executed = true;
            state_after_wait = GSM_STOP_CONNECTION;
            gsm_state = GSM_SUSPEND;
         }
         // fail
         else if (sim800_connection_status == 2) {
            is_client_connected = false;
            is_event_client_executed = true;
            is_error = true;
            gsm_state = GSM_WAIT_FOR_SWITCH_OFF;
         }
         // wait
      break;

      case GSM_STOP_CONNECTION:
         sim800_status = s800.stopConnection();

         // success
         if (sim800_status == SIM800_OK) {
            gsm_state = GSM_SWITCH_OFF;
         }
         // fail
         else if (sim800_status == SIM800_ERROR) {
            is_error = true;
            gsm_state = GSM_SWITCH_OFF;
         }
         // wait
      break;

      case GSM_SUSPEND:
         gsm_state = state_after_wait;
         noInterrupts();
         is_event_gsm = false;
         ready_tasks_count--;
         interrupts();
      break;

      case GSM_WAIT_FOR_SWITCH_OFF:
         delay_ms = SIM800_POWER_ON_TO_OFF_DELAY_MS;
         start_time_ms = millis();
         state_after_wait = GSM_SWITCH_OFF;
         gsm_state = GSM_WAIT_STATE;
      break;

      case GSM_SWITCH_OFF:
         sim800_status = s800.switchOff(power_off_mode);

         // success
         if (sim800_status == SIM800_OK) {
            gsm_state = GSM_END;
         }
         // fail
         else if (sim800_status == SIM800_ERROR) {
            if (power_off_mode == SIM800_POWER_OFF_BY_AT_COMMAND) {
               power_off_mode = SIM800_POWER_OFF_BY_SWITCH;
            }
            else {
               gsm_state = GSM_END;
            }
         }
         // wait...
      break;

      case GSM_END:
         is_event_client_executed = false;
         is_client_connected = false;
         is_client_udp_socket_open = false;
         noInterrupts();
         is_event_gsm = false;
         ready_tasks_count--;
         interrupts();
         gsm_state = GSM_INIT;
      break;

      case GSM_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            gsm_state = state_after_wait;
         }
      break;
   }
}

#endif

void sensors_reading_task () {
   static uint8_t i;
   static uint8_t retry;
   static sensors_reading_state_t state_after_wait;
   static uint32_t delay_ms;
   static uint32_t start_time_ms;
   static int32_t values_readed_from_sensor[2];

   switch (sensors_reading_state) {
      case SENSORS_READING_INIT:
         SERIAL_INFO("sensors reading...\r\n");
         for (i=0; i<sensors_count; i++) {
            sensors[i]->resetPrepared();
         }

         i = 0;
         retry = 0;
         state_after_wait = SENSORS_READING_INIT;
         sensors_reading_state = SENSORS_READING_PREPARE;
         SERIAL_TRACE("SENSORS_READING_INIT ---> SENSORS_READING_PREPARE\r\n");
      break;

      case SENSORS_READING_PREPARE:
         sensors[i]->prepare();
         delay_ms = sensors[i]->getDelay();
         start_time_ms = sensors[i]->getStartTime();

         if (delay_ms) {
            state_after_wait = SENSORS_READING_IS_PREPARED;
            sensors_reading_state = SENSORS_READING_WAIT_STATE;
            SERIAL_TRACE("SENSORS_READING_PREPARE ---> SENSORS_READING_WAIT_STATE\r\n");
         }
         else {
            sensors_reading_state = SENSORS_READING_IS_PREPARED;
            SERIAL_TRACE("SENSORS_READING_PREPARE ---> SENSORS_READING_IS_PREPARED\r\n");
         }
      break;

      case SENSORS_READING_IS_PREPARED:
         // success
         if (sensors[i]->isPrepared()) {
            sensors_reading_state = SENSORS_READING_GET;
            SERIAL_TRACE("SENSORS_READING_IS_PREPARED ---> SENSORS_READING_GET\r\n");
         }
         // retry
         else if ((++retry) < SENSORS_RETRY_COUNT_MAX) {
            delay_ms = SENSORS_RETRY_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SENSORS_READING_PREPARE;
            sensors_reading_state = SENSORS_READING_WAIT_STATE;
            SERIAL_TRACE("SENSORS_READING_IS_PREPARED ---> SENSORS_READING_WAIT_STATE\r\n");
         }
         // fail
         else {
            sensors_reading_state = SENSORS_READING_NEXT;
            SERIAL_TRACE("SENSORS_READING_IS_PREPARED ---> SENSORS_READING_NEXT\r\n");
         }
      break;

      case SENSORS_READING_GET:
         sensors[i]->getJson(values_readed_from_sensor, VALUES_TO_READ_FROM_SENSOR_COUNT, &json_sensors_data[i][0]);
         // sensors[i]->get(values_readed_from_sensor, VALUES_TO_READ_FROM_SENSOR_COUNT);
         delay_ms = sensors[i]->getDelay();
         start_time_ms = sensors[i]->getStartTime();

         if (delay_ms) {
            state_after_wait = SENSORS_READING_IS_GETTED;
            sensors_reading_state = SENSORS_READING_WAIT_STATE;
            SERIAL_TRACE("SENSORS_READING_GET ---> SENSORS_READING_WAIT_STATE\r\n");
         }
         else {
            sensors_reading_state = SENSORS_READING_IS_GETTED;
            SERIAL_TRACE("SENSORS_READING_GET ---> SENSORS_READING_IS_GETTED\r\n");
         }
      break;

      case SENSORS_READING_IS_GETTED:
         // success and end
         if (sensors[i]->isEnd() && !sensors[i]->isReaded() && sensors[i]->isSuccess()) {
            sensors_reading_state = SENSORS_READING_READ;
            SERIAL_TRACE("SENSORS_READING_IS_GETTED ---> SENSORS_READING_READ\r\n");
         }
         // success and not end
         else if (sensors[i]->isSuccess()) {
            sensors_reading_state = SENSORS_READING_GET;
            SERIAL_TRACE("SENSORS_READING_IS_GETTED ---> SENSORS_READING_GET\r\n");
         }
         // retry
         else if ((++retry) < SENSORS_RETRY_COUNT_MAX) {
            delay_ms = SENSORS_RETRY_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SENSORS_READING_GET;
            sensors_reading_state = SENSORS_READING_WAIT_STATE;
            SERIAL_TRACE("SENSORS_READING_IS_GETTED ---> SENSORS_READING_WAIT_STATE\r\n");
         }
         // fail
         else {
            sensors_reading_state = SENSORS_READING_NEXT;
            SERIAL_TRACE("SENSORS_READING_IS_GETTED ---> SENSORS_READING_NEXT\r\n");
         }
      break;

      case SENSORS_READING_READ:
         sensors_reading_state = SENSORS_READING_NEXT;
         SERIAL_TRACE("SENSORS_READING_READ ---> SENSORS_READING_NEXT\r\n");
      break;

      case SENSORS_READING_NEXT:
         // next sensor
         if ((++i) < sensors_count) {
            retry = 0;
            sensors_reading_state = SENSORS_READING_PREPARE;
            SERIAL_TRACE("SENSORS_READING_NEXT ---> SENSORS_READING_PREPARE\r\n");
         }
         // end
         else {

            // first time: read ptr data from sdcard
            if (is_first_run) {
               is_first_run = false;
            }
            // other time: save data to sdcard
            else {
               noInterrupts();
               if (!is_event_data_saving) {
                  is_event_data_saving = true;
                  ready_tasks_count++;
               }
               interrupts();
            }

            sensors_reading_state = SENSORS_READING_END;
            SERIAL_TRACE("SENSORS_READING_NEXT ---> SENSORS_READING_END\r\n");
         }
      break;

      case SENSORS_READING_END:
         noInterrupts();
         is_event_sensors_reading = false;
         ready_tasks_count--;
         interrupts();
         sensors_reading_state = SENSORS_READING_INIT;
         SERIAL_TRACE("SENSORS_READING_END ---> SENSORS_READING_INIT\r\n");
      break;

      case SENSORS_READING_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            sensors_reading_state = state_after_wait;
         }
      break;
   }
}

void data_saving_task() {
   static uint8_t retry;
   static data_saving_state_t state_after_wait;
   static uint32_t delay_ms;
   static uint32_t start_time_ms;
   static uint8_t i;
   static uint8_t k;
   static uint8_t data_count;
   static uint16_t sd_data_count;
   static char sd_buffer[MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH];
   static char topic_buffer[VALUES_TO_READ_FROM_SENSOR_COUNT][MQTT_SENSOR_TOPIC_LENGTH];
   static char message_buffer[VALUES_TO_READ_FROM_SENSOR_COUNT][MQTT_MESSAGE_LENGTH];
   char file_name[SDCARD_FILES_NAME_MAX_LENGTH];

   switch (data_saving_state) {
      case DATA_SAVING_INIT:
         retry = 0;
         sd_data_count = 0;

         if (is_sdcard_open) {
            data_saving_state = DATA_SAVING_OPEN_FILE;
            SERIAL_TRACE("DATA_SAVING_INIT ---> DATA_SAVING_OPEN_FILE\r\n");
         }
         else {
            data_saving_state = DATA_SAVING_OPEN_SDCARD;
            SERIAL_TRACE("DATA_SAVING_INIT ---> DATA_SAVING_OPEN_SDCARD\r\n");
         }
      break;

      case DATA_SAVING_OPEN_SDCARD:
         if (sdcard_init(&SD, SDCARD_CHIP_SELECT_PIN)) {
            retry = 0;
            is_sdcard_open = true;
            data_saving_state = DATA_SAVING_OPEN_FILE;
            SERIAL_TRACE("DATA_SAVING_OPEN_SDCARD ---> DATA_SAVING_OPEN_FILE\r\n");
         }
         // retry
         else if ((++retry) < DATA_SAVING_RETRY_COUNT_MAX) {
            delay_ms = DATA_SAVING_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = DATA_SAVING_OPEN_SDCARD;
            data_saving_state = DATA_SAVING_WAIT_STATE;
            SERIAL_TRACE("DATA_SAVING_OPEN_SDCARD ---> DATA_SAVING_WAIT_STATE\r\n");
         }
         // fail
         else {
            is_sdcard_error = true;
            is_sdcard_open = false;
            SERIAL_ERROR("SD Card... [ FAIL ]\r\n--> is card inserted?\r\n--> there is a valid FAT32 filesystem?\r\n\r\n");

            data_saving_state = DATA_SAVING_END;
            SERIAL_TRACE("DATA_SAVING_OPEN_SDCARD ---> DATA_SAVING_END\r\n");
         }
      break;

      case DATA_SAVING_OPEN_FILE:
         // open sdcard file: today!
         sdcard_make_filename(now(), file_name);

         // try to open file. if ok, write sensors data on it.
         if (sdcard_open_file(&SD, &write_data_file, file_name, O_RDWR | O_CREAT | O_APPEND)) {
            retry = 0;
            i = 0;
            data_saving_state = DATA_SAVING_SENSORS_LOOP;
            SERIAL_TRACE("DATA_SAVING_OPEN_FILE ---> DATA_SAVING_SENSORS_LOOP\r\n");
         }
         // retry
         else if ((++retry) < DATA_SAVING_RETRY_COUNT_MAX) {
            delay_ms = DATA_SAVING_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = DATA_SAVING_OPEN_FILE;
            data_saving_state = DATA_SAVING_WAIT_STATE;
            SERIAL_TRACE("DATA_SAVING_OPEN_SDCARD ---> DATA_SAVING_WAIT_STATE\r\n");
         }
         // fail
         else {
            SERIAL_ERROR("SD Card open file %s... [ FAIL ]\r\n", file_name);
            is_sdcard_error = true;
            data_saving_state = DATA_SAVING_END;
            SERIAL_TRACE("DATA_SAVING_OPEN_FILE ---> DATA_SAVING_END\r\n");
         }
      break;

      case DATA_SAVING_SENSORS_LOOP:
         if (i < sensors_count) {
            k = 0;
            data_count = jsonToMqtt(&json_sensors_data[i][0], configuration.sensors[i].mqtt_topic, topic_buffer, message_buffer, (tmElements_t *) &sensor_reading_time);
            data_saving_state = DATA_SAVING_DATA_LOOP;
            SERIAL_TRACE("DATA_SAVING_SENSORS_LOOP ---> DATA_SAVING_DATA_LOOP\r\n");
         }
         else {
            SERIAL_DEBUG("\r\n");
            data_saving_state = DATA_SAVING_CLOSE_FILE;
            SERIAL_TRACE("DATA_SAVING_SENSORS_LOOP ---> DATA_SAVING_CLOSE_FILE\r\n");
         }
      break;

      case DATA_SAVING_DATA_LOOP:
         if (k < data_count) {
            mqttToSd(&topic_buffer[k][0], &message_buffer[k][0], sd_buffer);
            data_saving_state = DATA_SAVING_WRITE_FILE;
            SERIAL_TRACE("DATA_SAVING_DATA_LOOP ---> DATA_SAVING_WRITE_FILE\r\n");
         }
         else {
            i++;
            data_saving_state = DATA_SAVING_SENSORS_LOOP;
            SERIAL_TRACE("DATA_SAVING_DATA_LOOP ---> DATA_SAVING_SENSORS_LOOP\r\n");
         }
      break;

      case DATA_SAVING_WRITE_FILE:
         // sdcard success
         if (write_data_file.write(sd_buffer, MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH) == (MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH)) {
            SERIAL_DEBUG("SD <-- %s %s\r\n", &topic_buffer[k][0], &message_buffer[k][0]);
            write_data_file.flush();
            retry = 0;
            k++;
            sd_data_count++;
            data_saving_state = DATA_SAVING_DATA_LOOP;
            SERIAL_TRACE("DATA_SAVING_WRITE_FILE ---> DATA_SAVING_DATA_LOOP\r\n");
         }
         // retry
         else if ((++retry) < DATA_SAVING_RETRY_COUNT_MAX) {
            delay_ms = DATA_SAVING_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = DATA_SAVING_WRITE_FILE;
            data_saving_state = DATA_SAVING_WAIT_STATE;
            SERIAL_TRACE("DATA_SAVING_WRITE_FILE ---> DATA_SAVING_WAIT_STATE\r\n");
         }
         // fail
         else {
            SERIAL_ERROR("SD Card writing data on file %s... [ FAIL ]\r\n", file_name);
            is_sdcard_error = true;
            data_saving_state = DATA_SAVING_CLOSE_FILE;
            SERIAL_TRACE("DATA_SAVING_WRITE_FILE ---> DATA_SAVING_CLOSE_FILE\r\n");
         }
      break;

      case DATA_SAVING_CLOSE_FILE:
            is_sdcard_error = !write_data_file.close();
            data_saving_state = DATA_SAVING_END;
            SERIAL_TRACE("DATA_SAVING_CLOSE_FILE ---> DATA_SAVING_END\r\n");
         break;

      case DATA_SAVING_END:
         SERIAL_INFO("[ %u ] data stored in sdcard... [ %s ]\r\n", sd_data_count, is_sdcard_error ? ERROR_STRING : OK_STRING);

         noInterrupts();
         if (!is_event_supervisor) {
            is_event_supervisor = true;
            ready_tasks_count++;
         }

         noInterrupts();
         is_event_data_saving = false;
         ready_tasks_count--;
         interrupts();

         data_saving_state = DATA_SAVING_INIT;
         SERIAL_TRACE("DATA_SAVING_END ---> DATA_SAVING_INIT\r\n");
      break;

      case DATA_SAVING_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            data_saving_state = state_after_wait;
         }
      break;
   }
}

void mqtt_task() {
   static uint8_t retry;
   static mqtt_state_t state_after_wait;
   static uint32_t delay_ms;
   static uint32_t start_time_ms;
   static uint8_t i;
   static uint8_t k;
   static uint16_t mqtt_data_count;
   static uint8_t data_count;
   static char sd_buffer[MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH];
   static char topic_buffer[VALUES_TO_READ_FROM_SENSOR_COUNT][MQTT_SENSOR_TOPIC_LENGTH];
   static char message_buffer[VALUES_TO_READ_FROM_SENSOR_COUNT][MQTT_MESSAGE_LENGTH];
   static char full_topic_buffer[MQTT_ROOT_TOPIC_LENGTH + MQTT_SENSOR_TOPIC_LENGTH];
   static bool is_mqtt_error;
   static bool is_mqtt_processing_sdcard;    // send data saved on sdcard
   static bool is_mqtt_processing_json;      // sd card fault fallback: send last acquired sensor data
   static bool is_eof_data_read;
   static tmElements_t datetime;
   static time_t current_ptr_time_data;
   static time_t next_ptr_time_data;
   uint8_t ipstack_status;
   char file_name[SDCARD_FILES_NAME_MAX_LENGTH];
   int read_bytes_count;

   switch (mqtt_state) {
      case MQTT_INIT:
         retry = 0;
         is_eof_data_read = false;
         mqtt_data_count = 0;
         is_mqtt_error = false;

         if (!is_sdcard_open && !is_sdcard_error) {
            mqtt_state = MQTT_OPEN_SDCARD;
            SERIAL_TRACE("MQTT_PTR_DATA_INIT ---> MQTT_OPEN_SDCARD\r\n");
         }
         else if (is_sdcard_open) {
            mqtt_state = MQTT_OPEN_PTR_FILE;
            SERIAL_TRACE("MQTT_PTR_DATA_INIT ---> MQTT_OPEN_PTR_FILE\r\n");
         }
         else {
            mqtt_state = MQTT_PTR_END;
            SERIAL_TRACE("MQTT_PTR_DATA_INIT ---> MQTT_PTR_END\r\n");
         }
      break;

      case MQTT_OPEN_SDCARD:
         if (sdcard_init(&SD, SDCARD_CHIP_SELECT_PIN)) {
            retry = 0;
            is_sdcard_open = true;
            is_sdcard_error = false;
            mqtt_state = MQTT_OPEN_PTR_FILE;
            SERIAL_TRACE("MQTT_OPEN_SDCARD ---> MQTT_OPEN_PTR_FILE\r\n");
         }
         // retry
         else if ((++retry) < MQTT_RETRY_COUNT_MAX) {
            delay_ms = MQTT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = MQTT_OPEN_SDCARD;
            mqtt_state = MQTT_WAIT_STATE;
            SERIAL_TRACE("MQTT_OPEN_SDCARD ---> MQTT_PTR_DATA_WAIT_STATE\r\n");
         }
         // fail
         else {
            is_sdcard_error = true;
            is_sdcard_open = false;
            SERIAL_ERROR("SD Card... [ FAIL ]\r\n--> is card inserted?\r\n--> there is a valid FAT32 filesystem?\r\n\r\n");

            mqtt_state = MQTT_PTR_END;
            SERIAL_TRACE("MQTT_OPEN_SDCARD ---> MQTT_PTR_END\r\n");
         }
         break;

      case MQTT_OPEN_PTR_FILE:
         // try to open file. if ok, read ptr data.
         if (sdcard_open_file(&SD, &mqtt_ptr_file, SDCARD_MQTT_PTR_FILE_NAME, O_RDWR | O_CREAT)) {
            retry = 0;
            mqtt_state = MQTT_PTR_READ;
            SERIAL_TRACE("MQTT_OPEN_PTR_FILE ---> MQTT_PTR_READ\r\n");
         }
         // retry
         else if ((++retry) < MQTT_RETRY_COUNT_MAX) {
            delay_ms = MQTT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = MQTT_OPEN_PTR_FILE;
            mqtt_state = MQTT_WAIT_STATE;
            SERIAL_TRACE("MQTT_OPEN_PTR_FILE ---> MQTT_PTR_DATA_WAIT_STATE\r\n");
         }
         // fail
         else {
            SERIAL_ERROR("SD Card open file %s... [ FAIL ]\r\n", SDCARD_MQTT_PTR_FILE_NAME);
            is_sdcard_error = true;
            mqtt_state = MQTT_PTR_END;
            SERIAL_TRACE("MQTT_OPEN_PTR_FILE ---> MQTT_PTR_END\r\n");
         }
      break;

      case MQTT_PTR_READ:
         ptr_date_time = UINT32_MAX;
         mqtt_ptr_file.seekSet(0);
         read_bytes_count = mqtt_ptr_file.read(&ptr_date_time, sizeof(time_t));

         // found
         if (read_bytes_count == sizeof(time_t) && ptr_date_time < now()) {
            is_ptr_found = true;
            mqtt_state = MQTT_PTR_FOUND;
            SERIAL_TRACE("MQTT_PTR_READ ---> MQTT_PTR_FOUND\r\n");
         }
         // not found (no sdcard error): find it by starting from 1th January of this year
         else if (read_bytes_count >= 0) {
            SERIAL_INFO("Data pointer... [ FIND ]\r\n");
            datetime.Year = CalendarYrToTm(year(now()));
            datetime.Month = 1;
            datetime.Day = 1;
            datetime.Hour = 0;
            datetime.Minute = 0;
            datetime.Second = 0;
            ptr_date_time = makeTime(datetime);
            is_ptr_found = false;
            mqtt_state = MQTT_PTR_FIND;
            SERIAL_TRACE("MQTT_PTR_READ ---> MQTT_PTR_FIND\r\n");
         }
         // not found (sdcard error)
         else {
            is_ptr_found = false;
            is_sdcard_error = true;
            mqtt_state = MQTT_PTR_END;
            SERIAL_TRACE("MQTT_PTR_READ ---> MQTT_PTR_END\r\n");
         }
      break;

      case MQTT_PTR_FIND:
         // ptr not found. find it by searching in file name until today is reach.
         // if there isn't file, ptr_date_time is set to current date at 00:00:00 time.
         if (!is_ptr_found && ptr_date_time < now()) {
            sdcard_make_filename(ptr_date_time, file_name);

            if (SD.exists(file_name)) {
               is_ptr_found = true;
               is_ptr_updated = true;
               is_eof_data_read = false;
               SERIAL_INFO("%s... [ FOUND ]\r\n", file_name);
               mqtt_state = MQTT_PTR_END;
               SERIAL_TRACE("MQTT_PTR_FOUND ---> MQTT_PTR_END\r\n");
            }
            else {
               SERIAL_INFO("%s... [ NOT FOUND ]\r\n", file_name);
               ptr_date_time += SECS_PER_DAY;
            }
         }
         // ptr not found: set ptr to yesterday at 23:60-REPORT_MINUTES:00 time.
         else if (!is_ptr_found && ptr_date_time >= now()) {
            datetime.Year = CalendarYrToTm(year());
            datetime.Month = month();
            datetime.Day = day();
            // datetime.Hour = 23;
            // datetime.Minute = 60 - REPORT_MINUTES;
            datetime.Hour = 0;
            datetime.Minute = 0;
            datetime.Second = 0;
            ptr_date_time = makeTime(datetime);
            is_ptr_found = true;
            is_ptr_updated = true;
         }
         // ptr found: sooner or later the ptr will be set in any case
         else if (is_ptr_found) {
            mqtt_state = MQTT_PTR_FOUND;
            SERIAL_TRACE("MQTT_PTR_FIND ---> MQTT_PTR_FOUND\r\n");
         }
      break;

      case MQTT_PTR_FOUND:
         // datafile read, reach eof and is today. END.
         if (is_eof_data_read && year() == year(ptr_date_time) && month() == month(ptr_date_time) && day() == day(ptr_date_time)) {
            mqtt_state = MQTT_CLOSE_DATA_FILE;
            SERIAL_TRACE("MQTT_PTR_FOUND ---> MQTT_CLOSE_DATA_FILE\r\n");
         }
         // datafile read, reach eof and NOT is today. go to end of this day.
         else if (is_eof_data_read) {
            datetime.Year = CalendarYrToTm(year(ptr_date_time));
            datetime.Month = month(ptr_date_time);
            datetime.Day = day(ptr_date_time);
            datetime.Hour = 23;
            datetime.Minute = 60 - REPORT_MINUTES;
            datetime.Second = 0;
            ptr_date_time = makeTime(datetime);
            is_ptr_updated = true;
            mqtt_state = MQTT_PTR_END;
            SERIAL_TRACE("MQTT_PTR_FOUND ---> MQTT_PTR_END\r\n");
         }
         else {
            // set ptr 1 second more for send next data to current ptr
            if (second(ptr_date_time) == 0) {
               ptr_date_time++;
            }

            is_eof_data_read = false;
            is_ptr_updated = true;
            mqtt_state = MQTT_PTR_END;
            SERIAL_TRACE("MQTT_PTR_FOUND ---> MQTT_PTR_END\r\n");
         }
      break;

      case MQTT_PTR_END:
         // ptr data is found: send data saved on sdcard
         if (is_ptr_found && is_sdcard_open && !is_sdcard_error) {
            SERIAL_INFO("Data pointer... [ %02u/%02u/%04u %02u:%02u:%02u ] [ %s ]\r\n", day(ptr_date_time), month(ptr_date_time), year(ptr_date_time), hour(ptr_date_time), minute(ptr_date_time), second(ptr_date_time), OK_STRING);
         }
         // ptr data is NOT found: sd card fault fallback: send last acquired sensor data
         else {
            SERIAL_INFO("Data pointer... [ --/--/---- --:--:-- ] [ %s ]\r\n", ERROR_STRING);
         }

         mqtt_state = MQTT_OPEN;
         SERIAL_TRACE("MQTT_PTR_END ---> MQTT_OPEN\r\n");
      break;

      case MQTT_OPEN:
         if (is_client_connected && mqtt_client.isConnected()) {
            mqtt_state = MQTT_CHECK;
            SERIAL_TRACE("MQTT_OPEN ---> MQTT_CHECK\r\n");
         }
         else if (is_client_connected) {
            mqtt_state = MQTT_CONNECT;
            SERIAL_TRACE("MQTT_OPEN ---> MQTT_CONNECT\r\n");
         }
         else {
            mqtt_state = MQTT_END;
            SERIAL_TRACE("MQTT_OPEN ---> MQTT_END\r\n");
         }
         break;

      case MQTT_CONNECT:
         ipstack_status = ipstack.connect(configuration.mqtt_server, configuration.mqtt_port);

         // success
         if (ipstack_status == 1 && mqttConnect(configuration.mqtt_username, configuration.mqtt_password)) {
            retry = 0;
            SERIAL_DEBUG("MQTT Connection... [ %s ]\r\n", OK_STRING);
            mqtt_state = MQTT_SUBSCRIBE;
            SERIAL_TRACE("MQTT_CONNECT ---> MQTT_SUBSCRIBE\r\n");
         }
         // retry
         else if (ipstack_status == 2 && (++retry) < MQTT_RETRY_COUNT_MAX) {
            delay_ms = MQTT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = MQTT_CONNECT;
            mqtt_state = MQTT_WAIT_STATE;
            SERIAL_TRACE("MQTT_CONNECT ---> MQTT_WAIT_STATE\r\n");
         }
         // fail
         else if (ipstack_status == 2) {
            SERIAL_ERROR("MQTT Connection... [ %s ]\r\n", FAIL_STRING);
            is_mqtt_error = true;
            mqtt_state = MQTT_DISCONNECT;
            SERIAL_TRACE("MQTT_CONNECT ---> MQTT_DISCONNECT\r\n");
         }
         // wait
      break;

      case MQTT_SUBSCRIBE:
         if (!is_mqtt_subscribed) {
            is_mqtt_subscribed = (mqtt_client.subscribe(configuration.mqtt_subscribe_topic, MQTT::QOS1, mqttRxCallback) == 0);
            is_mqtt_error = !is_mqtt_subscribed;
            SERIAL_DEBUG("MQTT Subscription... [ %s ]\r\n", is_mqtt_subscribed ? OK_STRING : FAIL_STRING);
         }

         mqtt_state = MQTT_CHECK;
         SERIAL_TRACE("MQTT_SUBSCRIBE ---> MQTT_CHECK\r\n");
      break;

      case MQTT_CHECK:
         // ptr data is found: send data saved on sdcard
         if (!is_sdcard_error) {
            is_mqtt_processing_json = false;
            is_mqtt_processing_sdcard = true;
            is_eof_data_read = false;
            mqtt_state = MQTT_OPEN_DATA_FILE;
            SERIAL_TRACE("MQTT_CHECK ---> MQTT_OPEN_DATA_FILE\r\n");
         }
         // ptr data is NOT found: sd card fault fallback: send last acquired sensor data
         else {
            is_mqtt_processing_json = true;
            is_mqtt_processing_sdcard = false;
            i = 0;
            mqtt_state = MQTT_SENSORS_LOOP;
            SERIAL_TRACE("MQTT_CHECK ---> MQTT_SENSORS_LOOP\r\n");
         }
      break;

      case MQTT_SENSORS_LOOP:
         if (i < sensors_count) {
            k = 0;
            data_count = jsonToMqtt(&json_sensors_data[i][0], configuration.sensors[i].mqtt_topic, topic_buffer, message_buffer, (tmElements_t *) &sensor_reading_time);
            mqtt_state = MQTT_DATA_LOOP;
            SERIAL_TRACE("MQTT_SENSORS_LOOP ---> MQTT_DATA_LOOP\r\n");
         }
         else if (is_mqtt_processing_json) {
            mqtt_state = MQTT_DISCONNECT;
            SERIAL_TRACE("MQTT_SENSORS_LOOP ---> MQTT_DISCONNECT\r\n");
         }
      break;

      case MQTT_SD_LOOP:
         memset(sd_buffer, 0, MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH);
         memset(topic_buffer, 0, sizeof(topic_buffer[0][0]) * VALUES_TO_READ_FROM_SENSOR_COUNT * MQTT_SENSOR_TOPIC_LENGTH);
         memset(message_buffer, 0, sizeof(message_buffer[0][0]) * VALUES_TO_READ_FROM_SENSOR_COUNT * MQTT_MESSAGE_LENGTH);

         read_bytes_count = read_data_file.read(sd_buffer, MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH);

         if (read_bytes_count == MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH) {
            sdToMqtt(sd_buffer, &topic_buffer[0][0], &message_buffer[0][0]);
            current_ptr_time_data = getDateFromMessage(&message_buffer[0][0]);

            if (current_ptr_time_data >= ptr_date_time) {
               mqtt_state = MQTT_DATA_LOOP;
               SERIAL_TRACE("MQTT_SD_LOOP ---> MQTT_DATA_LOOP\r\n");
            }
         }
         // EOF: End of File
         else {
            is_eof_data_read = true;
            mqtt_state = MQTT_PTR_FOUND;
            SERIAL_TRACE("MQTT_SD_LOOP ---> MQTT_PTR_FOUND\r\n");
         }
      break;

      case MQTT_DATA_LOOP:
         if ((k < data_count && is_mqtt_processing_json) || is_mqtt_processing_sdcard) {
            getFullTopic(full_topic_buffer, configuration.mqtt_root_topic, &topic_buffer[k][0]);
            mqtt_state = MQTT_PUBLISH;
            SERIAL_TRACE("MQTT_DATA_LOOP ---> MQTT_PUBLISH\r\n");
         }
         else {
            i++;
            mqtt_state = MQTT_SENSORS_LOOP;
            SERIAL_TRACE("MQTT_DATA_LOOP ---> MQTT_SENSORS_LOOP\r\n");
         }
      break;

      case MQTT_PUBLISH:
         // mqtt json success
         if (is_mqtt_processing_json && mqttPublish(full_topic_buffer, &message_buffer[k][0])) {
            SERIAL_DEBUG("MQTT <-- %s %s\r\n", &topic_buffer[k][0], &message_buffer[k][0]);
            retry = 0;
            k++;
            mqtt_data_count++;
            mqtt_state = MQTT_SENSORS_LOOP;
            SERIAL_TRACE("MQTT_PUBLISH ---> MQTT_SENSORS_LOOP\r\n");
         }
         // mqtt sdcard success
         else if (is_mqtt_processing_sdcard && mqttPublish(full_topic_buffer, &message_buffer[0][0])) {
            SERIAL_DEBUG("MQTT <-- %s %s\r\n", &topic_buffer[0][0], &message_buffer[0][0]);
            retry = 0;
            mqtt_data_count++;
            ptr_date_time = current_ptr_time_data;
            is_ptr_updated = true;
            mqtt_state = MQTT_SD_LOOP;
            SERIAL_TRACE("MQTT_PUBLISH ---> MQTT_SD_LOOP\r\n");
         }
         // retry
         else if ((++retry) < MQTT_RETRY_COUNT_MAX) {
            delay_ms = MQTT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = MQTT_PUBLISH;
            mqtt_state = MQTT_WAIT_STATE;
            SERIAL_TRACE("MQTT_PUBLISH ---> MQTT_WAIT_STATE\r\n");
         }
         // fail
         else {
            is_eof_data_read = true;
            is_mqtt_error = true;
            SERIAL_ERROR("MQTT publish... [ %s ]\r\n", FAIL_STRING);

            if (is_mqtt_processing_json) {
               mqtt_state = MQTT_DISCONNECT;
               SERIAL_TRACE("MQTT_PUBLISH ---> MQTT_DISCONNECT\r\n");
            }
            else if (is_mqtt_processing_sdcard) {
               mqtt_state = MQTT_CLOSE_DATA_FILE;
               SERIAL_TRACE("MQTT_PUBLISH ---> MQTT_CLOSE_DATA_FILE\r\n");
            }
         }
      break;

      case MQTT_OPEN_DATA_FILE:
         // open the file that corresponds to the next data to send
         next_ptr_time_data = ptr_date_time + REPORT_MINUTES * 60;
         sdcard_make_filename(next_ptr_time_data, file_name);

         // open file for read data
         if (sdcard_open_file(&SD, &read_data_file, file_name, O_READ)) {
            retry = 0;
            mqtt_state = MQTT_SD_LOOP;
            SERIAL_TRACE("MQTT_OPEN_DATA_FILE ---> MQTT_SD_LOOP\r\n");
         }
         // error: file doesn't exist but if is today, end.
         else if (!is_sdcard_error && year(next_ptr_time_data) == year() && month(next_ptr_time_data) == month() && day(next_ptr_time_data) == day()) {
            mqtt_state = MQTT_PTR_UPDATE;
            SERIAL_TRACE("MQTT_OPEN_DATA_FILE ---> MQTT_PTR_UPDATE\r\n");
         }
         // error: file doesn't exist and if it isn't today, jump to next day and search in it
         else if (!is_sdcard_error) {
            is_ptr_found = false;
            ptr_date_time = next_ptr_time_data;
            mqtt_state = MQTT_PTR_FIND;
            SERIAL_TRACE("MQTT_OPEN_DATA_FILE ---> MQTT_PTR_FIND\r\n");
         }
         // fail
         else {
            SERIAL_ERROR("SD Card open file %s... [ FAIL ]\r\n", file_name);
            is_sdcard_error = true;
            mqtt_state = MQTT_CHECK; // fallback
            SERIAL_TRACE("MQTT_OPEN_DATA_FILE ---> MQTT_CHECK\r\n");
         }
         break;

      case MQTT_CLOSE_DATA_FILE:
         if (is_mqtt_processing_sdcard) {
            is_sdcard_error = !read_data_file.close();
            mqtt_state = MQTT_DISCONNECT;
            SERIAL_TRACE("MQTT_OPEN_DATA_FILE ---> MQTT_DISCONNECT\r\n");
         }
         break;

      case MQTT_DISCONNECT:
         #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
         mqtt_client.disconnect();
         ipstack.disconnect();
         SERIAL_DEBUG("MQTT Disconnect... [ %s ]\r\n", OK_STRING);

         // resume GSM task for closing connection
         noInterrupts();
         if (!is_event_gsm) {
            is_event_gsm = true;
            ready_tasks_count++;
         }
         interrupts();
         #endif

         mqtt_state = MQTT_PTR_UPDATE;
         SERIAL_TRACE("MQTT_DISCONNECT ---> MQTT_PTR_UPDATE\r\n");
         break;

      case MQTT_PTR_UPDATE:
         // if (is_ptr_updated_ && ptr_date_time < now()) {
         if (is_ptr_updated) {
            // success
            if (mqtt_ptr_file.seekSet(0) && mqtt_ptr_file.write(&ptr_date_time, sizeof(time_t)) == sizeof(time_t)) {
               mqtt_ptr_file.flush();
               breakTime(ptr_date_time, datetime);
               SERIAL_INFO("Data pointer... [ %02u/%02u/%04u %02u:%02u:%02u ] [ %s ]\r\n", datetime.Day, datetime.Month, tmYearToCalendar(datetime.Year), datetime.Hour, datetime.Minute, datetime.Second, "UPDATE");
               mqtt_state = MQTT_CLOSE_PTR_FILE;
               SERIAL_TRACE("MQTT_PTR_UPDATE ---> MQTT_CLOSE_PTR_FILE\r\n");
            }
            // retry
            else if ((++retry) < MQTT_RETRY_COUNT_MAX) {
               delay_ms = MQTT_DELAY_MS;
               start_time_ms = millis();
               state_after_wait = MQTT_PTR_UPDATE;
               mqtt_state = MQTT_WAIT_STATE;
               SERIAL_TRACE("MQTT_PTR_UPDATE ---> MQTT_WAIT_STATE\r\n");
            }
            // fail
            else {
               SERIAL_ERROR("SD Card writing ptr data on file %s... [ %s ]\r\n", SDCARD_MQTT_PTR_FILE_NAME, FAIL_STRING);
               mqtt_state = MQTT_CLOSE_PTR_FILE;
               SERIAL_TRACE("MQTT_PTR_UPDATE ---> MQTT_CLOSE_PTR_FILE\r\n");
            }
         }
         else {
            mqtt_state = MQTT_CLOSE_PTR_FILE;
            SERIAL_TRACE("MQTT_PTR_UPDATE ---> MQTT_CLOSE_PTR_FILE\r\n");
         }
         break;

      case MQTT_CLOSE_PTR_FILE:
         mqtt_ptr_file.close();
         mqtt_state = MQTT_CLOSE_SDCARD;
         SERIAL_TRACE("MQTT_PTR_UPDATE ---> MQTT_CLOSE_SDCARD\r\n");
         break;

      case MQTT_CLOSE_SDCARD:
         is_sdcard_error = false;
         is_sdcard_open = false;
         mqtt_state = MQTT_END;
         SERIAL_TRACE("MQTT_PTR_UPDATE ---> DATA_PROCESSING_END\r\n");
         break;

      case MQTT_END:
         SERIAL_INFO("[ %u ] data send through mqtt... [ %s ]\r\n", mqtt_data_count, is_mqtt_error ? ERROR_STRING : OK_STRING);

         noInterrupts();
         is_event_mqtt = false;
         ready_tasks_count--;
         interrupts();

         mqtt_state = MQTT_INIT;
         SERIAL_TRACE("MQTT_END ---> MQTT_INIT\r\n");
      break;

      case MQTT_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            mqtt_state = state_after_wait;
         }
      break;
   }
}
