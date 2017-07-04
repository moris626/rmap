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
Debug level for sketch and library.
*/
#define SERIAL_TRACE_LEVEL RMAP_SERIAL_TRACE_LEVEL

#include "rmap.h"

void setup() {
  init_system();
}

void loop() {
  switch (state) {
    case INIT:
      init_buffers();
      init_tasks();
      init_pins();
      load_configuration();
      init_wire();
      init_spi();
      init_rtc();
      init_sensors();
      state = TASKS_EXECUTION;
      wdt_timer.interrupt_count = WDT_INTERRUPT_COUNT_DEFAULT;
    break;

    #if (USE_POWER_DOWN)
    case ENTER_POWER_DOWN:
      #if (USE_WDT_TO_WAKE_UP_FROM_SLEEP == false)
      wdt_disable();
      #endif

      power_down(&awakened_event_occurred_time_ms);

      #if (USE_WDT_TO_WAKE_UP_FROM_SLEEP == false)
      wdt_init(WDT_TIMER);
      #endif

      state = TASKS_EXECUTION;
      wdt_timer.interrupt_count = WDT_INTERRUPT_COUNT_DEFAULT;
      break
    #endif

    case TASKS_EXECUTION:
      if (is_event_sensors_reading)
        sensors_reading_task();

      #if (USE_WDT_TASK)
      if (is_event_wdt)
        wdt_task();
      #endif

      #if (USE_RTC_TASK)
      if (is_event_rtc)
        rtc_task();
      #endif

      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
      if (is_event_ethernet)
        ethernet_task();
      #endif

      // if (is_event_sd)
        // sd_task();

      if (is_event_time)
        time_task();

      if (ready_tasks_count == 0)
        state = END;

      wdt_timer.interrupt_count = WDT_INTERRUPT_COUNT_DEFAULT;
      break;

    case END:
      #if (USE_POWER_DOWN)
      state = ENTER_POWER_DOWN;
      #else
      state = TASKS_EXECUTION;
      #endif
      wdt_timer.interrupt_count = WDT_INTERRUPT_COUNT_DEFAULT;
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

  is_event_sensors_reading = false;

  is_event_time = true;
  ready_tasks_count++;

  // is_event_sd = true;
  // ready_tasks_count++;

  #if (USE_WDT_TASK)
  is_event_wdt = false;
  #endif

  #if (USE_RTC_TASK)
  is_event_rtc = false;
  #endif

  #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
  is_event_ethernet = true;
  ready_tasks_count++;
  #endif

  #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
  ethernet_task_attempt_occurred_time_ms = ETHERNET_ATTEMP_MS+1;
  #endif

  next_minute_for_sensor_reading = 255;
  time_state = INIT_TIME;
  sd_state = INIT_SD;

  interrupts();
}

void init_pins() {
  pinMode(CONFIGURATION_RESET_PIN, INPUT_PULLUP);

  pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), rtc_interrupt_handler, FALLING);

  pinMode(SDCARD_CHIP_SELECT_PIN, OUTPUT);
  digitalWrite(SDCARD_CHIP_SELECT_PIN, HIGH);

  #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
  Ethernet.w5500_cspin = W5500_CHIP_SELECT_PIN;
  #endif
}

void init_wire() {
  Wire.begin();
  Wire.setClock(I2C_BUS_CLOCK);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
}

void init_spi() {
  SPI.begin();
}

void init_rtc() {
  Pcf8563::disableAlarm();
  Pcf8563::disableTimer();
}

void init_system() {
  wdt_disable();
  TRACE_BEGIN(230400);
  wdt_timer.value = 0;
  wdt_timer.interrupt_count = WDT_INTERRUPT_COUNT_DEFAULT;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  awakened_event_occurred_time_ms = millis();
  wdt_init(WDT_TIMER);
  state = INIT;
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
  #endif

  SERIAL_INFO("--> mqtt server: %s\r\n", configuration.mqtt_server);
  SERIAL_INFO("--> mqtt root path: %s\r\n", configuration.mqtt_root_path);
  SERIAL_INFO("--> mqtt username: %s\r\n", configuration.mqtt_username);
  SERIAL_INFO("--> mqtt password: %s\r\n", configuration.mqtt_password);
  SERIAL_INFO("--> ntp server: %s\r\n\r\n", configuration.ntp_server);
}

void save_configuration(bool is_default) {
  char temp_string[20];
  if (is_default) {
    SERIAL_INFO("Save default configuration... [ OK ]\r\n");
    configuration.module_type = MODULE_TYPE;
    configuration.module_version = MODULE_VERSION;

    uint8_t i=0;

    strcpy(configuration.sensors[i].driver, SENSOR_DRIVER_I2C);
    strcpy(configuration.sensors[i].type, SENSOR_TYPE_TBS);
    strcpy(configuration.sensors[i].mqtt_path, "1,0,900/1,-,-,-/");
    configuration.sensors[i].address = CONFIGURATION_DEFAULT_RAIN_ADDRESS;
    i++;

    strcpy(configuration.sensors[i].driver, SENSOR_DRIVER_I2C);
    strcpy(configuration.sensors[i].type, SENSOR_TYPE_ITH);
    strcpy(configuration.sensors[i].mqtt_path, "254,0,0/103,2000,-,-/");
    configuration.sensors[i].address = CONFIGURATION_DEFAULT_TH_ADDRESS;
    i++;

    strcpy(configuration.sensors[i].driver, SENSOR_DRIVER_I2C);
    strcpy(configuration.sensors[i].type, SENSOR_TYPE_NTH);
    strcpy(configuration.sensors[i].mqtt_path, "3,0,0/103,2000,-,-/");
    configuration.sensors[i].address = CONFIGURATION_DEFAULT_TH_ADDRESS;
    i++;

    strcpy(configuration.sensors[i].driver, SENSOR_DRIVER_I2C);
    strcpy(configuration.sensors[i].type, SENSOR_TYPE_MTH);
    strcpy(configuration.sensors[i].mqtt_path, "0,0,0/103,2000,-,-/");
    configuration.sensors[i].address = CONFIGURATION_DEFAULT_TH_ADDRESS;
    i++;

    strcpy(configuration.sensors[i].driver, SENSOR_DRIVER_I2C);
    strcpy(configuration.sensors[i].type, SENSOR_TYPE_XTH);
    strcpy(configuration.sensors[i].mqtt_path, "2,0,0/103,2000,-,-/");
    configuration.sensors[i].address = CONFIGURATION_DEFAULT_TH_ADDRESS;
    i++;


    #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
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
    #endif

    strcpy(configuration.mqtt_server, CONFIGURATION_DEFAULT_MQTT_SERVER);
    strcpy(configuration.mqtt_root_path, CONFIGURATION_DEFAULT_MQTT_ROOT_PATH);
    strcpy(configuration.mqtt_username, CONFIGURATION_DEFAULT_MQTT_USERNAME);
    strcpy(configuration.mqtt_password, CONFIGURATION_DEFAULT_MQTT_PASSWORD);
    strcpy(configuration.ntp_server, CONFIGURATION_DEFAULT_NTP_SERVER);
  }
  else {
    SERIAL_INFO("Save configuration... [ OK ]\r\n");

    #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
    configuration.is_dhcp_enable = writable_data.is_dhcp_enable;
    #endif

    strcpy(configuration.mqtt_server, writable_data.mqtt_server);
    strcpy(configuration.mqtt_root_path, writable_data.mqtt_root_path);
    strcpy(configuration.mqtt_username, writable_data.mqtt_username);
    strcpy(configuration.mqtt_password, writable_data.mqtt_password);
    strcpy(configuration.ntp_server, writable_data.ntp_server);
  }

  ee_write(&configuration, CONFIGURATION_EEPROM_ADDRESS, sizeof(configuration));

  print_configuration();
}

void load_configuration() {
  ee_read(&configuration, CONFIGURATION_EEPROM_ADDRESS, sizeof(configuration));

  if (configuration.module_type != MODULE_TYPE || configuration.module_version != MODULE_VERSION || digitalRead(CONFIGURATION_RESET_PIN) == LOW) {
    save_configuration(CONFIGURATION_DEFAULT);
  }
  else {
    SERIAL_INFO("Load configuration... [ OK ]\r\n");
    print_configuration();
  }
}

void init_sensors () {
  is_first_run = true;
  sensors_count = 0;

  SERIAL_INFO("Sensors...\r\n");

  for (uint8_t i=0; i<USE_SENSORS_COUNT; i++) {
    SensorDriver::createAndSetup(configuration.sensors[i].driver, configuration.sensors[i].type, configuration.sensors[i].address, sensors, &sensors_count);
    SERIAL_INFO("--> %u: %s-%s: %s\t", i+1, configuration.sensors[i].driver, configuration.sensors[i].type, configuration.sensors[i].mqtt_path);

    if (sensors[i]->isSetted()) {
      SERIAL_INFO("[ OK ]\r\n");
    }
    else {
      SERIAL_INFO("[ FAIL ]\r\n");
    }
  }

  SERIAL_INFO("\r\n");
}

void rtc_interrupt_handler() {
  #if (USE_RTC_TASK)
  noInterrupts();
  if (!is_event_rtc) {
    is_event_rtc = true;
    ready_tasks_count++;
  }
  interrupts();
  #endif
}

ISR(WDT_vect) {
  wdt_timer.interrupt_count--;

  if (wdt_timer.interrupt_count == 0) {
    wdt_disable();
    wdt_reset();
    wdt_enable(WDTO_15MS);
    while(1);
  }

  if (wdt_timer.value == WDT_TIMER_MAX_VALUE)
  wdt_timer.value = 0;

  wdt_timer.value += WDT_OFFSET;

  #if (USE_WDT_TASK)
  noInterrupts();
  if (!is_event_wdt) {
    is_event_wdt = true;
    ready_tasks_count++;
  }
  interrupts();
  #endif
}

#if (USE_WDT_TASK)
void wdt_task() {
  noInterrupts();
  is_event_wdt = false;
  ready_tasks_count--;
  interrupts();
}
#endif

#if (USE_RTC_TASK)
void rtc_task() {

  if (Pcf8563::isAlarmActive())
    Pcf8563::disableAlarm();

  if (Pcf8563::isTimerActive())
    Pcf8563::resetTimer();

  if (second() == 0) {
    Pcf8563::enableTimer();

    if (minute() == next_minute_for_sensor_reading) {
      SERIAL_TRACE("Doing sensor reading...\r\n");
      sensor_reading_day = day();
      sensor_reading_month = month();
      sensor_reading_year = year(),
      sensor_reading_hour = hour();
      sensor_reading_minute = minute();

      sensor_state = INIT_SENSOR;

      noInterrupts();
      if (!is_event_sensors_reading) {
        is_event_sensors_reading = true;
        ready_tasks_count++;
      }
      interrupts();
    }

    setNextTimeForSensorReading(&next_hour_for_sensor_reading, &next_minute_for_sensor_reading);

    noInterrupts();
    is_event_rtc = false;
    ready_tasks_count--;
    interrupts();
  }
}
#endif

void time_task() {
  static uint8_t retry;
  static time_state_t state_after_wait;
  static uint32_t delay_ms;
  static uint32_t start_time_ms;

  switch (time_state) {
    case INIT_TIME:
      retry = 0;
      state_after_wait = INIT_TIME;
      time_state = SEND_REQUEST;
      break;

    case SEND_REQUEST:
      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
      //while (ethernetUdpClient.parsePacket() > 0);
      // success
      if (Ntp::sendRequest(&ethernetUdpClient, configuration.ntp_server)) {
        retry = 0;
        time_state = WAIT_RESPONSE;
      }
      // retry
      else if (++retry < NTP_RETRY_MAX_COUNT) {
        delay_ms = NTP_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = SEND_REQUEST;
        time_state = WAIT_NTP_STATE;
      }
      // fail
      else time_state = END_TIME;
      #endif
      break;

    case WAIT_RESPONSE:
      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
      // success
      if (Ntp::getResponse(&ethernetUdpClient)) {
        time_state = SET_SYNC_PROVIDER;
      }
      // retry
      else if (++retry < NTP_RETRY_MAX_COUNT) {
        delay_ms = NTP_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = WAIT_RESPONSE;
        time_state = WAIT_NTP_STATE;
      }
      // fail
      else time_state = END_TIME;
      #endif
      break;

    case SET_SYNC_PROVIDER:
      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
      setSyncProvider(Ntp::getTime);
      SERIAL_TRACE("Current NTP date and time: %02u/%02u/%04u %02u:%02u:%02u\r\n", day(), month(), year(), hour(), minute(), second());
      #endif

      time_state = SET_RTC_TIME;
      break;

    case SET_RTC_TIME:
      Pcf8563::reset();
      Pcf8563::setDate(day(), month(), year()-2000, weekday()-1, 0);
      Pcf8563::setTime(hour(), minute(), second());
      SERIAL_TRACE("Current RTC date and time: %02u/%02u/%04u %02u:%02u:%02u\r\n", day(), month(), year(), hour(), minute(), second());
      time_state = END_TIME;
      break;

    case END_TIME:
      setSyncProvider(Pcf8563::getTime);
      setNextTimeForSensorReading(&next_hour_for_sensor_reading, &next_minute_for_sensor_reading);
      Pcf8563::setAlarm(PCF8563_ALARM_DISABLE, next_minute_for_sensor_reading);
      Pcf8563::enableAlarm();
      Pcf8563::setTimer(RTC_FREQUENCY, RTC_TIMER);

      SERIAL_INFO("Current date and time is: %02u/%02u/%04u %02u:%02u:%02u\r\n\r\n", day(), month(), year(), hour(), minute(), second());

      SERIAL_INFO("Acquisition scheduling...\r\n");
      SERIAL_INFO("--> observations every %u minutes\r\n", OBSERVATIONS_MINUTES);
      SERIAL_INFO("--> report every %u minutes\r\n", REPORT_MINUTES);
      SERIAL_INFO("--> starting at: %02u:%02u:00\r\n\r\n", next_hour_for_sensor_reading, next_minute_for_sensor_reading);

      noInterrupts();
      is_event_time = false;
      ready_tasks_count--;
      interrupts();

      time_state = INIT_TIME;
      break;

    case WAIT_NTP_STATE:
      if (millis() - start_time_ms > delay_ms) {
        time_state = state_after_wait;
      }
      break;
  }
}

void setNextTimeForSensorReading (uint8_t *next_hour, uint8_t *next_minute) {
  bool is_update = false;
  *next_hour = hour();

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

    SERIAL_TRACE("Next acquisition scheduled at: %02u:%02u:00\r\n", *next_hour, *next_minute);
  }
}

#if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
void ethernet_task() {
  if (millis() - ethernet_task_attempt_occurred_time_ms > ETHERNET_ATTEMP_MS) {
    ethernet_task_attempt_occurred_time_ms = millis();

    bool is_ethernet_connected = false;

    if (configuration.is_dhcp_enable) {
      if (Ethernet.begin(configuration.ethernet_mac)) {
        is_ethernet_connected = true;
        SERIAL_INFO("Ethernet: DHCP [ OK ]\r\n");
      }
      else {
        SERIAL_ERROR("Ethernet: DHCP [ FAIL ]\r\n");
      }
    }

    if (!configuration.is_dhcp_enable) {
      Ethernet.begin(configuration.ethernet_mac, IPAddress(configuration.ip), IPAddress(configuration.primary_dns), IPAddress(configuration.gateway), IPAddress(configuration.netmask));
      is_ethernet_connected = true;
      SERIAL_INFO("Ethernet: Static [ OK ]\r\n");
    }

    if (is_ethernet_connected) {
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

      if (ethernetUdpClient.begin(ETHERNET_DEFAULT_LOCAL_UDP_PORT)) {
        SERIAL_TRACE("--> udp socket local port: %u [ OK ]\r\n", ETHERNET_DEFAULT_LOCAL_UDP_PORT);
      }
      else {
        SERIAL_ERROR("--> udp socket local port: %u [ FAIL ]\r\n", ETHERNET_DEFAULT_LOCAL_UDP_PORT);
      }

      SERIAL_INFO("\r\n");

      noInterrupts();
      is_event_ethernet = false;
      ready_tasks_count--;
      interrupts();
    }
  }
}
#endif

// void sd_task() {
//   static bool is_success;
//   static uint8_t i;
//   static uint8_t retry;
//   static uint32_t delay_ms;
//   static uint32_t start_time_ms;
//   static sd_state_t state_after_wait;
//
//   switch (sd_state) {
//     case INIT_SD:
//       i = 0;
//       retry = 0;
//       is_success = true;
//       strcpy(&files_name[i++][0], SDCARD_PTR_DATA_FILE_NAME);
//       snprintf(&files_name[i++][0], SDCARD_FILES_NAME_MAX_LENGTH, "%04u_%02u_%02u.txt", 2017, 7, 3);
//       // strcpy(&files_name[i++], SDCARD_PTR_DATA_FILE_NAME);
//       i = 0;
//       sd_state = OPEN_SD;
//       break;
//
//     case OPEN_SD:
//       // success
//       if (SD.begin(SDCARD_CHIP_SELECT_PIN)) {
//
//         if (SD.vol()->fatType() == 0) {
//           SERIAL_INFO("SD Card... [ FAIL ]\r\n--> Can't find a valid FAT16/FAT32 partition.\r\n\r\n");
//           is_success = false;
//           sd_state = END_SD;
//         }
//         else {
//           SERIAL_INFO("SD Card... [ OK ]\r\n--> Found %u MB card.\r\n\r\n", (uint32_t)(0.000512 * SD.card()->cardSize()));
//         }
//
//         retry = 0;
//         sd_state = OPEN_FILES;
//       }
//       // retry
//       else if (++retry < SDCARD_RETRY_MAX_COUNT) {
//         delay_ms = SDCARD_RETRY_DELAY_MS;
//         start_time_ms = millis();
//         state_after_wait = OPEN_SD;
//         sd_state = WAIT_SD_STATE;
//       }
//       // fail
//       else {
//         SERIAL_ERROR("SD Card... [ FAIL ]\r\n---> is card inserted?\r\n\r\n");
//         is_success = false;
//         sd_state = END_SD;
//       }
//       break;
//
//     case OPEN_FILES:
//       files[i] = SD.open(files_name[i], FILE_WRITE);
//
//       // success
//       if (files[i]) {
//         SERIAL_INFO("Open file %s... [ OK ]\r\n\r\n", files_name[i]);
//         // files[i].println("prova....");
//         // files[i].flush();
//         retry = 0;
//
//         if (++i >= SDCARD_FILES_COUNT)
//           sd_state = END_SD;
//       }
//       // retry
//       else if (++retry < SDCARD_RETRY_MAX_COUNT) {
//         delay_ms = SDCARD_RETRY_DELAY_MS;
//         start_time_ms = millis();
//         state_after_wait = OPEN_FILES;
//         sd_state = WAIT_SD_STATE;
//       }
//       // fail
//       else {
//         SERIAL_INFO("Open file %s... [ FAIL ]\r\n\r\n", files_name[i]);
//         retry = 0;
//
//         if (++i >= SDCARD_FILES_COUNT)
//           sd_state = END_SD;
//       }
//       break;
//
//     case END_SD:
//       if (is_success) {
//         noInterrupts();
//         is_event_sd = false;
//         ready_tasks_count--;
//         interrupts();
//       }
//       else {
//         delay_ms = SDCARD_RETRY_DELAY_MS*10;
//         start_time_ms = millis();
//         state_after_wait = INIT_SD;
//         sd_state = WAIT_SD_STATE;
//       }
//       break;
//
//     case WAIT_SD_STATE:
//       if (millis() - start_time_ms > delay_ms) {
//         sd_state = state_after_wait;
//       }
//       break;
//   }
// }

// char file_name[SDCARD_FILES_NAME_MAX_LENGTH];
// case OPEN_SDCARD_FILES:
//   if (SD.begin(SDCARD_CHIP_SELECT_PIN) && SD.vol()->fatType()) {
//     snprintf(file_name, SDCARD_FILES_NAME_MAX_LENGTH, "%04u_%02u_%02u.txt", year(), month(), day());
//     data_file = SD.open(file_name, FILE_WRITE);
//     ptr_data_file = SD.open(SDCARD_PTR_DATA_FILE_NAME, FILE_WRITE);
//
//     if (data_file && ptr_data_file) {
//       sensor_state = PREPARE_SENSOR;
//     }
//   }
//   break;

void sensors_reading_task () {
  static uint8_t i;
  static uint8_t retry;
  static sensor_state_t state_after_wait;
  static uint32_t delay_ms;
  static uint32_t start_time_ms;
  static int32_t values_readed_from_sensor[2];
  static char json_string[JSON_BUFFER_LENGTH];
  char data_string[VALUES_TO_READ_FROM_SENSOR_COUNT][MQTT_ROOT_PATH_LENGTH + MQTT_SENSOR_PATH_LENGTH];

  switch (sensor_state) {
    case INIT_SENSOR:
      for (i=0; i<USE_SENSORS_COUNT; i++)
        sensors[i]->resetPrepared();

      i = 0;
      retry = 0;
      memset(&temperature, UINT16_MAX, sizeof(value_t));
      memset(&humidity, UINT16_MAX, sizeof(value_t));
      memset(&rain, UINT16_MAX, sizeof(rain_t));

      state_after_wait = INIT_SENSOR;
      sensor_state = PREPARE_SENSOR;
      break;

    case PREPARE_SENSOR:
      sensors[i]->prepare();
      delay_ms = sensors[i]->getDelay();
      start_time_ms = sensors[i]->getStartTime();
      state_after_wait = IS_SENSOR_PREPARED;
      sensor_state = WAIT_SENSOR_STATE;
      break;

    case IS_SENSOR_PREPARED:
      // success
      if (sensors[i]->isPrepared()) {
        sensor_state = GET_SENSOR;
      }
      // retry
      else if ((++retry) < SENSORS_RETRY_COUNT_MAX) {
        delay_ms = SENSORS_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = PREPARE_SENSOR;
        sensor_state = WAIT_SENSOR_STATE;
      }
      // fail
      else sensor_state = END_SENSOR_READING;
      break;

    case GET_SENSOR:
      sensors[i]->getJson(values_readed_from_sensor, VALUES_TO_READ_FROM_SENSOR_COUNT, json_string);
      // sensors[i]->get(values_readed_from_sensor, VALUES_TO_READ_FROM_SENSOR_COUNT);
      delay_ms = sensors[i]->getDelay();
      start_time_ms = sensors[i]->getStartTime();
      state_after_wait = IS_SENSOR_GETTED;
      sensor_state = WAIT_SENSOR_STATE;
      break;

    case IS_SENSOR_GETTED:
      // success and end
      if (sensors[i]->isEnd() && !sensors[i]->isReaded() && sensors[i]->isSuccess()) {
        sensor_state = READ_SENSOR;
      }
      // success and not end
      else if (sensors[i]->isSuccess()) {
        sensor_state = GET_SENSOR;
      }
      // retry
      else if ((++retry) < SENSORS_RETRY_COUNT_MAX) {
        delay_ms = SENSORS_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = GET_SENSOR;
        sensor_state = WAIT_SENSOR_STATE;
      }
      // fail
      else sensor_state = END_SENSOR_READING;
      break;

    case READ_SENSOR:
      #if (USE_SENSOR_TBS || USE_SENSOR_TBR)
      if ((strcmp(sensors[i]->getType(), SENSOR_TYPE_TBS) == 0) || (strcmp(sensors[i]->getType(), SENSOR_TYPE_TBR) == 0)) {
        rain.tips_count = values_readed_from_sensor[0];
      }
      #endif

      #if (USE_SENSOR_STH)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_STH) == 0) {
        temperature.sample = values_readed_from_sensor[0];
        humidity.sample = values_readed_from_sensor[1];
      }
      #endif

      #if (USE_SENSOR_ITH)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_ITH) == 0) {
        temperature.med60 = values_readed_from_sensor[0];
        humidity.med60 = values_readed_from_sensor[1];
      }
      #endif

      #if (USE_SENSOR_MTH)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_MTH) == 0) {
        temperature.med = values_readed_from_sensor[0];
        humidity.med = values_readed_from_sensor[1];
      }
      #endif

      #if (USE_SENSOR_NTH)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_NTH) == 0) {
        temperature.min = values_readed_from_sensor[0];
        humidity.min = values_readed_from_sensor[1];
      }
      #endif

      #if (USE_SENSOR_XTH)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_XTH) == 0) {
        temperature.max = values_readed_from_sensor[0];
        humidity.max = values_readed_from_sensor[1];
      }
      #endif

      if (!is_first_run) {
        uint8_t data_count = makeRmapSensorString(data_string, json_string, configuration.mqtt_root_path, configuration.sensors[i].mqtt_path);
        #if (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_DEBUG)
        for (uint8_t k=0; k<data_count; k++)
          SERIAL_INFO("%s\r\n", &data_string[k][0]);
        #endif
      }

      sensor_state = END_SENSOR_READING;
      break;

    case END_SENSOR_READING:
      // next sensor
      if ((++i) < USE_SENSORS_COUNT) {
        retry = 0;
        sensor_state = PREPARE_SENSOR;
      }
      else {
        if (is_first_run) {
          #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_INFO)
          SERIAL_INFO("DATE      \tTIME    \tT-IST\tT-MIN\tT-MED\tT-MAX\tH-IST\tH-MIN\tH-MED\tH-MAX\tR-TPS\r\n");
          #elif (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_DEBUG)
          SERIAL_DEBUG("Start acquisition....\r\n\r\n");
          #endif
          is_first_run = false;
        }
        else {
          #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_INFO)
          SERIAL_INFO("%02u/%02u/%04u\t", sensor_reading_day, sensor_reading_month, sensor_reading_year);
          SERIAL_INFO("%02u:%02u:00\t", sensor_reading_hour, sensor_reading_minute);

          if (temperature.med60 != UINT16_MAX)
            SERIAL_INFO("%u\t", temperature.med60);
          else SERIAL_INFO("-----\t");

          if (temperature.min != UINT16_MAX)
            SERIAL_INFO("%u\t", temperature.min);
          else SERIAL_INFO("-----\t");

          if (temperature.med != UINT16_MAX)
            SERIAL_INFO("%u\t", temperature.med);
          else SERIAL_INFO("-----\t");

          if (temperature.max != UINT16_MAX)
            SERIAL_INFO("%u\t", temperature.max);
          else SERIAL_INFO("-----\t");

          if (humidity.med60 != UINT16_MAX)
            SERIAL_INFO("%u\t", humidity.med60);
          else SERIAL_INFO("-----\t");

          if (humidity.min != UINT16_MAX)
            SERIAL_INFO("%u\t", humidity.min);
          else SERIAL_INFO("-----\t");

          if (humidity.med != UINT16_MAX)
            SERIAL_INFO("%u\t", humidity.med);
          else SERIAL_INFO("-----\t");

          if (humidity.max != UINT16_MAX)
            SERIAL_INFO("%u\t", humidity.max);
          else SERIAL_INFO("-----\t");

          if (rain.tips_count != UINT16_MAX)
            SERIAL_INFO("%u\r\n", rain.tips_count);
          else SERIAL_INFO("-----\r\n");
          #elif (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_DEBUG)
          SERIAL_DEBUG("\r\n");
          #endif
        }

        #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_INFO)
        delay_ms = 10;
        start_time_ms = millis();
        state_after_wait = END_TASK;
        sensor_state = WAIT_SENSOR_STATE;
        #else
        sensor_state = END_TASK;
        #endif
      }
      break;

    case END_TASK:
      noInterrupts();
      is_event_sensors_reading = false;
      ready_tasks_count--;
      interrupts();
      sensor_state = END_SENSOR;
      break;

    case END_SENSOR:
      break;

    case WAIT_SENSOR_STATE:
      if (millis() - start_time_ms > delay_ms) {
        sensor_state = state_after_wait;
      }
      break;
  }
}

uint8_t makeRmapSensorString(char data_string[][MQTT_ROOT_PATH_LENGTH + MQTT_SENSOR_PATH_LENGTH], const char *json, const char *mqtt_root, const char *mqtt_sensor) {
  uint8_t i = 0;
  memset(data_string, 0, sizeof(data_string[0][0]) * (MQTT_ROOT_PATH_LENGTH + MQTT_SENSOR_PATH_LENGTH) * VALUES_TO_READ_FROM_SENSOR_COUNT);

  StaticJsonBuffer<JSON_BUFFER_LENGTH> buffer;
  JsonObject &root = buffer.parseObject(json);

  for (JsonObject::iterator it=root.begin(); it!=root.end(); ++it) {
    snprintf(&data_string[i][0], MQTT_ROOT_PATH_LENGTH + MQTT_SENSOR_PATH_LENGTH, "%s%s", mqtt_root, mqtt_sensor);
    snprintf(&data_string[i][0]+strlen(&data_string[i][0]), MQTT_ROOT_PATH_LENGTH + MQTT_SENSOR_PATH_LENGTH - strlen(&data_string[i][0]), "%s {\"v\":%ld,\"t\":\"", it->key, it->value.as<int32_t>());
    snprintf(&data_string[i][0]+strlen(&data_string[i][0]), MQTT_ROOT_PATH_LENGTH + MQTT_SENSOR_PATH_LENGTH - strlen(&data_string[i][0]), "%04u-%02u-%02uT%02u:%02u:00\"}", sensor_reading_year, sensor_reading_month, sensor_reading_day, sensor_reading_hour, sensor_reading_minute);
    i++;
  }

  return i;
}
