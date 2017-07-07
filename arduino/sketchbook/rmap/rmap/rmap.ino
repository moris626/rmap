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
      if (is_event_supervisor)
        supervisor_task();

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

      if (is_event_sensors_reading)
        sensors_reading_task();

      if (is_event_data_saving)
        data_saving_task();

      if (is_event_sdcard)
        sdcard_task();

      if (is_event_mqtt)
        mqtt_task();

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

  is_event_supervisor = true;
  ready_tasks_count++;

  is_event_time = false;
  is_event_sensors_reading = false;
  is_event_data_saving = false;
  is_event_mqtt = false;
  is_event_sdcard = false;

  #if (USE_WDT_TASK)
  is_event_wdt = false;
  #endif

  #if (USE_RTC_TASK)
  is_event_rtc = false;
  #endif

  #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
  is_event_ethernet = false;
  #endif

  supervisor_state = INIT_SUPERVISOR;
  time_state = INIT_TIME;
  sdcard_state = INIT_SDCARD;
  mqtt_state = INIT_MQTT;
  sensor_reading_state = INIT_SENSOR;
  data_saving_state = INIT_DATA_SAVING;

  rtc_event_occurred_time_ms = -DEBOUNCING_RTC_TIME_MS;

  is_ethernet_connected = false;
  is_ethernet_udp_socket_open = false;
  is_time_set = false;
  is_time_rtc = true;
  is_time_ntp = false;
  is_mqtt_connected = false;
  do_mqtt_disconnect = false;
  do_sensors_reading = false;

  interrupts();
}

void init_pins() {
  pinMode(CONFIGURATION_RESET_PIN, INPUT_PULLUP);

  pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), rtc_interrupt_handler, CHANGE);

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

  SERIAL_INFO("--> ntp server: %s\r\n", configuration.ntp_server);
  #endif

  SERIAL_INFO("--> mqtt server: %s\r\n", configuration.mqtt_server);
  SERIAL_INFO("--> mqtt port: %u\r\n", configuration.mqtt_port);
  SERIAL_INFO("--> mqtt root topic: %s\r\n", configuration.mqtt_root_topic);
  SERIAL_INFO("--> mqtt subscribe topic: %s\r\n", configuration.mqtt_subscribe_topic);
  SERIAL_INFO("--> mqtt username: %s\r\n", configuration.mqtt_username);
  SERIAL_INFO("--> mqtt password: %s\r\n", configuration.mqtt_password);

  SERIAL_INFO("\r\n");
}

void save_configuration(bool is_default) {
  char temp_string[20];
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
    strcpy(configuration.ntp_server, CONFIGURATION_DEFAULT_NTP_SERVER);
    #endif

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
    strcpy(configuration.ntp_server, writable_data.ntp_server);
    #endif

    strcpy(configuration.mqtt_server, writable_data.mqtt_server);
    strcpy(configuration.mqtt_root_topic, writable_data.mqtt_root_topic);
    strcpy(configuration.mqtt_subscribe_topic, writable_data.mqtt_subscribe_topic);
    strcpy(configuration.mqtt_username, writable_data.mqtt_username);
    strcpy(configuration.mqtt_password, writable_data.mqtt_password);
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
    SERIAL_INFO("--> %u: %s-%s: %s\t", i+1, configuration.sensors[i].driver, configuration.sensors[i].type, configuration.sensors[i].mqtt_topic);

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
  if (millis() - rtc_event_occurred_time_ms > DEBOUNCING_RTC_TIME_MS) {
    rtc_event_occurred_time_ms = millis();
    noInterrupts();
    if (!is_event_rtc) {
      is_event_rtc = true;
      ready_tasks_count++;
    }
    interrupts();
  }
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

void supervisor_task() {
  static bool is_supervisor_first_run = true;
  static bool do_rtc_timer_alarm = false;

  switch (supervisor_state) {
    case INIT_SUPERVISOR:
      do_sensors_reading = false;
      is_event_time_executed = false;
      is_event_ethernet_executed = false;
      is_event_sdcard_executed = false;
      is_event_mqtt_executed = false;

      if (is_supervisor_first_run)
        supervisor_state = INIT_RTC_LEVEL_TASKS;
      else supervisor_state = INIT_SDCARD_LEVEL_TASKS;
      break;

    case INIT_RTC_LEVEL_TASKS:
      // first time
      noInterrupts();
      if (!is_event_time && !is_event_time_executed && !is_time_set) {
        is_event_time = true;
        ready_tasks_count++;
      }
      interrupts();

      // success
      if (!is_event_time && is_event_time_executed && is_time_set) {
        do_rtc_timer_alarm = true;
        supervisor_state = INIT_SDCARD_LEVEL_TASKS;
      }

      // error: BLOCK !!!!
      if (!is_event_time && is_event_time_executed && !is_time_set) {
        supervisor_state = END_SUPERVISOR;
      }
      break;

    case INIT_SDCARD_LEVEL_TASKS:
      // first time
      noInterrupts();
      if (!is_event_sdcard && !is_event_sdcard_executed && !SD.fatType()) {
        is_event_sdcard = true;
        ready_tasks_count++;
      }
      interrupts();

      // success
      if (!is_event_sdcard && SD.fatType()) {
        do_sensors_reading = true;
        supervisor_state = INIT_CONNECTION_LEVEL_TASKS;
      }

      // error
      if (!is_event_sdcard && is_event_sdcard_executed && !SD.fatType()) {
        supervisor_state = INIT_CONNECTION_LEVEL_TASKS;
      }
      break;

    case INIT_CONNECTION_LEVEL_TASKS:
      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
      // first time
      noInterrupts();
      if (!is_event_ethernet && !is_event_ethernet_executed && !is_ethernet_connected) {
        is_event_ethernet = true;
        ready_tasks_count++;
      }
      interrupts();

      // success
      if (!is_event_ethernet && is_ethernet_connected) {
        // ntp sync
        if (is_supervisor_first_run) {
          is_event_time_executed = false;
          supervisor_state = INIT_NTP_LEVEL_TASKS;
        }
        else supervisor_state = INIT_MQTT_LEVEL_TASKS;
      }

      // error
      if (!is_event_ethernet && is_event_ethernet_executed && !is_ethernet_connected) {
        supervisor_state = END_SUPERVISOR;
      }

      #else
      supervisor_state = END_SUPERVISOR;
      #endif
      break;

    case INIT_NTP_LEVEL_TASKS:
      // first time
      noInterrupts();
      if (!is_event_time && !is_event_time_executed && is_time_set && is_ethernet_connected) {
        is_event_time = true;
        ready_tasks_count++;
      }
      interrupts();

      // success
      if (!is_event_time && is_event_time_executed && is_time_set && is_ethernet_connected) {
        do_rtc_timer_alarm = true;
        supervisor_state = INIT_MQTT_LEVEL_TASKS;
      }

      // error
      if (!is_event_time && is_event_time_executed && !is_time_set && is_ethernet_connected) {
        supervisor_state = INIT_MQTT_LEVEL_TASKS;
      }
      break;

    case INIT_MQTT_LEVEL_TASKS:
      // first time
      noInterrupts();
      if (!is_event_mqtt && !is_event_mqtt_executed && !mqtt_client.isConnected()) {
        is_event_mqtt = true;
        ready_tasks_count++;
      }
      interrupts();

      // success
      if (!is_event_mqtt && mqtt_client.isConnected()) {
        do_sensors_reading = true;
        supervisor_state = END_SUPERVISOR;
      }

      // error
      if (!is_event_mqtt && is_event_mqtt_executed && !mqtt_client.isConnected()) {
        supervisor_state = END_SUPERVISOR;
      }
      break;

    case END_SUPERVISOR:
      if (is_supervisor_first_run && do_rtc_timer_alarm)
        setRtcTimerAndAlarm();

      is_supervisor_first_run = false;
      noInterrupts();
      is_event_supervisor = false;
      ready_tasks_count--;
      interrupts();

      supervisor_state = INIT_SUPERVISOR;
      break;
  }
}

#if (USE_RTC_TASK)
void rtc_task() {
  if (Pcf8563::isAlarmActive())
    Pcf8563::disableAlarm();

  if (Pcf8563::isTimerActive())
    Pcf8563::resetTimer();

  if (second() == 0) {
    Pcf8563::enableTimer();

    noInterrupts();
    if (!is_event_supervisor) {
      is_event_supervisor = true;
      ready_tasks_count++;
    }
    interrupts();

    if (minute() == next_minute_for_sensor_reading) {
      SERIAL_TRACE("Doing sensor reading...\r\n");
      sensor_reading_time.Day = day();
      sensor_reading_time.Month = month();
      sensor_reading_time.Year = CalendarYrToTm(year());
      sensor_reading_time.Hour = hour();
      sensor_reading_time.Minute = minute();

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

void setRtcTimerAndAlarm() {
  next_minute_for_sensor_reading = 255;
  setNextTimeForSensorReading(&next_hour_for_sensor_reading, &next_minute_for_sensor_reading);
  Pcf8563::disableAlarm();
  Pcf8563::disableTimer();
  Pcf8563::setAlarm(PCF8563_ALARM_DISABLE, next_minute_for_sensor_reading);
  Pcf8563::enableAlarm();
  Pcf8563::setTimer(RTC_FREQUENCY, RTC_TIMER);

  SERIAL_INFO("Current date and time is: %02u/%02u/%04u %02u:%02u:%02u\r\n\r\n", day(), month(), year(), hour(), minute(), second());
  SERIAL_INFO("Acquisition scheduling...\r\n");
  SERIAL_INFO("--> observations every %u minutes\r\n", OBSERVATIONS_MINUTES);
  SERIAL_INFO("--> report every %u minutes\r\n", REPORT_MINUTES);
  SERIAL_INFO("--> starting at: %02u:%02u:00\r\n\r\n", next_hour_for_sensor_reading, next_minute_for_sensor_reading);
}

void time_task() {
  static uint8_t retry;
  static time_state_t state_after_wait;
  static uint32_t delay_ms;
  static uint32_t start_time_ms;

  switch (time_state) {
    case INIT_TIME:
      retry = 0;
      state_after_wait = INIT_TIME;

      if (is_ethernet_connected)
        time_state = TIME_SEND_ONLINE_REQUEST;
      else time_state = TIME_SET_SYNC_RTC_PROVIDER;
      break;

    case TIME_SEND_ONLINE_REQUEST:
      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
      //while (eth_udp_client.parsePacket() > 0);
      // success
      if (Ntp::sendRequest(&eth_udp_client, configuration.ntp_server)) {
        retry = 0;
        time_state = TIME_WAIT_ONLINE_RESPONSE;
      }
      // retry
      else if (++retry < NTP_RETRY_COUNT_MAX) {
        delay_ms = NTP_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = TIME_SEND_ONLINE_REQUEST;
        time_state = WAIT_TIME_STATE;
      }
      // fail: use old rtc time
      else time_state = TIME_SET_SYNC_RTC_PROVIDER;
      #endif
      break;

    case TIME_WAIT_ONLINE_RESPONSE:
      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
      // success
      if (Ntp::getResponse(&eth_udp_client)) {
        time_state = TIME_SET_SYNC_ONLINE_PROVIDER;
      }
      // retry
      else if (++retry < NTP_RETRY_COUNT_MAX) {
        delay_ms = NTP_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = TIME_WAIT_ONLINE_RESPONSE;
        time_state = WAIT_TIME_STATE;
      }
      // fail
      else time_state = TIME_SET_SYNC_RTC_PROVIDER;
      #endif
      break;

    case TIME_SET_SYNC_ONLINE_PROVIDER:
      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
      setSyncProvider(Ntp::getTime);
      SERIAL_TRACE("Current NTP date and time: %02u/%02u/%04u %02u:%02u:%02u\r\n", day(), month(), year(), hour(), minute(), second());
      #endif

      time_state = TIME_SET_RTC_TIME;
      break;

    case TIME_SET_RTC_TIME:
      Pcf8563::reset();
      Pcf8563::setDate(day(), month(), year()-2000, weekday()-1, 0);
      Pcf8563::setTime(hour(), minute(), second());
      time_state = TIME_SET_SYNC_RTC_PROVIDER;
      break;

    case TIME_SET_SYNC_RTC_PROVIDER:
      setSyncProvider(Pcf8563::getTime);
      SERIAL_TRACE("Current RTC date and time: %02u/%02u/%04u %02u:%02u:%02u\r\n", day(), month(), year(), hour(), minute(), second());
      time_state = END_TIME;
      break;

    case TIME_SET_ALARM:
      // next_minute_for_sensor_reading = 255;
      // setNextTimeForSensorReading(&next_hour_for_sensor_reading, &next_minute_for_sensor_reading);
      // Pcf8563::disableAlarm();
      // Pcf8563::disableTimer();
      // Pcf8563::setAlarm(PCF8563_ALARM_DISABLE, next_minute_for_sensor_reading);
      // Pcf8563::enableAlarm();
      // Pcf8563::setTimer(RTC_FREQUENCY, RTC_TIMER);
      time_state = END_TIME;
      break;

    case END_TIME:
      is_time_set = true;
      is_event_time_executed = true;
      noInterrupts();
      is_event_time = false;
      ready_tasks_count--;
      interrupts();
      time_state = INIT_TIME;
      break;

    case WAIT_TIME_STATE:
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
  static uint8_t retry;
  static ethernet_state_t state_after_wait;
  static uint32_t delay_ms;
  static uint32_t start_time_ms;

  switch (ethernet_state) {
    case INIT_ETHERNET:
      retry = 0;
      is_ethernet_connected = false;
      is_ethernet_udp_socket_open = false;
      state_after_wait = INIT_ETHERNET;
      ethernet_state = ETHERNET_CONNECT;
      break;

    case ETHERNET_CONNECT:
      if (configuration.is_dhcp_enable) {
        if (Ethernet.begin(configuration.ethernet_mac)) {
          is_ethernet_connected = true;
          SERIAL_INFO("Ethernet: DHCP [ OK ]\r\n");
        }
        else {
          SERIAL_ERROR("Ethernet: DHCP [ FAIL ]\r\n");
        }
      }
      else {
        Ethernet.begin(configuration.ethernet_mac, IPAddress(configuration.ip), IPAddress(configuration.primary_dns), IPAddress(configuration.gateway), IPAddress(configuration.netmask));
        is_ethernet_connected = true;
        SERIAL_INFO("Ethernet: Static [ OK ]\r\n");
      }

      // success
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
        ethernet_state = ETHERNET_OPEN_UDP_SOCKET;
      }
      // retry
      else if ((++retry) < ETHERNET_RETRY_COUNT_MAX) {
        delay_ms = ETHERNET_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = ETHERNET_CONNECT;
        ethernet_state = WAIT_ETHERNET_STATE;
      }
      // fail
      else {
        retry = 0;
        ethernet_state = END_ETHERNET;
      }
      break;

    case ETHERNET_OPEN_UDP_SOCKET:
      if (eth_udp_client.begin(ETHERNET_DEFAULT_LOCAL_UDP_PORT)) {
        SERIAL_TRACE("--> udp socket local port: %u [ OK ]\r\n", ETHERNET_DEFAULT_LOCAL_UDP_PORT);
        is_ethernet_udp_socket_open = true;
        ethernet_state = END_ETHERNET;
      }
      // retry
      else if ((++retry) < ETHERNET_RETRY_COUNT_MAX) {
        delay_ms = ETHERNET_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = ETHERNET_OPEN_UDP_SOCKET;
        ethernet_state = WAIT_ETHERNET_STATE;
      }
      // fail
      else {
        SERIAL_ERROR("--> udp socket on local port: %u [ FAIL ]\r\n", ETHERNET_DEFAULT_LOCAL_UDP_PORT);
        retry = 0;
        ethernet_state = INIT_ETHERNET;
      }
      break;

    case END_ETHERNET:
      SERIAL_INFO("\r\n");
      is_event_ethernet_executed = true;
      noInterrupts();
      is_event_ethernet = false;
      ready_tasks_count--;
      interrupts();
      ethernet_state = INIT_ETHERNET;
      break;

    case WAIT_ETHERNET_STATE:
      if (millis() - start_time_ms > delay_ms) {
        ethernet_state = state_after_wait;
      }
      break;
  }
}
#endif

void mqttRxCallback(MQTT::MessageData &md) {
  MQTT::Message &rx_message = md.message;
  SERIAL_DEBUG("%s\r\n", (char*)rx_message.payload);
  SERIAL_DEBUG("--> qos %u\r\n", rx_message.qos);
  SERIAL_DEBUG("--> retained %u\r\n", rx_message.retained);
  SERIAL_DEBUG("--> dup %u\r\n", rx_message.dup);
  SERIAL_DEBUG("--> id %u\r\n", rx_message.id);
}

bool mqttConnect(char *server, uint16_t port, char *username, char *password) {
  if (!mqtt_client.isConnected()) {

    if (ipstack.connect(server, port) != 1)
      return false;

    char id[MQTT_USERNAME_LENGTH+4];
    strcpy(id, username);
    id[MQTT_USERNAME_LENGTH] = hour();
    id[MQTT_USERNAME_LENGTH+1] = minute();
    id[MQTT_USERNAME_LENGTH+2] = second();
    id[MQTT_USERNAME_LENGTH+3] = '\0';

    MQTTPacket_connectData mqtt_connection_data;
    mqtt_connection_data = MQTTPacket_connectData_initializer;
    mqtt_connection_data.MQTTVersion = 3;
    mqtt_connection_data.clientID.cstring = (char*)id;
    mqtt_connection_data.username.cstring = (char*)username;
    mqtt_connection_data.password.cstring = (char*)password;
    mqtt_connection_data.cleansession = false;

    if (mqtt_client.connect(mqtt_connection_data))
      return false;
  }

  return mqtt_client.isConnected();
}

bool mqttPublish(char *topic, char *message) {
  MQTT::Message tx_message;
  tx_message.qos = MQTT::QOS1;
  tx_message.retained = false;
  tx_message.dup = false;
  tx_message.payload = (void*)message;
  tx_message.payloadlen = strlen(message) + 1;

  if (mqtt_client.publish(topic, tx_message))
    return false;

  return true;
}

uint8_t jsonToMqtt(const char *json, const char *mqtt_sensor, char topic[][MQTT_SENSOR_TOPIC_LENGTH], char message[][MQTT_MESSAGE_LENGTH]) {
  uint8_t i = 0;
  memset(topic, 0, sizeof(topic[0][0]) * MQTT_SENSOR_TOPIC_LENGTH * VALUES_TO_READ_FROM_SENSOR_COUNT);
  memset(message, 0, sizeof(message[0][0]) * MQTT_MESSAGE_LENGTH * VALUES_TO_READ_FROM_SENSOR_COUNT);

  StaticJsonBuffer<JSON_BUFFER_LENGTH> buffer;
  JsonObject &root = buffer.parseObject(json);

  for (JsonObject::iterator it=root.begin(); it!=root.end(); ++it) {
    snprintf(&topic[i][0], MQTT_SENSOR_TOPIC_LENGTH, "%s%s", mqtt_sensor, it->key);
    snprintf(&message[i][0], MQTT_MESSAGE_LENGTH, "{\"v\":%ld,\"t\":\"", it->value.as<int32_t>());
    snprintf(&message[i][0] + strlen(&message[i][0]), MQTT_MESSAGE_LENGTH - strlen(&message[i][0]), "%04u-%02u-%02uT%02u:%02u:00\"}", tmYearToCalendar(sensor_reading_time.Year), sensor_reading_time.Month, sensor_reading_time.Day, sensor_reading_time.Hour, sensor_reading_time.Minute);
    i++;
  }

  return i;
}

void mqttToSd(const char *topic, const char *message, char *sd) {
  memset(sd, 0, sizeof(topic[0]) * (MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH));
  snprintf(sd, MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH, "%s %s\r\n", topic, message);
}

void sdToMqtt(const char *sd, char *topic, char *message) {
  memset(topic, 0, sizeof(topic[0]) * MQTT_SENSOR_TOPIC_LENGTH);
  memset(message, 0, sizeof(message[0]) * MQTT_MESSAGE_LENGTH);

  uint8_t delimiter = (uint8_t) strcspn(sd, " ");

  strncpy(topic, sd, delimiter);
  topic[delimiter] = '\0';
  strcpy(message, sd + delimiter + 1);
}

time_t getDateFromMessage(char *message) {
  tmElements_t _datetime;
  char temp[JSON_BUFFER_LENGTH];
  char str_buffer[11];
  uint8_t dt[3];
  uint8_t delimiter = (uint8_t) strcspn(message, "{");
  strcpy(temp, message + delimiter);
  StaticJsonBuffer<JSON_BUFFER_LENGTH> buffer;
  JsonObject &root = buffer.parseObject(temp);
  const char *datetime = root.get<const char*>("t");
  delimiter = (uint8_t) strcspn(datetime, "T");
  strncpy(str_buffer, datetime+2, delimiter);
  str_buffer[delimiter] = '\0';
  dateStringToArray(dt, str_buffer);

  _datetime.Year = CalendarYrToTm(dt[0]+2000);
  _datetime.Month = dt[1];
  _datetime.Day = dt[2];

  strcpy(str_buffer, datetime + delimiter + 1);
  timeStringToArray(dt, str_buffer);

  _datetime.Hour = dt[0];
  _datetime.Minute = dt[1];
  _datetime.Second = dt[2];

  return makeTime(_datetime);
}

void mqtt_task() {
  static uint8_t retry;
  static mqtt_state_t state_after_wait;
  static uint32_t delay_ms;
  static uint32_t start_time_ms;
  static bool is_subscribed;

  switch (mqtt_state) {
    case INIT_MQTT:
      is_mqtt_connected = false;
      is_subscribed = false;
      do_mqtt_disconnect = false;
      state_after_wait = INIT_MQTT;
      mqtt_state = CHECK_MQTT_OPERATION;
      break;

    case CHECK_MQTT_OPERATION:
      retry = 0;
      if (!is_mqtt_connected)
        mqtt_state = CONNECT_MQTT;
      else if (!is_subscribed)
        mqtt_state = SUBSCRIBE_MQTT;
      else if (is_mqtt_connected && do_mqtt_disconnect)
        mqtt_state = DISCONNECT_MQTT;
      else mqtt_state = END_MQTT;
      break;

    case CONNECT_MQTT:
      // success
      if (mqttConnect(configuration.mqtt_server, configuration.mqtt_port, configuration.mqtt_username, configuration.mqtt_password)) {
        if (!is_mqtt_connected) {
          is_mqtt_connected = true;
          SERIAL_INFO("MQTT Connection... [ OK ]\r\n");
        }
        mqtt_state = CHECK_MQTT_OPERATION;
      }
      // retry
      else if ((++retry) < MQTT_RETRY_COUNT_MAX) {
        delay_ms = MQTT_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = CONNECT_MQTT;
        mqtt_state = WAIT_MQTT_STATE;
      }
      // fail
      else {
        SERIAL_ERROR("MQTT Connection... [ FAIL ]\r\n");
        retry = 0;
        mqtt_state = END_MQTT;
      }
      break;

    case SUBSCRIBE_MQTT:
      if (mqtt_client.subscribe(configuration.mqtt_subscribe_topic, MQTT::QOS1, mqttRxCallback) == 0) {
        SERIAL_INFO("MQTT Subscribed... [ OK ]\r\n\r\n");
        is_subscribed = true;
        mqtt_state = CHECK_MQTT_OPERATION;
      }
      // retry
      else if ((++retry) < MQTT_RETRY_COUNT_MAX) {
        delay_ms = MQTT_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = SUBSCRIBE_MQTT;
        mqtt_state = WAIT_MQTT_STATE;
      }
      // fail
      else {
        SERIAL_ERROR("MQTT Subscribed... [ FAIL ]\r\n\r\n");
        retry = 0;
        mqtt_state = END_MQTT;
      }
      break;

    case DISCONNECT_MQTT:
      // success
      if (mqtt_client.disconnect() == 0) {
        SERIAL_ERROR("MQTT Disconnected... [ OK ]\r\n\r\n");
        do_mqtt_disconnect = false;
        mqtt_state = END_MQTT;
      }
      // retry
      else if ((++retry) < MQTT_RETRY_COUNT_MAX) {
        delay_ms = MQTT_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = DISCONNECT_MQTT;
        mqtt_state = WAIT_MQTT_STATE;
      }
      // fail
      else {
        SERIAL_ERROR("MQTT Disconnected... [ FAIL ]\r\n\r\n");
        retry = 0;
        mqtt_state = END_MQTT;
      }
      break;

    case END_MQTT:
      is_event_mqtt_executed = true;
      noInterrupts();
      is_event_mqtt = false;
      ready_tasks_count--;
      interrupts();
      mqtt_state = INIT_MQTT;
      break;

    case WAIT_MQTT_STATE:
      if (millis() - start_time_ms > delay_ms) {
        mqtt_state = state_after_wait;
      }
      break;
  }
}

void sdcard_task() {
  static uint8_t retry;
  static sdcard_state_t state_after_wait;
  static uint32_t delay_ms;
  static uint32_t start_time_ms;

  switch (sdcard_state) {
    case INIT_SDCARD:
      retry = 0;
      state_after_wait = INIT_SDCARD;
      sdcard_state = OPEN_SDCARD;
      break;

    case OPEN_SDCARD:
      // success
      if (SD.begin(SDCARD_CHIP_SELECT_PIN) && SD.vol()->fatType()) {
        SERIAL_INFO("SD Card... [ OK ]\r\n\r\n");
        sdcard_state = END_SDCARD;
      }
      // retry
      else if ((++retry) < SDCARD_RETRY_COUNT_MAX) {
        delay_ms = SDCARD_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = OPEN_SDCARD;
        sdcard_state = WAIT_SDCARD_STATE;
      }
      // fail
      else {
        SERIAL_ERROR("SD Card... [ FAIL ]\r\n--> is card inserted?\r\n--> there is a valid FAT32 filesystem?\r\n\r\n");
        retry = 0;
        sdcard_state = END_SDCARD;
      }
      break;

    case END_SDCARD:
      noInterrupts();
      is_event_sdcard = false;
      ready_tasks_count--;
      interrupts();

      is_event_sdcard_executed = true;
      sdcard_state = INIT_SDCARD;
      break;

    case WAIT_SDCARD_STATE:
      if (millis() - start_time_ms > delay_ms) {
        sdcard_state = state_after_wait;
      }
      break;
  }
}

void sensors_reading_task () {
  static uint8_t i;
  static uint8_t retry;
  static sensor_reading_state_t state_after_wait;
  static uint32_t delay_ms;
  static uint32_t start_time_ms;
  static int32_t values_readed_from_sensor[2];

  switch (sensor_reading_state) {
    case INIT_SENSOR:
      for (i=0; i<sensors_count; i++)
        sensors[i]->resetPrepared();

      i = 0;
      retry = 0;
      state_after_wait = INIT_SENSOR;

      if (do_sensors_reading)
        sensor_reading_state = PREPARE_SENSOR;
      else sensor_reading_state = END_SENSOR;
      break;

    case PREPARE_SENSOR:
      sensors[i]->prepare();
      delay_ms = sensors[i]->getDelay();
      start_time_ms = sensors[i]->getStartTime();
      state_after_wait = IS_SENSOR_PREPARED;
      sensor_reading_state = WAIT_SENSOR_STATE;
      break;

    case IS_SENSOR_PREPARED:
      // success
      if (sensors[i]->isPrepared()) {
        sensor_reading_state = GET_SENSOR;
      }
      // retry
      else if ((++retry) < SENSORS_RETRY_COUNT_MAX) {
        delay_ms = SENSORS_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = PREPARE_SENSOR;
        sensor_reading_state = WAIT_SENSOR_STATE;
      }
      // fail
      else sensor_reading_state = END_SENSOR_READING;
      break;

    case GET_SENSOR:
      sensors[i]->getJson(values_readed_from_sensor, VALUES_TO_READ_FROM_SENSOR_COUNT, &json_sensors_data[i][0]);
      // sensors[i]->get(values_readed_from_sensor, VALUES_TO_READ_FROM_SENSOR_COUNT);
      delay_ms = sensors[i]->getDelay();
      start_time_ms = sensors[i]->getStartTime();
      state_after_wait = IS_SENSOR_GETTED;
      sensor_reading_state = WAIT_SENSOR_STATE;
      break;

    case IS_SENSOR_GETTED:
      // success and end
      if (sensors[i]->isEnd() && !sensors[i]->isReaded() && sensors[i]->isSuccess()) {
        sensor_reading_state = READ_SENSOR;
      }
      // success and not end
      else if (sensors[i]->isSuccess()) {
        sensor_reading_state = GET_SENSOR;
      }
      // retry
      else if ((++retry) < SENSORS_RETRY_COUNT_MAX) {
        delay_ms = SENSORS_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = GET_SENSOR;
        sensor_reading_state = WAIT_SENSOR_STATE;
      }
      // fail
      else sensor_reading_state = END_SENSOR_READING;
      break;

    case READ_SENSOR:
      sensor_reading_state = END_SENSOR_READING;
      break;

    case END_SENSOR_READING:
      // next sensor
      if ((++i) < sensors_count) {
        retry = 0;
        sensor_reading_state = PREPARE_SENSOR;
      }
      else {
        if (is_first_run) {
          is_first_run = false;

          #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_INFO)
          SERIAL_INFO("DATE      \tTIME    \tT-IST\tT-MIN\tT-MED\tT-MAX\tH-IST\tH-MIN\tH-MED\tH-MAX\tR-TPS\r\n");
          #elif (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_DEBUG)
          SERIAL_DEBUG("Start acquisition....\r\n\r\n");
          #endif
        }
        else {
          noInterrupts();
          if (!is_event_data_saving) {
            is_event_data_saving = true;
            ready_tasks_count++;
          }
          interrupts();
        }

        #if (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_INFO)
        delay_ms = 10;
        start_time_ms = millis();
        state_after_wait = END_SENSOR;
        sensor_reading_state = WAIT_SENSOR_STATE;
        #else
        sensor_reading_state = END_SENSOR;
        #endif
      }
      break;

    case END_SENSOR:
      noInterrupts();
      is_event_sensors_reading = false;
      ready_tasks_count--;
      interrupts();
      sensor_reading_state = INIT_SENSOR;
      break;

    case WAIT_SENSOR_STATE:
      if (millis() - start_time_ms > delay_ms) {
        sensor_reading_state = state_after_wait;
      }
      break;
  }
}

void data_saving_task() {
  static uint8_t i;
  static uint8_t k;
  static uint8_t retry;
  static data_saving_state_t state_after_wait;
  static uint32_t delay_ms;
  static uint32_t start_time_ms;
  static uint8_t data_count;
  static bool is_saving_sdcard;
  static bool is_sdcard_saved;
  static bool is_sdcard_error;
  static bool is_saving_mqtt;
  static bool is_mqtt_saved;
  static bool is_mqtt_error;
  static char file_name[SDCARD_FILES_NAME_MAX_LENGTH];
  char sd_buffer[MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH];
  char topic_buffer[VALUES_TO_READ_FROM_SENSOR_COUNT][MQTT_SENSOR_TOPIC_LENGTH];
  char message_buffer[VALUES_TO_READ_FROM_SENSOR_COUNT][MQTT_MESSAGE_LENGTH];

  switch (data_saving_state) {
    case INIT_DATA_SAVING:
      retry = 0;
      is_saving_sdcard = false;
      is_sdcard_saved = false;
      is_sdcard_error = false;
      is_saving_mqtt = false;
      is_mqtt_saved = false;
      is_mqtt_error = false;
      data_saving_state = CHEK_OPEN_DATA_SAVING_SERVICE;
      break;

    case CHEK_OPEN_DATA_SAVING_SERVICE:
      retry = 0;
      if (!is_sdcard_saved && !is_sdcard_error) { // is_sdcard_inserted
        is_saving_sdcard = true;
        is_saving_mqtt = false;
        data_saving_state = DATA_SAVING_ON_SDCARD;
      }
      // else if (!is_sdcard_inserted && !is_mqtt_saved && !is_mqtt_error) {
      //   is_saving_sdcard = false;
      //   is_saving_mqtt = true;
      //   data_saving_state = DATA_SAVING_ONLY_ON_MQTT;
      // }
      else data_saving_state = END_DATA_SAVING;
      break;

    case DATA_SAVING_ON_SDCARD:
      snprintf(file_name, SDCARD_FILES_NAME_MAX_LENGTH, "%04u_%02u_%02u.txt", tmYearToCalendar(sensor_reading_time.Year), sensor_reading_time.Month, sensor_reading_time.Day);
      data_file = SD.open(file_name, FILE_WRITE);

      if (data_file) {
        i = 0;
        data_saving_state = DATA_SAVING_LOOP_JSON_TO_MQTT;
      }
      // retry
      else if ((++retry) < DATA_SAVING_RETRY_COUNT_MAX) {
        delay_ms = DATA_SAVING_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = DATA_SAVING_ON_SDCARD;
        data_saving_state = WAIT_DATA_SAVING_STATE;
      }
      // fail
      else {
        SERIAL_ERROR("SD Card saving data in %s... [ FAIL ]\r\n", file_name);
        is_sdcard_saved = false;
        is_sdcard_error = true;
        data_saving_state = CHEK_OPEN_DATA_SAVING_SERVICE;
      }
      break;

    case DATA_SAVING_LOOP_JSON_TO_MQTT:
      if (i < sensors_count) {
        data_count = jsonToMqtt(&json_sensors_data[i][0], configuration.sensors[i].mqtt_topic, topic_buffer, message_buffer);
        k = 0;
        data_saving_state = DATA_SAVING_LOOP_MQTT_TO_SERVICE;
      }
      else {
        is_sdcard_saved = true;
        is_sdcard_error = data_file.close();
        data_saving_state = CHEK_OPEN_DATA_SAVING_SERVICE;
      }
      break;

    case DATA_SAVING_LOOP_MQTT_TO_SERVICE:
      if (k < data_count) {

        // preprocessing for sdcard
        if (is_saving_sdcard) {
          mqttToSd(&topic_buffer[k][0], &message_buffer[k][0], sd_buffer);
          SERIAL_INFO("%s", sd_buffer);
        }

        // success sdcard
        if (is_saving_sdcard && data_file.write(sd_buffer, MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH) == (MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH)) {
          data_file.flush();
          k++;
        }
        // success mqtt
        else if (is_saving_mqtt && mqttPublish(&topic_buffer[k][0], &message_buffer[k][0])) {
          k++;
        }
        // retry
        else if ((++retry) < DATA_SAVING_RETRY_COUNT_MAX) {
          delay_ms = DATA_SAVING_RETRY_DELAY_MS;
          start_time_ms = millis();
          state_after_wait = DATA_SAVING_LOOP_MQTT_TO_SERVICE;
          data_saving_state = WAIT_DATA_SAVING_STATE;
        }
        // fail
        else {
          if (is_saving_sdcard) {
            SERIAL_ERROR("SD Card saving data in %s... [ FAIL ]\r\n", file_name);
            is_sdcard_saved = false;
            is_sdcard_error = true;
          }
          else if (is_saving_mqtt) {
            SERIAL_ERROR("MQTT publishing... [ FAIL ]\r\n");
            is_mqtt_saved = false;
            is_mqtt_error = true;
          }

          data_saving_state = CHEK_OPEN_DATA_SAVING_SERVICE;
        }
      }
      else {
        i++;
        data_saving_state = DATA_SAVING_LOOP_JSON_TO_MQTT;
      }
      break;

    case DATA_SAVING_ONLY_ON_MQTT:
      if (mqtt_client.isConnected()) {
        for (uint8_t ii=i; ii<sensors_count; ii++) {
          uint8_t data_count = jsonToMqtt(&json_sensors_data[ii][0], configuration.sensors[ii].mqtt_topic, topic_buffer, message_buffer);
          for (uint8_t kk=k; kk<data_count; kk++) {
            if (mqttPublish(&topic_buffer[kk][0], &message_buffer[kk][0])) {
              SERIAL_INFO("%s %s\r\n", &topic_buffer[kk][0], &message_buffer[kk][0]);
              i = ii + 1;
              k = kk + 1;
            }
            else if ((++retry) < DATA_SAVING_RETRY_COUNT_MAX) {
              delay_ms = DATA_SAVING_RETRY_DELAY_MS;
              start_time_ms = millis();
              state_after_wait = DATA_SAVING_ONLY_ON_MQTT;
              data_saving_state = WAIT_DATA_SAVING_STATE;
            }
            // fail
            else {
              SERIAL_ERROR("MQTT publishing data... [ FAIL ]\r\n");
              is_mqtt_saved = false;
              is_mqtt_error = true;
              data_saving_state = CHEK_OPEN_DATA_SAVING_SERVICE;
            }
          }
          if (!is_mqtt_error) {
            is_mqtt_saved = true;
          }
        }
        data_saving_state = CHEK_OPEN_DATA_SAVING_SERVICE;
      }
      // retry
      else if ((++retry) < DATA_SAVING_RETRY_COUNT_MAX) {
        delay_ms = DATA_SAVING_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = DATA_SAVING_ONLY_ON_MQTT;
        data_saving_state = WAIT_DATA_SAVING_STATE;
      }
      // fail
      else {
        SERIAL_ERROR("MQTT publish data... [ FAIL ]\r\n");
        is_mqtt_saved = false;
        is_mqtt_error = true;
        data_saving_state = CHEK_OPEN_DATA_SAVING_SERVICE;
      }
      break;

    case END_DATA_SAVING:
      noInterrupts();
      is_event_data_saving = false;
      ready_tasks_count--;
      interrupts();
      data_saving_state = INIT_DATA_SAVING;
      break;

    case WAIT_DATA_SAVING_STATE:
      if (millis() - start_time_ms > delay_ms) {
        data_saving_state = state_after_wait;
      }
      break;
  }
}
