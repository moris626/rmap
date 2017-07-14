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
  wdt_disable();
  wdt_enable(WDTO_4S);
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
        data_processing_task();

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
  time_state = END_TIME_TASK;
  sensor_reading_state = END_SENSOR_TASK;
  data_processing_state = END_DATA_PROCESSING;

  rtc_event_occurred_time_ms = -DEBOUNCING_RTC_TIME_MS;

  #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
  is_ethernet_connected = false;
  is_ethernet_udp_socket_open = false;
  #endif

  is_time_set = false;

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
      is_event_time_executed = false;

      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
      is_event_ethernet_executed = false;
      #endif

      if (is_supervisor_first_run) {
        supervisor_state = INIT_RTC_LEVEL_TASKS;
        SERIAL_INFO("INIT_SUPERVISOR ---> INIT_RTC_LEVEL_TASKS\r\n");
      }
      else {
        supervisor_state = INIT_CONNECTION_LEVEL_TASKS;
        SERIAL_INFO("INIT_SUPERVISOR ---> INIT_CONNECTION_LEVEL_TASKS\r\n");
      }
      break;

    case INIT_RTC_LEVEL_TASKS:
      // first time
      noInterrupts();
      if (!is_event_time && !is_event_time_executed && !is_time_set) {
        time_state = INIT_TIME;
        is_event_time = true;
        ready_tasks_count++;
      }
      interrupts();

      // success
      if (!is_event_time && is_event_time_executed && is_time_set) {
        do_rtc_timer_alarm = true;
        supervisor_state = INIT_CONNECTION_LEVEL_TASKS;
        SERIAL_INFO("INIT_RTC_LEVEL_TASKS ---> INIT_CONNECTION_LEVEL_TASKS\r\n");
      }

      // error: BLOCK !!!!
      if (!is_event_time && is_event_time_executed && !is_time_set) {
        supervisor_state = END_SUPERVISOR;
        SERIAL_INFO("INIT_RTC_LEVEL_TASKS ---> END_SUPERVISOR\r\n");
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
          SERIAL_INFO("INIT_CONNECTION_LEVEL_TASKS ---> INIT_NTP_LEVEL_TASKS\r\n");
        }
        else {
          supervisor_state = END_SUPERVISOR;
          SERIAL_INFO("INIT_CONNECTION_LEVEL_TASKS ---> END_SUPERVISOR\r\n");
        }
      }

      // error
      if (!is_event_ethernet && is_event_ethernet_executed && !is_ethernet_connected) {
        supervisor_state = END_SUPERVISOR;
        SERIAL_INFO("INIT_CONNECTION_LEVEL_TASKS ---> END_SUPERVISOR\r\n");
      }

      #else
      supervisor_state = END_SUPERVISOR;
      SERIAL_INFO("INIT_CONNECTION_LEVEL_TASKS ---> END_SUPERVISOR\r\n");
      #endif
      break;

    case INIT_NTP_LEVEL_TASKS:
      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
      // first time
      noInterrupts();
      if (!is_event_time && !is_event_time_executed && is_time_set && is_ethernet_connected) {
        time_state = INIT_TIME;
        is_event_time = true;
        ready_tasks_count++;
      }
      interrupts();

      // success
      if (!is_event_time && is_event_time_executed && is_time_set && is_ethernet_connected) {
        do_rtc_timer_alarm = true;
        supervisor_state = END_SUPERVISOR;
        SERIAL_INFO("INIT_NTP_LEVEL_TASKS ---> END_SUPERVISOR\r\n");
      }

      // error
      if (!is_event_time && is_event_time_executed && !is_time_set && is_ethernet_connected) {
        supervisor_state = END_SUPERVISOR;
        SERIAL_INFO("INIT_NTP_LEVEL_TASKS ---> END_SUPERVISOR\r\n");
      }
      #endif
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
      SERIAL_INFO("END_SUPERVISOR ---> INIT_SUPERVISOR\r\n");
      break;
  }
}

#if (USE_RTC_TASK)
void rtc_task() {
  if (Pcf8563::isAlarmActive())
    Pcf8563::disableAlarm();

  if (second() == 0) {
    Pcf8563::resetTimer();

    if (is_first_run)
      Pcf8563::enableTimer();

    noInterrupts();
    if (!is_event_supervisor) {
      is_event_supervisor = true;
      ready_tasks_count++;
    }
    interrupts();

    if (minute() == next_minute_for_sensor_reading) {
      SERIAL_DEBUG("Doing sensor reading...\r\n");
      sensor_reading_time.Day = day();
      sensor_reading_time.Month = month();
      sensor_reading_time.Year = CalendarYrToTm(year());
      sensor_reading_time.Hour = hour();
      sensor_reading_time.Minute = minute();
      sensor_reading_time.Second = 0;

      noInterrupts();
      if (!is_event_sensors_reading) {
        sensor_reading_state = INIT_SENSOR;
        is_event_sensors_reading = true;
        ready_tasks_count++;
      }
      interrupts();
    }

    setNextTimeForSensorReading(&next_hour_for_sensor_reading, &next_minute_for_sensor_reading);
    Pcf8563::setAlarm(PCF8563_ALARM_DISABLE, next_minute_for_sensor_reading);
    Pcf8563::enableAlarm();

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

      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_ETH || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_ETH)
      if (is_ethernet_connected)
        time_state = TIME_SEND_ONLINE_REQUEST;
      #elif (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
      if (true)
        time_state = TIME_SEND_ONLINE_REQUEST;
      #endif
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

    case END_TIME:
      is_time_set = true;
      is_event_time_executed = true;
      noInterrupts();
      is_event_time = false;
      ready_tasks_count--;
      interrupts();
      time_state = END_TIME_TASK;
      break;

    case END_TIME_TASK:
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
  if (ipstack.connect(server, port) != 1)
    return false;

  // char id[MQTT_USERNAME_LENGTH+4];
  // strcpy(id, username);
  // id[MQTT_USERNAME_LENGTH] = hour();
  // id[MQTT_USERNAME_LENGTH+1] = minute();
  // id[MQTT_USERNAME_LENGTH+2] = second();
  // id[MQTT_USERNAME_LENGTH+3] = '\0';

  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  data.MQTTVersion = 3;
  data.clientID.cstring = (char*)"eth12345";
  data.username.cstring = (char*)username;
  data.password.cstring = (char*)password;
  data.cleansession = false;

  if (mqtt_client.connect(data))
    return false;

  return true;
}

bool mqttPublish(const char *topic, const char *message) {
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
  memset(sd, 0, sizeof(sd[0]) * (MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH));
  snprintf(sd, MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH, "%s %s", topic, message);
}

void sdToMqtt(const char *sd, char *topic, char *message) {
  memset(topic, 0, sizeof(topic[0]) * MQTT_SENSOR_TOPIC_LENGTH);
  memset(message, 0, sizeof(message[0]) * MQTT_MESSAGE_LENGTH);

  uint8_t delimiter = (uint8_t) strcspn(sd, " ");

  strncpy(topic, sd, delimiter);
  topic[delimiter] = '\0';
  strcpy(message, sd + delimiter + 1);
}

void getFullTopic(char *full_topic, const char *root_topic, const char *sensor_topic) {
  memset(full_topic, 0, MQTT_ROOT_TOPIC_LENGTH + MQTT_SENSOR_TOPIC_LENGTH);
  strncpy(full_topic, root_topic, MQTT_ROOT_TOPIC_LENGTH);
  strncpy(full_topic + strlen(root_topic), sensor_topic, MQTT_SENSOR_TOPIC_LENGTH);
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
      sensor_reading_state = PREPARE_SENSOR;
      SERIAL_TRACE("INIT_SENSOR ---> PREPARE_SENSOR\r\n");
      break;

    case PREPARE_SENSOR:
      sensors[i]->prepare();
      delay_ms = sensors[i]->getDelay();
      start_time_ms = sensors[i]->getStartTime();

      if (delay_ms) {
        state_after_wait = IS_SENSOR_PREPARED;
        sensor_reading_state = WAIT_SENSOR_STATE;
        SERIAL_TRACE("PREPARE_SENSOR ---> WAIT_SENSOR_STATE\r\n");
      }
      else {
        sensor_reading_state = IS_SENSOR_PREPARED;
        SERIAL_TRACE("PREPARE_SENSOR ---> IS_SENSOR_PREPARED\r\n");
      }
      break;

    case IS_SENSOR_PREPARED:
      // success
      if (sensors[i]->isPrepared()) {
        sensor_reading_state = GET_SENSOR;
        SERIAL_TRACE("IS_SENSOR_PREPARED ---> GET_SENSOR\r\n");
      }
      // retry
      else if ((++retry) < SENSORS_RETRY_COUNT_MAX) {
        delay_ms = SENSORS_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = PREPARE_SENSOR;
        sensor_reading_state = WAIT_SENSOR_STATE;
        SERIAL_TRACE("IS_SENSOR_PREPARED ---> WAIT_SENSOR_STATE\r\n");
      }
      // fail
      else {
        sensor_reading_state = END_SENSOR_READING;
        SERIAL_TRACE("IS_SENSOR_PREPARED ---> END_SENSOR_READING\r\n");
      }
      break;

    case GET_SENSOR:
      sensors[i]->getJson(values_readed_from_sensor, VALUES_TO_READ_FROM_SENSOR_COUNT, &json_sensors_data[i][0]);
      // sensors[i]->get(values_readed_from_sensor, VALUES_TO_READ_FROM_SENSOR_COUNT);
      delay_ms = sensors[i]->getDelay();
      start_time_ms = sensors[i]->getStartTime();

      if (delay_ms) {
        state_after_wait = IS_SENSOR_GETTED;
        sensor_reading_state = WAIT_SENSOR_STATE;
        SERIAL_TRACE("GET_SENSOR ---> WAIT_SENSOR_STATE\r\n");
      }
      else {
        sensor_reading_state = IS_SENSOR_GETTED;
        SERIAL_TRACE("GET_SENSOR ---> IS_SENSOR_GETTED\r\n");
      }
      break;

    case IS_SENSOR_GETTED:
      // success and end
      if (sensors[i]->isEnd() && !sensors[i]->isReaded() && sensors[i]->isSuccess()) {
        sensor_reading_state = READ_SENSOR;
        SERIAL_TRACE("IS_SENSOR_GETTED ---> READ_SENSOR\r\n");
      }
      // success and not end
      else if (sensors[i]->isSuccess()) {
        sensor_reading_state = GET_SENSOR;
        SERIAL_TRACE("IS_SENSOR_GETTED ---> GET_SENSOR\r\n");
      }
      // retry
      else if ((++retry) < SENSORS_RETRY_COUNT_MAX) {
        delay_ms = SENSORS_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = GET_SENSOR;
        sensor_reading_state = WAIT_SENSOR_STATE;
        SERIAL_TRACE("IS_SENSOR_GETTED ---> WAIT_SENSOR_STATE\r\n");
      }
      // fail
      else {
        sensor_reading_state = END_SENSOR_READING;
        SERIAL_TRACE("IS_SENSOR_GETTED ---> END_SENSOR_READING\r\n");
      }
      break;

    case READ_SENSOR:
      sensor_reading_state = END_SENSOR_READING;
      SERIAL_TRACE("READ_SENSOR ---> END_SENSOR_READING\r\n");
      break;

    case END_SENSOR_READING:
      // next sensor
      if ((++i) < sensors_count) {
        retry = 0;
        sensor_reading_state = PREPARE_SENSOR;
        SERIAL_TRACE("END_SENSOR_READING ---> PREPARE_SENSOR\r\n");
      }
      // end
      else {
        noInterrupts();
        if (!is_event_data_saving) {
          data_processing_state = INIT_DATA_PROCESSING;
          is_event_data_saving = true;
          ready_tasks_count++;
        }
        interrupts();

        sensor_reading_state = END_SENSOR;
        SERIAL_TRACE("END_SENSOR_READING ---> END_SENSOR\r\n");
      }
      break;

    case END_SENSOR:
      noInterrupts();
      is_event_sensors_reading = false;
      ready_tasks_count--;
      interrupts();
      sensor_reading_state = END_SENSOR_TASK;
      SERIAL_TRACE("END_SENSOR ---> END_SENSOR_TASK\r\n");
      break;

    case END_SENSOR_TASK:
      SERIAL_ERROR("END_SENSOR_TASK ?!?!?!?!?!?!?!?!?\r\n");
      break;

    case WAIT_SENSOR_STATE:
      if (millis() - start_time_ms > delay_ms) {
        sensor_reading_state = state_after_wait;
      }
      break;
  }
}

bool sdcard_init(SdFat *SD, uint8_t chip_select) {
  if (SD->begin(chip_select) && SD->vol()->fatType()) {
    return true;
  }

  SERIAL_ERROR("SD Card... [ FAIL ]\r\n--> is card inserted?\r\n--> there is a valid FAT32 filesystem?\r\n\r\n");
  return false;
}

bool sdcard_open_file(SdFat *SD, File *file, const char *file_name, uint8_t param) {
  *file = SD->open(file_name, param);

  if (*file) {
    return true;
  }

  SERIAL_ERROR("SD Card open file %s... [ FAIL ]\r\n", file_name);
  return false;
}

void sdcard_make_filename(time_t time, char *file_name) {
  snprintf(file_name, SDCARD_FILES_NAME_MAX_LENGTH, "%04u_%02u_%02u.txt", year(time), month(time), day(time));
}

void data_processing_task() {
  static uint8_t i;
  static uint8_t k;
  static uint16_t sd_data_count;
  static uint16_t mqtt_data_count;
  static uint8_t retry;
  static data_processing_state_t state_after_wait;
  static uint32_t delay_ms;
  static uint32_t start_time_ms;
  static uint8_t data_count;
  static char sd_buffer[MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH];
  static char topic_buffer[VALUES_TO_READ_FROM_SENSOR_COUNT][MQTT_SENSOR_TOPIC_LENGTH];
  static char message_buffer[VALUES_TO_READ_FROM_SENSOR_COUNT][MQTT_MESSAGE_LENGTH];
  static char full_topic_buffer[MQTT_ROOT_TOPIC_LENGTH + MQTT_SENSOR_TOPIC_LENGTH];
  char file_name[SDCARD_FILES_NAME_MAX_LENGTH];
  static bool is_sdcard_processing;         // saving data on sdcard
  static bool is_mqtt_processing_sdcard;    // send data saved on sdcard
  static bool is_mqtt_processing_json;      // sd card fault fallback: send data in json
  static bool is_sdcard_error;
  static bool is_mqtt_error;
  static bool is_ptr_found;
  static bool is_ptr_updated;
  static bool is_eof_data_read;
  static tmElements_t datetime;
  static time_t current_ptr_time_data;
  static bool is_mqtt_subscribed = false;
  int read_bytes_count;

  switch (data_processing_state) {
    case INIT_DATA_PROCESSING:
      retry = 0;
      is_eof_data_read = false;
      is_sdcard_error = false;
      is_mqtt_error = false;
      sd_data_count = 0;
      mqtt_data_count = 0;
      data_processing_state = INIT_SDCARD_SERVICE;
      SERIAL_TRACE("INIT_DATA_PROCESSING ---> INIT_SDCARD_SERVICE\r\n");
      break;

    case INIT_SDCARD_SERVICE:
      is_sdcard_processing = true;
      is_mqtt_processing_json = false;
      is_mqtt_processing_sdcard = false;

      if (sdcard_init(&SD, SDCARD_CHIP_SELECT_PIN)) {
        retry = 0;
        // first time, there isn't new data. check for old not sent data by find ptr_date_time
        if (is_first_run) {
          is_first_run = false;
          data_processing_state = OPEN_SDCARD_PTR_DATA_FILES;
          SERIAL_TRACE("INIT_SDCARD_SERVICE ---> OPEN_SDCARD_PTR_DATA_FILES\r\n");
        }
        // other time, try to save data in sdcard
        else {
          data_processing_state = OPEN_SDCARD_WRITE_DATA_FILE;
          SERIAL_TRACE("INIT_SDCARD_SERVICE ---> OPEN_SDCARD_WRITE_DATA_FILE\r\n");
        }
      }
      // retry
      else if ((++retry) < DATA_PROCESSING_RETRY_COUNT_MAX) {
        delay_ms = DATA_PROCESSING_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = INIT_SDCARD_SERVICE;
        data_processing_state = WAIT_DATA_PROCESSING_STATE;
        SERIAL_TRACE("INIT_SDCARD_SERVICE ---> WAIT_DATA_PROCESSING_STATE\r\n");
      }
      // fail
      else {
        is_sdcard_error = true;

        // first time: nothing to do.
        if (is_first_run) {
          data_processing_state = END_DATA_PROCESSING;
          SERIAL_TRACE("INIT_SDCARD_SERVICE ---> END_DATA_PROCESSING\r\n");
        }
        // other time, send sensors data directly through mqtt
        else {
          data_processing_state = INIT_MQTT_SERVICE;
          SERIAL_TRACE("INIT_SDCARD_SERVICE ---> INIT_MQTT_SERVICE\r\n");
        }
      }
      break;

    case OPEN_SDCARD_WRITE_DATA_FILE:
      // open sdcard file: today!
      sdcard_make_filename(now(), file_name);

      // try to open file. if ok, write sensors data on it.
      if (sdcard_open_file(&SD, &data_file, file_name, O_RDWR | O_CREAT | O_APPEND)) {
        i = 0;
        data_processing_state = LOOP_JSON_TO_MQTT;
        SERIAL_TRACE("OPEN_SDCARD_WRITE_DATA_FILE ---> LOOP_JSON_TO_MQTT\r\n");
      }
      else {
        is_sdcard_error = true;
        data_processing_state = END_SDCARD_SERVICE;
        SERIAL_TRACE("OPEN_SDCARD_WRITE_DATA_FILE ---> END_SDCARD_SERVICE\r\n");
      }
      break;

    case END_SDCARD_SERVICE:
      if (!is_sdcard_error && data_file.close()) {
        data_processing_state = OPEN_SDCARD_PTR_DATA_FILES;
        SERIAL_TRACE("OPEN_SDCARD_WRITE_DATA_FILE ---> OPEN_SDCARD_PTR_DATA_FILES\r\n");
      }
      else {
        is_sdcard_error = true;
        data_processing_state = INIT_MQTT_SERVICE;
        SERIAL_TRACE("OPEN_SDCARD_WRITE_DATA_FILE ---> INIT_MQTT_SERVICE\r\n");
      }
      break;

    case OPEN_SDCARD_PTR_DATA_FILES:
      is_ptr_found = false;

      if (sdcard_open_file(&SD, &ptr_data_file, SDCARD_PTR_DATA_FILE_NAME, O_RDWR | O_CREAT)) {
        data_processing_state = READ_PTR_DATA;
        SERIAL_TRACE("OPEN_SDCARD_PTR_DATA_FILES ---> READ_PTR_DATA\r\n");
      }
      else {
        is_sdcard_error = true;
        data_processing_state = END_FIND_PTR;
        SERIAL_TRACE("OPEN_SDCARD_PTR_DATA_FILES ---> END_FIND_PTR\r\n");
      }
      break;

    case READ_PTR_DATA:
      ptr_date_time = UINT32_MAX;
      ptr_data_file.seekSet(0);
      read_bytes_count = ptr_data_file.read(&ptr_date_time, sizeof(time_t));

      if (read_bytes_count == sizeof(time_t) && ptr_date_time < now()) {
        is_ptr_found = true;
        data_processing_state = FOUND_PTR_DATA;
        SERIAL_TRACE("READ_PTR_DATA ---> FOUND_PTR_DATA\r\n");
      }
      else if (read_bytes_count >= 0) {
        SERIAL_INFO("Data pointer... [ FAIL ]\r\n--> Find it...\r\n\r\n");
        datetime.Year = year(now());
        datetime.Month = 1;
        datetime.Day = 1;
        datetime.Hour = 0;
        datetime.Minute = 0;
        datetime.Second = 0;
        ptr_date_time = makeTime(datetime);
        is_ptr_found = false;
        data_processing_state = FIND_PTR_DATA;
        SERIAL_TRACE("READ_PTR_DATA ---> FIND_PTR_DATA\r\n");
      }
      else {
        is_sdcard_error = true;
        data_processing_state = END_FIND_PTR;
        SERIAL_TRACE("READ_PTR_DATA ---> END_FIND_PTR\r\n");
      }
      break;

    case FIND_PTR_DATA:
      // ptr not found. find it by searching in file name. if there isn't file, ptr_date_time is set to current date at 00:00:00 time.
      if (!is_ptr_found && (year(ptr_date_time) != year() || month(ptr_date_time) != month() || day(ptr_date_time) != day()) && ptr_date_time < now()) {
        sdcard_make_filename(ptr_date_time, file_name);
        SERIAL_INFO("%s\r\n", file_name);

        if (SD.exists(file_name))
          is_ptr_found = true;
        else ptr_date_time += SECS_PER_DAY;
      }
      else {
        // ptr not found: set it to yesterday at 23:60-REPORT_MINUTES:00 time.
        if (!is_ptr_found) {
          datetime.Year = CalendarYrToTm(year());
          datetime.Month = month();
          datetime.Day = day() - 1;
          datetime.Hour = 23;
          datetime.Minute = 60 - REPORT_MINUTES;
          datetime.Second = 0;
          ptr_date_time = makeTime(datetime);
          is_ptr_found = true;
        }

        is_ptr_updated = true;
        data_processing_state = FOUND_PTR_DATA;
        SERIAL_TRACE("FIND_PTR_DATA ---> FOUND_PTR_DATA\r\n");
      }
      break;

    // ptr_date_time is set in any case.
    case FOUND_PTR_DATA:
      // datafile read, reach eof and is today. END.
      if (is_eof_data_read && year() == year(ptr_date_time) && month() == month(ptr_date_time) && day() == day(ptr_date_time)) {
        data_processing_state = END_MQTT_SERVICE;
        SERIAL_TRACE("FOUND_PTR_DATA ---> END_MQTT_SERVICE\r\n");
      }
      // datafile read, reach eof and NOT is today. go to end of this day.
      else if (is_eof_data_read) {
        datetime.Year = year(ptr_date_time);
        datetime.Month = month(ptr_date_time);
        datetime.Day = day(ptr_date_time);
        datetime.Hour = 23;
        datetime.Minute = 60 - REPORT_MINUTES;
        datetime.Second = 0;
        ptr_date_time = makeTime(datetime);
        data_processing_state = END_FIND_PTR;
        SERIAL_TRACE("FOUND_PTR_DATA ---> END_FIND_PTR\r\n");
      }
      else {
        ptr_date_time++;
        is_eof_data_read = false;
        data_processing_state = END_FIND_PTR;
        SERIAL_TRACE("FOUND_PTR_DATA ---> END_FIND_PTR\r\n");
      }
      break;

    case END_FIND_PTR:
      if (is_ptr_found) {
        SERIAL_INFO("Data pointer... [ OK ]\r\n--> %02u/%02u/%04u %02u:%02u:%02u\r\n\r\n", day(ptr_date_time), month(ptr_date_time), year(ptr_date_time), hour(ptr_date_time), minute(ptr_date_time), second(ptr_date_time));
        data_processing_state = OPEN_SDCARD_READ_DATA_FILE;
        SERIAL_TRACE("END_FIND_PTR ---> OPEN_SDCARD_READ_DATA_FILE\r\n");
      }
      else {
        data_processing_state = INIT_MQTT_SERVICE;
        SERIAL_TRACE("END_FIND_PTR ---> INIT_MQTT_SERVICE\r\n");
      }
      break;

    case OPEN_SDCARD_READ_DATA_FILE:
      // open read data file
      sdcard_make_filename(ptr_date_time + REPORT_MINUTES * 60, file_name);

      if (sdcard_open_file(&SD, &data_file, file_name, O_READ)) {
        retry = 0;
        data_processing_state = INIT_MQTT_SERVICE;
        SERIAL_TRACE("OPEN_SDCARD_READ_DATA_FILE ---> INIT_MQTT_SERVICE\r\n");
      }
      // retry
      else if ((++retry) < DATA_PROCESSING_RETRY_COUNT_MAX) {
        delay_ms = DATA_PROCESSING_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = OPEN_SDCARD_READ_DATA_FILE;
        data_processing_state = WAIT_DATA_PROCESSING_STATE;
        SERIAL_TRACE("OPEN_SDCARD_READ_DATA_FILE ---> WAIT_DATA_PROCESSING_STATE\r\n");
      }
      // fail
      else {
        is_sdcard_error = true;
        data_processing_state = INIT_MQTT_SERVICE;
        SERIAL_TRACE("OPEN_SDCARD_READ_DATA_FILE ---> INIT_MQTT_SERVICE\r\n");
      }
      break;

    case INIT_MQTT_SERVICE:
      i = 0;
      retry = 0;
      is_sdcard_processing = false;

      if (is_sdcard_error) {
        is_mqtt_processing_json = true;
        is_mqtt_processing_sdcard = false;
      }
      else {
        is_mqtt_processing_json = false;
        is_mqtt_processing_sdcard = true;
      }

      if (!mqtt_client.isConnected()) {
        if (mqttConnect(configuration.mqtt_server, configuration.mqtt_port, configuration.mqtt_username, configuration.mqtt_password)) {
          data_processing_state = SUBSCRIBE_MQTT_SERVICE;
          SERIAL_TRACE("INIT_MQTT_SERVICE ---> SUBSCRIBE_MQTT_SERVICE\r\n");
        }
        else {
          SERIAL_ERROR("MQTT Connection... [ FAIL ]\r\n");
          is_mqtt_error = true;
          data_processing_state = END_MQTT_SERVICE;
          SERIAL_TRACE("INIT_MQTT_SERVICE ---> END_MQTT_SERVICE\r\n");
        }
      }
      else {
        data_processing_state = SUBSCRIBE_MQTT_SERVICE;
        SERIAL_TRACE("INIT_MQTT_SERVICE ---> SUBSCRIBE_MQTT_SERVICE\r\n");
      }
      break;

    case SUBSCRIBE_MQTT_SERVICE:
      if (!is_mqtt_subscribed) {
        is_mqtt_subscribed = (mqtt_client.subscribe(configuration.mqtt_subscribe_topic, MQTT::QOS1, mqttRxCallback) == 0);

        if (!is_mqtt_subscribed) {
          SERIAL_ERROR("MQTT Subscription... [ FAIL ]\r\n");
          is_mqtt_error = true;
        }
      }

      if (is_mqtt_processing_json) {
        data_processing_state = LOOP_JSON_TO_MQTT;
        SERIAL_TRACE("SUBSCRIBE_MQTT_SERVICE ---> LOOP_JSON_TO_MQTT\r\n");
      }
      else if (is_mqtt_processing_sdcard) {
        is_eof_data_read = false;
        data_processing_state = LOOP_SD_TO_MQTT;
        SERIAL_TRACE("SUBSCRIBE_MQTT_SERVICE ---> LOOP_SD_TO_MQTT\r\n");
      }
      break;

    case END_MQTT_SERVICE:
      if (is_mqtt_processing_sdcard) {
        data_file.close();
      }

      #if (MODULE_TYPE == STIMA_MODULE_TYPE_SAMPLE_GSM || MODULE_TYPE == STIMA_MODULE_TYPE_REPORT_GSM)
      mqtt_client.disconnect();
      #endif

      data_processing_state = UPDATE_PTR_DATA;
      SERIAL_TRACE("END_MQTT_SERVICE ---> UPDATE_PTR_DATA\r\n");
      break;

    case LOOP_JSON_TO_MQTT:
      if (i < sensors_count) {
        k = 0;
        data_count = jsonToMqtt(&json_sensors_data[i][0], configuration.sensors[i].mqtt_topic, topic_buffer, message_buffer);
        data_processing_state = LOOP_MQTT_TO_X;
        SERIAL_TRACE("LOOP_JSON_TO_MQTT ---> LOOP_MQTT_TO_X\r\n");
      }
      else if (is_sdcard_processing) {
        SERIAL_DEBUG("\r\n");
        data_processing_state = END_SDCARD_SERVICE;
        SERIAL_TRACE("LOOP_JSON_TO_MQTT ---> END_SDCARD_SERVICE\r\n");
      }
      else if (is_mqtt_processing_json) {
        SERIAL_DEBUG("\r\n");
        data_processing_state = END_MQTT_SERVICE;
        SERIAL_TRACE("LOOP_JSON_TO_MQTT ---> END_MQTT_SERVICE\r\n");
      }
      break;

    case LOOP_SD_TO_MQTT:
      memset(sd_buffer, 0, MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH);
      memset(topic_buffer, 0, sizeof(topic_buffer[0][0]) * VALUES_TO_READ_FROM_SENSOR_COUNT * MQTT_SENSOR_TOPIC_LENGTH);
      memset(message_buffer, 0, sizeof(message_buffer[0][0]) * VALUES_TO_READ_FROM_SENSOR_COUNT * MQTT_MESSAGE_LENGTH);

      read_bytes_count = data_file.read(sd_buffer, MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH);

      if (read_bytes_count == MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH) {
        sdToMqtt(sd_buffer, &topic_buffer[0][0], &message_buffer[0][0]);
        current_ptr_time_data = getDateFromMessage(&message_buffer[0][0]);

        if (current_ptr_time_data >= ptr_date_time) {
          data_processing_state = LOOP_MQTT_TO_X;
          SERIAL_TRACE("LOOP_SD_TO_MQTT ---> LOOP_MQTT_TO_X\r\n");
        }
      }
      // EOF: End of File
      else {
        is_eof_data_read = true;
        data_processing_state = FOUND_PTR_DATA;
        SERIAL_TRACE("LOOP_SD_TO_MQTT ---> FOUND_PTR_DATA\r\n");
      }
      break;

    case LOOP_MQTT_TO_X:
      if (k < data_count && is_sdcard_processing) {
        mqttToSd(&topic_buffer[k][0], &message_buffer[k][0], sd_buffer);
        data_processing_state = WRITE_DATA_TO_X;
        SERIAL_TRACE("LOOP_MQTT_TO_X ---> WRITE_DATA_TO_X\r\n");
      }
      else if (k < data_count && is_mqtt_processing_json) {
        getFullTopic(full_topic_buffer, configuration.mqtt_root_topic, &topic_buffer[k][0]);
        data_processing_state = WRITE_DATA_TO_X;
        SERIAL_TRACE("LOOP_MQTT_TO_X ---> WRITE_DATA_TO_X\r\n");
      }
      else if (is_mqtt_processing_sdcard) {
        getFullTopic(full_topic_buffer, configuration.mqtt_root_topic, &topic_buffer[0][0]);
        data_processing_state = WRITE_DATA_TO_X;
        SERIAL_TRACE("LOOP_MQTT_TO_X ---> WRITE_DATA_TO_X\r\n");
      }
      else {
        i++;
        data_processing_state = LOOP_JSON_TO_MQTT;
        SERIAL_TRACE("LOOP_MQTT_TO_X ---> LOOP_JSON_TO_MQTT\r\n");
      }
      break;

    case WRITE_DATA_TO_X:
      // sdcard success
      if (is_sdcard_processing && data_file.write(sd_buffer, MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH) == (MQTT_SENSOR_TOPIC_LENGTH + MQTT_MESSAGE_LENGTH)) {
        SERIAL_DEBUG("SD <-- %s %s\r\n", &topic_buffer[k][0], &message_buffer[k][0]);
        data_file.flush();
        retry = 0;
        k++;
        sd_data_count++;
        data_processing_state = LOOP_MQTT_TO_X;
        SERIAL_TRACE("WRITE_DATA_TO_X ---> LOOP_MQTT_TO_X\r\n");
      }
      // mqtt json success
      else if (is_mqtt_processing_json && mqttPublish(full_topic_buffer, &message_buffer[k][0])) {
        SERIAL_DEBUG("MQTT <-- %s %s\r\n", &topic_buffer[k][0], &message_buffer[k][0]);
        retry = 0;
        k++;
        mqtt_data_count++;
        data_processing_state = LOOP_MQTT_TO_X;
        SERIAL_TRACE("WRITE_DATA_TO_X ---> LOOP_MQTT_TO_X\r\n");
      }
      // mqtt sdcard success
      else if (is_mqtt_processing_sdcard && mqttPublish(full_topic_buffer, &message_buffer[0][0])) {
        SERIAL_DEBUG("MQTT <-- %s %s\r\n", &topic_buffer[0][0], &message_buffer[0][0]);
        retry = 0;
        mqtt_data_count++;
        ptr_date_time = current_ptr_time_data;
        is_ptr_updated = true;
        data_processing_state = LOOP_SD_TO_MQTT;
        SERIAL_TRACE("WRITE_DATA_TO_X ---> LOOP_SD_TO_MQTT\r\n");
      }
      // retry
      else if ((++retry) < DATA_PROCESSING_RETRY_COUNT_MAX) {
        delay_ms = DATA_PROCESSING_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = WRITE_DATA_TO_X;
        data_processing_state = WAIT_DATA_PROCESSING_STATE;
        SERIAL_TRACE("WRITE_DATA_TO_X ---> WAIT_DATA_PROCESSING_STATE\r\n");
      }
      // fail
      else {
        if (is_sdcard_processing) {
          SERIAL_ERROR("SD Card writing data on file %s... [ FAIL ]\r\n", file_name);
          is_sdcard_error = true;
          data_processing_state = END_SDCARD_SERVICE;
          SERIAL_TRACE("WRITE_DATA_TO_X ---> END_SDCARD_SERVICE\r\n");
        }

        if (is_mqtt_processing_json || is_mqtt_processing_sdcard) {
          is_eof_data_read = true;
          is_mqtt_error = true;
          SERIAL_ERROR("MQTT publish... [ FAIL ]\r\n");
          data_processing_state = END_MQTT_SERVICE;
          SERIAL_TRACE("WRITE_DATA_TO_X ---> END_MQTT_SERVICE\r\n");
        }
      }
      break;

    case UPDATE_PTR_DATA:
      if (is_ptr_updated && ptr_date_time < now()) {
        if (ptr_data_file.seekSet(0) && ptr_data_file.write(&ptr_date_time, sizeof(time_t)) == sizeof(time_t)) {
          ptr_data_file.flush();
          ptr_data_file.close();
          breakTime(ptr_date_time, datetime);
          SERIAL_INFO("Last data send: %02u/%02u/%04u %02u:%02u:00\r\n\r\n", datetime.Day, datetime.Month, tmYearToCalendar(datetime.Year), datetime.Hour, datetime.Minute);
          data_processing_state = END_DATA_PROCESSING;
          SERIAL_TRACE("UPDATE_PTR_DATA ---> END_DATA_PROCESSING\r\n");
        }
        else if ((++retry) < DATA_PROCESSING_RETRY_COUNT_MAX) {
          delay_ms = DATA_PROCESSING_RETRY_DELAY_MS;
          start_time_ms = millis();
          state_after_wait = UPDATE_PTR_DATA;
          data_processing_state = WAIT_DATA_PROCESSING_STATE;
          SERIAL_TRACE("UPDATE_PTR_DATA ---> WAIT_DATA_PROCESSING_STATE\r\n");
        }
        // fail
        else {
          SERIAL_ERROR("SD Card writing ptr data on file %s... [ FAIL ]\r\n", SDCARD_PTR_DATA_FILE_NAME);
          data_processing_state = END_DATA_PROCESSING;
          SERIAL_TRACE("UPDATE_PTR_DATA ---> END_DATA_PROCESSING\r\n");
        }
      }
      else {
        data_processing_state = END_DATA_PROCESSING;
        SERIAL_TRACE("UPDATE_PTR_DATA ---> END_DATA_PROCESSING\r\n");
      }
      break;

    case END_DATA_PROCESSING:
      if (!is_sdcard_error) {
        SERIAL_INFO("%u data stored in sdcard\r\n", sd_data_count);
      }

      if (!is_mqtt_error) {
        SERIAL_INFO("%u data send through MQTT\r\n", mqtt_data_count);
      }

      noInterrupts();
      is_event_data_saving = false;
      ready_tasks_count--;
      interrupts();
      data_processing_state = END_DATA_PROCESSING_TASK;
      SERIAL_TRACE("END_DATA_PROCESSING ---> END_DATA_PROCESSING_TASK\r\n");
      break;

    case END_DATA_PROCESSING_TASK:
      SERIAL_ERROR("END_DATA_PROCESSING ?!?!?!?!?!?!?!?!?\r\n");
      break;

    case WAIT_DATA_PROCESSING_STATE:
      if (millis() - start_time_ms > delay_ms) {
        data_processing_state = state_after_wait;
      }
      break;
  }
}
