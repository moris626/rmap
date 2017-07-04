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
/*! \file i2c-th2.h
    \brief i2c-th include file

    i2c-th include file
*/
#include <debug_config.h>

/*!
  \def SERIAL_TRACE_LEVEL
  Debug level for sketch and library.
*/
#define SERIAL_TRACE_LEVEL I2C_TH_SERIAL_TRACE_LEVEL

#include "i2c-th.h"

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
      break;
    #endif

    case TASKS_EXECUTION:
      if (is_event_sensors_reading)
        sensors_reading_task();

      #if (USE_WDT_TASK)
      if (is_event_wdt)
        wdt_task();
      #endif

      if (is_event_command_task)
        command_task();

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
  reset_buffers();
  memcpy((void *) readable_data_read_ptr, (const void*) readable_data_write_ptr, sizeof(readable_data_t));
}

void init_tasks() {
  noInterrupts();
  ready_tasks_count = 0;

  is_event_command_task = false;
  is_event_sensors_reading = false;

  #if (USE_WDT_TASK)
  is_event_wdt = false;
  #endif

  interrupts();
}

void init_pins() {
  pinMode(CONFIGURATION_RESET_PIN, INPUT_PULLUP);
}

void init_wire() {
  Wire.begin(configuration.i2c_address);
  Wire.setClock(I2C_BUS_CLOCK);
  Wire.onRequest(i2c_request_interrupt_handler);
  Wire.onReceive(i2c_receive_interrupt_handler);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
}

void init_spi() {
}

void init_rtc() {
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
  SERIAL_INFO("--> i2c address: 0x%x (%d)\r\n", configuration.i2c_address, configuration.i2c_address);
  SERIAL_INFO("--> oneshot: %s\r\n", configuration.is_oneshot ? "on" : "off");
  SERIAL_INFO("--> continuous: %s\r\n", configuration.is_continuous ? "on" : "off");
  SERIAL_INFO("--> i2c temperature address: 0x%x (%d)\r\n", configuration.i2c_temperature_address, configuration.i2c_temperature_address);
  SERIAL_INFO("--> i2c humidity address: 0x%x (%d)\r\n\r\n", configuration.i2c_humidity_address, configuration.i2c_temperature_address);
}

void save_configuration(bool is_default) {
  if (is_default) {
    SERIAL_INFO("Save default configuration... [ OK ]\r\n");
    configuration.module_type = MODULE_TYPE;
    configuration.module_version = MODULE_VERSION;
    configuration.i2c_address = CONFIGURATION_DEFAULT_I2C_ADDRESS;
    configuration.is_oneshot = CONFIGURATION_DEFAULT_IS_ONESHOT;
    configuration.is_continuous = CONFIGURATION_DEFAULT_IS_CONTINUOUS;
    configuration.i2c_temperature_address = CONFIGURATION_DEFAULT_TEMPERATURE_ADDRESS;
    configuration.i2c_humidity_address = CONFIGURATION_DEFAULT_HUMIDITY_ADDRESS;

    #if (USE_SENSOR_ADT)
    configuration.i2c_temperature_address = 0x28;
    #endif

    #if (USE_SENSOR_HIH)
    configuration.i2c_humidity_address = 0x28;
    #endif

    #if (USE_SENSOR_HYT)
    configuration.i2c_temperature_address = HYT271_DEFAULT_ADDRESS;
    configuration.i2c_humidity_address = HYT271_DEFAULT_ADDRESS;
    #endif
  }
  else {
    SERIAL_INFO("Save configuration... [ OK ]\r\n");
    configuration.i2c_address = writable_data.i2c_address;
    configuration.is_oneshot = writable_data.is_oneshot;
    configuration.is_continuous = writable_data.is_continuous;
    configuration.i2c_temperature_address = writable_data.i2c_temperature_address;
    configuration.i2c_humidity_address = writable_data.i2c_humidity_address;
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

  writable_data.i2c_address = configuration.i2c_address;
  writable_data.is_oneshot = configuration.is_oneshot;
}

void init_sensors () {
  is_first_run = true;
  sensors_count = 0;

  SERIAL_INFO("Sensors...\r\n");

  #if (USE_SENSOR_ADT)
  SensorDriver::createAndSetup(SENSOR_DRIVER_I2C, SENSOR_TYPE_ADT, &is_sensor_adt_setted, &is_sensor_adt_prepared, configuration.i2c_temperature_address, sensors, &sensors_count);
  SERIAL_INFO("--> %u: %s-%s\r\n", sensors_count, SENSOR_DRIVER_I2C, SENSOR_TYPE_ADT);
  #endif

  #if (USE_SENSOR_HIH)
  SensorDriver::createAndSetup(SENSOR_DRIVER_I2C, SENSOR_TYPE_HIH, &is_sensor_hih_setted, &is_sensor_hih_prepared, configuration.i2c_humidity_address, sensors, &sensors_count);
  SERIAL_INFO("--> %u: %s-%s\r\n", sensors_count, SENSOR_DRIVER_I2C, SENSOR_TYPE_HIH);
  #endif

  #if (USE_SENSOR_HYT)
  SensorDriver::createAndSetup(SENSOR_DRIVER_I2C, SENSOR_TYPE_HYT, &is_sensor_hyt_setted, &is_sensor_hyt_prepared, configuration.i2c_temperature_address, sensors, &sensors_count);
  SERIAL_INFO("--> %u: %s-%s\r\n", sensors_count, SENSOR_DRIVER_I2C, SENSOR_TYPE_HYT);
  #endif

  SERIAL_INFO("\r\n");

  if (configuration.is_continuous) {
    #if (REPORT_MINUTES != OBSERVATIONS_MINUTES)
    SERIAL_INFO("--> acquiring %u~%u samples in %u minutes\r\n\r\n", SAMPLE_COUNT_MIN, SAMPLE_COUNT_MAX, OBSERVATIONS_MINUTES);
    #else
    SERIAL_INFO("--> acquiring %u samples in %u minutes\r\n\r\n", SAMPLE_COUNT_MIN, OBSERVATIONS_MINUTES);
    #endif
    SERIAL_INFO("T-IST\tT-MIN\tT-MED\tT-MAX\tH-IST\tH-MIN\tH-MED\tH-MAX");
    SERIAL_DEBUG("\tT-CNT\tH-CNT");
    SERIAL_INFO("\r\n");
  }
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

void i2c_request_interrupt_handler() {
  // for (uint8_t i=0; i<readable_data_length; i++)
    // Wire.write(((uint8_t *)readable_data_read_ptr+readable_data_address)[i]);

  Wire.write((uint8_t *)readable_data_read_ptr+readable_data_address, readable_data_length);
}

void i2c_receive_interrupt_handler(int rx_data_length) {
  for (uint8_t i=0; i<rx_data_length; i++)
    i2c_rx_data[i] = Wire.read();

  if (rx_data_length == 2 && is_readable_register(i2c_rx_data[0])) {
    readable_data_address = i2c_rx_data[0];
    readable_data_length = i2c_rx_data[1];
  }
  else if (rx_data_length == 2 && is_command(i2c_rx_data[0])) {
    noInterrupts();
    if (!is_event_command_task) {
      is_event_command_task = true;
      ready_tasks_count++;
    }
    interrupts();
  }
  else if (is_writable_register(i2c_rx_data[0])) {
    for (uint8_t i=1; i<rx_data_length; i++)
      ((uint8_t *)writable_data_ptr)[i2c_rx_data[0] - I2C_WRITE_REGISTER_START_ADDRESS] = i2c_rx_data[i];
  }
}

#if (USE_WDT_TASK)
void wdt_task() {

  if (executeWdtTaskEach(SAMPLE_SECONDS) && configuration.is_continuous && is_continuous && is_start) {
    sensor_state = INIT_SENSOR;
    noInterrupts();
    if (!is_event_sensors_reading) {
      is_event_sensors_reading = true;
      ready_tasks_count++;
    }
    interrupts();
  }

  noInterrupts();
  is_event_wdt = false;
  ready_tasks_count--;
  interrupts();
}
#endif

void samples_processing() {
  uint8_t i;
  uint32_t temperature = 0;
  uint32_t humidity = 0;

  bool is_processing_temperature = (temperature_samples.count + temperature_samples.error_count == samples_count && temperature_samples.error_count < SAMPLE_COUNT_TOLLERANCE);
  bool is_processing_humidity = (humidity_samples.count + humidity_samples.error_count == samples_count && humidity_samples.error_count < SAMPLE_COUNT_TOLLERANCE);

  if (is_processing_temperature) {
    for (i = 0; i < temperature_samples.count; i++)
      temperature += temperature_samples.values[i];

    // average
    temperature = temperature / temperature_samples.count;
    temperature_observations.med[temperature_observations.count++] = temperature;
    SERIAL_DEBUG("%u\t \t \t \t", temperature);
  }

  if (is_processing_humidity) {
    for (i = 0; i < humidity_samples.count; i++)
      humidity += humidity_samples.values[i];

    // average
    humidity = humidity / humidity_samples.count;
    humidity_observations.med[humidity_observations.count++] = humidity;
    SERIAL_DEBUG("%u\t \t \t \t", humidity);
  }

  if (is_processing_temperature || is_processing_humidity) {
    SERIAL_DEBUG("%u/%u\t%u/%u\r\n", temperature_samples.count, samples_count, humidity_samples.count, samples_count);
    temperature_samples.count = 0;
    humidity_samples.count = 0;
    #if (REPORT_MINUTES != OBSERVATIONS_MINUTES)
    samples_count = (samples_count == SAMPLE_COUNT_MAX ? SAMPLE_COUNT_MIN : SAMPLE_COUNT_MAX);
    #endif
  }
}

//------------------------------------------------------------------------------
// I2C-TH
// STH: oneshot                   --> xxx.sample
// ITH: continuous istantaneous   --> xxx.med60
// MTH: continuous average        --> xxx.med
// NTH: continuous min            --> xxx.min
// XTH: continuous max            --> xxx.max
//------------------------------------------------------------------------------
void observations_processing() {
  uint8_t i;
  uint32_t temperature = 0;
  uint32_t humidity = 0;
  float sigma_temperature = 0;
  float sigma_humidity = 0;

  bool is_processing_temperature = (temperature_observations.count == OBSERVATION_COUNT);
  bool is_processing_humidty = (humidity_observations.count == OBSERVATION_COUNT);

  if (is_processing_temperature) {
    readable_data_write_ptr->temperature.max = 0;

    for (i = 0; i < temperature_observations.count; i++) {
      temperature += temperature_observations.med[i];
      readable_data_write_ptr->temperature.min = min(readable_data_write_ptr->temperature.min, temperature_observations.med[i]);
      readable_data_write_ptr->temperature.max = max(readable_data_write_ptr->temperature.max, temperature_observations.med[i]);
    }

    // average
    readable_data_write_ptr->temperature.med = temperature / temperature_observations.count;

    // last observation
    readable_data_write_ptr->temperature.med60 = temperature_observations.med[temperature_observations.count-1];

    // last sample
    readable_data_write_ptr->temperature.sample = temperature_samples.values[temperature_samples.count-1];

    // standard deviation
    for (i = 0; i < temperature_observations.count; i++)
      sigma_temperature += pow(temperature_observations.med[i] - readable_data_write_ptr->temperature.med, 2);

    sigma_temperature = sqrt(sigma_temperature/temperature_observations.count);

    // da controllare !!!
    readable_data_write_ptr->temperature.sigma = sigma_temperature * 100;
  }

  if (is_processing_humidty) {
    readable_data_write_ptr->humidity.max = 0;

    for (i = 0; i < humidity_observations.count; i++) {
      humidity += humidity_observations.med[i];
      readable_data_write_ptr->humidity.min = min(readable_data_write_ptr->humidity.min, humidity_observations.med[i]);
      readable_data_write_ptr->humidity.max = max(readable_data_write_ptr->humidity.max, humidity_observations.med[i]);
    }

    // average
    readable_data_write_ptr->humidity.med = humidity / humidity_observations.count;

    // last observation
    readable_data_write_ptr->humidity.med60 = humidity_observations.med[humidity_observations.count-1];

    // last sample
    readable_data_write_ptr->humidity.sample = humidity_samples.values[humidity_samples.count-1];

    // standard deviation
    for (i = 0; i < humidity_observations.count; i++)
      sigma_humidity += pow(humidity_observations.med[i] - readable_data_write_ptr->humidity.med, 2);

    sigma_humidity = sqrt(sigma_humidity / humidity_observations.count);

    // da controllare !!!
    readable_data_write_ptr->humidity.sigma = sigma_humidity * 100;
  }

  #if (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_INFO)
  if (is_processing_temperature || is_processing_humidty) {
    if (readable_data_write_ptr->temperature.med60 != UINT16_MAX)
    SERIAL_INFO("%u\t", readable_data_write_ptr->temperature.med60);
    else SERIAL_INFO("-----\t");

    if (readable_data_write_ptr->temperature.min != UINT16_MAX)
    SERIAL_INFO("%u\t", readable_data_write_ptr->temperature.min);
    else SERIAL_INFO("-----\t");

    if (readable_data_write_ptr->temperature.med != UINT16_MAX)
    SERIAL_INFO("%u\t", readable_data_write_ptr->temperature.med);
    else SERIAL_INFO("-----\t");

    if (readable_data_write_ptr->temperature.max != UINT16_MAX)
    SERIAL_INFO("%u\t", readable_data_write_ptr->temperature.max);
    else SERIAL_INFO("-----\t");

    if (readable_data_write_ptr->humidity.med60 != UINT16_MAX)
    SERIAL_INFO("%u\t", readable_data_write_ptr->humidity.med60);
    else SERIAL_INFO("-----\t");

    if (readable_data_write_ptr->humidity.min != UINT16_MAX)
    SERIAL_INFO("%u\t", readable_data_write_ptr->humidity.min);
    else SERIAL_INFO("-----\t");

    if (readable_data_write_ptr->humidity.med != UINT16_MAX)
    SERIAL_INFO("%u\t", readable_data_write_ptr->humidity.med);
    else SERIAL_INFO("-----\t");

    if (readable_data_write_ptr->humidity.max != UINT16_MAX)
    SERIAL_INFO("%u\r\n", readable_data_write_ptr->humidity.max);
    else SERIAL_INFO("-----\r\n");
  }
  #endif
}

void observations_processing_debug() {
  readable_data_write_ptr->temperature.med60 = 29999;
  readable_data_write_ptr->temperature.max = 30010;
  readable_data_write_ptr->temperature.med = 30000;
  readable_data_write_ptr->temperature.min = 30000;

  readable_data_write_ptr->humidity.med60 = 60;
  readable_data_write_ptr->humidity.max = 100;
  readable_data_write_ptr->humidity.med = 50;
  readable_data_write_ptr->humidity.min = 0;

  #if (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_INFO)
  if (readable_data_write_ptr->temperature.med60 != UINT16_MAX)
  SERIAL_INFO("%u\t", readable_data_write_ptr->temperature.med60);
  else SERIAL_INFO("-----\t");

  if (readable_data_write_ptr->temperature.min != UINT16_MAX)
  SERIAL_INFO("%u\t", readable_data_write_ptr->temperature.min);
  else SERIAL_INFO("-----\t");

  if (readable_data_write_ptr->temperature.med != UINT16_MAX)
  SERIAL_INFO("%u\t", readable_data_write_ptr->temperature.med);
  else SERIAL_INFO("-----\t");

  if (readable_data_write_ptr->temperature.max != UINT16_MAX)
  SERIAL_INFO("%u\t", readable_data_write_ptr->temperature.max);
  else SERIAL_INFO("-----\t");

  if (readable_data_write_ptr->humidity.med60 != UINT16_MAX)
  SERIAL_INFO("%u\t", readable_data_write_ptr->humidity.med60);
  else SERIAL_INFO("-----\t");

  if (readable_data_write_ptr->humidity.min != UINT16_MAX)
  SERIAL_INFO("%u\t", readable_data_write_ptr->humidity.min);
  else SERIAL_INFO("-----\t");

  if (readable_data_write_ptr->humidity.med != UINT16_MAX)
  SERIAL_INFO("%u\t", readable_data_write_ptr->humidity.med);
  else SERIAL_INFO("-----\t");

  if (readable_data_write_ptr->humidity.max != UINT16_MAX)
  SERIAL_INFO("%u\r\n", readable_data_write_ptr->humidity.max);
  else SERIAL_INFO("-----\r\n");
  #endif
}

void sensors_reading_task () {
  static uint8_t i;
  static uint8_t retry;
  static sensor_state_t state_after_wait;
  static uint32_t delay_ms;
  static uint32_t start_time_ms;
  static int32_t values_readed_from_sensor[2];

  switch (sensor_state) {
    case INIT_SENSOR:
      i = 0;
      retry = 0;
      state_after_wait = INIT_SENSOR;
      sensor_state = PREPARE_SENSOR;
      break;

    case PREPARE_SENSOR:
      sensors[i]->prepare();
      delay_ms = sensors[i]->getDelay();
      start_time_ms = sensors[i]->getStartTime();
      state_after_wait = IS_SENSOR_PREPARED;
      sensor_state = WAIT_STATE;
      break;

    case IS_SENSOR_PREPARED:
      // success
      if (sensors[i]->isPrepared()) {
        sensor_state = GET_SENSOR;
      }
      // retry
      else if (++retry < SENSORS_RETRY_COUNT_MAX) {
        delay_ms = SENSORS_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = PREPARE_SENSOR;
        sensor_state = WAIT_STATE;
      }
      // fail
      else sensor_state = END_SENSOR_READING;
      break;

    case GET_SENSOR:
      sensors[i]->get(values_readed_from_sensor, VALUES_TO_READ_FROM_SENSOR_COUNT);
      delay_ms = sensors[i]->getDelay();
      start_time_ms = sensors[i]->getStartTime();
      state_after_wait = IS_SENSOR_GETTED;
      sensor_state = WAIT_STATE;
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
      else if (++retry < SENSORS_RETRY_COUNT_MAX) {
        delay_ms = SENSORS_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = PREPARE_SENSOR;
        sensor_state = WAIT_STATE;
      }
      // fail
      else {
        #if (USE_SENSOR_HYT || USE_SENSOR_ADT)
        temperature_samples.error_count++;
        #endif

        #if (USE_SENSOR_HYT || USE_SENSOR_HIH)
        humidity_samples.error_count++;
        #endif

        sensor_state = END_SENSOR_READING;
      }
      break;

    case READ_SENSOR:
      #if (USE_SENSOR_HYT)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_HYT) == 0) {
        humidity_samples.values[humidity_samples.count++] = values_readed_from_sensor[0];
        temperature_samples.values[temperature_samples.count++] = values_readed_from_sensor[1];
      }
      #endif
      sensor_state = END_SENSOR_READING;
      break;

    case END_SENSOR_READING:
      // next sensor
      if (++i < sensors_count) {
        retry = 0;
        sensor_state = PREPARE_SENSOR;
      }
      // end
      else {
        if (configuration.is_continuous) {
          samples_processing();
          // observations_processing_debug();

          if (!is_first_run)
            observations_processing();
          else is_first_run = false;
        }

        for (i=0; i<sensors_count; i++)
          sensors[i]->resetPrepared();

        #if (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_INFO)
        delay_ms = 5;
        start_time_ms = millis();
        state_after_wait = END_TASK;
        sensor_state = WAIT_STATE;
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

    case WAIT_STATE:
      if (millis() - start_time_ms > delay_ms) {
        sensor_state = state_after_wait;
      }
      break;
  }
}

void exchange_buffers() {
  readable_data_temp_ptr = readable_data_write_ptr;
  readable_data_write_ptr = readable_data_read_ptr;
  readable_data_read_ptr = readable_data_temp_ptr;
}

void reset_buffers() {
  memset((void *) &readable_data_write_ptr->humidity, UINT16_MAX, sizeof(value_t));
  memset((void *) &readable_data_write_ptr->temperature, UINT16_MAX, sizeof(value_t));
  humidity_samples.count = 0;
  temperature_samples.count = 0;
  humidity_samples.error_count = 0;
  temperature_samples.error_count = 0;
  humidity_observations.count = 0;
  temperature_observations.count = 0;

  #if ((REPORT_MINUTES / OBSERVATIONS_MINUTES) % 2 == 0)
  samples_count = SAMPLE_COUNT_MAX;
  #else
  samples_count = SAMPLE_COUNT_MIN;
  #endif
}

void command_task() {
  #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
  char buffer[30];
  #endif

  switch(i2c_rx_data[1]) {
    case I2C_TH_COMMAND_ONESHOT_START:
    #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
    strcpy(buffer, "ONESHOT START");
    #endif
    is_oneshot = true;
    is_continuous = false;
    is_start = true;
    is_stop = false;
    commands();
    break;

    case I2C_TH_COMMAND_ONESHOT_STOP:
    #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
    strcpy(buffer, "ONESHOT STOP");
    #endif
    is_oneshot = true;
    is_continuous = false;
    is_start = false;
    is_stop = true;
    commands();
    break;

    case I2C_TH_COMMAND_ONESHOT_START_STOP:
    #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
    strcpy(buffer, "ONESHOT START-STOP");
    #endif
    is_oneshot = true;
    is_continuous = false;
    is_start = true;
    is_stop = true;
    commands();
    break;

    case I2C_TH_COMMAND_CONTINUOUS_START:
    #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
    strcpy(buffer, "CONTINUOUS START");
    #endif
    is_oneshot = false;
    is_continuous = true;
    is_start = true;
    is_stop = false;
    commands();
    // starttime = millis();
    break;

    case I2C_TH_COMMAND_CONTINUOUS_STOP:
    #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
    strcpy(buffer, "CONTINUOUS STOP");
    #endif
    is_oneshot = false;
    is_continuous = true;
    is_start = false;
    is_stop = true;
    commands();
    break;

    case I2C_TH_COMMAND_CONTINUOUS_START_STOP:
    #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
    strcpy(buffer, "CONTINUOUS START-STOP");
    #endif
    is_oneshot = false;
    is_continuous = true;
    is_start = true;
    is_stop = true;
    commands();
    // starttime = millis();
    break;

    case I2C_TH_COMMAND_SAVE:
    is_oneshot = false;
    is_continuous = false;
    is_start = false;
    is_stop = false;
    SERIAL_TRACE("Execute command [ SAVE ]\r\n");
    save_configuration(CONFIGURATION_CURRENT);
    break;
  }

  #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
  if (configuration.is_oneshot == is_oneshot || configuration.is_continuous == is_continuous) {
    SERIAL_TRACE("Execute [ %s ]\r\n", buffer);
  }
  else if (configuration.is_oneshot == is_continuous || configuration.is_continuous == is_oneshot) {
    SERIAL_TRACE("Ignore [ %s ]\r\n", buffer);
  }
  #endif

  noInterrupts();
  is_event_command_task = false;
  ready_tasks_count--;
  interrupts();
}

void commands() {
  noInterrupts();

  if (configuration.is_oneshot && is_oneshot && is_stop) {
    if (temperature_samples.count == 1) {
      readable_data_write_ptr->temperature.sample = temperature_samples.values[0];
      SERIAL_INFO("--> temperature: %u\r\n", readable_data_write_ptr->temperature.sample);
    }
    else SERIAL_INFO("--> temperature: ---\r\n");

    if (humidity_samples.count == 1) {
      readable_data_write_ptr->humidity.sample = humidity_samples.values[0];
      SERIAL_INFO("--> humidity: %u\r\n", readable_data_write_ptr->humidity.sample);
    }
    else SERIAL_INFO("--> humidity: ---\r\n");

    exchange_buffers();
  }
  else if (configuration.is_continuous && is_continuous && is_stop) {
    exchange_buffers();
  }

  if (configuration.is_oneshot && is_oneshot && is_start) {
    reset_buffers();
    sensor_state = INIT_SENSOR;
    if(!is_event_sensors_reading) {
      is_event_sensors_reading = true;
      ready_tasks_count++;
    }
  }
  else if (configuration.is_continuous && is_continuous && is_start) {
    #if (USE_WDT_TASK)
    // wdt_disable();
    // wdt_init(WDT_TIMER);
    wdt_reset();
    #endif
    reset_buffers();
  }
  else if (is_start) {
    reset_buffers();
    exchange_buffers();
  }

  interrupts();
}
