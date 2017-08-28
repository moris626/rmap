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
/*! \file i2c-rain2.h
    \brief i2c-rain include file

    i2c-rain include file
*/
#include <debug_config.h>

/*!
  \def SERIAL_TRACE_LEVEL
  Debug level for sketch and library.
*/
#define SERIAL_TRACE_LEVEL I2C_RAIN_SERIAL_TRACE_LEVEL

#include "i2c-rain.h"

void setup() {
  init_wdt();
  TRACE_BEGIN(115200);
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
      if (is_event_tipping_bucket && configuration.is_oneshot && is_oneshot && is_start) {
        tipping_bucket_task();
        wdt_reset();
      }

      if (is_event_command_task) {
        command_task();
        wdt_reset();
      }

      wdt_reset();

      if (ready_tasks_count == 0)
        state = END;
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
  reset_buffers();
  memcpy((void *) readable_data_read_ptr, (const void*) readable_data_write_ptr, sizeof(readable_data_t));
}

void init_tasks() {
  noInterrupts();
  ready_tasks_count = 0;

  is_event_command_task = false;
  is_event_tipping_bucket = false;

  rain_tips_event_occurred_time_ms = 0;
  interrupts();
}

void init_pins() {
  pinMode(CONFIGURATION_RESET_PIN, INPUT_PULLUP);
  pinMode(TIPPING_BUCKET_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TIPPING_BUCKET_PIN), tipping_bucket_interrupt_handler, FALLING);
}

void init_wire() {
  Wire.begin(configuration.i2c_address);
  Wire.setClock(I2C_BUS_CLOCK);
  Wire.onRequest(i2c_request_interrupt_handler);
  Wire.onReceive(i2c_receive_interrupt_handler);
}

void init_spi() {
}

void init_rtc() {
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
}

void print_configuration() {
  char stima_name[20];
  getStimaNameByType(stima_name, configuration.module_type);
  SERIAL_INFO("--> type: %s\r\n", stima_name);
  SERIAL_INFO("--> version: %d\r\n", configuration.module_version);
  SERIAL_INFO("--> i2c address: 0x%x (%d)\r\n", configuration.i2c_address, configuration.i2c_address);
  SERIAL_INFO("--> oneshot: %s\r\n", configuration.is_oneshot ? "on" : "off");
  SERIAL_INFO("--> continuous: %s\r\n", configuration.is_continuous ? "on" : "off");
}

void save_configuration(bool is_default) {
  if (is_default) {
    SERIAL_INFO("Save default configuration... [ OK ]\r\n");
    configuration.module_type = MODULE_TYPE;
    configuration.module_version = MODULE_VERSION;
    configuration.i2c_address = CONFIGURATION_DEFAULT_I2C_ADDRESS;
    configuration.is_oneshot = CONFIGURATION_DEFAULT_IS_ONESHOT;
    configuration.is_continuous = CONFIGURATION_DEFAULT_IS_CONTINUOUS;
  }
  else {
    SERIAL_INFO("Save configuration... [ OK ]\r\n");
    configuration.i2c_address = writable_data.i2c_address;
    configuration.is_oneshot = writable_data.is_oneshot;
    configuration.is_continuous = writable_data.is_continuous;
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

void tipping_bucket_interrupt_handler() {
  if (digitalRead(TIPPING_BUCKET_PIN) == LOW) {
    rain_tips_event_occurred_time_ms = millis();
    noInterrupts();
    if (!is_event_tipping_bucket) {
      is_event_tipping_bucket = true;
      ready_tasks_count++;
    }
    interrupts();
  }
}

void i2c_request_interrupt_handler() {
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

void tipping_bucket_task() {
  if (millis() - rain_tips_event_occurred_time_ms > DEBOUNCING_TIPPING_BUCKET_TIME_MS) {
    rain.tips_count++;
    rain_tips_event_occurred_time_ms = millis();

    noInterrupts();
    is_event_tipping_bucket = false;
    ready_tasks_count--;
    interrupts();
  }
}

void exchange_buffers() {
  readable_data_temp_ptr = readable_data_write_ptr;
  readable_data_write_ptr = readable_data_read_ptr;
  readable_data_read_ptr = readable_data_temp_ptr;
}

void reset_buffers() {
  memset((void *) &readable_data_write_ptr->rain, UINT16_MAX, sizeof(rain_t));
  rain.tips_count = 0;
}

void command_task() {
  #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
  char buffer[30];
  #endif

  switch(i2c_rx_data[1]) {
    case I2C_RAIN_COMMAND_ONESHOT_START:
      #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
      strcpy(buffer, "ONESHOT START");
      #endif
      is_oneshot = true;
      is_continuous = false;
      is_start = true;
      is_stop = false;
      commands();
    break;

    case I2C_RAIN_COMMAND_ONESHOT_STOP:
      #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
      strcpy(buffer, "ONESHOT STOP");
      #endif
      is_oneshot = true;
      is_continuous = false;
      is_start = false;
      is_stop = true;
      commands();
    break;

    case I2C_RAIN_COMMAND_ONESHOT_START_STOP:
      #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
      strcpy(buffer, "ONESHOT START-STOP");
      #endif
      is_oneshot = true;
      is_continuous = false;
      is_start = true;
      is_stop = true;
      commands();
    break;

    case I2C_RAIN_COMMAND_SAVE:
      is_oneshot = false;
      is_continuous = false;
      is_start = false;
      is_stop = false;
      SERIAL_DEBUG("Execute command [ SAVE ]\r\n");
      save_configuration(CONFIGURATION_CURRENT);
    break;
  }

  #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
  if (configuration.is_oneshot == is_oneshot || configuration.is_continuous == is_continuous) {
    SERIAL_DEBUG("Execute [ %s ]\r\n", buffer);
  }
  else if (configuration.is_oneshot == is_continuous || configuration.is_continuous == is_oneshot) {
    SERIAL_DEBUG("Ignore [ %s ]\r\n", buffer);
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
    readable_data_write_ptr->rain.tips_count = rain.tips_count * RAIN_FOR_TIP;
    SERIAL_INFO("Total rain : %u\r\n", readable_data_write_ptr->rain.tips_count);
    exchange_buffers();
  }

  if (configuration.is_oneshot && is_oneshot && is_start) {
    reset_buffers();
  }
  else if (is_start) {
    reset_buffers();
    exchange_buffers();
  }

  interrupts();
}
