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

#include "i2c-rain-config.h"
#include <debug.h>
#include <hardware_config.h>
#include <rmap_util.h>
#include <sleep_utility.h>
#include <eeprom_utility.h>
#include <Wire.h>
#include <Time.h>
#include <typedef.h>
#include <registers-rain.h>

/**********************************************************************
 * TYPEDEF
 *********************************************************************/
/*!
  \typedef
  \struct configuration_t
  EEPROM saved configuration this module.
*/
typedef struct {
  uint8_t module_version;   //!< module version saved in eeprom. If matching the MODULE_VERSION, the configuration is up to date
  uint8_t module_type;      //!< module type saved in eeprom. If matching the MODULE_TYPE, the configuration is up to date
  uint8_t i2c_address;      //!< i2c address saved in eeprom
  bool is_oneshot;          //!< oneshot modality saved in eeprom
  bool is_continuous;
} configuration_t;

/*!
  \typedef
  \struct readable_data_t
  Readable data through i2c bus.
*/
typedef struct {
  uint8_t module_type;      //!< module type defined in MODULE_TYPE
  uint8_t module_version;   //!< module version defined in MODULE_VERSION
  rain_t rain;              //!< rain data
} readable_data_t;

/*!
  \typedef
  \struct writable_data_t
  Writable data through i2c bus.
*/
typedef struct {
  uint8_t i2c_address;    //!< i2c address for this module
  bool is_oneshot;        //!< oneshot modality for this module
  bool is_continuous;
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
  \var readable_data_address
  Address of readable i2c register.
*/
volatile uint8_t readable_data_address;

/*!
  \var readable_data_length
  Number of bytes to read at readable_data_address.
*/
volatile uint8_t readable_data_length;

/*!
  \var i2c_rx_data
  Buffer for i2c receive data.
*/
volatile uint8_t i2c_rx_data[I2C_MAX_DATA_LENGTH];

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

/*!
  \var rain_tips_event_occurred_time_ms
  System time (in millisecond) when rain tips occured.
*/
volatile uint32_t rain_tips_event_occurred_time_ms;

bool is_start;
bool is_stop;
bool is_oneshot;
bool is_continuous;

rain_t rain;

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

void commands(void);

void reset_buffers(void);
void exchange_buffers(void);

/**********************************************************************
 * TASKS
 *********************************************************************/
volatile bool is_event_tipping_bucket;

/*! \fn void tipping_bucket_task(void)
 *  \brief Tipping bucket task.
 *  \return void.
 */
void tipping_bucket_task(void);

volatile bool is_event_command_task;

/*! \fn void command_task(void)
 *  \brief Executes the command received on i2c bus.
 *  \return void.
 */
void command_task(void);

/**********************************************************************
 * INTERRUPT HANDLER
 *********************************************************************/

/*! \fn void tipping_bucket_interrupt_handler(void)
 *  \brief Tipping bucket interrupt handler.
 *  \return void.
 */
void tipping_bucket_interrupt_handler(void);

/*! \fn void i2c_request_interrupt_handler(void)
 *  \brief I2C request interrupt handler.
 *  \return void.
 */
void i2c_request_interrupt_handler(void);

/*! \fn void i2c_receive_interrupt_handler(void)
 *  \brief I2C receive interrupt handler.
 *  \return void.
 */
void i2c_receive_interrupt_handler(void);
