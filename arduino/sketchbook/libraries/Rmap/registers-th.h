/**********************************************************************
 * Copyright (C) 2017  Marco Baldinetti <m.baldinetti@digiteco.it>
 * authors:
 * Marco Baldinetti <m.baldinetti@digiteco.it>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 **********************************************************************/

#ifndef _REGISTERS_TH_H
#define _REGISTERS_TH_H

#include "registers.h"

#define I2C_TH_DEFAULT_ADDRESS                  (0x23)
#define I2C_TH_TEMPERATURE_DEFAULT_ADDRESS      (0x49)
#define I2C_TH_HUMIDITY_DEFAULT_ADDRESS         (0x27)

#define I2C_TH_COMMAND_SAVE                     (0x01)
#define I2C_TH_COMMAND_ONESHOT_START            (0x02)
#define I2C_TH_COMMAND_ONESHOT_STOP             (0x03)
#define I2C_TH_COMMAND_ONESHOT_START_STOP       (0x04)
#define I2C_TH_COMMAND_CONTINUOUS_START         (0x05)
#define I2C_TH_COMMAND_CONTINUOUS_STOP          (0x06)
#define I2C_TH_COMMAND_CONTINUOUS_START_STOP    (0x07)

// Specifying the length in bytes of the data by I2C_RAIN_{DATA_NAME}_LENGTH, the corresponding address is calculated automatically

//------------------------------------------------------------------------------
// Readable registers
//------------------------------------------------------------------------------
#define I2C_TH_TYPE_LENGTH                      (0x01)
#define I2C_TH_TYPE_ADDRESS                     (I2C_READ_REGISTER_START_ADDRESS)

#define I2C_TH_VERSION_LENGTH                   (0x01)
#define I2C_TH_VERSION_ADDRESS                  (I2C_TH_TYPE_ADDRESS + I2C_TH_TYPE_LENGTH)

#define I2C_TH_TEMPERATURE_SAMPLE_LENGTH        (0x02)
#define I2C_TH_TEMPERATURE_SAMPLE_ADDRESS       (I2C_TH_VERSION_ADDRESS + I2C_TH_VERSION_LENGTH)

#define I2C_TH_TEMPERATURE_MED60_LENGTH         (0x02)
#define I2C_TH_TEMPERATURE_MED60_ADDRESS        (I2C_TH_TEMPERATURE_SAMPLE_ADDRESS + I2C_TH_TEMPERATURE_SAMPLE_LENGTH)

#define I2C_TH_TEMPERATURE_MED_LENGTH           (0x02)
#define I2C_TH_TEMPERATURE_MED_ADDRESS          (I2C_TH_TEMPERATURE_MED60_ADDRESS + I2C_TH_TEMPERATURE_MED60_LENGTH)

#define I2C_TH_TEMPERATURE_MAX_LENGTH           (0x02)
#define I2C_TH_TEMPERATURE_MAX_ADDRESS          (I2C_TH_TEMPERATURE_MED_ADDRESS + I2C_TH_TEMPERATURE_MED_LENGTH)

#define I2C_TH_TEMPERATURE_MIN_LENGTH           (0x02)
#define I2C_TH_TEMPERATURE_MIN_ADDRESS          (I2C_TH_TEMPERATURE_MAX_ADDRESS + I2C_TH_TEMPERATURE_MAX_LENGTH)

#define I2C_TH_TEMPERATURE_SIGMA_LENGTH         (0x02)
#define I2C_TH_TEMPERATURE_SIGMA_ADDRESS        (I2C_TH_TEMPERATURE_MIN_ADDRESS + I2C_TH_TEMPERATURE_MIN_LENGTH)

// update with precedent values
#define I2C_TH_TEMPERATURE_DATA_MAX_LENGTH      (0x02)

#define I2C_TH_HUMIDITY_SAMPLE_LENGTH           (0x02)
#define I2C_TH_HUMIDITY_SAMPLE_ADDRESS          (I2C_TH_TEMPERATURE_SIGMA_ADDRESS + I2C_TH_TEMPERATURE_SIGMA_LENGTH)

#define I2C_TH_HUMIDITY_MED60_LENGTH            (0x02)
#define I2C_TH_HUMIDITY_MED60_ADDRESS           (I2C_TH_HUMIDITY_SAMPLE_ADDRESS + I2C_TH_HUMIDITY_SAMPLE_LENGTH)

#define I2C_TH_HUMIDITY_MED_LENGTH              (0x02)
#define I2C_TH_HUMIDITY_MED_ADDRESS             (I2C_TH_HUMIDITY_MED60_ADDRESS + I2C_TH_HUMIDITY_MED60_LENGTH)

#define I2C_TH_HUMIDITY_MAX_LENGTH              (0x02)
#define I2C_TH_HUMIDITY_MAX_ADDRESS             (I2C_TH_HUMIDITY_MED_ADDRESS + I2C_TH_HUMIDITY_MED_LENGTH)

#define I2C_TH_HUMIDITY_MIN_LENGTH              (0x02)
#define I2C_TH_HUMIDITY_MIN_ADDRESS             (I2C_TH_HUMIDITY_MAX_ADDRESS + I2C_TH_HUMIDITY_MAX_LENGTH)

#define I2C_TH_HUMIDITY_SIGMA_LENGTH            (0x02)
#define I2C_TH_HUMIDITY_SIGMA_ADDRESS           (I2C_TH_HUMIDITY_MIN_ADDRESS + I2C_TH_HUMIDITY_MIN_LENGTH)

// update with precedent values
#define I2C_TH_HUMIDITY_DATA_MAX_LENGTH         (0x02)

// update with last 2 define of writable registers
#define I2C_TH_READABLE_DATA_LENGTH             (I2C_TH_HUMIDITY_SIGMA_ADDRESS + I2C_TH_HUMIDITY_SIGMA_LENGTH - I2C_READ_REGISTER_START_ADDRESS)

//------------------------------------------------------------------------------
// Writeable registers
//------------------------------------------------------------------------------
#define I2C_TH_ADDRESS_LENGTH                   (0x01)
#define I2C_TH_ADDRESS_ADDRESS                  (I2C_WRITE_REGISTER_START_ADDRESS)

#define I2C_TH_ONESHOT_LENGTH                   (0x01)
#define I2C_TH_ONESHOT_ADDRESS                  (I2C_TH_ADDRESS_ADDRESS + I2C_TH_ADDRESS_LENGTH)

#define I2C_TH_TEMPERATURE_ADDRESS_LENGTH       (0x01)
#define I2C_TH_TEMPERATURE_ADDRESS_ADDRESS      (I2C_TH_ONESHOT_ADDRESS + I2C_TH_ONESHOT_LENGTH)

#define I2C_TH_HUMIDITY_ADDRESS_LENGTH          (0x01)
#define I2C_TH_HUMIDITY_ADDRESS_ADDRESS         (I2C_TH_TEMPERATURE_ADDRESS_ADDRESS + I2C_TH_TEMPERATURE_ADDRESS_LENGTH)

// update with last 2 define of writable registers
#define I2C_TH_WRITABLE_DATA_LENGTH             (I2C_TH_HUMIDITY_ADDRESS_ADDRESS + I2C_TH_HUMIDITY_ADDRESS_LENGTH - I2C_WRITE_REGISTER_START_ADDRESS)

// Readable registers errors checking
#if I2C_TH_READ_REGISTERS_LENGTH > I2C_READ_REGISTER_END_ADDRESS
#error "ERROR! Too many readable registers found in TH module!!!"
#endif

// Writeable registers errors checking
#if I2C_TH_WRITE_REGISTERS_LENGTH > I2C_WRITE_REGISTER_END_ADDRESS
#error "ERROR! Too many writable registers found in TH module!!!"
#endif

#endif
