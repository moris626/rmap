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

#ifndef _REGISTERS_RAIN_H
#define _REGISTERS_RAIN_H

#include "registers.h"

#define I2C_RAIN_DEFAULT_ADDRESS              (0x21)

#define I2C_RAIN_COMMAND_SAVE                 (0x01)
#define I2C_RAIN_COMMAND_ONESHOT_START        (0x02)
#define I2C_RAIN_COMMAND_ONESHOT_STOP         (0x03)
#define I2C_RAIN_COMMAND_ONESHOT_START_STOP   (0x04)

// Specifying the length in bytes of the data by I2C_RAIN_{DATA_NAME}_LENGTH, the corresponding address is calculated automatically

// Readable registers
#define I2C_RAIN_TYPE_LENGTH                  (0x01)
#define I2C_RAIN_TYPE_ADDRESS                 (I2C_READ_REGISTER_START_ADDRESS)

#define I2C_RAIN_VERSION_LENGTH               (0x01)
#define I2C_RAIN_VERSION_ADDRESS              (I2C_RAIN_TYPE_ADDRESS + I2C_RAIN_TYPE_LENGTH)

#define I2C_RAIN_TIPS_LENGTH                  (0x02)
#define I2C_RAIN_TIPS_ADDRESS                 (I2C_RAIN_VERSION_ADDRESS + I2C_RAIN_VERSION_LENGTH)

// update with last 2 define of writable registers
#define I2C_RAIN_READABLE_DATA_LENGTH         (I2C_RAIN_TIPS_ADDRESS + I2C_RAIN_TIPS_LENGTH - I2C_READ_REGISTER_START_ADDRESS)

// Writeable registers
#define I2C_RAIN_ADDRESS_LENGTH               (0x01)
#define I2C_RAIN_ADDRESS_ADDRESS              (I2C_WRITE_REGISTER_START_ADDRESS)

#define I2C_RAIN_ONESHOT_LENGTH               (0x01)
#define I2C_RAIN_ONESHOT_ADDRESS              (I2C_RAIN_ADDRESS_ADDRESS + I2C_RAIN_ADDRESS_LENGTH)

// update with last 2 define of writable registers
#define I2C_RAIN_WRITABLE_DATA_LENGTH         (I2C_RAIN_ADDRESS_ADDRESS + I2C_RAIN_ADDRESS_LENGTH - I2C_WRITE_REGISTER_START_ADDRESS)

// Readable registers errors checking
#if I2C_RAIN_READ_REGISTERS_LENGTH > I2C_READ_REGISTER_END_ADDRESS
#error "ERROR! Too many readable registers found in RAIN module!!!"
#endif

// Writeable registers errors checking
#if I2C_RAIN_WRITE_REGISTERS_LENGTH > I2C_WRITE_REGISTER_END_ADDRESS
#error "ERROR! Too many writable registers found in RAIN module!!!"
#endif

#endif
