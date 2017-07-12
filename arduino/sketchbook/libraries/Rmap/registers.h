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

#define I2C_READ_REGISTER_START_ADDRESS     (0x00)
#define I2C_READ_REGISTER_END_ADDRESS       (0x1E)

#define I2C_WRITE_REGISTER_START_ADDRESS    (0x1F)
#define I2C_WRITE_REGISTER_END_ADDRESS      (0xFE)

#define CONFIGURATION_EEPROM_ADDRESS        (0x00)

#define I2C_COMMAND_ID                      (0xFF)

#define MISS_INT_VALUE                      (0xFFFF)

#define is_readable_register(register)      (register <= I2C_READ_REGISTER_END_ADDRESS)
#define is_writable_register(register)      (I2C_WRITE_REGISTER_START_ADDRESS <= register && register <= I2C_WRITE_REGISTER_END_ADDRESS)
#define is_command(value)                   (value == I2C_COMMAND_ID)
