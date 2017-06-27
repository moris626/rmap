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

#include "hyt271.h"

namespace Hyt271 {

  uint32_t initRead(uint8_t address) {
    Wire.beginTransmission(address);
    return HYT271_CONVERSION_TIME_MS;
  }

  /*
  * Function:  read
  * ----------------------------
  *   Returns the humidty and temperature from hyt271 sensor at address
  *
  *   raw_data: 4 byte raw data readed from the sensor: msb humidity | lsb humidity | msb temperature | lsb temperature
  *   humidity: pointer to humidity variable
  *   temperature: pointer to temperature variable
  *
  *   returns: value of humidity and temperature
  */
  bool read(int8_t address, float *humidity, float *temperature) {
    unsigned long raw_data = 0xFFFF;

    // Request 4 bytes: 2 bytes for Humidity and 2 bytes for Temperature
    Wire.requestFrom(address, HYT271_READ_HT_DATA_LENGTH);
	 
	 if (Wire.available() < HYT271_READ_HT_DATA_LENGTH)
		 return false;

    // read 4 bytes of raw data
    raw_data = (unsigned long) Wire.read() << 24 | (unsigned long) Wire.read() << 16 | (unsigned long) Wire.read() << 8 | (unsigned long) Wire.read();

    // extract 14 bit humidity right adjusted (bit 0-14)
    *humidity = 100.0 / 0x3FFF * (raw_data >> 16 & 0x3FFF);

	 // extract 14 bit temperature left adjusted (bit 2-16)
    *temperature = 165.0 / 0x3FFF * (((unsigned int) raw_data) >> 2) - 40;

    if (Wire.endTransmission())
		return false;
	 
	 return true;
  }

  void send(int8_t address, uint8_t data_0, uint8_t data_1, uint8_t data_2) {
    Wire.beginTransmission(address);
    Wire.write(data_0);
    Wire.write(data_1);
    Wire.write(data_2);
    Wire.endTransmission();
  }

  void changeAddress(uint8_t power_pin, int8_t address, int8_t new_address) {
    off(power_pin);
    on(power_pin);
    send(address, HYT271_ENTER_COMMAND_MODE, 0x00, 0x00);
    send(address, HYT271_WRITE_ADDRESS, 0x00, new_address);
    send(address, HYT271_EXIT_COMMAND_MODE, 0x00, 0x00);
  }

  void init(uint8_t power_pin) {
    pinMode(power_pin, OUTPUT);
  }

  void on(uint8_t power_pin) {
    digitalWrite(power_pin, HIGH);
  }

  void off(uint8_t power_pin) {
    digitalWrite(power_pin, LOW);
  }
}
