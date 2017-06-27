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

#include "ntp.h"

namespace Ntp {
  uint8_t ntp_buffer[NTP_PACKET_LENGTH];
  uint32_t seconds_since_1900;

  bool sendRequest(EthernetUDP *client, const char *server) {
    memset(ntp_buffer, 0, NTP_PACKET_LENGTH);
    ntp_buffer[0] = 0b11100011;   // LI, Version, Mode
    ntp_buffer[1] = 0;     // Stratum, or type of clock
    ntp_buffer[2] = 6;     // Polling Interval
    ntp_buffer[3] = 0xEC;  // Peer Clock Precision
    ntp_buffer[12]  = 49;
    ntp_buffer[13]  = 0x4E;
    ntp_buffer[14]  = 49;
    ntp_buffer[15]  = 52;

    if (!client->beginPacket(server, NTP_SERVER_PORT))
      return false;

    if (!client->write(ntp_buffer, NTP_PACKET_LENGTH))
      return false;

    if (!client->endPacket())
      return false;

    return true;
  }

  bool getResponse (EthernetUDP *client) {
    if (client->parsePacket() < NTP_PACKET_LENGTH)
      return false;

    if (client->read(ntp_buffer, NTP_PACKET_LENGTH) <= 0)
      return false;

    seconds_since_1900 =  (uint32_t) ntp_buffer[NTP_RECEIVE_TIMESTAMP_OFFSET] << 24;
    seconds_since_1900 |= (uint32_t) ntp_buffer[NTP_RECEIVE_TIMESTAMP_OFFSET+1] << 16;
    seconds_since_1900 |= (uint32_t) ntp_buffer[NTP_RECEIVE_TIMESTAMP_OFFSET+2] << 8;
    seconds_since_1900 |= (uint32_t) ntp_buffer[NTP_RECEIVE_TIMESTAMP_OFFSET+3];
    seconds_since_1900 = seconds_since_1900 - NTP_70_YEARS_SECONDS + NTP_TIMEZONE * NTP_1_HOUR_SECONDS;
    return true;
  }

  uint32_t getTime() {
    return seconds_since_1900;
  }
}
